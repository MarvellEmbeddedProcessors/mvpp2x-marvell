/*
 * Driver for Marvell PPv2x family network controllers.
 *
 * Copyright (C) 2015 Marvell
 *
 *.
 *.This program is free software; you can redistribute it and/or modify it
 *.under the terms and conditions of the GNU General Public License,
 *.version 2, as published by the Free Software Foundation.
 *
 *.This program is distributed in the hope it will be useful, but WITHOUT
 *.ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *.FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *.more details.
 *
 *.You should have received a copy of the GNU General Public License along with
 *.this program; if not, write to the Free Software Foundation, Inc.,
 *.51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *.The full GNU General Public License is included in this distribution in
 *.the file called "COPYING".
 *
 *
 * Contact Information:
 * Linux Network Driver <XXX.YYYY@marvell.com>
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/inetdevice.h>
#include <linux/mbus.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/cpumask.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#include <linux/phy.h>
#include <linux/clk.h>
#include <uapi/linux/ppp_defs.h>
#include <net/ip.h>
#include <net/ipv6.h>


#include "mvpp2.h"
#include "mvpp2_hw.h"



/* Declaractions */

/* Number of RXQs used by single port */
int mvpp2_rxq_number = MVPP2_DEFAULT_RXQ;
/* Number of TXQs used by single port */
int mvpp2_txq_number = MVPP2_MAX_TXQ;


static void mvpp2_txq_inc_get(struct mvpp2_txq_pcpu *txq_pcpu)
{
	txq_pcpu->txq_get_index++;
	if (txq_pcpu->txq_get_index == txq_pcpu->size)
		txq_pcpu->txq_get_index = 0;
}

static void mvpp2_txq_inc_put(struct mvpp2_txq_pcpu *txq_pcpu,
			      struct sk_buff *skb)
{
	txq_pcpu->tx_skb[txq_pcpu->txq_put_index] = skb;
	txq_pcpu->txq_put_index++;
	if (txq_pcpu->txq_put_index == txq_pcpu->size)
		txq_pcpu->txq_put_index = 0;
}


/* Buffer Manager configuration routines */

/* Create pool */
static int mvpp2_bm_pool_create(struct platform_device *pdev,
				struct mvpp2 *priv,
				struct mvpp2_bm_pool *bm_pool, int size)
{
	int size_bytes;

	/* Driver enforces size= x16 both for PPv21 and for PPv22, even though 
	    PPv22 HW allows size= x8 */
	if (!IS_ALIGNED(size, (1<<MVPP21_BM_POOL_SIZE_OFFSET)))
		return -EINVAL;
		
	size_bytes = 2 * sizeof(uintptr_t) * size; //YuvalC: Two pointers per buffer, bugfix.
	bm_pool->virt_addr = dma_alloc_coherent(&pdev->dev, size_bytes,
						&bm_pool->phys_addr,
						GFP_KERNEL);
	if (!bm_pool->virt_addr)
		return -ENOMEM;

	if (!IS_ALIGNED((uintptr_t)bm_pool->virt_addr, MVPP2_BM_POOL_PTR_ALIGN)) {
		dma_free_coherent(&pdev->dev, size_bytes, bm_pool->virt_addr,
				  bm_pool->phys_addr);
		dev_err(&pdev->dev, "BM pool %d is not %d bytes aligned\n",
			bm_pool->id, MVPP2_BM_POOL_PTR_ALIGN);
		return -ENOMEM;
	}

	mvpp2_bm_hw_pool_create(priv, bm_pool->id, bm_pool->phys_addr, size);

	bm_pool->type = MVPP2_BM_FREE;
	bm_pool->size = size;
	bm_pool->pkt_size = 0;
	bm_pool->buf_num = 0;
	atomic_set(&bm_pool->in_use, 0);
	spin_lock_init(&bm_pool->lock);

	return 0;
}

void mvpp2_bm_bufs_free(struct mvpp2 *priv, struct mvpp2_bm_pool *bm_pool)
{
	int i;

	for (i = 0; i < bm_pool->buf_num; i++) {
		struct sk_buff *vaddr;

		/* Get buffer virtual adress (indirect access) */
		vaddr = mvpp2_bm_virt_addr_get(priv, bm_pool->id);
		if (!vaddr)
			break;	
		dev_kfree_skb_any(vaddr);
	}

	/* Update BM driver with number of buffers removed from pool */
	bm_pool->buf_num -= i;
}


/* Cleanup pool */
static int mvpp2_bm_pool_destroy(struct platform_device *pdev,
				 struct mvpp2 *priv,
				 struct mvpp2_bm_pool *bm_pool)
{
	u32 val;

	mvpp2_bm_bufs_free(priv, bm_pool);
	if (bm_pool->buf_num) {
		WARN(1, "cannot free all buffers in pool %d\n", bm_pool->id);
		return 0;
	}

	val = mvpp2_read(priv, MVPP2_BM_POOL_CTRL_REG(bm_pool->id));
	val |= MVPP2_BM_STOP_MASK;
	mvpp2_write(priv, MVPP2_BM_POOL_CTRL_REG(bm_pool->id), val);

	dma_free_coherent(&pdev->dev, 2 * sizeof(uintptr_t) * bm_pool->size,
			  bm_pool->virt_addr,
			  bm_pool->phys_addr);
	return 0;
}

static int mvpp2_bm_pools_init(struct platform_device *pdev,
			       struct mvpp2 *priv)
{
	int i, err, size;
	struct mvpp2_bm_pool *bm_pool;

	/* Create all pools with maximum size */
	size = MVPP2_BM_POOL_SIZE_MAX;
	for (i = 0; i < MVPP2_BM_POOLS_NUM; i++) {
		bm_pool = &priv->bm_pools[i];
		bm_pool->id = i;
		err = mvpp2_bm_pool_create(pdev, priv, bm_pool, size);
		if (err)
			goto err_unroll_pools;
		mvpp2_bm_pool_bufsize_set(priv, bm_pool, 0);
	}
	return 0;

err_unroll_pools:
	dev_err(&pdev->dev, "failed to create BM pool %d, size %d\n", i, size);
	for (i = i - 1; i >= 0; i--)
		mvpp2_bm_pool_destroy(pdev, priv, &priv->bm_pools[i]);
	return err;
}

static int mvpp2_bm_init(struct platform_device *pdev, struct mvpp2 *priv)
{
	int i, err;

	for (i = 0; i < MVPP2_BM_POOLS_NUM; i++) {
		/* Mask BM all interrupts */
		mvpp2_write(priv, MVPP2_BM_INTR_MASK_REG(i), 0);
		/* Clear BM cause register */
		mvpp2_write(priv, MVPP2_BM_INTR_CAUSE_REG(i), 0);
	}

	/* Allocate and initialize BM pools */
	priv->bm_pools = devm_kcalloc(&pdev->dev, MVPP2_BM_POOLS_NUM,
				     sizeof(struct mvpp2_bm_pool), GFP_KERNEL);
	if (!priv->bm_pools)
		return -ENOMEM;

	err = mvpp2_bm_pools_init(pdev, priv);
	if (err < 0)
		return err;
	return 0;
}



/* Allocate skb for BM pool */
static struct sk_buff *mvpp2_skb_alloc(struct mvpp2_port *port,
				       struct mvpp2_bm_pool *bm_pool,
				       dma_addr_t *buf_phys_addr,
				       gfp_t gfp_mask)
{
	struct sk_buff *skb;
	dma_addr_t phys_addr;

	skb = __dev_alloc_skb(bm_pool->pkt_size, gfp_mask);
	if (!skb)
		return NULL;

	phys_addr = dma_map_single(port->dev->dev.parent, skb->head,
				   MVPP2_RX_BUF_SIZE(bm_pool->pkt_size),
				    DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(port->dev->dev.parent, phys_addr))) {
		dev_kfree_skb_any(skb);
		return NULL;
	}
	*buf_phys_addr = phys_addr;

	return skb;
}



/* Allocate buffers for the pool */
static int mvpp2_bm_bufs_add(struct mvpp2_port *port,
			     struct mvpp2_bm_pool *bm_pool, int buf_num)
{
	struct sk_buff *skb;
	int i, buf_size, total_size;
	dma_addr_t phys_addr;

	buf_size = MVPP2_RX_BUF_SIZE(bm_pool->pkt_size);
	total_size = MVPP2_RX_TOTAL_SIZE(buf_size);

	if (buf_num < 0 ||
	    (buf_num + bm_pool->buf_num > bm_pool->size)) {
		netdev_err(port->dev,
			   "cannot allocate %d buffers for pool %d\n",
			   buf_num, bm_pool->id);
		return 0;
	}

	for (i = 0; i < buf_num; i++) {
		skb = mvpp2_skb_alloc(port, bm_pool, &phys_addr, GFP_KERNEL);
		if (!skb)
			break;

		mvpp2_pool_refill(port->priv, (u32)bm_pool->id, phys_addr, skb);
	}

	/* Update BM driver with number of buffers added to pool */
	bm_pool->buf_num += i;
	bm_pool->in_use_thresh = bm_pool->buf_num / 4;

	netdev_dbg(port->dev,
		   "%s pool %d: pkt_size=%4d, buf_size=%4d, total_size=%4d\n",
		   bm_pool->type == MVPP2_BM_SWF_SHORT ? "short" : " long",
		   bm_pool->id, bm_pool->pkt_size, buf_size, total_size);

	netdev_dbg(port->dev,
		   "%s pool %d: %d of %d buffers added\n",
		   bm_pool->type == MVPP2_BM_SWF_SHORT ? "short" : " long",
		   bm_pool->id, i, buf_num);
	return i;
}




/* Notify the driver that BM pool is being used as specific type and return the
 * pool pointer on success
 */
static struct mvpp2_bm_pool * mvpp2_bm_pool_use(struct mvpp2_port *port, int pool, enum mvpp2_bm_type type,
		  int pkt_size)
{
	unsigned long flags = 0;
	struct mvpp2_bm_pool *new_pool = &port->priv->bm_pools[pool];
	int num;

	if (new_pool->type != MVPP2_BM_FREE && new_pool->type != type) {
		netdev_err(port->dev, "mixing pool types is forbidden\n");
		return NULL;
	}

	spin_lock_irqsave(&new_pool->lock, flags);

	if (new_pool->type == MVPP2_BM_FREE)
		new_pool->type = type;

	/* Allocate buffers in case BM pool is used as long pool, but packet
	 * size doesn't match MTU or BM pool hasn't being used yet
	 */
	if (((type == MVPP2_BM_SWF_LONG) && (pkt_size > new_pool->pkt_size)) ||
	    (new_pool->pkt_size == 0)) {
		int pkts_num;

		/* Set default buffer number or free all the buffers in case
		 * the pool is not empty
		 */
		pkts_num = new_pool->buf_num;
		if (pkts_num == 0)
			pkts_num = type == MVPP2_BM_SWF_LONG ?
				   MVPP2_BM_LONG_BUF_NUM :
				   MVPP2_BM_SHORT_BUF_NUM;
		else
			mvpp2_bm_bufs_free(port->priv, new_pool);

		new_pool->pkt_size = pkt_size;

		/* Allocate buffers for this pool */
		num = mvpp2_bm_bufs_add(port, new_pool, pkts_num);
		if (num != pkts_num) {
			WARN(1, "pool %d: %d of %d allocated\n",
			     new_pool->id, num, pkts_num);
			/* We need to undo the bufs_add() allocations */
			spin_unlock_irqrestore(&new_pool->lock, flags);
			return NULL;
		}
	}

	mvpp2_bm_pool_bufsize_set(port->priv, new_pool,
				  MVPP2_RX_BUF_SIZE(new_pool->pkt_size));

	spin_unlock_irqrestore(&new_pool->lock, flags);

	return new_pool;
}

/* Initialize pools for swf */
static int mvpp2_swf_bm_pool_init(struct mvpp2_port *port)
{
	unsigned long flags = 0;
	int rxq;

	if (!port->pool_long) {
		port->pool_long =
		       mvpp2_bm_pool_use(port, MVPP2_BM_SWF_LONG_POOL(port->id),
					 MVPP2_BM_SWF_LONG,
					 port->pkt_size);
		if (!port->pool_long)
			return -ENOMEM;

		spin_lock_irqsave(&port->pool_long->lock, flags);
		port->pool_long->port_map |= (1 << port->id);
		spin_unlock_irqrestore(&port->pool_long->lock, flags);

		for (rxq = 0; rxq < mvpp2_rxq_number; rxq++)
			mvpp2_rxq_long_pool_set(port, rxq, port->pool_long->id);
	}

	if (!port->pool_short) {
		port->pool_short =
			mvpp2_bm_pool_use(port, MVPP2_BM_SWF_SHORT_POOL,
					  MVPP2_BM_SWF_SHORT,
					  MVPP2_BM_SHORT_PKT_SIZE);
		if (!port->pool_short)
			return -ENOMEM;

		spin_lock_irqsave(&port->pool_short->lock, flags);
		port->pool_short->port_map |= (1 << port->id);
		spin_unlock_irqrestore(&port->pool_short->lock, flags);

		for (rxq = 0; rxq < mvpp2_rxq_number; rxq++)
			mvpp2_rxq_short_pool_set(port, rxq,
						 port->pool_short->id);
	}

	return 0;
}

static int mvpp2_bm_update_mtu(struct net_device *dev, int mtu)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_bm_pool *port_pool = port->pool_long;
	int num, pkts_num = port_pool->buf_num;
	int pkt_size = MVPP2_RX_PKT_SIZE(mtu);

	/* Update BM pool with new buffer size */
	mvpp2_bm_bufs_free(port->priv, port_pool);
	if (port_pool->buf_num) {
		WARN(1, "cannot free all buffers in pool %d\n", port_pool->id);
		return -EIO;
	}

	port_pool->pkt_size = pkt_size;
	num = mvpp2_bm_bufs_add(port, port_pool, pkts_num);
	if (num != pkts_num) {
		WARN(1, "pool %d: %d of %d allocated\n",
		     port_pool->id, num, pkts_num);
		return -EIO;
	}

	mvpp2_bm_pool_bufsize_set(port->priv, port_pool,
				  MVPP2_RX_BUF_SIZE(port_pool->pkt_size));
	dev->mtu = mtu;
	netdev_update_features(dev);
	return 0;
}



/* Set defaults to the MVPP2 port */
static void mvpp2_defaults_set(struct mvpp2_port *port)
{
	int tx_port_num, val, queue, ptxq, lrxq;

	/* Configure port to loopback if needed */
	if (port->flags & MVPP2_F_LOOPBACK)
		mvpp2_port_loopback_set(port);

	/* Update TX FIFO MIN Threshold */
	val = readl(port->base + MVPP2_GMAC_PORT_FIFO_CFG_1_REG);
	val &= ~MVPP2_GMAC_TX_FIFO_MIN_TH_ALL_MASK;
	/* Min. TX threshold must be less than minimal packet length */
	val |= MVPP2_GMAC_TX_FIFO_MIN_TH_MASK(64 - 4 - 2);
	writel(val, port->base + MVPP2_GMAC_PORT_FIFO_CFG_1_REG);

	/* Disable Legacy WRR, Disable EJP, Release from reset */
	tx_port_num = mvpp2_egress_port(port);
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG,
		    tx_port_num);
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_CMD_1_REG, 0);

	/* Close bandwidth for all queues */
	for (queue = 0; queue < MVPP2_MAX_TXQ; queue++) {
		ptxq = mvpp2_txq_phys(port->id, queue);
		mvpp2_write(port->priv,
			    MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(ptxq), 0);
	}

	/* Set refill period to 1 usec, refill tokens
	 * and bucket size to maximum
	 */
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PERIOD_REG,
		    port->priv->tclk / USEC_PER_SEC);
	val = mvpp2_read(port->priv, MVPP2_TXP_SCHED_REFILL_REG);
	val &= ~MVPP2_TXP_REFILL_PERIOD_ALL_MASK;
	val |= MVPP2_TXP_REFILL_PERIOD_MASK(1);
	val |= MVPP2_TXP_REFILL_TOKENS_ALL_MASK;
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_REFILL_REG, val);
	val = MVPP2_TXP_TOKEN_SIZE_MAX;
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, val);

	/* Set MaximumLowLatencyPacketSize value to 256 */
	mvpp2_write(port->priv, MVPP2_RX_CTRL_REG(port->id),
		    MVPP2_RX_USE_PSEUDO_FOR_CSUM_MASK |
		    MVPP2_RX_LOW_LATENCY_PKT_SIZE(256));

	/* Enable Rx cache snoop */
	for (lrxq = 0; lrxq < mvpp2_rxq_number; lrxq++) {
		queue = port->rxqs[lrxq]->id;
		val = mvpp2_read(port->priv, MVPP21_RXQ_CONFIG_REG(queue));
		val |= MVPP21_SNOOP_PKT_SIZE_MASK |
			   MVPP21_SNOOP_BUF_HDR_MASK;
		mvpp2_write(port->priv, MVPP21_RXQ_CONFIG_REG(queue), val);
	}

	/* At default, mask all interrupts to all present cpus */
	mvpp2_interrupts_disable(port);
}







/* Check if there are enough reserved descriptors for transmission.
 * If not, request chunk of reserved descriptors and check again.
 */
static int mvpp2_txq_reserved_desc_num_proc(struct mvpp2 *priv,
					    struct mvpp2_tx_queue *txq,
					    struct mvpp2_txq_pcpu *txq_pcpu,
					    int num)
{
	int req, cpu, desc_count;

	if (txq_pcpu->reserved_num >= num)
		return 0;

	/* Not enough descriptors reserved! Update the reserved descriptor
	 * count and check again.
	 */

	desc_count = 0;
	/* Compute total of used descriptors */
	for_each_present_cpu(cpu) {
		struct mvpp2_txq_pcpu *txq_pcpu_aux;

		txq_pcpu_aux = per_cpu_ptr(txq->pcpu, cpu);
		desc_count += txq_pcpu_aux->count;
		desc_count += txq_pcpu_aux->reserved_num;
	}

	req = max(MVPP2_CPU_DESC_CHUNK, num - txq_pcpu->reserved_num);
	desc_count += req;

	if (desc_count >
	   (txq->size - (num_present_cpus() * MVPP2_CPU_DESC_CHUNK)))
		return -ENOMEM;

	txq_pcpu->reserved_num += mvpp2_txq_alloc_reserved_desc(priv, txq, req);

	/* OK, the descriptor cound has been updated: check again. */
	if (txq_pcpu->reserved_num < num)
		return -ENOMEM;
	return 0;
}

/* Release the last allocated Tx descriptor. Useful to handle DMA
 * mapping failures in the Tx path.
 */






/* Free Tx queue skbuffs */
static void mvpp2_txq_bufs_free(struct mvpp2_port *port,
				struct mvpp2_tx_queue *txq,
				struct mvpp2_txq_pcpu *txq_pcpu, int num)
{
	int i;
	dma_addr_t buf_phys_addr;

	for (i = 0; i < num; i++) {
		struct mvpp2_tx_desc *tx_desc = txq->descs +
							txq_pcpu->txq_get_index;
		struct sk_buff *skb = txq_pcpu->tx_skb[txq_pcpu->txq_get_index];

		mvpp2_txq_inc_get(txq_pcpu);

		if (!skb)
			continue;
		buf_phys_addr = mvpp2x_txdesc_phys_addr_get(port->priv->pp2_version,
			tx_desc);
		
		dma_unmap_single(port->dev->dev.parent, buf_phys_addr,
				 tx_desc->data_size, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
	}
}



/* Handle end of transmission */
static void mvpp2_txq_done(struct mvpp2_port *port, struct mvpp2_tx_queue *txq,
			   struct mvpp2_txq_pcpu *txq_pcpu)
{
	struct netdev_queue *nq = netdev_get_tx_queue(port->dev, txq->log_id);
	int tx_done;

	if (txq_pcpu->cpu != smp_processor_id())
		netdev_err(port->dev, "wrong cpu on the end of Tx processing\n");

	tx_done = mvpp2_txq_sent_desc_proc(port, txq);
	if (!tx_done)
		return;
	mvpp2_txq_bufs_free(port, txq, txq_pcpu, tx_done);

	txq_pcpu->count -= tx_done;

	if (netif_tx_queue_stopped(nq))
		if (txq_pcpu->size - txq_pcpu->count >= MAX_SKB_FRAGS + 1)
			netif_tx_wake_queue(nq);
}

/* Rx/Tx queue initialization/cleanup methods */

/* Allocate and initialize descriptors for aggr TXQ */
static int mvpp2_aggr_txq_init(struct platform_device *pdev,
			       struct mvpp2_aggr_tx_queue *aggr_txq,
			       int desc_num, int cpu,
			       struct mvpp2 *priv)
{
	/* Allocate memory for TX descriptors */
	aggr_txq->descs = dma_alloc_coherent(&pdev->dev,
				desc_num * MVPP2_DESC_ALIGNED_SIZE,
				&aggr_txq->descs_phys, GFP_KERNEL);
	if (!aggr_txq->descs)
		return -ENOMEM;

	/* Make sure descriptor address is cache line size aligned  */
	BUG_ON(aggr_txq->descs !=
	       PTR_ALIGN(aggr_txq->descs, MVPP2_CPU_D_CACHE_LINE_SIZE));

	aggr_txq->last_desc = aggr_txq->size - 1;

	/* Aggr TXQ no reset WA */
	aggr_txq->next_desc_to_proc = mvpp2_read(priv,
						 MVPP2_AGGR_TXQ_INDEX_REG(cpu));

	/* Set Tx descriptors queue starting address */
	/* indirect access */
	mvpp2_write(priv, MVPP21_AGGR_TXQ_DESC_ADDR_REG(cpu),
		    aggr_txq->descs_phys);
	mvpp2_write(priv, MVPP2_AGGR_TXQ_DESC_SIZE_REG(cpu), desc_num);

	return 0;
}

/* Create a specified Rx queue */
static int mvpp2_rxq_init(struct mvpp2_port *port,
			  struct mvpp2_rx_queue *rxq)

{
	rxq->size = port->rx_ring_size;

	/* Allocate memory for RX descriptors */
	rxq->descs = dma_alloc_coherent(port->dev->dev.parent,
					rxq->size * MVPP2_DESC_ALIGNED_SIZE,
					&rxq->descs_phys, GFP_KERNEL);
	if (!rxq->descs)
		return -ENOMEM;

	BUG_ON(rxq->descs !=
	       PTR_ALIGN(rxq->descs, MVPP2_CPU_D_CACHE_LINE_SIZE));

	rxq->last_desc = rxq->size - 1;

	/* Zero occupied and non-occupied counters - direct access */
	mvpp2_write(port->priv, MVPP2_RXQ_STATUS_REG(rxq->id), 0);

	/* Set Rx descriptors queue starting address - indirect access */
	mvpp2_write(port->priv, MVPP2_RXQ_NUM_REG, rxq->id);
	mvpp2_write(port->priv, MVPP21_RXQ_DESC_ADDR_REG, rxq->descs_phys);
	mvpp2_write(port->priv, MVPP2_RXQ_DESC_SIZE_REG, rxq->size);
	mvpp2_write(port->priv, MVPP2_RXQ_INDEX_REG, 0);

	/* Set Offset */
	mvpp2_rxq_offset_set(port, rxq->id, NET_SKB_PAD);

	/* Set coalescing pkts and time */
	mvpp2_rx_pkts_coal_set(port, rxq, rxq->pkts_coal);
	mvpp2_rx_time_coal_set(port, rxq, rxq->time_coal);

	/* Add number of descriptors ready for receiving packets */
	mvpp2_rxq_status_update(port, rxq->id, 0, rxq->size);

	return 0;
}

/* Push packets received by the RXQ to BM pool */
static void mvpp2_rxq_drop_pkts(struct mvpp2_port *port,
				struct mvpp2_rx_queue *rxq)
{
	int rx_received, i;
	struct sk_buff *buf_cookie;
	dma_addr_t buf_phys_addr;

	rx_received = mvpp2_rxq_received(port, rxq->id);
	if (!rx_received)
		return;

	for (i = 0; i < rx_received; i++) {
		struct mvpp2_rx_desc *rx_desc = mvpp2_rxq_next_desc_get(rxq);
		if (port->priv->pp2_version == PPV21) {			
			buf_cookie = mvpp21_rxdesc_cookie_get(rx_desc);
			buf_phys_addr = mvpp21_rxdesc_phys_addr_get(rx_desc);
		} else {
			buf_cookie = mvpp22_rxdesc_cookie_get(rx_desc);
			buf_phys_addr = mvpp22_rxdesc_phys_addr_get(rx_desc);
		}		
		mvpp2_pool_refill(port->priv, MVPP2_RX_DESC_POOL(rx_desc), 
			buf_phys_addr, buf_cookie);
	}
	mvpp2_rxq_status_update(port, rxq->id, rx_received, rx_received);
}

/* Cleanup Rx queue */
static void mvpp2_rxq_deinit(struct mvpp2_port *port,
			     struct mvpp2_rx_queue *rxq)
{
	mvpp2_rxq_drop_pkts(port, rxq);

	if (rxq->descs)
		dma_free_coherent(port->dev->dev.parent,
				  rxq->size * MVPP2_DESC_ALIGNED_SIZE,
				  rxq->descs,
				  rxq->descs_phys);

	rxq->descs             = NULL;
	rxq->last_desc         = 0;
	rxq->next_desc_to_proc = 0;
	rxq->descs_phys        = 0;

	/* Clear Rx descriptors queue starting address and size;
	 * free descriptor number
	 */
	mvpp2_write(port->priv, MVPP2_RXQ_STATUS_REG(rxq->id), 0);
	mvpp2_write(port->priv, MVPP2_RXQ_NUM_REG, rxq->id);
	mvpp2_write(port->priv, MVPP21_RXQ_DESC_ADDR_REG, 0);
	mvpp2_write(port->priv, MVPP2_RXQ_DESC_SIZE_REG, 0);
}

/* Create and initialize a Tx queue */
static int mvpp2_txq_init(struct mvpp2_port *port,
			  struct mvpp2_tx_queue *txq)
{
	u32 val;
	int cpu, desc, desc_per_txq, tx_port_num;
	struct mvpp2_txq_pcpu *txq_pcpu;

	txq->size = port->tx_ring_size;

	/* Allocate memory for Tx descriptors */
	txq->descs = dma_alloc_coherent(port->dev->dev.parent,
				txq->size * MVPP2_DESC_ALIGNED_SIZE,
				&txq->descs_phys, GFP_KERNEL);
	if (!txq->descs)
		return -ENOMEM;

	/* Make sure descriptor address is cache line size aligned  */
	BUG_ON(txq->descs !=
	       PTR_ALIGN(txq->descs, MVPP2_CPU_D_CACHE_LINE_SIZE));

	txq->last_desc = txq->size - 1;

	/* Set Tx descriptors queue starting address - indirect access */
	mvpp2_write(port->priv, MVPP2_TXQ_NUM_REG, txq->id);
	mvpp2_write(port->priv, MVPP2_TXQ_DESC_ADDR_LOW_REG, txq->descs_phys);
	mvpp2_write(port->priv, MVPP2_TXQ_DESC_SIZE_REG, txq->size &
					     MVPP2_TXQ_DESC_SIZE_MASK);
	mvpp2_write(port->priv, MVPP2_TXQ_INDEX_REG, 0);
	mvpp2_write(port->priv, MVPP2_TXQ_RSVD_CLR_REG,
		    txq->id << MVPP2_TXQ_RSVD_CLR_OFFSET);
	val = mvpp2_read(port->priv, MVPP2_TXQ_PENDING_REG);
	val &= ~MVPP2_TXQ_PENDING_MASK;
	mvpp2_write(port->priv, MVPP2_TXQ_PENDING_REG, val);

	/* Calculate base address in prefetch buffer. We reserve 16 descriptors
	 * for each existing TXQ.
	 * TCONTS for PON port must be continuous from 0 to MVPP2_MAX_TCONT
	 * GBE ports assumed to be continious from 0 to MVPP2_MAX_PORTS
	 */
	desc_per_txq = 16;
	desc = (port->id * MVPP2_MAX_TXQ * desc_per_txq) +
	       (txq->log_id * desc_per_txq);

	mvpp2_write(port->priv, MVPP2_TXQ_PREF_BUF_REG,
		    MVPP2_PREF_BUF_PTR(desc) | MVPP2_PREF_BUF_SIZE_16 |
		    MVPP2_PREF_BUF_THRESH(desc_per_txq/2));

	/* WRR / EJP configuration - indirect access */
	tx_port_num = mvpp2_egress_port(port);
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);

	val = mvpp2_read(port->priv, MVPP2_TXQ_SCHED_REFILL_REG(txq->log_id));
	val &= ~MVPP2_TXQ_REFILL_PERIOD_ALL_MASK;
	val |= MVPP2_TXQ_REFILL_PERIOD_MASK(1);
	val |= MVPP2_TXQ_REFILL_TOKENS_ALL_MASK;
	mvpp2_write(port->priv, MVPP2_TXQ_SCHED_REFILL_REG(txq->log_id), val);

	val = MVPP2_TXQ_TOKEN_SIZE_MAX;
	mvpp2_write(port->priv, MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq->log_id),
		    val);

	for_each_present_cpu(cpu) {
		txq_pcpu = per_cpu_ptr(txq->pcpu, cpu);
		txq_pcpu->size = txq->size;
		txq_pcpu->tx_skb = kmalloc(txq_pcpu->size *
					   sizeof(*txq_pcpu->tx_skb),
					   GFP_KERNEL);
		if (!txq_pcpu->tx_skb) {
			dma_free_coherent(port->dev->dev.parent,
					  txq->size * MVPP2_DESC_ALIGNED_SIZE,
					  txq->descs, txq->descs_phys);
			return -ENOMEM;
		}

		txq_pcpu->count = 0;
		txq_pcpu->reserved_num = 0;
		txq_pcpu->txq_put_index = 0;
		txq_pcpu->txq_get_index = 0;
	}

	return 0;
}

/* Free allocated TXQ resources */
static void mvpp2_txq_deinit(struct mvpp2_port *port,
			     struct mvpp2_tx_queue *txq)
{
	struct mvpp2_txq_pcpu *txq_pcpu;
	int cpu;

	for_each_present_cpu(cpu) {
		txq_pcpu = per_cpu_ptr(txq->pcpu, cpu);
		kfree(txq_pcpu->tx_skb);
	}

	if (txq->descs)
		dma_free_coherent(port->dev->dev.parent,
				  txq->size * MVPP2_DESC_ALIGNED_SIZE,
				  txq->descs, txq->descs_phys);

	txq->descs             = NULL;
	txq->last_desc         = 0;
	txq->next_desc_to_proc = 0;
	txq->descs_phys        = 0;

	/* Set minimum bandwidth for disabled TXQs */
	mvpp2_write(port->priv, MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(txq->id), 0);

	/* Set Tx descriptors queue starting address and size */
	mvpp2_write(port->priv, MVPP2_TXQ_NUM_REG, txq->id);
	mvpp2_write(port->priv, MVPP2_TXQ_DESC_ADDR_LOW_REG, 0);
	mvpp2_write(port->priv, MVPP2_TXQ_DESC_SIZE_REG, 0);
}

/* Cleanup Tx ports */
static void mvpp2_txq_clean(struct mvpp2_port *port, struct mvpp2_tx_queue *txq)
{
	struct mvpp2_txq_pcpu *txq_pcpu;
	int delay, pending, cpu;
	u32 val;

	mvpp2_write(port->priv, MVPP2_TXQ_NUM_REG, txq->id);
	val = mvpp2_read(port->priv, MVPP2_TXQ_PREF_BUF_REG);
	val |= MVPP2_TXQ_DRAIN_EN_MASK;
	mvpp2_write(port->priv, MVPP2_TXQ_PREF_BUF_REG, val);

	/* The napi queue has been stopped so wait for all packets
	 * to be transmitted.
	 */
	delay = 0;
	do {
		if (delay >= MVPP2_TX_PENDING_TIMEOUT_MSEC) {
			netdev_warn(port->dev,
				    "port %d: cleaning queue %d timed out\n",
				    port->id, txq->log_id);
			break;
		}
		mdelay(1);
		delay++;

		pending = mvpp2_txq_pend_desc_num_get(port, txq);
	} while (pending);

	val &= ~MVPP2_TXQ_DRAIN_EN_MASK;
	mvpp2_write(port->priv, MVPP2_TXQ_PREF_BUF_REG, val);

	for_each_present_cpu(cpu) {
		txq_pcpu = per_cpu_ptr(txq->pcpu, cpu);

		/* Release all packets */
		mvpp2_txq_bufs_free(port, txq, txq_pcpu, txq_pcpu->count);

		/* Reset queue */
		txq_pcpu->count = 0;
		txq_pcpu->txq_put_index = 0;
		txq_pcpu->txq_get_index = 0;
	}
}

/* Cleanup all Tx queues */
void mvpp2_cleanup_txqs(struct mvpp2_port *port)
{
	struct mvpp2_tx_queue *txq;
	int queue;
	u32 val;

	val = mvpp2_read(port->priv, MVPP2_TX_PORT_FLUSH_REG);

	/* Reset Tx ports and delete Tx queues */
	val |= MVPP2_TX_PORT_FLUSH_MASK(port->id);
	mvpp2_write(port->priv, MVPP2_TX_PORT_FLUSH_REG, val);

	for (queue = 0; queue < mvpp2_txq_number; queue++) {
		txq = port->txqs[queue];
		mvpp2_txq_clean(port, txq);
		mvpp2_txq_deinit(port, txq);
	}

	on_each_cpu(mvpp2_txq_sent_counter_clear, port, 1);

	val &= ~MVPP2_TX_PORT_FLUSH_MASK(port->id);
	mvpp2_write(port->priv, MVPP2_TX_PORT_FLUSH_REG, val);
}

/* Cleanup all Rx queues */
void mvpp2_cleanup_rxqs(struct mvpp2_port *port)
{
	int queue;

	for (queue = 0; queue < mvpp2_rxq_number; queue++)
		mvpp2_rxq_deinit(port, port->rxqs[queue]);
}

/* Init all Rx queues for port */
int mvpp2_setup_rxqs(struct mvpp2_port *port)
{
	int queue, err;

	for (queue = 0; queue < mvpp2_rxq_number; queue++) {
		err = mvpp2_rxq_init(port, port->rxqs[queue]);
		if (err)
			goto err_cleanup;
	}
	return 0;

err_cleanup:
	mvpp2_cleanup_rxqs(port);
	return err;
}

/* Init all tx queues for port */
int mvpp2_setup_txqs(struct mvpp2_port *port)
{
	struct mvpp2_tx_queue *txq;
	int queue, err;

	for (queue = 0; queue < mvpp2_txq_number; queue++) {
		txq = port->txqs[queue];
		err = mvpp2_txq_init(port, txq);
		if (err)
			goto err_cleanup;
	}

	on_each_cpu(mvpp2_tx_done_pkts_coal_set, port, 1);
	on_each_cpu(mvpp2_txq_sent_counter_clear, port, 1);
	return 0;

err_cleanup:
	mvpp2_cleanup_txqs(port);
	return err;
}

/* The callback for per-port interrupt */
static irqreturn_t mvpp2_isr(int irq, void *dev_id)
{
	struct mvpp2_port *port = (struct mvpp2_port *)dev_id;

	mvpp2_interrupts_disable(port);

	napi_schedule(&port->napi);

	return IRQ_HANDLED;
}

/* Adjust link */
static void mvpp2_link_event(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct phy_device *phydev = port->phy_dev;
	int status_change = 0;
	u32 val;

	if (phydev->link) {
		if ((port->speed != phydev->speed) ||
		    (port->duplex != phydev->duplex)) {
			u32 val;

			val = readl(port->base + MVPP2_GMAC_AUTONEG_CONFIG);
			val &= ~(MVPP2_GMAC_CONFIG_MII_SPEED |
				 MVPP2_GMAC_CONFIG_GMII_SPEED |
				 MVPP2_GMAC_CONFIG_FULL_DUPLEX |
				 MVPP2_GMAC_AN_SPEED_EN |
				 MVPP2_GMAC_AN_DUPLEX_EN);

			if (phydev->duplex)
				val |= MVPP2_GMAC_CONFIG_FULL_DUPLEX;

			if (phydev->speed == SPEED_1000)
				val |= MVPP2_GMAC_CONFIG_GMII_SPEED;
			else if (phydev->speed == SPEED_100)
				val |= MVPP2_GMAC_CONFIG_MII_SPEED;

			writel(val, port->base + MVPP2_GMAC_AUTONEG_CONFIG);

			port->duplex = phydev->duplex;
			port->speed  = phydev->speed;
		}
	}

	if (phydev->link != port->link) {
		if (!phydev->link) {
			port->duplex = -1;
			port->speed = 0;
		}

		port->link = phydev->link;
		status_change = 1;
	}

	if (status_change) {
		if (phydev->link) {
			val = readl(port->base + MVPP2_GMAC_AUTONEG_CONFIG);
			val |= (MVPP2_GMAC_FORCE_LINK_PASS |
				MVPP2_GMAC_FORCE_LINK_DOWN);
			writel(val, port->base + MVPP2_GMAC_AUTONEG_CONFIG);
			mvpp2_egress_enable(port);
			mvpp2_ingress_enable(port);
		} else {
			mvpp2_ingress_disable(port);
			mvpp2_egress_disable(port);
		}
		phy_print_status(phydev);
	}
}

/* Main RX/TX processing routines */


/* Reuse skb if possible, or allocate a new skb and add it to BM pool */
static int mvpp2_rx_refill(struct mvpp2_port *port,
			   struct mvpp2_bm_pool *bm_pool,
			   u32 pool, int is_recycle)
{
	struct sk_buff *skb;
	dma_addr_t phys_addr;

	if (is_recycle &&
	    (atomic_read(&bm_pool->in_use) < bm_pool->in_use_thresh))
		return 0;

	/* No recycle or too many buffers are in use, so allocate a new skb */
	skb = mvpp2_skb_alloc(port, bm_pool, &phys_addr, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	mvpp2_pool_refill(port->priv, pool, phys_addr, skb);
	atomic_dec(&bm_pool->in_use);
	return 0;
}

/* Handle tx checksum */
static u32 mvpp2_skb_tx_csum(struct mvpp2_port *port, struct sk_buff *skb)
{
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		int ip_hdr_len = 0;
		u8 l4_proto;

		if (skb->protocol == htons(ETH_P_IP)) {
			struct iphdr *ip4h = ip_hdr(skb);

			/* Calculate IPv4 checksum and L4 checksum */
			ip_hdr_len = ip4h->ihl;
			l4_proto = ip4h->protocol;
		} else if (skb->protocol == htons(ETH_P_IPV6)) {
			struct ipv6hdr *ip6h = ipv6_hdr(skb);

			/* Read l4_protocol from one of IPv6 extra headers */
			if (skb_network_header_len(skb) > 0)
				ip_hdr_len = (skb_network_header_len(skb) >> 2);
			l4_proto = ip6h->nexthdr;
		} else {
			return MVPP2_TXD_L4_CSUM_NOT;
		}

		return mvpp2_txq_desc_csum(skb_network_offset(skb),
				skb->protocol, ip_hdr_len, l4_proto);
	}

	return MVPP2_TXD_L4_CSUM_NOT | MVPP2_TXD_IP_CSUM_DISABLE;
}

static void mvpp2_buff_hdr_rx(struct mvpp2_port *port,
			      struct mvpp2_rx_desc *rx_desc)
{
	struct mvpp2_buff_hdr *buff_hdr;
	struct sk_buff *skb;
	u32 rx_status = rx_desc->status;
	u32 buff_phys_addr;
	u32 buff_virt_addr;
	u32 buff_phys_addr_next;
	u32 buff_virt_addr_next;
	int mc_id;
	int pool_id;

	pool_id = (rx_status & MVPP2_RXD_BM_POOL_ID_MASK) >>
		   MVPP2_RXD_BM_POOL_ID_OFFS;
//TODO : YuvalC, this is just workaround to compile. Need to handle mvpp2_buff_hdr_rx().	
	buff_phys_addr = rx_desc->u.pp21.buf_phys_addr;
	buff_virt_addr = rx_desc->u.pp21.buf_cookie;

	do {
		skb = (struct sk_buff *)buff_virt_addr;
		buff_hdr = (struct mvpp2_buff_hdr *)skb->head;

		mc_id = MVPP2_B_HDR_INFO_MC_ID(buff_hdr->info);

		buff_phys_addr_next = buff_hdr->next_buff_phys_addr;
		buff_virt_addr_next = buff_hdr->next_buff_virt_addr;

		/* Release buffer */
		mvpp2_bm_pool_mc_put(port, pool_id, buff_phys_addr,
				     buff_virt_addr, mc_id);

		buff_phys_addr = buff_phys_addr_next;
		buff_virt_addr = buff_virt_addr_next;

	} while (!MVPP2_B_HDR_INFO_IS_LAST(buff_hdr->info));
}

/* Main rx processing */
static int mvpp2_rx(struct mvpp2_port *port, int rx_todo,
		    struct mvpp2_rx_queue *rxq)
{
	struct net_device *dev = port->dev;
	int rx_received, rx_filled, i;
	u32 rcvd_pkts = 0;
	u32 rcvd_bytes = 0;

	/* Get number of received packets and clamp the to-do */
	rx_received = mvpp2_rxq_received(port, rxq->id);
	if (rx_todo > rx_received)
		rx_todo = rx_received;

	rx_filled = 0;
	for (i = 0; i < rx_todo; i++) {
		struct mvpp2_rx_desc *rx_desc = mvpp2_rxq_next_desc_get(rxq);
		struct mvpp2_bm_pool *bm_pool;
		struct sk_buff *skb;
		u32 rx_status, pool;
		int rx_bytes, err;		
		dma_addr_t buf_phys_addr;

		rx_filled++;
		rx_status = rx_desc->status;
		rx_bytes = rx_desc->data_size - MVPP2_MH_SIZE;

		pool = MVPP2_RX_DESC_POOL(rx_desc);
		bm_pool = &port->priv->bm_pools[pool];
		/* Check if buffer header is used */
		if (rx_status & MVPP2_RXD_BUF_HDR) {
			mvpp2_buff_hdr_rx(port, rx_desc);
			continue;
		}

		if (port->priv->pp2_version == PPV21) {			
			skb = mvpp21_rxdesc_cookie_get(rx_desc);
			buf_phys_addr = mvpp21_rxdesc_phys_addr_get(rx_desc);
		} else {
			skb = mvpp22_rxdesc_cookie_get(rx_desc);
			buf_phys_addr = mvpp22_rxdesc_phys_addr_get(rx_desc);
		}


		/* In case of an error, release the requested buffer pointer
		 * to the Buffer Manager. This request process is controlled
		 * by the hardware, and the information about the buffer is
		 * comprised by the RX descriptor.
		 */
		if (rx_status & MVPP2_RXD_ERR_SUMMARY) {
			dev->stats.rx_errors++;
			mvpp2_rx_error(port, rx_desc);
			mvpp2_pool_refill(port->priv, pool, buf_phys_addr, skb);
			continue;
		}


		rcvd_pkts++;
		rcvd_bytes += rx_bytes;
		atomic_inc(&bm_pool->in_use);

		skb_reserve(skb, MVPP2_MH_SIZE);
		skb_put(skb, rx_bytes);
		skb->protocol = eth_type_trans(skb, dev);
		mvpp2_rx_csum(port, rx_status, skb);

		napi_gro_receive(&port->napi, skb);

		err = mvpp2_rx_refill(port, bm_pool, pool, 0);
		if (err) {
			netdev_err(port->dev, "failed to refill BM pools\n");
			rx_filled--;
		}
	}

	if (rcvd_pkts) {
		struct mvpp2_pcpu_stats *stats = this_cpu_ptr(port->stats);

		u64_stats_update_begin(&stats->syncp);
		stats->rx_packets += rcvd_pkts;
		stats->rx_bytes   += rcvd_bytes;
		u64_stats_update_end(&stats->syncp);
	}

	/* Update Rx queue management counters */
	wmb();
	mvpp2_rxq_status_update(port, rxq->id, rx_todo, rx_filled);

	return rx_todo;
}

static inline void tx_desc_unmap_put(struct device *dev, struct mvpp2_tx_queue *txq,
		  struct mvpp2_tx_desc *desc)
{
	dma_addr_t buf_phys_addr;

	buf_phys_addr = mvpp2x_txdesc_phys_addr_get(
		((struct mvpp2 *)(dev_get_drvdata(dev)))->pp2_version, desc);
	dma_unmap_single(dev, buf_phys_addr,
			 desc->data_size, DMA_TO_DEVICE);
	mvpp2_txq_desc_put(txq);
}

/* Handle tx fragmentation processing */
static int mvpp2_tx_frag_process(struct mvpp2_port *port, struct sk_buff *skb,
				 struct mvpp2_aggr_tx_queue *aggr_txq,
				 struct mvpp2_tx_queue *txq)
{
	struct mvpp2_txq_pcpu *txq_pcpu = this_cpu_ptr(txq->pcpu);
	struct mvpp2_tx_desc *tx_desc;
	int i;
	dma_addr_t buf_phys_addr;

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		void *addr = page_address(frag->page.p) + frag->page_offset;

		tx_desc = mvpp2_txq_next_desc_get(aggr_txq);
		tx_desc->phys_txq = txq->id;
		tx_desc->data_size = frag->size;

		buf_phys_addr = dma_map_single(port->dev->dev.parent, addr,
					       tx_desc->data_size,
					       DMA_TO_DEVICE);
		if (dma_mapping_error(port->dev->dev.parent, buf_phys_addr)) {
			mvpp2_txq_desc_put(txq);
			goto error;
		}

		mvpp2x_txdesc_phys_addr_set(port->priv->pp2_version, 
			buf_phys_addr & MVPP2_TX_DESC_ALIGN, tx_desc);

		if (i == (skb_shinfo(skb)->nr_frags - 1)) {
			/* Last descriptor */
			tx_desc->command = MVPP2_TXD_L_DESC;
			mvpp2_txq_inc_put(txq_pcpu, skb);
		} else {
			/* Descriptor in the middle: Not First, Not Last */
			tx_desc->command = 0;
			mvpp2_txq_inc_put(txq_pcpu, NULL);
		}
	}

	return 0;

error:
	/* Release all descriptors that were used to map fragments of
	 * this packet, as well as the corresponding DMA mappings
	 */
	for (i = i - 1; i >= 0; i--) {
		tx_desc = txq->descs + i;
		tx_desc_unmap_put(port->dev->dev.parent, txq, tx_desc);
	}

	return -ENOMEM;
}

/* Main tx processing */
static int mvpp2_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_tx_queue *txq;
	struct mvpp2_aggr_tx_queue *aggr_txq;
	struct mvpp2_txq_pcpu *txq_pcpu;
	struct mvpp2_tx_desc *tx_desc;
	dma_addr_t buf_phys_addr;
	int frags = 0;
	u16 txq_id;
	u32 tx_cmd;

	txq_id = skb_get_queue_mapping(skb);
	txq = port->txqs[txq_id];
	txq_pcpu = this_cpu_ptr(txq->pcpu);
	aggr_txq = &port->priv->aggr_txqs[smp_processor_id()];

	frags = skb_shinfo(skb)->nr_frags + 1;

	/* Check number of available descriptors */
	if (mvpp2_aggr_desc_num_check(port->priv, aggr_txq, frags) ||
	    mvpp2_txq_reserved_desc_num_proc(port->priv, txq,
					     txq_pcpu, frags)) {
		frags = 0;
		goto out;
	}

	/* Get a descriptor for the first part of the packet */
	tx_desc = mvpp2_txq_next_desc_get(aggr_txq);
	tx_desc->phys_txq = txq->id;
	tx_desc->data_size = skb_headlen(skb);

	buf_phys_addr = dma_map_single(dev->dev.parent, skb->data,
				       tx_desc->data_size, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev->dev.parent, buf_phys_addr))) {
		mvpp2_txq_desc_put(txq);
		frags = 0;
		goto out;
	}
	tx_desc->packet_offset = buf_phys_addr & MVPP2_TX_DESC_ALIGN;
	mvpp2x_txdesc_phys_addr_set(port->priv->pp2_version, 
		buf_phys_addr & MVPP2_TX_DESC_ALIGN, tx_desc);
	
	tx_cmd = mvpp2_skb_tx_csum(port, skb);

	if (frags == 1) {
		/* First and Last descriptor */
		tx_cmd |= MVPP2_TXD_F_DESC | MVPP2_TXD_L_DESC;
		tx_desc->command = tx_cmd;
		mvpp2_txq_inc_put(txq_pcpu, skb);
	} else {
		/* First but not Last */
		tx_cmd |= MVPP2_TXD_F_DESC | MVPP2_TXD_PADDING_DISABLE;
		tx_desc->command = tx_cmd;
		mvpp2_txq_inc_put(txq_pcpu, NULL);

		/* Continue with other skb fragments */
		if (mvpp2_tx_frag_process(port, skb, aggr_txq, txq)) {
			tx_desc_unmap_put(port->dev->dev.parent, txq, tx_desc);
			frags = 0;
			goto out;
		}
	}

	txq_pcpu->reserved_num -= frags;
	txq_pcpu->count += frags;
	aggr_txq->count += frags;

	/* Enable transmit */
	wmb();
	mvpp2_aggr_txq_pend_desc_add(port, frags);

	if (txq_pcpu->size - txq_pcpu->count < MAX_SKB_FRAGS + 1) {
		struct netdev_queue *nq = netdev_get_tx_queue(dev, txq_id);

		netif_tx_stop_queue(nq);
	}
out:
	if (frags > 0) {
		struct mvpp2_pcpu_stats *stats = this_cpu_ptr(port->stats);

		u64_stats_update_begin(&stats->syncp);
		stats->tx_packets++;
		stats->tx_bytes += skb->len;
		u64_stats_update_end(&stats->syncp);
	} else {
		dev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
	}

	return NETDEV_TX_OK;
}



static void mvpp2_txq_done_percpu(void *arg)
{
	struct mvpp2_port *port = arg;
	u32 cause_rx_tx, cause_tx, cause_misc;

	/* Rx/Tx cause register
	 *
	 * Bits 0-15: each bit indicates received packets on the Rx queue
	 * (bit 0 is for Rx queue 0).
	 *
	 * Bits 16-23: each bit indicates transmitted packets on the Tx queue
	 * (bit 16 is for Tx queue 0).
	 *
	 * Each CPU has its own Rx/Tx cause register
	 */
	cause_rx_tx = mvpp2_read(port->priv,
				 MVPP21_ISR_RX_TX_CAUSE_REG(port->id));
	cause_tx = cause_rx_tx & MVPP21_CAUSE_TXQ_OCCUP_DESC_ALL_MASK;
	cause_misc = cause_rx_tx & MVPP21_CAUSE_MISC_SUM_MASK;

	if (cause_misc) {
		mvpp2_cause_error(port->dev, cause_misc);

		/* Clear the cause register */
		mvpp2_write(port->priv, MVPP2_ISR_MISC_CAUSE_REG, 0);
		mvpp2_write(port->priv, MVPP21_ISR_RX_TX_CAUSE_REG(port->id),
			    cause_rx_tx & ~MVPP21_CAUSE_MISC_SUM_MASK);
	}

	/* Release TX descriptors */
	if (cause_tx) {
		struct mvpp2_tx_queue *txq = mvpp2_get_tx_queue(port, cause_tx);
		struct mvpp2_txq_pcpu *txq_pcpu = this_cpu_ptr(txq->pcpu);

		if (txq_pcpu->count)
			mvpp2_txq_done(port, txq, txq_pcpu);
	}
}

static int mvpp2_poll(struct napi_struct *napi, int budget)
{
	u32 cause_rx_tx, cause_rx;
	int rx_done = 0;
	struct mvpp2_port *port = netdev_priv(napi->dev);

	on_each_cpu(mvpp2_txq_done_percpu, port, 1);

	cause_rx_tx = mvpp2_read(port->priv,
				 MVPP21_ISR_RX_TX_CAUSE_REG(port->id));
	cause_rx = cause_rx_tx & MVPP21_CAUSE_RXQ_OCCUP_DESC_ALL_MASK;

	/* Process RX packets */
	cause_rx |= port->pending_cause_rx;
	while (cause_rx && budget > 0) {
		int count;
		struct mvpp2_rx_queue *rxq;

		rxq = mvpp2_get_rx_queue(port, cause_rx);
		if (!rxq)
			break;

		count = mvpp2_rx(port, budget, rxq);
		rx_done += count;
		budget -= count;
		if (budget > 0) {
			/* Clear the bit associated to this Rx queue
			 * so that next iteration will continue from
			 * the next Rx queue.
			 */
			cause_rx &= ~(1 << rxq->log_id);
		}
	}

	if (budget > 0) {
		cause_rx = 0;
		napi_complete(napi);

		mvpp2_interrupts_enable(port);
	}
	port->pending_cause_rx = cause_rx;
	return rx_done;
}

/* Set hw internals when starting port */
void mvpp2_start_dev(struct mvpp2_port *port)
{
	mvpp2_gmac_max_rx_size_set(port);
	mvpp2_txp_max_tx_size_set(port);

	napi_enable(&port->napi);

	/* Enable interrupts on all CPUs */
	mvpp2_interrupts_enable(port);

	mvpp2_port_enable(port);
	phy_start(port->phy_dev);
	netif_tx_start_all_queues(port->dev);
}

/* Set hw internals when stopping port */
void mvpp2_stop_dev(struct mvpp2_port *port)
{
	/* Stop new packets from arriving to RXQs */
	mvpp2_ingress_disable(port);

	mdelay(10);

	/* Disable interrupts on all CPUs */
	mvpp2_interrupts_disable(port);

	napi_disable(&port->napi);

	netif_carrier_off(port->dev);
	netif_tx_stop_all_queues(port->dev);

	mvpp2_egress_disable(port);
	mvpp2_port_disable(port);
	phy_stop(port->phy_dev);
}

/* Return positive if MTU is valid */
static inline int mvpp2_check_mtu_valid(struct net_device *dev, int mtu)
{
	if (mtu < 68) {
		netdev_err(dev, "cannot change mtu to less than 68\n");
		return -EINVAL;
	}

	/* 9676 == 9700 - 20 and rounding to 8 */
	if (mtu > 9676) {
		netdev_info(dev, "illegal MTU value %d, round to 9676\n", mtu);
		mtu = 9676;
	}

	if (!IS_ALIGNED(MVPP2_RX_PKT_SIZE(mtu), 8)) {
		netdev_info(dev, "illegal MTU value %d, round to %d\n", mtu,
			    ALIGN(MVPP2_RX_PKT_SIZE(mtu), 8));
		mtu = ALIGN(MVPP2_RX_PKT_SIZE(mtu), 8);
	}

	return mtu;
}

int mvpp2_check_ringparam_valid(struct net_device *dev,
				       struct ethtool_ringparam *ring)
{
	u16 new_rx_pending = ring->rx_pending;
	u16 new_tx_pending = ring->tx_pending;

	if (ring->rx_pending == 0 || ring->tx_pending == 0)
		return -EINVAL;

	if (ring->rx_pending > MVPP2_MAX_RXD)
		new_rx_pending = MVPP2_MAX_RXD;
	else if (!IS_ALIGNED(ring->rx_pending, 16))
		new_rx_pending = ALIGN(ring->rx_pending, 16);

	if (ring->tx_pending > MVPP2_MAX_TXD)
		new_tx_pending = MVPP2_MAX_TXD;
	else if (!IS_ALIGNED(ring->tx_pending, 32))
		new_tx_pending = ALIGN(ring->tx_pending, 32);

	if (ring->rx_pending != new_rx_pending) {
		netdev_info(dev, "illegal Rx ring size value %d, round to %d\n",
			    ring->rx_pending, new_rx_pending);
		ring->rx_pending = new_rx_pending;
	}

	if (ring->tx_pending != new_tx_pending) {
		netdev_info(dev, "illegal Tx ring size value %d, round to %d\n",
			    ring->tx_pending, new_tx_pending);
		ring->tx_pending = new_tx_pending;
	}

	return 0;
}



static int mvpp2_phy_connect(struct mvpp2_port *port)
{
	struct phy_device *phy_dev;

	phy_dev = of_phy_connect(port->dev, port->phy_node, mvpp2_link_event, 0,
				 port->phy_interface);
	if (!phy_dev) {
		netdev_err(port->dev, "cannot connect to phy\n");
		return -ENODEV;
	}
	phy_dev->supported &= PHY_GBIT_FEATURES;
	phy_dev->advertising = phy_dev->supported;

	port->phy_dev = phy_dev;
	port->link    = 0;
	port->duplex  = 0;
	port->speed   = 0;

	return 0;
}

static void mvpp2_phy_disconnect(struct mvpp2_port *port)
{
	phy_disconnect(port->phy_dev);
	port->phy_dev = NULL;
}

static int mvpp2_open(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);
	unsigned char mac_bcast[ETH_ALEN] = {
			0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	int err;

	err = mvpp2_prs_mac_da_accept(port->priv, port->id, mac_bcast, true);
	if (err) {
		netdev_err(dev, "mvpp2_prs_mac_da_accept BC failed\n");
		return err;
	}
	err = mvpp2_prs_mac_da_accept(port->priv, port->id,
				      dev->dev_addr, true);
	if (err) {
		netdev_err(dev, "mvpp2_prs_mac_da_accept MC failed\n");
		return err;
	}
	err = mvpp2_prs_tag_mode_set(port->priv, port->id, MVPP2_TAG_TYPE_MH);
	if (err) {
		netdev_err(dev, "mvpp2_prs_tag_mode_set failed\n");
		return err;
	}
	err = mvpp2_prs_def_flow(port);
	if (err) {
		netdev_err(dev, "mvpp2_prs_def_flow failed\n");
		return err;
	}

	/* Allocate the Rx/Tx queues */
	err = mvpp2_setup_rxqs(port);
	if (err) {
		netdev_err(port->dev, "cannot allocate Rx queues\n");
		return err;
	}

	err = mvpp2_setup_txqs(port);
	if (err) {
		netdev_err(port->dev, "cannot allocate Tx queues\n");
		goto err_cleanup_rxqs;
	}

	err = request_irq(port->irq, mvpp2_isr, 0, dev->name, port);
	if (err) {
		netdev_err(port->dev, "cannot request IRQ %d\n", port->irq);
		goto err_cleanup_txqs;
	}

	/* In default link is down */
	netif_carrier_off(port->dev);

	err = mvpp2_phy_connect(port);
	if (err < 0)
		goto err_free_irq;

	/* Unmask interrupts on all CPUs */
	on_each_cpu(mvpp2_interrupts_unmask, port, 1);

	mvpp2_start_dev(port);

	return 0;

err_free_irq:
	free_irq(port->irq, port);
err_cleanup_txqs:
	mvpp2_cleanup_txqs(port);
err_cleanup_rxqs:
	mvpp2_cleanup_rxqs(port);
	return err;
}

static int mvpp2_stop(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);

	mvpp2_stop_dev(port);
	mvpp2_phy_disconnect(port);

	/* Mask interrupts on all CPUs */
	on_each_cpu(mvpp2_interrupts_mask, port, 1);

	free_irq(port->irq, port);
	mvpp2_cleanup_rxqs(port);
	mvpp2_cleanup_txqs(port);

	return 0;
}

static void mvpp2_set_rx_mode(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2 *priv = port->priv;
	struct netdev_hw_addr *ha;
	int id = port->id;
	bool allmulti = dev->flags & IFF_ALLMULTI;

	mvpp2_prs_mac_promisc_set(priv, id, dev->flags & IFF_PROMISC);
	mvpp2_prs_mac_multi_set(priv, id, MVPP2_PE_MAC_MC_ALL, allmulti);
	mvpp2_prs_mac_multi_set(priv, id, MVPP2_PE_MAC_MC_IP6, allmulti);

	/* Remove all port->id's mcast enries */
	mvpp2_prs_mcast_del_all(priv, id);

	if (allmulti && !netdev_mc_empty(dev)) {
		netdev_for_each_mc_addr(ha, dev)
			mvpp2_prs_mac_da_accept(priv, id, ha->addr, true);
	}
}

static int mvpp2_set_mac_address(struct net_device *dev, void *p)
{
	struct mvpp2_port *port = netdev_priv(dev);
	const struct sockaddr *addr = p;
	int err;

	if (!is_valid_ether_addr(addr->sa_data)) {
		err = -EADDRNOTAVAIL;
		goto error;
	}

	if (!netif_running(dev)) {
		err = mvpp2_prs_update_mac_da(dev, addr->sa_data);
		if (!err)
			return 0;
		/* Reconfigure parser to accept the original MAC address */
		err = mvpp2_prs_update_mac_da(dev, dev->dev_addr);
		if (err)
			goto error;
	}

	mvpp2_stop_dev(port);

	err = mvpp2_prs_update_mac_da(dev, addr->sa_data);
	if (!err)
		goto out_start;

	/* Reconfigure parser accept the original MAC address */
	err = mvpp2_prs_update_mac_da(dev, dev->dev_addr);
	if (err)
		goto error;
out_start:
	mvpp2_start_dev(port);
	mvpp2_egress_enable(port);
	mvpp2_ingress_enable(port);
	return 0;

error:
	netdev_err(dev, "fail to change MAC address\n");
	return err;
}

static int mvpp2_change_mtu(struct net_device *dev, int mtu)
{
	struct mvpp2_port *port = netdev_priv(dev);
	int err;

	mtu = mvpp2_check_mtu_valid(dev, mtu);
	if (mtu < 0) {
		err = mtu;
		goto error;
	}

	if (!netif_running(dev)) {
		err = mvpp2_bm_update_mtu(dev, mtu);
		if (!err) {
			port->pkt_size =  MVPP2_RX_PKT_SIZE(mtu);
			return 0;
		}

		/* Reconfigure BM to the original MTU */
		err = mvpp2_bm_update_mtu(dev, dev->mtu);
		if (err)
			goto error;
	}

	mvpp2_stop_dev(port);

	err = mvpp2_bm_update_mtu(dev, mtu);
	if (!err) {
		port->pkt_size =  MVPP2_RX_PKT_SIZE(mtu);
		goto out_start;
	}

	/* Reconfigure BM to the original MTU */
	err = mvpp2_bm_update_mtu(dev, dev->mtu);
	if (err)
		goto error;

out_start:
	mvpp2_start_dev(port);
	mvpp2_egress_enable(port);
	mvpp2_ingress_enable(port);

	return 0;

error:
	netdev_err(dev, "fail to change MTU\n");
	return err;
}

static struct rtnl_link_stats64 *
mvpp2_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	struct mvpp2_port *port = netdev_priv(dev);
	unsigned int start;
	int cpu;

	for_each_possible_cpu(cpu) {
		struct mvpp2_pcpu_stats *cpu_stats;
		u64 rx_packets;
		u64 rx_bytes;
		u64 tx_packets;
		u64 tx_bytes;

		cpu_stats = per_cpu_ptr(port->stats, cpu);
		do {
			start = u64_stats_fetch_begin_irq(&cpu_stats->syncp);
			rx_packets = cpu_stats->rx_packets;
			rx_bytes   = cpu_stats->rx_bytes;
			tx_packets = cpu_stats->tx_packets;
			tx_bytes   = cpu_stats->tx_bytes;
		} while (u64_stats_fetch_retry_irq(&cpu_stats->syncp, start));

		stats->rx_packets += rx_packets;
		stats->rx_bytes   += rx_bytes;
		stats->tx_packets += tx_packets;
		stats->tx_bytes   += tx_bytes;
	}

	stats->rx_errors	= dev->stats.rx_errors;
	stats->rx_dropped	= dev->stats.rx_dropped;
	stats->tx_dropped	= dev->stats.tx_dropped;

	return stats;
}

static int mvpp2_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct mvpp2_port *port = netdev_priv(dev);
	int ret;

	if (!port->phy_dev)
		return -ENOTSUPP;

	ret = phy_mii_ioctl(port->phy_dev, ifr, cmd);
	if (!ret)
		mvpp2_link_event(dev);

	return ret;
}



/* Device ops */

static const struct net_device_ops mvpp2_netdev_ops = {
	.ndo_open		= mvpp2_open,
	.ndo_stop		= mvpp2_stop,
	.ndo_start_xmit		= mvpp2_tx,
	.ndo_set_rx_mode	= mvpp2_set_rx_mode,
	.ndo_set_mac_address	= mvpp2_set_mac_address,
	.ndo_change_mtu		= mvpp2_change_mtu,
	.ndo_get_stats64	= mvpp2_get_stats64,
	.ndo_do_ioctl		= mvpp2_ioctl,
};

/* Driver initialization */

static void mvpp2_port_power_up(struct mvpp2_port *port)
{
	mvpp2_port_mii_set(port);
	mvpp2_port_periodic_xon_disable(port);
	mvpp2_port_fc_adv_enable(port);
	mvpp2_port_reset(port);
}

/* Initialize port HW */
static int mvpp2_port_init(struct mvpp2_port *port)
{
	struct device *dev = port->dev->dev.parent;
	struct mvpp2 *priv = port->priv;
	struct mvpp2_txq_pcpu *txq_pcpu;
	int queue, cpu, err;

	if (port->first_rxq + mvpp2_rxq_number > MVPP2_RXQ_TOTAL_NUM)
		return -EINVAL;

	/* Disable port */
	mvpp2_egress_disable(port);
	mvpp2_port_disable(port);

	port->txqs = devm_kcalloc(dev, mvpp2_txq_number, sizeof(*port->txqs),
				  GFP_KERNEL);
	if (!port->txqs)
		return -ENOMEM;

	/* Associate physical Tx queues to this port and initialize.
	 * The mapping is predefined.
	 */
	for (queue = 0; queue < mvpp2_txq_number; queue++) {
		int queue_phy_id = mvpp2_txq_phys(port->id, queue);
		struct mvpp2_tx_queue *txq;

		txq = devm_kzalloc(dev, sizeof(*txq), GFP_KERNEL);
		if (!txq)
			return -ENOMEM;

		txq->pcpu = alloc_percpu(struct mvpp2_txq_pcpu);
		if (!txq->pcpu) {
			err = -ENOMEM;
			goto err_free_percpu;
		}

		txq->id = queue_phy_id;
		txq->log_id = queue;
		txq->pkts_coal = MVPP2_TXDONE_COAL_PKTS_THRESH;
		for_each_present_cpu(cpu) {
			txq_pcpu = per_cpu_ptr(txq->pcpu, cpu);
			txq_pcpu->cpu = cpu;
		}

		port->txqs[queue] = txq;
	}

	port->rxqs = devm_kcalloc(dev, mvpp2_rxq_number, sizeof(*port->rxqs),
				  GFP_KERNEL);
	if (!port->rxqs) {
		err = -ENOMEM;
		goto err_free_percpu;
	}

	/* Allocate and initialize Rx queue for this port */
	for (queue = 0; queue < mvpp2_rxq_number; queue++) {
		struct mvpp2_rx_queue *rxq;

		/* Map physical Rx queue to port's logical Rx queue */
		rxq = devm_kzalloc(dev, sizeof(*rxq), GFP_KERNEL);
		if (!rxq)
			goto err_free_percpu;
		/* Map this Rx queue to a physical queue */
		rxq->id = port->first_rxq + queue;
		rxq->port = port->id;
		rxq->log_id = queue;

		port->rxqs[queue] = rxq;
	}

	/* Configure Rx queue group interrupt for this port */
	mvpp2_write(priv, MVPP21_ISR_RXQ_GROUP_REG(port->id), mvpp2_rxq_number);

	/* Create Rx descriptor rings */
	for (queue = 0; queue < mvpp2_rxq_number; queue++) {
		struct mvpp2_rx_queue *rxq = port->rxqs[queue];

		rxq->size = port->rx_ring_size;
		rxq->pkts_coal = MVPP2_RX_COAL_PKTS;
		rxq->time_coal = MVPP2_RX_COAL_USEC;
	}

	mvpp2_ingress_disable(port);

	/* Port default configuration */
	mvpp2_defaults_set(port);

	/* Port's classifier configuration */
	mvpp2_cls_oversize_rxq_set(port);
	mvpp2_cls_port_config(port);

	/* Provide an initial Rx packet size */
	port->pkt_size = MVPP2_RX_PKT_SIZE(port->dev->mtu);

	/* Initialize pools for swf */
	err = mvpp2_swf_bm_pool_init(port);
	if (err)
		goto err_free_percpu;

	return 0;

err_free_percpu:
	for (queue = 0; queue < mvpp2_txq_number; queue++) {
		if (!port->txqs[queue])
			continue;
		free_percpu(port->txqs[queue]->pcpu);
	}
	return err;
}

/* Ports initialization */
static int mvpp2_port_probe(struct platform_device *pdev,
			    struct device_node *port_node,
			    struct mvpp2 *priv,
			    int *next_first_rxq)
{
	struct device_node *phy_node;
	struct mvpp2_port *port;
	struct net_device *dev;
	struct resource *res;
	const char *dt_mac_addr;
	const char *mac_from;
	char hw_mac_addr[ETH_ALEN];
	u32 id;
	int features;
	int phy_mode;
	int priv_common_regs_num = 2;
	int err, i;

	dev = alloc_etherdev_mqs(sizeof(struct mvpp2_port), mvpp2_txq_number,
				 mvpp2_rxq_number);
	if (!dev)
		return -ENOMEM;

	phy_node = of_parse_phandle(port_node, "phy", 0);
	if (!phy_node) {
		dev_err(&pdev->dev, "missing phy\n");
		err = -ENODEV;
		goto err_free_netdev;
	}

	phy_mode = of_get_phy_mode(port_node);
	if (phy_mode < 0) {
		dev_err(&pdev->dev, "incorrect phy mode\n");
		err = phy_mode;
		goto err_free_netdev;
	}

	if (of_property_read_u32(port_node, "port-id", &id)) {
		err = -EINVAL;
		dev_err(&pdev->dev, "missing port-id value\n");
		goto err_free_netdev;
	}

	dev->tx_queue_len = MVPP2_MAX_TXD;
	dev->watchdog_timeo = 5 * HZ;
	dev->netdev_ops = &mvpp2_netdev_ops;
	mvpp2_set_ethtool_ops(dev);

	port = netdev_priv(dev);

	port->irq = irq_of_parse_and_map(port_node, 0);
	if (port->irq <= 0) {
		err = -EINVAL;
		goto err_free_netdev;
	}

	if (of_property_read_bool(port_node, "marvell,loopback"))
		port->flags |= MVPP2_F_LOOPBACK;

	port->priv = priv;
	port->id = id;
	port->first_rxq = *next_first_rxq;
	port->phy_node = phy_node;
	port->phy_interface = phy_mode;

	res = platform_get_resource(pdev, IORESOURCE_MEM,
				    priv_common_regs_num + id);
	port->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(port->base)) {
		err = PTR_ERR(port->base);
		goto err_free_irq;
	}

	/* Alloc per-cpu stats */
	port->stats = netdev_alloc_pcpu_stats(struct mvpp2_pcpu_stats);
	if (!port->stats) {
		err = -ENOMEM;
		goto err_free_irq;
	}

	dt_mac_addr = of_get_mac_address(port_node);
	if (dt_mac_addr && is_valid_ether_addr(dt_mac_addr)) {
		mac_from = "device tree";
		ether_addr_copy(dev->dev_addr, dt_mac_addr);
	} else {
		mvpp2_get_mac_address(port, hw_mac_addr);
		if (is_valid_ether_addr(hw_mac_addr)) {
			mac_from = "hardware";
			ether_addr_copy(dev->dev_addr, hw_mac_addr);
		} else {
			mac_from = "random";
			eth_hw_addr_random(dev);
		}
	}

	port->tx_ring_size = MVPP2_MAX_TXD;
	port->rx_ring_size = MVPP2_MAX_RXD;
	port->dev = dev;
	SET_NETDEV_DEV(dev, &pdev->dev);

	err = mvpp2_port_init(port);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to init port %d\n", id);
		goto err_free_stats;
	}
	mvpp2_port_power_up(port);

	netif_napi_add(dev, &port->napi, mvpp2_poll, NAPI_POLL_WEIGHT);
	features = NETIF_F_SG | NETIF_F_IP_CSUM;
	dev->features = features | NETIF_F_RXCSUM;
	dev->hw_features |= features | NETIF_F_RXCSUM | NETIF_F_GRO;
	dev->vlan_features |= features;

	err = register_netdev(dev);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register netdev\n");
		goto err_free_txq_pcpu;
	}
	netdev_info(dev, "Using %s mac address %pM\n", mac_from, dev->dev_addr);

	/* Increment the first Rx queue number to be used by the next port */
	*next_first_rxq += mvpp2_rxq_number;
	priv->port_list[id] = port;
	return 0;

err_free_txq_pcpu:
	for (i = 0; i < mvpp2_txq_number; i++)
		free_percpu(port->txqs[i]->pcpu);
err_free_stats:
	free_percpu(port->stats);
err_free_irq:
	irq_dispose_mapping(port->irq);
err_free_netdev:
	free_netdev(dev);
	return err;
}

/* Ports removal routine */
static void mvpp2_port_remove(struct mvpp2_port *port)
{
	int i;

	unregister_netdev(port->dev);
	free_percpu(port->stats);
	for (i = 0; i < mvpp2_txq_number; i++)
		free_percpu(port->txqs[i]->pcpu);
	irq_dispose_mapping(port->irq);
	free_netdev(port->dev);
}


/* Initialize decoding windows */
static void mvpp2_conf_mbus_windows(const struct mbus_dram_target_info *dram,
				    struct mvpp2 *priv)
{
	u32 win_enable;
	int i;

	for (i = 0; i < 6; i++) {
		mvpp2_write(priv, MVPP2_WIN_BASE(i), 0);
		mvpp2_write(priv, MVPP2_WIN_SIZE(i), 0);

		if (i < 4)
			mvpp2_write(priv, MVPP2_WIN_REMAP(i), 0);
	}

	win_enable = 0;

	for (i = 0; i < dram->num_cs; i++) {
		const struct mbus_dram_window *cs = dram->cs + i;

		mvpp2_write(priv, MVPP2_WIN_BASE(i),
			    (cs->base & 0xffff0000) | (cs->mbus_attr << 8) |
			    dram->mbus_dram_target_id);

		mvpp2_write(priv, MVPP2_WIN_SIZE(i),
			    (cs->size - 1) & 0xffff0000);

		win_enable |= (1 << i);
	}

	mvpp2_write(priv, MVPP2_BASE_ADDR_ENABLE, win_enable);
}

/* Initialize Rx FIFO's */
static void mvpp2_rx_fifo_init(struct mvpp2 *priv)
{
	int port;

	for (port = 0; port < MVPP2_MAX_PORTS; port++) {
		mvpp2_write(priv, MVPP2_RX_DATA_FIFO_SIZE_REG(port),
			    MVPP2_RX_FIFO_PORT_DATA_SIZE);
		mvpp2_write(priv, MVPP2_RX_ATTR_FIFO_SIZE_REG(port),
			    MVPP2_RX_FIFO_PORT_ATTR_SIZE);
	}

	mvpp2_write(priv, MVPP2_RX_MIN_PKT_SIZE_REG,
		    MVPP2_RX_FIFO_PORT_MIN_PKT);
	mvpp2_write(priv, MVPP2_RX_FIFO_INIT_REG, 0x1);
}

/* Initialize network controller common part HW */
static int mvpp2_init(struct platform_device *pdev, struct mvpp2 *priv)
{
	const struct mbus_dram_target_info *dram_target_info;
	int err, i;
	u32 val;

	/* Checks for hardware constraints */
	if (mvpp2_rxq_number % 4 || (mvpp2_rxq_number > MVPP2_MAX_RXQ) ||
	    (mvpp2_txq_number > MVPP2_MAX_TXQ)) {
		dev_err(&pdev->dev, "invalid queue size parameter\n");
		return -EINVAL;
	}

	/* MBUS windows configuration */
	dram_target_info = mv_mbus_dram_info();
	if (dram_target_info)
		mvpp2_conf_mbus_windows(dram_target_info, priv);

	/* Disable HW PHY polling */
	val = readl(priv->lms_base + MVPP2_PHY_AN_CFG0_REG);
	val |= MVPP2_PHY_AN_STOP_SMI0_MASK;
	writel(val, priv->lms_base + MVPP2_PHY_AN_CFG0_REG);

	/* Allocate and initialize aggregated TXQs */
	priv->aggr_txqs = devm_kcalloc(&pdev->dev, num_present_cpus(),
				       sizeof(struct mvpp2_aggr_tx_queue),
				       GFP_KERNEL);
	if (!priv->aggr_txqs)
		return -ENOMEM;

	for_each_present_cpu(i) {
		priv->aggr_txqs[i].id = i;
		priv->aggr_txqs[i].size = MVPP2_AGGR_TXQ_SIZE;
		err = mvpp2_aggr_txq_init(pdev, &priv->aggr_txqs[i],
					  MVPP2_AGGR_TXQ_SIZE, i, priv);
		if (err < 0)
			return err;
	}

	/* Rx Fifo Init */
	mvpp2_rx_fifo_init(priv);

	/* Reset Rx queue group interrupt configuration */
	for (i = 0; i < MVPP2_MAX_PORTS; i++)
		mvpp2_write(priv, MVPP21_ISR_RXQ_GROUP_REG(i), mvpp2_rxq_number);

	writel(MVPP2_EXT_GLOBAL_CTRL_DEFAULT,
	       priv->lms_base + MVPP2_MNG_EXTENDED_GLOBAL_CTRL_REG);

	/* Allow cache snoop when transmiting packets */
	mvpp2_write(priv, MVPP21_TX_SNOOP_REG, 0x1);

	/* Buffer Manager initialization */
	err = mvpp2_bm_init(pdev, priv);
	if (err < 0)
		return err;

	/* Parser default initialization */
	err = mvpp2_prs_default_init(pdev, priv);
	if (err < 0)
		return err;

	/* Classifier default initialization */
	mvpp2_cls_init(priv);

	return 0;
}


static const struct mvpp2x_platform_data pp21_pdata = {
	.pp2x_ver = PPV21,	
};

static const struct mvpp2x_platform_data pp22_pdata = {
	.pp2x_ver = PPV22,
};



static const struct of_device_id mvpp2_match_tbl[] = {
        {
                .compatible = "marvell,armada-375-pp21",
                .data = &pp21_pdata,
        },
        {
                .compatible = "marvell,pp22",
                .data = &pp22_pdata,
        },
        { /* end */ }
};


static int mvpp2_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	struct device_node *port_node;
	struct mvpp2 *priv;
	struct resource *res;
	const struct of_device_id *match;	
	int port_count, first_rxq;
	int err;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct mvpp2), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	match = of_match_node(mvpp2_match_tbl, dn);

	if (!match)
		return -ENODEV;
	priv->pp2xdata = (const struct mvpp2x_platform_data *) match->data;
	priv->pp2_version = priv->pp2xdata->pp2x_ver;
		
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->lms_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->lms_base))
		return PTR_ERR(priv->lms_base);

	priv->pp_clk = devm_clk_get(&pdev->dev, "pp_clk");
	if (IS_ERR(priv->pp_clk))
		return PTR_ERR(priv->pp_clk);
	err = clk_prepare_enable(priv->pp_clk);
	if (err < 0)
		return err;

	priv->gop_clk = devm_clk_get(&pdev->dev, "gop_clk");
	if (IS_ERR(priv->gop_clk)) {
		err = PTR_ERR(priv->gop_clk);
		goto err_pp_clk;
	}
	err = clk_prepare_enable(priv->gop_clk);
	if (err < 0)
		goto err_pp_clk;

	/* Get system's tclk rate */
	priv->tclk = clk_get_rate(priv->pp_clk);

	/* Initialize network controller */
	err = mvpp2_init(pdev, priv);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to initialize controller\n");
		goto err_gop_clk;
	}

	port_count = of_get_available_child_count(dn);
	if (port_count == 0) {
		dev_err(&pdev->dev, "no ports enabled\n");
		err = -ENODEV;
		goto err_gop_clk;
	}

	priv->port_list = devm_kcalloc(&pdev->dev, port_count,
				      sizeof(struct mvpp2_port *),
				      GFP_KERNEL);
	if (!priv->port_list) {
		err = -ENOMEM;
		goto err_gop_clk;
	}

	/* Initialize ports */
	first_rxq = 0;
	for_each_available_child_of_node(dn, port_node) {
		err = mvpp2_port_probe(pdev, port_node, priv, &first_rxq);
		if (err < 0)
			goto err_gop_clk;
	}

	platform_set_drvdata(pdev, priv);
	return 0;

err_gop_clk:
	clk_disable_unprepare(priv->gop_clk);
err_pp_clk:
	clk_disable_unprepare(priv->pp_clk);
	return err;
}

static int mvpp2_remove(struct platform_device *pdev)
{
	struct mvpp2 *priv = platform_get_drvdata(pdev);
	struct device_node *dn = pdev->dev.of_node;
	struct device_node *port_node;
	int i = 0;

	for_each_available_child_of_node(dn, port_node) {
		if (priv->port_list[i])
			mvpp2_port_remove(priv->port_list[i]);
		i++;
	}

	for (i = 0; i < MVPP2_BM_POOLS_NUM; i++) {
		struct mvpp2_bm_pool *bm_pool = &priv->bm_pools[i];

		mvpp2_bm_pool_destroy(pdev, priv, bm_pool);
	}

	for_each_present_cpu(i) {
		struct mvpp2_aggr_tx_queue *aggr_txq = &priv->aggr_txqs[i];

		dma_free_coherent(&pdev->dev,
				  MVPP2_AGGR_TXQ_SIZE * MVPP2_DESC_ALIGNED_SIZE,
				  aggr_txq->descs,
				  aggr_txq->descs_phys);
	}

	clk_disable_unprepare(priv->pp_clk);
	clk_disable_unprepare(priv->gop_clk);

	return 0;
}




MODULE_DEVICE_TABLE(of, mvpp2_match_tbl);

static struct platform_driver mvpp2_driver = {
	.probe = mvpp2_probe,
	.remove = mvpp2_remove,
	.driver = {
		.name = MVPP2_DRIVER_NAME,
		.of_match_table = mvpp2_match_tbl,
	},
};

module_platform_driver(mvpp2_driver);

MODULE_DESCRIPTION("Marvell PPv2 Ethernet Driver - www.marvell.com");
MODULE_AUTHOR("Marcin Wojtas <mw@semihalf.com>");
MODULE_LICENSE("GPL v2");
