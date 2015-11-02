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
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <uapi/linux/ppp_defs.h>
#include <net/ip.h>
#include <net/ipv6.h>

#ifdef CONFIG_MV_PP2_FPGA
#include <linux/pci.h>
#endif


#include "mvpp2.h"
#include "mvpp2_hw.h"
#include "mvpp2_debug.h"




/* Declaractions */
u8 mvpp2_num_cos_queues = 4;
static bool mvpp2_queue_mode = MVPP2_QDIST_MULTI_MODE;
static bool rss_mode = 0;
static u8 default_cpu = 0;
static bool cos_classifer = 0;
static u32 pri_map = 0x0;
static u8 default_cos = 0;
static bool jumbo_pool = false;
static u16 rx_queue_size = MVPP2_MAX_RXD;
static u16 tx_queue_size = MVPP2_MAX_TXD;
static u16 buffer_scaling = 100;
static u8 first_bm_pool = 0;
static u8 first_addr_space = 0;
static u8 first_log_rxq_queue = 0;

#ifdef CONFIG_MV_PP2_FPGA
#define FPGA_PORTS_BASE          0
#define MV_PP2_FPGA_PERODIC_TIME 10
#define FPGA_PORT_0_OFFSET       0x104000

u32 mv_pp2_vfpga_address;
struct timer_list cpu_poll_timer;
static void mv_pp22_cpu_timer_callback(unsigned long data);
#endif

module_param_named(num_cos_queues, mvpp2_num_cos_queues, byte, S_IRUGO);
MODULE_PARM_DESC(num_cos_queues,"Set number of cos_queues (1-8), def=4");

module_param_named(queue_mode, mvpp2_queue_mode, bool, S_IRUGO);
MODULE_PARM_DESC(queue_mode, "Set queue_mode (single=0, multi=1)");

module_param(rss_mode, bool, S_IRUGO);
MODULE_PARM_DESC(rss_mode, "Set rss_mode (UDP_2T=0, UDP_5T=1)");

module_param(default_cpu, byte, S_IRUGO);
MODULE_PARM_DESC(default_cpu, "Set default CPU for non RSS frames");

module_param(cos_classifer, bool, S_IRUGO);
MODULE_PARM_DESC(cos_classifer, "Cos Classifier (vlan_pri=0, dscp=1)");

module_param(pri_map, uint, S_IRUGO);
MODULE_PARM_DESC(pri_map, "Set priority_map, nibble for each cos.");

module_param(default_cos, byte, S_IRUGO);
MODULE_PARM_DESC(default_cos, "Set default cos (0-(num_cose_queues-1)).");

module_param(jumbo_pool, bool, S_IRUGO);
MODULE_PARM_DESC(jumbo_pool, "no_jumbo_support(0), jumbo_support(1)");

module_param(rx_queue_size, ushort, S_IRUGO);
MODULE_PARM_DESC(rx_queue_size, "Rx queue size");

module_param(tx_queue_size, ushort, S_IRUGO);
MODULE_PARM_DESC(tx_queue_size, "Tx queue size");

module_param(buffer_scaling, ushort, S_IRUGO);
MODULE_PARM_DESC(buffer_scaling, "Buffer scaling (TBD)");


/*TODO:  Below module_params will not go to ML. Created for testing. */
module_param(first_bm_pool, byte, S_IRUGO);
MODULE_PARM_DESC(first_bm_pool, "First used buffer pool (0-11)");

module_param(first_addr_space, byte, S_IRUGO);
MODULE_PARM_DESC(first_addr_space, "First used PPV22 address space (0-8)");

module_param(first_log_rxq_queue, byte, S_IRUGO);
MODULE_PARM_DESC(first_log_rxq_queue, "First logical rx_queue (0-31)");



/* Number of RXQs used by single port */
static int mvpp2_rxq_number;
/* Number of TXQs used by single port */
static int mvpp2_txq_number;



struct mvpp2_pool_attributes mvpp2_pools[] = {
	{
		.description =  "short",
		/*.pkt_size    = 	MVPP2_BM_SHORT_PKT_SIZE, */
		.buf_num     =  MVPP2_BM_SHORT_BUF_NUM,
	},
	{
		.description =  "long",
		/*.pkt_size    = 	MVPP2_BM_LONG_PKT_SIZE,*/
		.buf_num     =  MVPP2_BM_LONG_BUF_NUM,
	},
	{
		.description =	"jumbo",
		/*.pkt_size    =	MVPP2_BM_JUMBO_PKT_SIZE, */
		.buf_num     =  MVPP2_BM_JUMBO_BUF_NUM,
	}
};




static void mvpp2_txq_inc_get(struct mvpp2_txq_pcpu *txq_pcpu)
{
	txq_pcpu->txq_get_index++;
	if (txq_pcpu->txq_get_index == txq_pcpu->size)
		txq_pcpu->txq_get_index = 0;
}

static void mvpp2_txq_inc_put(enum mvppv2_version pp2_ver,
			      struct mvpp2_txq_pcpu *txq_pcpu,
			      struct sk_buff *skb,
			      struct mvpp2_tx_desc *tx_desc)
{
	txq_pcpu->tx_skb[txq_pcpu->txq_put_index] = skb;
	if (skb)
		txq_pcpu->tx_buffs[txq_pcpu->txq_put_index] =
				mvpp2x_txdesc_phys_addr_get(pp2_ver, tx_desc);
	txq_pcpu->txq_put_index++;
	if (txq_pcpu->txq_put_index == txq_pcpu->size)
		txq_pcpu->txq_put_index = 0;
}

static inline u8 mvpp2_first_pool_get(struct mvpp2 *priv) {
	return(priv->pp2_cfg.first_bm_pool);
}

static inline u8 mvpp2_num_pools_get(struct mvpp2 *priv) {
	return((priv->pp2_cfg.jumbo_pool == true)? 3:2);
}

static inline u8 mvpp2_last_pool_get(struct mvpp2 *priv) {
	return(mvpp2_first_pool_get(priv) + mvpp2_num_pools_get(priv));
}

static inline int mvpp2_pool_pkt_size_get(enum mvpp2_bm_pool_log_num  log_id) {
	return mvpp2_pools[log_id].pkt_size;

}

static inline int mvpp2_pool_buf_num_get(enum mvpp2_bm_pool_log_num  log_id) {
	return mvpp2_pools[log_id].buf_num;

}


static inline const char * mvpp2_pool_description_get(enum mvpp2_bm_pool_log_num  log_id) {
	return mvpp2_pools[log_id].description;

}




/* Buffer Manager configuration routines */

/* Create pool */
static int mvpp2_bm_pool_create(struct platform_device *pdev,
				struct mvpp2_hw *hw,
				struct mvpp2_bm_pool *bm_pool, int size)
{
	int size_bytes;

	/* Driver enforces size= x16 both for PPv21 and for PPv22, even though
	    PPv22 HW allows size= x8 */
	if (!IS_ALIGNED(size, (1<<MVPP21_BM_POOL_SIZE_OFFSET)))
		return -EINVAL;

	size_bytes = 2 * sizeof(uintptr_t) * size; /*YuvalC: Two pointers per buffer, existing bug fixed. */
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

	mvpp2_bm_hw_pool_create(hw, bm_pool->id, bm_pool->phys_addr, size);

	bm_pool->size = size;
	bm_pool->pkt_size = mvpp2_pool_pkt_size_get(bm_pool->log_id);
	bm_pool->buf_num = 0;
	mvpp2_bm_pool_bufsize_set(hw, bm_pool, MVPP2_RX_BUF_SIZE(bm_pool->pkt_size));
	atomic_set(&bm_pool->in_use, 0);

	return 0;
}

void mvpp2_bm_bufs_free(struct mvpp2_hw *hw, struct mvpp2_bm_pool *bm_pool,
			int buf_num)
{
	int i;

	if (buf_num > bm_pool->buf_num) {
		WARN(1, "Pool does not have so many bufs pool(%d) bufs(%d)\n",
			bm_pool->id, buf_num);
		buf_num = bm_pool->buf_num;

	}
	for (i = 0; i < buf_num; i++) {
		struct sk_buff *vaddr;

		/* Get buffer virtual adress (indirect access) */
		vaddr = mvpp2_bm_virt_addr_get(hw, bm_pool->id);
		if (!vaddr)
			break;
		dev_kfree_skb_any(vaddr);
	}

	/* Update BM driver with number of buffers removed from pool */
	bm_pool->buf_num -= i;
}


/* Cleanup pool */
static int mvpp2_bm_pool_destroy(struct platform_device *pdev,
				 struct mvpp2_hw *hw,
				 struct mvpp2_bm_pool *bm_pool)
{
	u32 val;
	int size_bytes;

	mvpp2_bm_bufs_free(hw, bm_pool, bm_pool->buf_num);
	if (bm_pool->buf_num) {
		WARN(1, "cannot free all buffers in pool %d\n", bm_pool->id);
		return 0;
	}

	val = mvpp2_read(hw, MVPP2_BM_POOL_CTRL_REG(bm_pool->id));
	val |= MVPP2_BM_STOP_MASK;
	mvpp2_write(hw, MVPP2_BM_POOL_CTRL_REG(bm_pool->id), val);

	size_bytes = 2 * sizeof(uintptr_t) * bm_pool->size;
	dma_free_coherent(&pdev->dev, size_bytes, bm_pool->virt_addr,
			  bm_pool->phys_addr);
	mvpp2_bm_pool_bufsize_set(hw, bm_pool,0);
	return 0;
}

static int mvpp2_bm_pools_init(struct platform_device *pdev,
			       struct mvpp2 *priv, u8 first_pool, u8 num_pools)
{
	int i, err, size;
	struct mvpp2_bm_pool *bm_pool;
	struct mvpp2_hw *hw = &priv->hw;

	/* Create all pools with maximum size */
	size = MVPP2_BM_POOL_SIZE_MAX;
	for (i = 0; i < num_pools; i++) {
		bm_pool = &priv->bm_pools[i];
		bm_pool->log_id = i;
		bm_pool->id = first_pool + i;
		err = mvpp2_bm_pool_create(pdev, hw, bm_pool, size);
		if (err)
			goto err_unroll_pools;
	}
	priv->num_pools = num_pools;
	return 0;

err_unroll_pools:
	dev_err(&pdev->dev, "failed to create BM pool %d, size %d\n", i, size);
	for (i = i - 1; i >= 0; i--)
		mvpp2_bm_pool_destroy(pdev, &priv->hw, &priv->bm_pools[i]);
		return err;
}

static int mvpp2_bm_init(struct platform_device *pdev, struct mvpp2 *priv)
{
	int i, err;
	u8 first_pool = mvpp2_first_pool_get(priv);
	u8 num_pools = mvpp2_num_pools_get(priv);

	for (i = first_pool; i < (first_pool+num_pools); i++) {
		/* Mask BM all interrupts */
		mvpp2_write(&priv->hw, MVPP2_BM_INTR_MASK_REG(i), 0);
		/* Clear BM cause register */
		mvpp2_write(&priv->hw, MVPP2_BM_INTR_CAUSE_REG(i), 0);
	}

	/* Allocate and initialize BM pools */
	priv->bm_pools = devm_kcalloc(&pdev->dev, num_pools,
				     sizeof(struct mvpp2_bm_pool), GFP_KERNEL);
	if (!priv->bm_pools)
		return -ENOMEM;

	err = mvpp2_bm_pools_init(pdev, priv, first_pool, num_pools);
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
		   mvpp2_pool_description_get(bm_pool->log_id),
		   bm_pool->id, bm_pool->pkt_size, buf_size, total_size);

	netdev_dbg(port->dev,
		   "%s pool %d: %d of %d buffers added\n",
		   mvpp2_pool_description_get(bm_pool->log_id),
		   bm_pool->id, i, buf_num);
	return i;
}

static int mvpp2_bm_buf_calc(enum mvpp2_bm_pool_log_num log_pool, u32 port_map) {

	/*TODO: Code algo based  on port_map/num_rx_queues/num_tx_queues/queue_sizes */
	int num_ports = hweight_long(port_map);
	return(num_ports*mvpp2_pool_buf_num_get(log_pool));
}



/* Notify the driver that BM pool is being used as specific type and return the
 * pool pointer on success
 */

static struct mvpp2_bm_pool * mvpp2_bm_pool_use_internal(
		struct mvpp2_port *port, enum mvpp2_bm_pool_log_num log_pool,
		bool add_port)
{
	int pkts_num, add_num, num;
	struct mvpp2_bm_pool *pool = &port->priv->bm_pools[log_pool];
	struct mvpp2_hw *hw = &(port->priv->hw);

	if (log_pool < MVPP2_BM_SWF_SHORT_POOL || log_pool > MVPP2_BM_SWF_JUMBO_POOL) {
		netdev_err(port->dev, "pool does not exist\n");
		return NULL;
	}

	if (add_port) {
		pkts_num = mvpp2_bm_buf_calc(log_pool, pool->port_map |(1 << port->id));
	}
	else {
		pkts_num = mvpp2_bm_buf_calc(log_pool, pool->port_map & ~(1 << port->id));
	}

	add_num = pkts_num - pool->buf_num;

	/* Allocate buffers for this pool */
	if (add_num > 0) {
		num = mvpp2_bm_bufs_add(port, pool, add_num);
		if (num != add_num) {
			WARN(1, "pool %d: %d of %d allocated\n",
			     pool->id, num, pkts_num);
			/* We need to undo the bufs_add() allocations */
			return NULL;
		}
	} else if (add_num < 0) {
		mvpp2_bm_bufs_free(hw, pool, -add_num);
	}

	return pool;
}



static struct mvpp2_bm_pool * mvpp2_bm_pool_use(struct mvpp2_port *port,
			enum mvpp2_bm_pool_log_num log_pool)
{
	return(mvpp2_bm_pool_use_internal(port, log_pool, true));
}


static struct mvpp2_bm_pool * mvpp2_bm_pool_stop_use(struct mvpp2_port *port,
			enum mvpp2_bm_pool_log_num log_pool)
{
	return(mvpp2_bm_pool_use_internal(port, log_pool, false));
}


/* Initialize pools for swf */
static int mvpp2_swf_bm_pool_init(struct mvpp2_port *port)
{
	int rxq;
	struct mvpp2_hw *hw = &(port->priv->hw);

	if (!port->pool_long) {
		port->pool_long =
		       mvpp2_bm_pool_use(port, MVPP2_BM_SWF_LONG_POOL);
		if (!port->pool_long)
			return -ENOMEM;
		port->pool_long->port_map |= (1 << port->id);

		for (rxq = 0; rxq < port->num_rx_queues; rxq++) {
			port->priv->pp2xdata->mvpp2x_rxq_long_pool_set(hw,
				port->rxqs[rxq]->id, port->pool_long->id);
		}
	}

	if (!port->pool_short) {
		port->pool_short =
			mvpp2_bm_pool_use(port, MVPP2_BM_SWF_SHORT_POOL);
		if (!port->pool_short)
			return -ENOMEM;

		port->pool_short->port_map |= (1 << port->id);

		for (rxq = 0; rxq < port->num_rx_queues; rxq++)
			port->priv->pp2xdata->mvpp2x_rxq_short_pool_set(hw,
			port->rxqs[rxq]->id, port->pool_short->id);
	}

	return 0;
}

static int mvpp2_bm_update_mtu(struct net_device *dev, int mtu)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_bm_pool *old_port_pool = port->pool_long;
	struct mvpp2_hw *hw = &(port->priv->hw);
	enum mvpp2_bm_pool_log_num new_log_pool, old_log_pool = old_port_pool->log_id;
	int rxq;
	int pkt_size = MVPP2_RX_PKT_SIZE(mtu);

	if (pkt_size > MVPP2_BM_LONG_PKT_SIZE)
		new_log_pool = MVPP2_BM_SWF_JUMBO_POOL;
	else
		new_log_pool = MVPP2_BM_SWF_LONG_POOL;

	if (new_log_pool != old_log_pool) {
		/* Add port to new pool */
		port->pool_long = mvpp2_bm_pool_use(port, new_log_pool);
		if (!port->pool_long)
			return -ENOMEM;
		port->pool_long->port_map |= (1 << port->id);
		for (rxq = 0; rxq < port->num_rx_queues; rxq++)
			port->priv->pp2xdata->mvpp2x_rxq_long_pool_set(hw,
			port->rxqs[rxq]->id, port->pool_long->id);

		/* Remove port from old pool */
		mvpp2_bm_pool_stop_use(port, old_log_pool);
		old_port_pool->port_map &= ~(1 << port->id);
	}

	dev->mtu = mtu;
	netdev_update_features(dev);
	return 0;
}

#ifdef CONFIG_MV_PP2_FPGA
static void print_regs(struct mvpp2_port *port)
{
	int val;
	val = readl(port->base + 0x10);
	pr_debug("******* print_reg(%d):[0x%x] = 0x%x\n", __LINE__, (unsigned int)port->base + 0x10, val);
}
#endif

/* Set defaults to the MVPP2 port */
static void mvpp2_defaults_set(struct mvpp2_port *port)
{
	int tx_port_num, val, queue, ptxq, lrxq;
	struct mvpp2_hw *hw = &(port->priv->hw);

	/* Configure port to loopback if needed */
	if (port->flags & MVPP2_F_LOOPBACK)
		mvpp2_port_loopback_set(port);

#ifdef CONFIG_MV_PP2_FPGA
	print_regs(port);
	writel(0x8be4, port->base);
	writel(0xc200, port->base + 0x8);
	writel(0x3, port->base + 0x90);

	writel(0x902A, port->base + 0xC);    /*force link to 100Mb*/
	writel(0x8be5, port->base);          /*enable port        */
	print_regs(port);
#endif

	/* Update TX FIFO MIN Threshold */
	val = readl(port->base + MVPP2_GMAC_PORT_FIFO_CFG_1_REG);
	val &= ~MVPP2_GMAC_TX_FIFO_MIN_TH_ALL_MASK;
	/* Min. TX threshold must be less than minimal packet length */
	val |= MVPP2_GMAC_TX_FIFO_MIN_TH_MASK(64 - 4 - 2);
	writel(val, port->base + MVPP2_GMAC_PORT_FIFO_CFG_1_REG);

	/* Disable Legacy WRR, Disable EJP, Release from reset */
	tx_port_num = mvpp2_egress_port(port);
	mvpp2_write(hw , MVPP2_TXP_SCHED_PORT_INDEX_REG,
		    tx_port_num);
	mvpp2_write(hw , MVPP2_TXP_SCHED_CMD_1_REG, 0);

	/* Close bandwidth for all queues */
	for (queue = 0; queue < MVPP2_MAX_TXQ; queue++) {
		ptxq = mvpp2_txq_phys(port->id, queue);
		mvpp2_write(hw ,
			    MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(ptxq), 0);
	}

	/* Set refill period to 1 usec, refill tokens
	 * and bucket size to maximum
	 */
	mvpp2_write(hw , MVPP2_TXP_SCHED_PERIOD_REG,
		    hw->tclk / USEC_PER_SEC);
	val = mvpp2_read(hw, MVPP2_TXP_SCHED_REFILL_REG);
	val &= ~MVPP2_TXP_REFILL_PERIOD_ALL_MASK;
	val |= MVPP2_TXP_REFILL_PERIOD_MASK(1);
	val |= MVPP2_TXP_REFILL_TOKENS_ALL_MASK;
	mvpp2_write(hw, MVPP2_TXP_SCHED_REFILL_REG, val);
	val = MVPP2_TXP_TOKEN_SIZE_MAX;
	mvpp2_write(hw, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, val);

	/* Set MaximumLowLatencyPacketSize value to 256 */
	mvpp2_write(hw, MVPP2_RX_CTRL_REG(port->id),
		    MVPP2_RX_USE_PSEUDO_FOR_CSUM_MASK |
		    MVPP2_RX_LOW_LATENCY_PKT_SIZE(256));

	/* Enable Rx cache snoop */
	for (lrxq = 0; lrxq < port->num_rx_queues; lrxq++) {
		queue = port->rxqs[lrxq]->id;
		val = mvpp2_read(hw, MVPP2_RXQ_CONFIG_REG(queue));
		val |= MVPP2_SNOOP_PKT_SIZE_MASK |
			   MVPP2_SNOOP_BUF_HDR_MASK;
		mvpp2_write(hw, MVPP2_RXQ_CONFIG_REG(queue), val);
	}

	/* At default, mask all interrupts to all present cpus */
	mvpp2_port_interrupts_disable(port);
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
	   (txq->size - (num_active_cpus() * MVPP2_CPU_DESC_CHUNK)))
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

	for (i = 0; i < num; i++) {
		dma_addr_t buf_phys_addr =
				    txq_pcpu->tx_buffs[txq_pcpu->txq_get_index];
		struct sk_buff *skb = txq_pcpu->tx_skb[txq_pcpu->txq_get_index];

		mvpp2_txq_inc_get(txq_pcpu);

		if (!skb)
			continue;

		dma_unmap_single(port->dev->dev.parent, buf_phys_addr,
				 skb_headlen(skb), DMA_TO_DEVICE);
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


static unsigned int mvpp2_tx_done(struct mvpp2_port *port, u32 cause)
{
	struct mvpp2_tx_queue *txq;
	struct mvpp2_txq_pcpu *txq_pcpu;
	unsigned int tx_todo = 0;

	while (cause) {
		txq = mvpp2_get_tx_queue(port, cause);
		if (!txq)
			break;

		txq_pcpu = this_cpu_ptr(txq->pcpu);

		if (txq_pcpu->count) {
			mvpp2_txq_done(port, txq, txq_pcpu);
			tx_todo += txq_pcpu->count;
		}

		cause &= ~(1 << txq->log_id);
	}
	return tx_todo;
}


/* Rx/Tx queue initialization/cleanup methods */

/* Allocate and initialize descriptors for aggr TXQ */
static int mvpp2_aggr_txq_init(struct platform_device *pdev,
			       struct mvpp2_aggr_tx_queue *aggr_txq,
			       int desc_num, int cpu,
			       struct mvpp2 *priv)
{
	struct mvpp2_hw *hw = &priv->hw;

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
	aggr_txq->next_desc_to_proc = mvpp2_read(hw,
						 MVPP2_AGGR_TXQ_INDEX_REG(cpu));

	/* Set Tx descriptors queue starting address */
	/* indirect access */
	mvpp2_write(hw, MVPP21_AGGR_TXQ_DESC_ADDR_REG(cpu),
		    aggr_txq->descs_phys);
	mvpp2_write(hw, MVPP2_AGGR_TXQ_DESC_SIZE_REG(cpu), desc_num);

	return 0;
}

/* Create a specified Rx queue */
static int mvpp2_rxq_init(struct mvpp2_port *port,
			  struct mvpp2_rx_queue *rxq)

{
	struct mvpp2_hw *hw = &(port->priv->hw);

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
	mvpp2_write(hw, MVPP2_RXQ_STATUS_REG(rxq->id), 0);

	/* Set Rx descriptors queue starting address - indirect access */
	mvpp2_write(hw, MVPP2_RXQ_NUM_REG, rxq->id);

	MVPP2_PRINT_LINE();
	if (port->priv->pp2_version == PPV21)
		mvpp2_write(hw, MVPP21_RXQ_DESC_ADDR_REG, rxq->descs_phys);
	else
		mvpp2_write(hw, MVPP22_RXQ_DESC_ADDR_REG, (rxq->descs_phys>> MVPP22_RXQ_DESC_ADDR_SHIFT));
	MVPP2_PRINT_LINE();
	mvpp2_write(hw, MVPP2_RXQ_DESC_SIZE_REG, rxq->size);
	mvpp2_write(hw, MVPP2_RXQ_INDEX_REG, 0);

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
	struct mvpp2_hw *hw = &(port->priv->hw);

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
	mvpp2_write(hw, MVPP2_RXQ_STATUS_REG(rxq->id), 0);
	mvpp2_write(hw, MVPP2_RXQ_NUM_REG, rxq->id);
	mvpp2_write(hw, MVPP21_RXQ_DESC_ADDR_REG, 0);
	mvpp2_write(hw, MVPP2_RXQ_DESC_SIZE_REG, 0);
}

/* Create and initialize a Tx queue */
static int mvpp2_txq_init(struct mvpp2_port *port,
			  struct mvpp2_tx_queue *txq)
{
	u32 val;
	int cpu, desc, desc_per_txq, tx_port_num;
	struct mvpp2_hw *hw = &(port->priv->hw);
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
	mvpp2_write(hw, MVPP2_TXQ_NUM_REG, txq->id);
	mvpp2_write(hw, MVPP2_TXQ_DESC_ADDR_LOW_REG, txq->descs_phys);
	mvpp2_write(hw, MVPP2_TXQ_DESC_SIZE_REG, txq->size &
					     MVPP2_TXQ_DESC_SIZE_MASK);
	mvpp2_write(hw, MVPP2_TXQ_INDEX_REG, 0);
	mvpp2_write(hw, MVPP2_TXQ_RSVD_CLR_REG,
		    txq->id << MVPP2_TXQ_RSVD_CLR_OFFSET);
	val = mvpp2_read(hw, MVPP2_TXQ_PENDING_REG);
	val &= ~MVPP2_TXQ_PENDING_MASK;
	mvpp2_write(hw, MVPP2_TXQ_PENDING_REG, val);

	/* Calculate base address in prefetch buffer. We reserve 16 descriptors
	 * for each existing TXQ.
	 * TCONTS for PON port must be continuous from 0 to MVPP2_MAX_TCONT
	 * GBE ports assumed to be continious from 0 to MVPP2_MAX_PORTS
	 */
	desc_per_txq = 16;
	desc = (port->id * MVPP2_MAX_TXQ * desc_per_txq) +
	       (txq->log_id * desc_per_txq);

	mvpp2_write(hw, MVPP2_TXQ_PREF_BUF_REG,
		    MVPP2_PREF_BUF_PTR(desc) | MVPP2_PREF_BUF_SIZE_16 |
		    MVPP2_PREF_BUF_THRESH(desc_per_txq/2));

	/* WRR / EJP configuration - indirect access */
	tx_port_num = mvpp2_egress_port(port);
	mvpp2_write(hw, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);

	val = mvpp2_read(hw, MVPP2_TXQ_SCHED_REFILL_REG(txq->log_id));
	val &= ~MVPP2_TXQ_REFILL_PERIOD_ALL_MASK;
	val |= MVPP2_TXQ_REFILL_PERIOD_MASK(1);
	val |= MVPP2_TXQ_REFILL_TOKENS_ALL_MASK;
	mvpp2_write(hw, MVPP2_TXQ_SCHED_REFILL_REG(txq->log_id), val);

	val = MVPP2_TXQ_TOKEN_SIZE_MAX;
	mvpp2_write(hw, MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq->log_id),
		    val);

	for_each_present_cpu(cpu) {
		txq_pcpu = per_cpu_ptr(txq->pcpu, cpu);
		txq_pcpu->size = txq->size;
		txq_pcpu->tx_skb = kmalloc(txq_pcpu->size *
					   sizeof(*txq_pcpu->tx_skb),
					   GFP_KERNEL);
		if (!txq_pcpu->tx_skb)
			goto error;

		txq_pcpu->tx_buffs = kmalloc(txq_pcpu->size *
					     sizeof(dma_addr_t), GFP_KERNEL);
		if (!txq_pcpu->tx_buffs)
			goto error;

		txq_pcpu->count = 0;
		txq_pcpu->reserved_num = 0;
		txq_pcpu->txq_put_index = 0;
		txq_pcpu->txq_get_index = 0;
	}

	return 0;

error:
	for_each_present_cpu(cpu) {
		txq_pcpu = per_cpu_ptr(txq->pcpu, cpu);
		kfree(txq_pcpu->tx_skb);
		kfree(txq_pcpu->tx_buffs);
	}

	dma_free_coherent(port->dev->dev.parent,
			  txq->size * MVPP2_DESC_ALIGNED_SIZE,
			  txq->descs, txq->descs_phys);

	return -ENOMEM;
}

/* Free allocated TXQ resources */
static void mvpp2_txq_deinit(struct mvpp2_port *port,
			     struct mvpp2_tx_queue *txq)
{
	struct mvpp2_txq_pcpu *txq_pcpu;
	struct mvpp2_hw *hw = &(port->priv->hw);
	int cpu;

	for_each_present_cpu(cpu) {
		txq_pcpu = per_cpu_ptr(txq->pcpu, cpu);
		kfree(txq_pcpu->tx_skb);
		kfree(txq_pcpu->tx_buffs);
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
	mvpp2_write(hw, MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(txq->id), 0);

	/* Set Tx descriptors queue starting address and size */
	mvpp2_write(hw, MVPP2_TXQ_NUM_REG, txq->id);
	mvpp2_write(hw, MVPP2_TXQ_DESC_ADDR_LOW_REG, 0);
	mvpp2_write(hw, MVPP2_TXQ_DESC_SIZE_REG, 0);
}

/* Cleanup Tx ports */
static void mvpp2_txq_clean(struct mvpp2_port *port, struct mvpp2_tx_queue *txq)
{
	struct mvpp2_txq_pcpu *txq_pcpu;
	int delay, pending, cpu;
	u32 val;
	struct mvpp2_hw *hw = &(port->priv->hw);

	mvpp2_write(hw, MVPP2_TXQ_NUM_REG, txq->id);
	val = mvpp2_read(hw, MVPP2_TXQ_PREF_BUF_REG);
	val |= MVPP2_TXQ_DRAIN_EN_MASK;
	mvpp2_write(hw, MVPP2_TXQ_PREF_BUF_REG, val);

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
	mvpp2_write(hw, MVPP2_TXQ_PREF_BUF_REG, val);

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
	struct mvpp2_hw *hw = &(port->priv->hw);

	val = mvpp2_read(hw, MVPP2_TX_PORT_FLUSH_REG);

	/* Reset Tx ports and delete Tx queues */
	val |= MVPP2_TX_PORT_FLUSH_MASK(port->id);
	mvpp2_write(hw, MVPP2_TX_PORT_FLUSH_REG, val);

	for (queue = 0; queue < port->num_tx_queues; queue++) {
		txq = port->txqs[queue];
		mvpp2_txq_clean(port, txq);
		mvpp2_txq_deinit(port, txq);
	}

	on_each_cpu(mvpp2_txq_sent_counter_clear, port, 1);

	val &= ~MVPP2_TX_PORT_FLUSH_MASK(port->id);
	mvpp2_write(hw, MVPP2_TX_PORT_FLUSH_REG, val);
}

/* Cleanup all Rx queues */
void mvpp2_cleanup_rxqs(struct mvpp2_port *port)
{
	int queue;

	for (queue = 0; queue < port->num_rx_queues; queue++)
		mvpp2_rxq_deinit(port, port->rxqs[queue]);
}

/* Init all Rx queues for port */
int mvpp2_setup_rxqs(struct mvpp2_port *port)
{
	int queue, err;

	for (queue = 0; queue < port->num_rx_queues; queue++) {
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

	for (queue = 0; queue < port->num_tx_queues; queue++) {
		txq = port->txqs[queue];
		err = mvpp2_txq_init(port, txq);
		if (err)
			goto err_cleanup;
	}
	if (port->priv->pp2xdata->interrupt_tx_done == true) {
		mvpp2_tx_done_time_coal_set(port, port->tx_time_coal);
		on_each_cpu(mvpp2_tx_done_pkts_coal_set, port, 1);
	}
	on_each_cpu(mvpp2_txq_sent_counter_clear, port, 1);
	return 0;

err_cleanup:
	mvpp2_cleanup_txqs(port);
	return err;
}


void mvpp2_cleanup_irqs(struct mvpp2_port *port)
{
	int qvec;
	/*YuvalC TODO: Check, according to free_irq(), it is safe to free a non-requested irq */
	for (qvec=0;qvec<port->num_qvector;qvec++) {
		free_irq(port->q_vector[qvec].irq, &port->q_vector[qvec]);
	}
}


/* The callback for per-q_vector interrupt */
static irqreturn_t mvpp2_isr(int irq, void *dev_id)
{
	struct queue_vector *q_vec = (struct queue_vector *)dev_id;
	mvpp2_qvector_interrupt_disable(q_vec);
	napi_schedule(&q_vec->napi);

	return IRQ_HANDLED;
}

int mvpp2_setup_irqs(struct net_device * dev, struct mvpp2_port *port)
{
	int qvec, err;
	char temp_buf[32];
	for (qvec=0;qvec<port->num_qvector;qvec++) {
		sprintf(temp_buf, "%s.q_vec[%d]", dev->name, qvec);
		err = request_irq(port->q_vector[qvec].irq, mvpp2_isr, 0,
			temp_buf, &port->q_vector[qvec]);
		if (err) {
			netdev_err(port->dev, "cannot request IRQ %d\n",
				port->q_vector[qvec].irq);
			goto err_cleanup;
		}
	}
	return(0);
err_cleanup:
	mvpp2_cleanup_irqs(port);
	return(err);
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

static void mvpp2_timer_set(struct mvpp2_port_pcpu *port_pcpu)
{
	ktime_t interval;

	if (!port_pcpu->timer_scheduled) {
		port_pcpu->timer_scheduled = true;
		interval = ktime_set(0, MVPP2_TXDONE_HRTIMER_PERIOD_NS);
		hrtimer_start(&port_pcpu->tx_done_timer, interval,
			      HRTIMER_MODE_REL_PINNED);
	}
}

static void mvpp2_tx_proc_cb(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_port_pcpu *port_pcpu = this_cpu_ptr(port->pcpu);
	unsigned int tx_todo, cause;

	if (!netif_running(dev))
		return;
	port_pcpu->timer_scheduled = false;

	/* Process all the Tx queues */
	cause = (1 << mvpp2_txq_number) - 1;
	tx_todo = mvpp2_tx_done(port, cause);

	/* Set the timer in case not all the packets were processed */
	if (tx_todo)
		mvpp2_timer_set(port_pcpu);
}

static enum hrtimer_restart mvpp2_hr_timer_cb(struct hrtimer *timer)
{
	struct mvpp2_port_pcpu *port_pcpu = container_of(timer,
							 struct mvpp2_port_pcpu,
							 tx_done_timer);

	tasklet_schedule(&port_pcpu->tx_done_tasklet);

	return HRTIMER_NORESTART;
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
/*TODO : YuvalC, this is just workaround to compile. Need to handle mvpp2_buff_hdr_rx().*/
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
static int mvpp2_rx(struct mvpp2_port *port, struct napi_struct *napi,
			int rx_todo, struct mvpp2_rx_queue *rxq)
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

		napi_gro_receive(napi, skb);

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
		tx_desc->packet_offset = buf_phys_addr & MVPP2_TX_DESC_ALIGN;
		mvpp2x_txdesc_phys_addr_set(port->priv->pp2_version,
			buf_phys_addr & ~MVPP2_TX_DESC_ALIGN, tx_desc);

		if (i == (skb_shinfo(skb)->nr_frags - 1)) {
			/* Last descriptor */
			tx_desc->command = MVPP2_TXD_L_DESC;
			mvpp2_txq_inc_put(port->priv->pp2_version,
				txq_pcpu, skb, tx_desc);
		} else {
			/* Descriptor in the middle: Not First, Not Last */
			tx_desc->command = 0;
			mvpp2_txq_inc_put(port->priv->pp2_version,
				txq_pcpu, NULL, tx_desc);
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



static inline void mvpp2_tx_done_post_proc(struct mvpp2_tx_queue *txq,
	struct mvpp2_txq_pcpu *txq_pcpu, struct mvpp2_port *port, int frags)
{

	/* Finalize TX processing */
	if (txq_pcpu->count >= txq->pkts_coal)
		mvpp2_txq_done(port, txq, txq_pcpu);

	/* Set the timer in case not all frags were processed */
	if (txq_pcpu->count <= frags && txq_pcpu->count > 0) {
		struct mvpp2_port_pcpu *port_pcpu = this_cpu_ptr(port->pcpu);

		mvpp2_timer_set(port_pcpu);
	}
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
		buf_phys_addr & ~MVPP2_TX_DESC_ALIGN, tx_desc);

	tx_cmd = mvpp2_skb_tx_csum(port, skb);
	pr_debug(KERN_EMERG "mvpp2_tx(%d): trace\n", __LINE__);
	if (frags == 1) {
		/* First and Last descriptor */
		tx_cmd |= MVPP2_TXD_F_DESC | MVPP2_TXD_L_DESC;
		tx_desc->command = tx_cmd;
		mvpp2_txq_inc_put(port->priv->pp2_version,
			txq_pcpu, skb, tx_desc);
	} else {
		/* First but not Last */
		tx_cmd |= MVPP2_TXD_F_DESC | MVPP2_TXD_PADDING_DISABLE;
		tx_desc->command = tx_cmd;
		mvpp2_txq_inc_put(port->priv->pp2_version,
			txq_pcpu, NULL, tx_desc);

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
	/* PPV21 TX Post-Processing */
	if (port->priv->pp2xdata->interrupt_tx_done == false)
		mvpp2_tx_done_post_proc(txq, txq_pcpu, port, frags);
	return NETDEV_TX_OK;
}
static inline void mvpp2_cause_misc_handle(struct mvpp2_port *port,
	struct mvpp2_hw *hw, u32 cause_rx_tx)
{
	u32 cause_misc = cause_rx_tx & MVPP2_CAUSE_MISC_SUM_MASK;

	if (cause_misc) {
		mvpp2_cause_error(port->dev, cause_misc);

		/* Clear the cause register */
		mvpp2_write(hw, MVPP2_ISR_MISC_CAUSE_REG, 0);
		mvpp2_write(hw, MVPP2_ISR_RX_TX_CAUSE_REG(port->id),
			    cause_rx_tx & ~MVPP2_CAUSE_MISC_SUM_MASK);
	}
}


static inline int mvpp2_cause_rx_handle(struct mvpp2_port *port,
		struct queue_vector *q_vec, struct napi_struct *napi,
		int budget, u32 cause_rx)
{
	int rx_done = 0, count = 0;
	struct mvpp2_rx_queue *rxq;

	while (cause_rx && budget > 0) {
		rxq = mvpp2_get_rx_queue(port, cause_rx);
		if (!rxq)
			break;

		count = mvpp2_rx(port, &q_vec->napi, budget, rxq);
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
		mvpp2_qvector_interrupt_enable(q_vec);
	}
	q_vec->pending_cause_rx = cause_rx;

	return(rx_done);
}



static int mvpp21_poll(struct napi_struct *napi, int budget)
{
	u32 cause_rx_tx, cause_rx;
	int rx_done = 0;
	struct mvpp2_port *port = netdev_priv(napi->dev);
	struct mvpp2_hw *hw = &port->priv->hw;
 	struct queue_vector *q_vec = container_of(napi, struct queue_vector, napi);

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
	cause_rx_tx = mvpp2_read(hw, MVPP2_ISR_RX_TX_CAUSE_REG(port->id));

	cause_rx_tx &= ~MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK;

	/*Process misc errors */
	mvpp2_cause_misc_handle(port, hw, cause_rx_tx);

	/* Process RX packets */
	cause_rx = cause_rx_tx & MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK;
	cause_rx |= q_vec->pending_cause_rx;

	rx_done = mvpp2_cause_rx_handle(port, q_vec, napi, budget, cause_rx);

	return rx_done;
}


static int mvpp22_poll(struct napi_struct *napi, int budget)
{
	u32 cause_rx_tx, cause_rx, cause_tx;
	int rx_done = 0;
	struct mvpp2_port *port = netdev_priv(napi->dev);
	struct mvpp2_hw *hw = &port->priv->hw;
	struct queue_vector *q_vec = container_of(napi, struct queue_vector, napi);

	/* Rx/Tx cause register
	 * Each CPU has its own Tx cause register
	 */

	/*The read is in the q_vector's sw_thread_id  address_space */
	cause_rx_tx = mvpp22_thread_read(hw, q_vec->sw_thread_id,
			MVPP2_ISR_RX_TX_CAUSE_REG(port->id));


	/*Process misc errors */
	mvpp2_cause_misc_handle(port, hw, cause_rx_tx);

	/* Release TX descriptors */
	cause_tx = (cause_rx_tx & MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK) >>
			MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_OFFSET;
	if (cause_tx)
		mvpp2_tx_done(port, cause_tx);

	/* Process RX packets */
	cause_rx = cause_rx_tx & MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK;
	cause_rx <<= q_vec->first_rx_queue; /*Convert queues from subgroup-relative to port-relative */
	cause_rx |= q_vec->pending_cause_rx;

	rx_done = mvpp2_cause_rx_handle(port, q_vec, napi, budget, cause_rx);

	return rx_done;
}

void mvpp2_port_napi_enable(struct mvpp2_port *port)
{
	int i;

	for (i=0;i<port->num_qvector;i++) {
		napi_enable(&port->q_vector[i].napi);
	}
}

void mvpp2_port_napi_disable(struct mvpp2_port *port)
{
	int i;

	for (i=0;i<port->num_qvector;i++) {
		napi_disable(&port->q_vector[i].napi);
	}
}

static inline void mvpp2_port_irqs_dispose_mapping(struct mvpp2_port *port)
{
	int i;
	for (i=0;i<port->num_irqs;i++)
		irq_dispose_mapping(port->of_irqs[i]);
}


/* Set hw internals when starting port */
void mvpp2_start_dev(struct mvpp2_port *port)
{
	mvpp2_gmac_max_rx_size_set(port);
	mvpp2_txp_max_tx_size_set(port);

	mvpp2_port_napi_enable(port);

	/* Enable interrupts on all CPUs */
	mvpp2_port_interrupts_enable(port);

	mvpp2_port_enable(port);
#ifndef CONFIG_MV_PP2_FPGA
	phy_start(port->phy_dev);
#endif
	netif_tx_start_all_queues(port->dev);
}

/* Set hw internals when stopping port */
void mvpp2_stop_dev(struct mvpp2_port *port)
{
	/* Stop new packets from arriving to RXQs */
	mvpp2_ingress_disable(port);

	mdelay(10);

	/* Disable interrupts on all CPUs */
	mvpp2_port_interrupts_disable(port);

	mvpp2_port_napi_disable(port);

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
	if (MVPP2_RX_PKT_SIZE(mtu) > MVPP2_BM_LONG_PKT_SIZE &&
		jumbo_pool == false) {
		netdev_err(dev, "jumbo packet not supported (%d)\n", mtu);
		return -EINVAL;
	}

	/* 9676 == 9700 - 20 and rounding to 8 */
	if (mtu > 9676) {
		netdev_info(dev, "illegal MTU value %d, round to 9676\n", mtu);
		mtu = 9676;
	}

/*TOO: Below code is incorrect. Check if rounding to 8 is still relevant. */
#if 0

	if (!IS_ALIGNED(MVPP2_RX_PKT_SIZE(mtu), 8)) {
		netdev_info(dev, "illegal MTU value %d, round to %d\n", mtu,
			    ALIGN(MVPP2_RX_PKT_SIZE(mtu), 8));
		mtu = ALIGN(MVPP2_RX_PKT_SIZE(mtu), 8);
	}
#endif

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
	struct mvpp2_hw *hw = &(port->priv->hw);
	int err;

	err = mvpp2_prs_mac_da_accept(hw, port->id, mac_bcast, true);
	if (err) {
		netdev_err(dev, "mvpp2_prs_mac_da_accept BC failed\n");
		return err;
	}
	err = mvpp2_prs_mac_da_accept(hw, port->id,
				      dev->dev_addr, true);
	if (err) {
		netdev_err(dev, "mvpp2_prs_mac_da_accept MC failed\n");
		return err;
	}
	err = mvpp2_prs_tag_mode_set(hw, port->id, MVPP2_TAG_TYPE_MH);
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

#ifndef CONFIG_MV_PP2_FPGA
	err = mvpp2_setup_irqs(dev, port);
	if (err) {
		netdev_err(port->dev, "cannot allocate irq's \n");
		goto err_cleanup_txqs;
	}
#endif
	/* In default link is down */
	netif_carrier_off(port->dev);

#ifndef CONFIG_MV_PP2_FPGA
	err = mvpp2_phy_connect(port);
	if (err < 0)
		goto err_free_irq;
#endif

#ifndef CONFIG_MV_PP2_FPGA

	/* Unmask interrupts on all CPUs */
	on_each_cpu(mvpp2_interrupts_unmask, port, 1);

	/* Unmask shared interrupts */
	mvpp2_shared_thread_interrupts_unmask(port);
#endif

	mvpp2_start_dev(port);

#ifdef CONFIG_MV_PP2_FPGA
	netif_carrier_on(port->dev);
	netif_tx_start_all_queues(port->dev);
#endif
	return 0;

err_free_irq:
	mvpp2_cleanup_irqs(port);
err_cleanup_txqs:
	mvpp2_cleanup_txqs(port);
err_cleanup_rxqs:
	mvpp2_cleanup_rxqs(port);
	return err;
}

static int mvpp2_stop(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_port_pcpu *port_pcpu;
	int cpu;


	mvpp2_stop_dev(port);
	mvpp2_phy_disconnect(port);

	/* Mask interrupts on all CPUs */
	on_each_cpu(mvpp2_interrupts_mask, port, 1);

	/* Mask shared interrupts */
	mvpp2_shared_thread_interrupts_mask(port);

	mvpp2_cleanup_irqs(port);
	if (port->priv->pp2xdata->interrupt_tx_done == false) {
		for_each_present_cpu(cpu) {
			port_pcpu = per_cpu_ptr(port->pcpu, cpu);
			hrtimer_cancel(&port_pcpu->tx_done_timer);
			port_pcpu->timer_scheduled = false;
			tasklet_kill(&port_pcpu->tx_done_tasklet);
		}
	}
	mvpp2_cleanup_rxqs(port);
	mvpp2_cleanup_txqs(port);

	return 0;
}

static void mvpp2_set_rx_mode(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_hw *hw = &(port->priv->hw);
	struct netdev_hw_addr *ha;
	int id = port->id;
	bool allmulti = dev->flags & IFF_ALLMULTI;

	mvpp2_prs_mac_promisc_set(hw, id, dev->flags & IFF_PROMISC);
	mvpp2_prs_mac_multi_set(hw, id, MVPP2_PE_MAC_MC_ALL, allmulti);
	mvpp2_prs_mac_multi_set(hw, id, MVPP2_PE_MAC_MC_IP6, allmulti);

	/* Remove all port->id's mcast enries */
	mvpp2_prs_mcast_del_all(hw, id);

	if (allmulti && !netdev_mc_empty(dev)) {
		netdev_for_each_mc_addr(ha, dev)
			mvpp2_prs_mac_da_accept(hw, id, ha->addr, true);
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

/*of_irq_count is not exported */
int mvpp2_of_irq_count(struct device_node *dev)
{
	struct of_phandle_args irq;
	int nr = 0;

	while (of_irq_parse_one(dev, nr, &irq) == 0)
		nr++;

	return nr;
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

static int  mvpp2_port_txqs_init(struct device *dev, struct mvpp2_port *port)
{
	int queue, cpu;
	struct mvpp2_txq_pcpu *txq_pcpu;

	port->tx_time_coal = MVPP2_TXDONE_COAL_USEC;

	for (queue = 0; queue < port->num_tx_queues; queue++) {
		int queue_phy_id = mvpp2_txq_phys(port->id, queue);
		struct mvpp2_tx_queue *txq;

		txq = devm_kzalloc(dev, sizeof(*txq), GFP_KERNEL);
		if (!txq)
			return -ENOMEM;

		txq->pcpu = alloc_percpu(struct mvpp2_txq_pcpu);
		if (!txq->pcpu) {
			return(-ENOMEM);
		}

		txq->id = queue_phy_id;
		txq->log_id = queue;
		txq->pkts_coal = MVPP2_TXDONE_COAL_PKTS;

		for_each_present_cpu(cpu) {
			txq_pcpu = per_cpu_ptr(txq->pcpu, cpu);
			txq_pcpu->cpu = cpu;
		}

		port->txqs[queue] = txq;
	}

	return(0);
}

static int  mvpp2_port_rxqs_init(struct device *dev, struct mvpp2_port *port)
{
	int queue;

	/* Allocate and initialize Rx queue for this port */
	for (queue = 0; queue < port->num_rx_queues; queue++) {
		struct mvpp2_rx_queue *rxq;

		/* Map physical Rx queue to port's logical Rx queue */
		rxq = devm_kzalloc(dev, sizeof(*rxq), GFP_KERNEL);
		if (!rxq)
			return(-ENOMEM);
		/* Map this Rx queue to a physical queue */
		rxq->id = port->first_rxq + queue;
		rxq->port = port->id;
		rxq->log_id = queue;

		port->rxqs[queue] = rxq;
	}

	return(0);
}

static void mvpp21_port_queue_vectors_init(struct mvpp2_port *port)
{
	struct queue_vector *q_vec = &port->q_vector[0];

	q_vec[0].first_rx_queue = 0;
	q_vec[0].num_rx_queues = port->num_rx_queues;
	q_vec[0].parent = port;
	q_vec[0].pending_cause_rx = 0;
	q_vec[0].qv_type = MVPP2_SHARED;
	q_vec[0].sw_thread_id = 0;
	q_vec[0].sw_thread_mask = port->priv->cpu_map;
	q_vec[0].irq = port->of_irqs[0];
	netif_napi_add(port->dev, &q_vec[0].napi, mvpp21_poll,
		NAPI_POLL_WEIGHT);

	port->num_qvector = 1;
}

static void mvpp22_port_queue_vectors_init(struct mvpp2_port *port)
{
	int cpu;
	struct queue_vector *q_vec = &port->q_vector[0];

	/* Each cpu has zero private rx_queues */
	for (cpu = 0;cpu < num_active_cpus();cpu++) {
		q_vec[cpu].parent = port;
		q_vec[cpu].qv_type = MVPP2_PRIVATE;
		q_vec[cpu].sw_thread_id = first_addr_space+cpu;
		q_vec[cpu].sw_thread_mask = (1<<q_vec[cpu].sw_thread_id);
		q_vec[cpu].pending_cause_rx = 0;
#ifndef CONFIG_MV_PP2_FPGA
		q_vec[cpu].irq = port->of_irqs[first_addr_space+cpu];
#endif
		netif_napi_add(port->dev, &q_vec[cpu].napi, mvpp22_poll,
			NAPI_POLL_WEIGHT);
		if (mvpp2_queue_mode == MVPP2_QDIST_MULTI_MODE) {
			q_vec[cpu].num_rx_queues = mvpp2_num_cos_queues;
			q_vec[cpu].first_rx_queue = cpu*mvpp2_num_cos_queues;
		} else {
			q_vec[cpu].first_rx_queue = 0;
			q_vec[cpu].num_rx_queues = 0;
		}
		port->num_qvector++;
	}
	/*Additional queue_vector for Shared RX */
	if (mvpp2_queue_mode == MVPP2_QDIST_SINGLE_MODE) {
		q_vec[cpu].parent = port;
		q_vec[cpu].qv_type = MVPP2_SHARED;
		q_vec[cpu].sw_thread_id = first_addr_space+cpu;
		q_vec[cpu].sw_thread_mask = (1<<q_vec[cpu].sw_thread_id);
		q_vec[cpu].pending_cause_rx = 0;
#ifndef CONFIG_MV_PP2_FPGA
		q_vec[cpu].irq = port->of_irqs[first_addr_space+cpu];
#endif
		netif_napi_add(port->dev, &q_vec[cpu].napi, mvpp22_poll,
			NAPI_POLL_WEIGHT);
		q_vec[cpu].first_rx_queue = 0;
		q_vec[cpu].num_rx_queues = port->num_rx_queues;

		port->num_qvector++;
	}
}

static void mvpp21x_port_isr_rx_group_cfg(struct mvpp2_port *port)
{
	mvpp21_isr_rx_group_write(&port->priv->hw, port->id, port->num_rx_queues);
}

static void mvpp22_port_isr_rx_group_cfg(struct mvpp2_port *port)
{
	int i;
//	u8 cur_rx_queue;
	struct mvpp2_hw *hw = &port->priv->hw;

	for (i = 0; i < port->num_qvector;i++) {
		if (port->q_vector[i].num_rx_queues != 0) {
			mvpp22_isr_rx_group_write(hw, port->id,
				port->q_vector[i].sw_thread_id,
				port->q_vector[i].first_rx_queue,
				port->q_vector[i].num_rx_queues);
		}
	}
#if 0
	if (mvpp2_queue_mode == MVPP2_QDIST_MULTI_MODE) {
		/* Each cpu has num_cos_queues private rx_queues */
		cur_rx_queue = first_log_rxq_queue;
		for (cpu = 0;cpu < num_active_cpus();cpu++) {
			mvpp22_isr_rx_group_write(hw, port->id,
				first_addr_space+cpu,cur_rx_queue,
				mvpp2_num_cos_queues);
			cur_rx_queue += mvpp2_num_cos_queues;
		}
	} else {
		/* Each cpu has zero private rx_queues */
		for (cpu = 0;cpu < num_active_cpus();cpu++) {
			mvpp22_isr_rx_group_write(hw, port->id,
				first_addr_space+cpu, 0, 0);
		}
		/* Additional shared group */
		mvpp22_isr_rx_group_write(hw, port->id,
			first_addr_space+cpu, first_log_rxq_queue,
			port->num_rx_queues);

	}
#endif
}




/* Initialize port HW */
static int mvpp2_port_init(struct mvpp2_port *port)
{
	struct device *dev = port->dev->dev.parent;
	struct mvpp2 *priv = port->priv;
	int queue, err;


	/* Disable port */
	mvpp2_egress_disable(port);
	mvpp2_port_disable(port);

	/* Allocate queues */
	port->txqs = devm_kcalloc(dev, port->num_tx_queues, sizeof(*port->txqs),
				  GFP_KERNEL);
	if (!port->txqs)
		return -ENOMEM;

	port->rxqs = devm_kcalloc(dev, port->num_rx_queues, sizeof(*port->rxqs),
				  GFP_KERNEL);
	if (!port->rxqs) {
		return -ENOMEM;
	}

	/* Associate physical Tx queues to port and initialize.  */
	err = mvpp2_port_txqs_init(dev, port);
	if (err)
		goto err_free_percpu;

	/* Associate physical Rx queues to port and initialize.  */
	err = mvpp2_port_rxqs_init(dev, port);
	if (err)
		goto err_free_percpu;

	/* Configure queue_vectors */
	priv->pp2xdata->mvpp2x_port_queue_vectors_init(port);

	/* Configure Rx queue group interrupt for this port */
	priv->pp2xdata->mvpp2x_port_isr_rx_group_cfg(port);


	/* Create Rx descriptor rings */
	for (queue = 0; queue < port->num_rx_queues; queue++) {
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
	for (queue = 0; queue < port->num_tx_queues; queue++) {
		if (!port->txqs[queue])
			continue;
		free_percpu(port->txqs[queue]->pcpu);
	}
	return err;
}

#ifndef CONFIG_MV_PP2_FPGA

/* Ports initialization */
static int mvpp2_port_probe(struct platform_device *pdev,
			    struct device_node *port_node,
			    struct mvpp2 *priv)
{
	struct device_node *phy_node;
	struct mvpp2_port *port;
	struct mvpp2_port_pcpu *port_pcpu;
	struct net_device *dev;
	struct resource *res;
	const char *dt_mac_addr;
	const char *mac_from;
	char hw_mac_addr[ETH_ALEN];
	u32 id;
	unsigned int * port_irqs;
	int features, phy_mode, err=0, i, port_num_irq, cpu;
	int priv_common_regs_num = 2;

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

	port_num_irq = mvpp2_of_irq_count(port_node);
	if (port_num_irq != priv->pp2xdata->num_port_irq) {
		dev_err(&pdev->dev, "port(%d)-number of irq's doesn't match hw\n", id);
		goto err_free_netdev;
	}
	port_irqs = devm_kcalloc(&pdev->dev, port_num_irq, sizeof(u32), GFP_KERNEL);
	port->of_irqs = port_irqs;
	port->num_irqs = 0;

	for (i=0;i<port_num_irq;i++) {
		port_irqs[i] = irq_of_parse_and_map(port_node, i);
		if (port_irqs[i] == 0) {
			dev_err(&pdev->dev, "Fail to parse port(%d), irq(%d)\n", id, i);
			err = -EINVAL;
			goto err_free_irq;
		}
		port->num_irqs++;
	}

	if (of_property_read_bool(port_node, "marvell,loopback"))
		port->flags |= MVPP2_F_LOOPBACK;

	port->priv = priv;
	port->id = id;
	port->num_tx_queues = mvpp2_txq_number;
	port->num_rx_queues = mvpp2_rxq_number;

	/*YuvalC: Port first_rxq relative to port->id, not dependent on board topology, i.e. not dynamically allocated */
	port->first_rxq = (port->id)*(priv->pp2xdata->pp2x_max_port_rxqs) +
		first_log_rxq_queue;
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

	port->tx_ring_size = tx_queue_size;
	port->rx_ring_size = rx_queue_size;
	port->dev = dev;
	SET_NETDEV_DEV(dev, &pdev->dev);

	err = mvpp2_port_init(port);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to init port %d\n", id);
		goto err_free_stats;
	}
	mvpp2_port_power_up(port);

	port->pcpu = alloc_percpu(struct mvpp2_port_pcpu);
	if (!port->pcpu) {
		err = -ENOMEM;
		goto err_free_txq_pcpu;
	}

	if (port->priv->pp2xdata->interrupt_tx_done == false) {
		for_each_present_cpu(cpu) {
			port_pcpu = per_cpu_ptr(port->pcpu, cpu);

			hrtimer_init(&port_pcpu->tx_done_timer, CLOCK_MONOTONIC,
				     HRTIMER_MODE_REL_PINNED);
			port_pcpu->tx_done_timer.function = mvpp2_hr_timer_cb;
			port_pcpu->timer_scheduled = false;

			tasklet_init(&port_pcpu->tx_done_tasklet, mvpp2_tx_proc_cb,
				     (unsigned long)dev);
		}
	}
	features = NETIF_F_SG | NETIF_F_IP_CSUM;
	dev->features = features | NETIF_F_RXCSUM;
	dev->hw_features |= features | NETIF_F_RXCSUM | NETIF_F_GRO;
	dev->vlan_features |= features;

	err = register_netdev(dev);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register netdev\n");
		goto err_free_port_pcpu;
	}
	netdev_info(dev, "Using %s mac address %pM\n", mac_from, dev->dev_addr);


	priv->port_list[id] = port;
	return 0;
	dev_err(&pdev->dev, "%s failed for port_id(%d)\n", __func__, id);

err_free_port_pcpu:
	free_percpu(port->pcpu);
err_free_txq_pcpu:
	for (i = 0; i < mvpp2_txq_number; i++)
		free_percpu(port->txqs[i]->pcpu);
err_free_stats:
	free_percpu(port->stats);
err_free_irq:
#ifndef CONFIG_MV_PP2_FPGA
	mvpp2_port_irqs_dispose_mapping(port);
#endif
err_free_netdev:
	free_netdev(dev);
	return err;
}

#else

static int mvpp2_port_probe_fpga(struct platform_device *pdev,
				int port_i,
			    struct mvpp2 *priv)
{
	struct device_node *phy_node;
	struct mvpp2_port *port;
	struct mvpp2_port_pcpu *port_pcpu;
	struct net_device *dev;
	struct resource *res;
	const char *dt_mac_addr;
	const char *mac_from;
	char hw_mac_addr[ETH_ALEN];
	u32 id;
	unsigned int * port_irqs;
	int features, phy_mode, err=0, i, port_num_irq, cpu;
	int priv_common_regs_num = 2;

	dev = alloc_etherdev_mqs(sizeof(struct mvpp2_port), mvpp2_txq_number,
				 mvpp2_rxq_number);
	if (!dev)
		return -ENOMEM;
	MVPP2_PRINT_LINE();

	dev->tx_queue_len = MVPP2_MAX_TXD;
	dev->watchdog_timeo = 5 * HZ;
	dev->netdev_ops = &mvpp2_netdev_ops;
	mvpp2_set_ethtool_ops(dev);

	port = netdev_priv(dev);

	port->priv = priv;
	port->id = port_i;
	port->num_tx_queues = mvpp2_txq_number;
	port->num_rx_queues = mvpp2_rxq_number;

	/*YuvalC: Port first_rxq relative to port->id, not dependent on board topology, i.e. not dynamically allocated */
	port->first_rxq = (port->id)*(priv->pp2xdata->pp2x_max_port_rxqs) +
		first_log_rxq_queue;
	port->phy_node = phy_node;
	port->phy_interface = phy_mode;
	port->base = (void *) ((mv_pp2_vfpga_address + FPGA_PORT_0_OFFSET) + ((port->id) * 0x1000));
	DBG_MSG("mvpp2(%d): mvpp2_port_probe: port_id-%d mv_pp2_vfpga_address=0x%x port->base=0x%p\n",
		   __LINE__, port->id, mv_pp2_vfpga_address, port->base);
	MVPP2_PRINT_LINE();

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
	MVPP2_PRINT_LINE();


	mac_from = "hardware";
	hw_mac_addr[0] = 0x02;
	hw_mac_addr[1] = 0x68;
	hw_mac_addr[2] = 0xb3;
	hw_mac_addr[3] = 0x29;
	hw_mac_addr[4] = 0xda;
	hw_mac_addr[5] = 0x98 | port_i;

	ether_addr_copy(dev->dev_addr, hw_mac_addr);

	port->tx_ring_size = tx_queue_size;
	port->rx_ring_size = rx_queue_size;
	port->dev = dev;
	SET_NETDEV_DEV(dev, &pdev->dev);
	err = mvpp2_port_init(port);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to init port %d\n", port->id);
		goto err_free_stats;
	}
	mvpp2_port_power_up(port);
	MVPP2_PRINT_LINE();
	mvpp2_ingress_enable(port); //Enable here, because there is no link event

	port->pcpu = alloc_percpu(struct mvpp2_port_pcpu);
	if (!port->pcpu) {
		err = -ENOMEM;
		goto err_free_txq_pcpu;
	}
	MVPP2_PRINT_LINE();

	if (port->priv->pp2xdata->interrupt_tx_done == false) {
		for_each_present_cpu(cpu) {
			port_pcpu = per_cpu_ptr(port->pcpu, cpu);
			MVPP2_PRINT_LINE();

			hrtimer_init(&port_pcpu->tx_done_timer, CLOCK_MONOTONIC,
				     HRTIMER_MODE_REL_PINNED);
			port_pcpu->tx_done_timer.function = mvpp2_hr_timer_cb;
			port_pcpu->timer_scheduled = false;
			MVPP2_PRINT_LINE();

			tasklet_init(&port_pcpu->tx_done_tasklet, mvpp2_tx_proc_cb,
				     (unsigned long)dev);

			MVPP2_PRINT_LINE();
		}
	}

	MVPP2_PRINT_LINE();
	features = NETIF_F_SG | NETIF_F_IP_CSUM;
	dev->features = features | NETIF_F_RXCSUM;
	dev->hw_features |= features | NETIF_F_RXCSUM | NETIF_F_GRO;
	dev->vlan_features |= features;

	err = register_netdev(dev);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register netdev\n");
		goto err_free_port_pcpu;
	}

	MVPP2_PRINT_LINE();
	netdev_info(dev, "Using %s mac address %pM\n", mac_from, dev->dev_addr);
	priv->port_list[port_i] = port;
	return 0;
	dev_err(&pdev->dev, "%s failed for port_id(%d)\n", __func__, id);

err_free_port_pcpu:
	free_percpu(port->pcpu);
err_free_txq_pcpu:
	for (i = 0; i < mvpp2_txq_number; i++)
		free_percpu(port->txqs[i]->pcpu);
err_free_stats:
	free_percpu(port->stats);
err_free_irq:
#ifndef CONFIG_MV_PP2_FPGA
	mvpp2_port_irqs_dispose_mapping(port);
#endif
err_free_netdev:
	free_netdev(dev);
	return err;
}

#endif

/* Ports removal routine */
static void mvpp2_port_remove(struct mvpp2_port *port)
{
	int i;

	unregister_netdev(port->dev);
	free_percpu(port->pcpu);
	free_percpu(port->stats);
	for (i = 0; i < port->num_tx_queues; i++)
		free_percpu(port->txqs[i]->pcpu);
#ifndef CONFIG_MV_PP2_FPGA
	mvpp2_port_irqs_dispose_mapping(port);
#endif
	free_netdev(port->dev);
}


/* Initialize decoding windows */
static void mvpp2_conf_mbus_windows(const struct mbus_dram_target_info *dram,
				    struct mvpp2_hw *hw)
{
	u32 win_enable;
	int i;

	for (i = 0; i < 6; i++) {
		mvpp2_write(hw, MVPP2_WIN_BASE(i), 0);
		mvpp2_write(hw, MVPP2_WIN_SIZE(i), 0);

		if (i < 4)
			mvpp2_write(hw, MVPP2_WIN_REMAP(i), 0);
	}

	win_enable = 0;

	for (i = 0; i < dram->num_cs; i++) {
		const struct mbus_dram_window *cs = dram->cs + i;

		mvpp2_write(hw, MVPP2_WIN_BASE(i),
			    (cs->base & 0xffff0000) | (cs->mbus_attr << 8) |
			    dram->mbus_dram_target_id);

		mvpp2_write(hw, MVPP2_WIN_SIZE(i),
			    (cs->size - 1) & 0xffff0000);

		win_enable |= (1 << i);
	}

	mvpp2_write(hw, MVPP2_BASE_ADDR_ENABLE, win_enable);
}

/* Initialize Rx FIFO's */
static void mvpp2_rx_fifo_init(struct mvpp2_hw *hw)
{
	int port;

	for (port = 0; port < MVPP2_MAX_PORTS; port++) {
		mvpp2_write(hw, MVPP2_RX_DATA_FIFO_SIZE_REG(port),
			    MVPP2_RX_FIFO_PORT_DATA_SIZE);
		mvpp2_write(hw, MVPP2_RX_ATTR_FIFO_SIZE_REG(port),
			    MVPP2_RX_FIFO_PORT_ATTR_SIZE);
	}

	mvpp2_write(hw, MVPP2_RX_MIN_PKT_SIZE_REG,
		    MVPP2_RX_FIFO_PORT_MIN_PKT);
	mvpp2_write(hw, MVPP2_RX_FIFO_INIT_REG, 0x1);
}

/* Initialize network controller common part HW */
static int mvpp2_init(struct platform_device *pdev, struct mvpp2 *priv)
{
	int err, i;
	int last_log_rx_queue;
	u32 val;
	const struct mbus_dram_target_info *dram_target_info;
	u8 pp2_ver = priv->pp2xdata->pp2x_ver;
	struct mvpp2_hw *hw = &priv->hw;

	/* Checks for hardware constraints */
	last_log_rx_queue = first_log_rxq_queue + mvpp2_rxq_number;
	if (last_log_rx_queue > priv->pp2xdata->pp2x_max_port_rxqs) {
		dev_err(&pdev->dev, "too high num_cos_queue parameter\n");
		return -EINVAL;
	}
	/*TODO: YuvalC, replace this with a per-pp2x validation function. */
	if ((pp2_ver == PPV21) && (mvpp2_rxq_number % 4)) {
		dev_err(&pdev->dev, "invalid num_cos_queue parameter\n");
		return -EINVAL;
	}

	if (mvpp2_txq_number > MVPP2_MAX_TXQ) {
		dev_err(&pdev->dev, "invalid num_cos_queue parameter\n");
		return -EINVAL;
	}

	/* MBUS windows configuration */
	dram_target_info = mv_mbus_dram_info();
	if (dram_target_info)
		mvpp2_conf_mbus_windows(dram_target_info, hw);

	/* Disable HW PHY polling */
#ifndef CONFIG_MV_PP2_FPGA
	val = readl(hw->lms_base + MVPP2_PHY_AN_CFG0_REG);
	val |= MVPP2_PHY_AN_STOP_SMI0_MASK;
	writel(val, hw->lms_base + MVPP2_PHY_AN_CFG0_REG);
#endif

	/* Allocate and initialize aggregated TXQs */
	priv->aggr_txqs = devm_kcalloc(&pdev->dev, num_active_cpus(),
				       sizeof(struct mvpp2_aggr_tx_queue),
				       GFP_KERNEL);
	if (!priv->aggr_txqs)
		return -ENOMEM;
	/*TODO: FIXME: See num_present_cpus() above , below will OOPS for cpu_present_mask = cpu0|cpu3 */
	for_each_present_cpu(i) {
		priv->aggr_txqs[i].id = i;
		priv->aggr_txqs[i].size = MVPP2_AGGR_TXQ_SIZE;
		err = mvpp2_aggr_txq_init(pdev, &priv->aggr_txqs[i],
					  MVPP2_AGGR_TXQ_SIZE, i, priv);
		if (err < 0)
			return err;
	}

	/* Rx Fifo Init */
	mvpp2_rx_fifo_init(hw);

#ifndef CONFIG_MV_PP2_FPGA
	writel(MVPP2_EXT_GLOBAL_CTRL_DEFAULT,
	       hw->lms_base + MVPP2_MNG_EXTENDED_GLOBAL_CTRL_REG);
#endif

	/* Allow cache snoop when transmiting packets */
	mvpp2_write(hw, MVPP21_TX_SNOOP_REG, 0x1);

	/* Buffer Manager initialization */
	err = mvpp2_bm_init(pdev, priv);
	if (err < 0)
		return err;

	/* Parser default initialization */
	err = mvpp2_prs_default_init(pdev, hw);
	if (err < 0)
		return err;

	/* Classifier default initialization */
	mvpp2_cls_init(hw);


	/*NEW : Disable all rx_queues */
	/*TBD - this is TEMP_HACK */
	if (pp2_ver == PPV22) {
		for (i = 0; i < 128; i++) {
			val = mvpp2_read(hw, MVPP2_RXQ_CONFIG_REG(i));
			val |= MVPP2_RXQ_DISABLE_MASK;
			mvpp2_write(hw, MVPP2_RXQ_CONFIG_REG(i), val);
		}
	}

	return 0;
}


static const struct mvpp2x_platform_data pp21_pdata = {
	.pp2x_ver = PPV21,
	.pp2x_max_port_rxqs = 8,
	.mvpp2x_rxq_short_pool_set = mvpp21_rxq_short_pool_set,
	.mvpp2x_rxq_long_pool_set = mvpp21_rxq_long_pool_set,
	.multi_addr_space = false,
	.interrupt_tx_done = false,
	.multi_hw_instance = false,
	.mvpp2x_port_queue_vectors_init = mvpp21_port_queue_vectors_init,
	.mvpp2x_port_isr_rx_group_cfg = mvpp21x_port_isr_rx_group_cfg,
	.num_port_irq = 1,
};

static const struct mvpp2x_platform_data pp22_pdata = {
	.pp2x_ver = PPV22,
	.pp2x_max_port_rxqs = 32,
	.mvpp2x_rxq_short_pool_set = mvpp22_rxq_short_pool_set,
	.mvpp2x_rxq_long_pool_set = mvpp22_rxq_long_pool_set,
	.multi_addr_space = true,
	.interrupt_tx_done = false,
	.multi_hw_instance = true,
	.mvpp2x_port_queue_vectors_init = mvpp22_port_queue_vectors_init,
	.mvpp2x_port_isr_rx_group_cfg = mvpp22_port_isr_rx_group_cfg,
	.num_port_irq = 9,
};



static const struct of_device_id mvpp2_match_tbl[] = {
        {
                .compatible = "marvell,armada-375-pp2",
                .data = &pp21_pdata,
        },
        {
                .compatible = "marvell,pp22",
                .data = &pp22_pdata,
        },
	{ }
};

static void mvpp2_init_config(struct mvpp2_param_config *pp2_cfg, u32 cell_index)
{
	pp2_cfg->cell_index = cell_index;
	pp2_cfg->first_bm_pool = first_bm_pool;
	pp2_cfg->first_sw_thread = first_addr_space;
	pp2_cfg->jumbo_pool = jumbo_pool;
	pp2_cfg->queue_mode = mvpp2_queue_mode;

	pp2_cfg->cos_cfg.cos_classifier = cos_classifer;
	pp2_cfg->cos_cfg.default_cos = default_cos;
	pp2_cfg->cos_cfg.num_cos_queues = mvpp2_num_cos_queues;
	pp2_cfg->cos_cfg.pri_map = pri_map;

	pp2_cfg->rss_cfg.dflt_cpu = default_cpu;
	pp2_cfg->rss_cfg.queue_mode = mvpp2_queue_mode; /*TODO : This param is redundant, reduce from rss */
	pp2_cfg->rss_cfg.reserved = 0;
	pp2_cfg->rss_cfg.rss_mode = rss_mode;
}

void mvpp2_pp2_basic_print(struct platform_device *pdev, struct mvpp2 *priv)
{

	printk("%s\n",__func__);


	DBG_MSG("num_present_cpus(%d) num_active_cpus(%d) num_online_cpus(%d)\n",
		num_present_cpus(), num_active_cpus(),num_online_cpus());
	DBG_MSG("cpu_map(%x)\n", priv->cpu_map);

	DBG_MSG("pdev->name(%s) pdev->id(%d)\n", pdev->name, pdev->id);
	DBG_MSG("dev.init_name(%s) dev.id(%d)\n", pdev->dev.init_name, pdev->dev.id);
	DBG_MSG("dev.kobj.name(%s)\n", pdev->dev.kobj.name);
	DBG_MSG("dev->bus.name(%s) pdev.dev->bus.dev_name(%s)\n",
		pdev->dev.bus->name, pdev->dev.bus->dev_name);

	DBG_MSG("pp2_ver(%d)\n", priv->pp2_version);
	DBG_MSG("queue_mode(%d)\n", priv->pp2_cfg.queue_mode);
	DBG_MSG("first_bm_pool(%d) jumbo_pool(%d)\n", priv->pp2_cfg.first_bm_pool, priv->pp2_cfg.jumbo_pool);
	DBG_MSG("cell_index(%d) num_ports(%d)\n", priv->pp2_cfg.cell_index, priv->num_ports);

	DBG_MSG("hw->base(%p)\n", priv->hw.base);
}
EXPORT_SYMBOL(mvpp2_pp2_basic_print);

void mvpp2_pp2_port_print(struct mvpp2_port *port)
{
	int i;

	DBG_MSG("%s port_id(%d)\n",__func__, port->id);
	DBG_MSG("\t ifname(%s)\n", port->dev->name);
	DBG_MSG("\t first_rxq(%d)\n", port->first_rxq);
	DBG_MSG("\t num_irqs(%d)\n", port->num_irqs);
	DBG_MSG("\t pkt_size(%d)\n", port->pkt_size);
	DBG_MSG("\t flags(%lx)\n", port->flags);
	DBG_MSG("\t tx_ring_size(%d)\n", port->tx_ring_size);
	DBG_MSG("\t rx_ring_size(%d)\n", port->rx_ring_size);
	DBG_MSG("\t time_coal(%d)\n", port->tx_time_coal);
	DBG_MSG("\t pool_long(%p)\n", port->pool_long);
	DBG_MSG("\t pool_short(%p)\n", port->pool_short);
	DBG_MSG("\t first_rxq(%d)\n", port->first_rxq);
	DBG_MSG("\t num_rx_queues(%d)\n", port->num_rx_queues);
	DBG_MSG("\t num_tx_queues(%d)\n", port->num_tx_queues);
	DBG_MSG("\t num_qvector(%d)\n", port->num_qvector);

	for (i=0;i<port->num_qvector;i++) {
		DBG_MSG("\t qvector_index(%d)\n", i);
		DBG_MSG("\t\t irq(%d)\n", port->q_vector[i].irq);
		DBG_MSG("\t\t qv_type(%d)\n", port->q_vector[i].qv_type);
		DBG_MSG("\t\t sw_thread_id	(%d)\n", port->q_vector[i].sw_thread_id);
		DBG_MSG("\t\t sw_thread_mask(%d)\n", port->q_vector[i].sw_thread_mask);
		DBG_MSG("\t\t first_rx_queue(%d)\n", port->q_vector[i].first_rx_queue);
		DBG_MSG("\t\t num_rx_queues(%d)\n", port->q_vector[i].num_rx_queues);
		DBG_MSG("\t\t pending_cause_rx(%d)\n", port->q_vector[i].pending_cause_rx);

	}

}
EXPORT_SYMBOL(mvpp2_pp2_port_print);

static void mvpp2_pp2_ports_print(struct mvpp2 *priv)
{
	int i;
	struct mvpp2_port *port;

	for (i = 0; i < priv->num_ports; i++) {
		if (priv->port_list[i] == NULL) {
			pr_emerg("\t port_list[%d]= NULL!\n", i);
			continue;
		}
		port = priv->port_list[i];
		mvpp2_pp2_port_print(port);
	}
}
EXPORT_SYMBOL(mvpp2_pp2_ports_print);


static int mvpp2_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	struct device_node *port_node;
	struct mvpp2 *priv;
	struct mvpp2_hw *hw;
#ifndef CONFIG_MV_PP2_FPGA
	struct resource *res;
#endif
	const struct of_device_id *match;
	int port_count, cpu;
	int i, err;
	u16 cpu_map;
	u32 cell_index = 0;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct mvpp2), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	hw = &priv->hw;

#ifndef CONFIG_MV_PP2_FPGA
	match = of_match_node(mvpp2_match_tbl, dn);
#else
	match = &mvpp2_match_tbl[1];
#endif

	if (!match)
		return -ENODEV;

#ifdef TEST_FPGA_MODE // Single CPU, FPGA, Test
	mvpp2_queue_mode = MVPP2_QDIST_MULTI_MODE;
#endif


MVPP2_PRINT_LINE();
	priv->pp2xdata = (const struct mvpp2x_platform_data *) match->data;
	priv->pp2_version = priv->pp2xdata->pp2x_ver;
#ifndef CONFIG_MV_PP2_FPGA
	if (priv->pp2xdata->multi_hw_instance) {
		if (of_property_read_u32(pdev->dev.of_node, "cell-index",
					  &cell_index)) {
			dev_err(&pdev->dev, "missing cell_index value\n");
			return -ENODEV;
		}
	}
#endif
MVPP2_PRINT_LINE();

#ifndef CONFIG_MV_PP2_FPGA
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hw->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hw->base))
		return PTR_ERR(hw->base);



	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	hw->lms_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hw->lms_base))
		return PTR_ERR(hw->lms_base);
#else
	hw->base = (void *)mv_pp2_vfpga_address;
	pr_debug("mvpp2(%d): mvpp2_probe:mv_pp2_vfpga_address=0x%x\n", __LINE__, mv_pp2_vfpga_address);
#endif
MVPP2_PRINT_LINE();

#ifndef CONFIG_MV_PP2_FPGA
	hw->pp_clk = devm_clk_get(&pdev->dev, "pp_clk");
	if (IS_ERR(hw->pp_clk))
		return PTR_ERR(hw->pp_clk);
	err = clk_prepare_enable(hw->pp_clk);
	if (err < 0)
		return err;

	hw->gop_clk = devm_clk_get(&pdev->dev, "gop_clk");
	if (IS_ERR(hw->gop_clk)) {
		err = PTR_ERR(hw->gop_clk);
		goto err_pp_clk;
	}
	err = clk_prepare_enable(hw->gop_clk);
	if (err < 0)
		goto err_pp_clk;

	/* Get system's tclk rate */
	hw->tclk = clk_get_rate(hw->pp_clk);
#else
	hw->tclk = 25000000;
#endif
	/* Save cpu_present_mask + populate the per_cpu address space */
	cpu_map = 0;
	i = 0;
	for_each_present_cpu(cpu) {
		cpu_map |= (1<<cpu);
		hw->cpu_base[cpu] = hw->base;
		if (priv->pp2xdata->multi_addr_space) {
			hw->cpu_base[cpu] += (first_addr_space + i)*MVPP2_ADDR_SPACE_SIZE;
			i++;
		}
	}
	priv->cpu_map = cpu_map;
	MVPP2_PRINT_LINE();


	/* Initialize network controller */
	err = mvpp2_init(pdev, priv);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to initialize controller\n");
		goto err_gop_clk;
	}
	MVPP2_PRINT_LINE();

#ifndef CONFIG_MV_PP2_FPGA
	port_count = of_get_available_child_count(dn);
	if (port_count == 0) {
		dev_err(&pdev->dev, "no ports enabled\n");
		err = -ENODEV;
		goto err_gop_clk;
	}
#else
	port_count = 2;
#endif
	MVPP2_PRINT_LINE();

	priv->port_list = devm_kcalloc(&pdev->dev, port_count,
				      sizeof(struct mvpp2_port *),
				      GFP_KERNEL);
	if (!priv->port_list) {
		err = -ENOMEM;
		goto err_gop_clk;
	}
	priv->num_ports = port_count;

	/*Init PP2 Configuration */
	mvpp2_init_config(&priv->pp2_cfg, cell_index);
	mvpp2_pp2_basic_print(pdev, priv);

	/* Initialize ports */
#ifndef CONFIG_MV_PP2_FPGA
	for_each_available_child_of_node(dn, port_node) {
		err = mvpp2_port_probe(pdev, port_node, priv);
		if (err < 0)
			goto err_gop_clk;
	}
#else
	for(i = 0 ; i < port_count ; i++) {
		err = mvpp2_port_probe_fpga(pdev, i, priv);
		if (err < 0)
			goto err_gop_clk;
	}
#endif
	mvpp2_pp2_ports_print(priv);
#ifdef CONFIG_MV_PP2_FPGA
	init_timer(&cpu_poll_timer);
	cpu_poll_timer.function = mv_pp22_cpu_timer_callback;
	cpu_poll_timer.expires  = jiffies + msecs_to_jiffies(MV_PP2_FPGA_PERODIC_TIME);
	cpu_poll_timer.data     = pdev;
	add_timer(&cpu_poll_timer);
#endif

	platform_set_drvdata(pdev, priv);
	return 0;

err_gop_clk:
	clk_disable_unprepare(hw->gop_clk);
err_pp_clk:
	clk_disable_unprepare(hw->pp_clk);
	return err;
}

static int mvpp2_remove(struct platform_device *pdev)
{
	struct mvpp2 *priv = platform_get_drvdata(pdev);
	struct mvpp2_hw *hw = &priv->hw;
	struct device_node *dn = pdev->dev.of_node;
	struct device_node *port_node;
	int i;

	for(i = 0; i < priv->num_ports; i++) {
		if (priv->port_list[i])
		mvpp2_port_remove(priv->port_list[i]);
	}

	for (i = 0; i < priv->num_pools; i++) {
		struct mvpp2_bm_pool *bm_pool = &priv->bm_pools[i];

		mvpp2_bm_pool_destroy(pdev, hw, bm_pool);
	}

	for_each_present_cpu(i) {
		struct mvpp2_aggr_tx_queue *aggr_txq = &priv->aggr_txqs[i];

		dma_free_coherent(&pdev->dev,
				  MVPP2_AGGR_TXQ_SIZE * MVPP2_DESC_ALIGNED_SIZE,
				  aggr_txq->descs,
				  aggr_txq->descs_phys);
	}

	clk_disable_unprepare(hw->pp_clk);
	clk_disable_unprepare(hw->gop_clk);

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

static int mvpp2_rxq_number_get(void) {

	int rx_queue_num;

	if (mvpp2_queue_mode == MVPP2_QDIST_SINGLE_MODE)
		rx_queue_num = mvpp2_num_cos_queues;
	else
		rx_queue_num = mvpp2_num_cos_queues * num_active_cpus();

	return (rx_queue_num);
}


#ifdef CONFIG_MV_PP2_FPGA

static int mv_pp2_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	/* code below relevant for FPGA only */
	if (pci_enable_device(pdev)) {
		pr_err("mvpp2: can not enable PCI device\n");
		return -1;
	}

	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM)) {
		pr_err("mvpp2: can not find proper PCI device base address\n");
		return -ENODEV;
	}

	if (pci_request_regions(pdev, "mv_pp2_pci")) {
		pr_err("mvpp2: can not obtain PCI resources\n");
		return -ENODEV;
	}

	if (pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) {
		pr_err("mvpp2: no usable DMA configuration\n");
		return -ENODEV;
	}

	mv_pp2_vfpga_address = (u32)pci_iomap(pdev, 0, 16 * 1024 * 1024);

	if (!mv_pp2_vfpga_address)
		pr_err("mvpp2: can not map device registers\n");

	pr_debug("mvpp2: fpga base: VIRT=0x%0x, size=%d KBytes\n", mv_pp2_vfpga_address, 16 * 1024);
	return 0;
}


static void mv_pp2_pci_remove(struct pci_dev *pdev)
{
	pr_info("mvpp2: PCI device removed\n");
}

static const struct pci_device_id fpga_id_table[] = {
	{ 0x1234, 0x1234, PCI_ANY_ID, PCI_ANY_ID, 2, 0, 0}, {0}
};

MODULE_DEVICE_TABLE(pci, fpga_id_table);

static struct resource mvpp2_resources[] = {
	{
		.name = MVPP2_DRIVER_NAME,
/*		.start = __NSS_MGMT_PHYS_BASE + 0x200000,
		.end   = __NSS_MGMT_PHYS_BASE + 0x200000 + 0x200000 - 1,*/
		.flags = IORESOURCE_MEM,
	},
};

static struct pci_driver mv_pp2_pci_driver = {
	.name	= "mv_pp2_pci",
	.id_table = fpga_id_table,
	.probe		= mv_pp2_pci_probe,
	.remove		= mv_pp2_pci_remove,
};

static struct platform_device mvpp2_device = {
	.name           = MVPP2_DRIVER_NAME,
	.id             = 0,
	.num_resources	= ARRAY_SIZE(mvpp2_resources),
	.resource		= mvpp2_resources,
	.dev            = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = 0,
		.init_name = "f10f0000.ethernet",
	},
};

#endif

static int __init mpp2_module_init(void)
{
	int ret;

#ifdef CONFIG_MV_PP2_FPGA

	if(platform_device_register(&mvpp2_device)) {
		pr_debug("mvpp2(%d): platform_device_register failed\n", __LINE__);
		return -1;
	}

	ret = pci_register_driver(&mv_pp2_pci_driver);
	if (ret < 0) {
		pr_err("mvpp2: PCI card not found, driver not installed. rc=%d\n", ret);
		return ret;
	}
	mvpp2_num_cos_queues=4;

#endif
	mvpp2_rxq_number = mvpp2_rxq_number_get();
	mvpp2_txq_number = mvpp2_num_cos_queues;

	/* Compiler does not allow below Init in structure definition */
	mvpp2_pools[MVPP2_BM_SWF_SHORT_POOL].pkt_size = MVPP2_BM_SHORT_PKT_SIZE;
	mvpp2_pools[MVPP2_BM_SWF_LONG_POOL].pkt_size = MVPP2_BM_LONG_PKT_SIZE;
	mvpp2_pools[MVPP2_BM_SWF_JUMBO_POOL].pkt_size = MVPP2_BM_JUMBO_PKT_SIZE;

	ret = platform_driver_register(&mvpp2_driver);

	return ret;
}

static void __exit mpp2_module_exit(void)
{
#ifdef CONFIG_MV_PP2_FPGA
	pci_unregister_driver(&mv_pp2_pci_driver);
#endif
	platform_driver_unregister(&mvpp2_driver);

}

#ifdef CONFIG_MV_PP2_FPGA

static void mv_pp22_cpu_timer_callback(unsigned long data)
{
	struct platform_device *pdev = (struct platform_device *)data;
	struct mvpp2 *priv = platform_get_drvdata(pdev);
	struct device_node *dn = pdev->dev.of_node;
	struct device_node *port_node;
	int i = 0, j, err;
	struct mvpp2_port *port;

	for(i = 0 ; i < priv->num_ports; i++) {
		port = priv->port_list[i];
		if (port && netif_carrier_ok(port->dev)) {
			for(j = 0 ; j < num_active_cpus(); i++) {
				if (j==smp_processor_id())
					napi_schedule(&port->q_vector[j].napi);
				else {
					err = smp_call_function_single(j, napi_schedule, &port->q_vector[j].napi, 0);
					if (!err)
						pr_crit("%s:napi_schedule error:\n", __func__);
				}
			}
		}
		else if(port) {
			pr_debug("mvpp2(%d): port=%p netif_carrier_ok=%d\n", __LINE__, port, netif_carrier_ok(port->dev));
		}
		else {
			pr_debug("mvpp2(%d): mv_pp22_cpu_timer_callback. PORT NULL !!!!\n", __LINE__);
		}
	}

	mod_timer(&cpu_poll_timer, jiffies + msecs_to_jiffies(MV_PP2_FPGA_PERODIC_TIME));
}

#endif
module_init(mpp2_module_init);
module_exit(mpp2_module_exit);


MODULE_DESCRIPTION("Marvell PPv2 Ethernet Driver - www.marvell.com");
MODULE_AUTHOR("Marcin Wojtas <mw@semihalf.com>");
MODULE_LICENSE("GPL v2");
