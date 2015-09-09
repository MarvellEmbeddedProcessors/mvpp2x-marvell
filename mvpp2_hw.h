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

#ifndef _MVPP2_HW_H_
#define _MVPP2_HW_H_

#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>


static inline void mvpp2_write(struct mvpp2 *priv, u32 offset, u32 data)
{
	writel(data, priv->base + offset);
}

static inline u32 mvpp2_read(struct mvpp2 *priv, u32 offset)
{
	return readl(priv->base + offset);
}

#if 0
/* Obtain BM cookie information from descriptor */
static inline u32 mvpp2_bm_cookie_build(struct mvpp2_rx_desc *rx_desc)
{
	int pool = (rx_desc->status & MVPP2_RXD_BM_POOL_ID_MASK) >>
		   MVPP2_RXD_BM_POOL_ID_OFFS;
	int cpu = smp_processor_id();

	return ((pool & 0xFF) << MVPP2_BM_COOKIE_POOL_OFFS) |
	       ((cpu & 0xFF) << MVPP2_BM_COOKIE_CPU_OFFS);
}
#endif


/* Get number of physical egress port */
static inline int mvpp2_egress_port(struct mvpp2_port *port)
{
	return MVPP2_MAX_TCONT + port->id;
}

/* Get number of physical TXQ */
static inline int mvpp2_txq_phys(int port, int txq)
{
	return (MVPP2_MAX_TCONT + port) * MVPP2_MAX_TXQ + txq;
}


/* Rx descriptors helper methods */

/* Get number of Rx descriptors occupied by received packets */
static inline int mvpp2_rxq_received(struct mvpp2_port *port, int rxq_id)
{
	u32 val = mvpp2_read(port->priv, MVPP2_RXQ_STATUS_REG(rxq_id));

	return val & MVPP2_RXQ_OCCUPIED_MASK;
}

/* Update Rx queue status with the number of occupied and available
 * Rx descriptor slots.
 */
static inline void mvpp2_rxq_status_update(struct mvpp2_port *port,
				int rxq_id, int used_count, int free_count)
{
	/* Decrement the number of used descriptors and increment count
	 * increment the number of free descriptors.
	 */
	u32 val = used_count | (free_count << MVPP2_RXQ_NUM_NEW_OFFSET);

	mvpp2_write(port->priv, MVPP2_RXQ_STATUS_UPDATE_REG(rxq_id), val);
}

/* Get pointer to next RX descriptor to be processed by SW */
static inline struct mvpp2_rx_desc *
mvpp2_rxq_next_desc_get(struct mvpp2_rx_queue *rxq)
{
	int rx_desc = rxq->next_desc_to_proc;

	rxq->next_desc_to_proc = MVPP2_QUEUE_NEXT_DESC(rxq, rx_desc);
	prefetch(rxq->descs + rxq->next_desc_to_proc);
	return rxq->descs + rx_desc;
}

/* Mask the current CPU's Rx/Tx interrupts */
static inline void mvpp2_interrupts_mask(void *arg)
{
	struct mvpp2_port *port = arg;

	mvpp2_write(port->priv, MVPP2_ISR_RX_TX_MASK_REG(port->id), 0);
}

/* Unmask the current CPU's Rx/Tx interrupts */
static inline void mvpp2_interrupts_unmask(void *arg)
{
	struct mvpp2_port *port = arg;

	mvpp2_write(port->priv, MVPP2_ISR_RX_TX_MASK_REG(port->id),
		    (MVPP21_CAUSE_MISC_SUM_MASK |
		     MVPP21_CAUSE_RXQ_OCCUP_DESC_ALL_MASK));
}

#if 0
/* Set pool number in a BM cookie */
static inline u32 mvpp2_bm_cookie_pool_set(u32 cookie, int pool)
{
	u32 bm;

	bm = cookie & ~(0xFF << MVPP2_BM_COOKIE_POOL_OFFS);
	bm |= ((pool & 0xFF) << MVPP2_BM_COOKIE_POOL_OFFS);

	return bm;
}
#endif

static inline struct mvpp2_rx_queue *mvpp2_get_rx_queue(struct mvpp2_port *port,
							u32 cause)
{
	int queue = fls(cause) - 1;

	return port->rxqs[queue];
}

static inline struct mvpp2_tx_queue *mvpp2_get_tx_queue(struct mvpp2_port *port,
							u32 cause)
{
	int queue = fls(cause) - 1;


	return port->txqs[queue];
}

#if 0
/* Get pool number from a BM cookie */
static inline int mvpp2_bm_cookie_pool_get(u32 cookie)
{
	return (cookie >> MVPP2_BM_COOKIE_POOL_OFFS) & 0xFF;
}
#endif

/* Release buffer to BM */
static inline void mvpp2_bm_pool_put(struct mvpp2 *priv, u32 pool,
				     dma_addr_t buf_phys_addr, struct sk_buff *buf_virt_addr)
{

#ifdef CONFIG_PHYS_ADDR_T_64BIT //TODO: Validate this is  correct CONFIG_XXX for (sk_buff *),   it is a kmem_cache address (YuvalC).
	u32 val = 0;
	val = (upper_32_bits((uintptr_t)buf_virt_addr) && MVPP22_ADDR_HIGH_MASK) << MVPP22_BM_VIRT_HIGH_ALLOC_OFFST;
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	val |= (upper_32_bits(buf_phys_addr) && MVPP22_ADDR_HIGH_MASK) << MVPP22_BM_PHY_HIGH_ALLOC_OFFSET;

#endif
	mvpp2_write(priv, MVPP22_BM_PHY_VIRT_HIGH_ALLOC_REG, val);
#endif

	mvpp2_write(priv, MVPP2_BM_VIRT_RLS_REG, lower_32_bits((uintptr_t)buf_virt_addr));
	mvpp2_write(priv, MVPP2_BM_PHY_RLS_REG(pool), lower_32_bits(buf_phys_addr));

}

/* Release multicast buffer */
static inline void mvpp2_bm_pool_mc_put(struct mvpp2_port *port, int pool,
				 u32 buf_phys_addr, u32 buf_virt_addr,
				 int mc_id)
{
	u32 val = 0;

	val |= (mc_id & MVPP21_BM_MC_ID_MASK);
	mvpp2_write(port->priv, MVPP21_BM_MC_RLS_REG, val);
	//TODO : YuvalC, this is just workaround to compile. Need to handle mvpp2_buff_hdr_rx().
	mvpp2_bm_pool_put(port->priv, pool,
			  (dma_addr_t)(buf_phys_addr | MVPP2_BM_PHY_RLS_MC_BUFF_MASK),
			  (struct sk_buff *)(buf_virt_addr));
}

static inline void mvpp2_interrupts_enable(struct mvpp2_port *port)
{
	int cpu, cpu_mask = 0;

	for_each_present_cpu(cpu)
		cpu_mask |= 1 << cpu;
	mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_ENABLE_INTERRUPT(cpu_mask));
}


static inline void mvpp2_interrupts_disable(struct mvpp2_port *port)
{
	int cpu, cpu_mask = 0;

	for_each_present_cpu(cpu)
		cpu_mask |= 1 << cpu;
	mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_DISABLE_INTERRUPT(cpu_mask));
}


static inline int mvpp2_txq_sent_desc_proc(struct mvpp2_port *port,
					   struct mvpp2_tx_queue *txq)
{
	u32 val;

	/* Reading status reg resets transmitted descriptor counter */
	val = mvpp2_read(port->priv, MVPP21_TXQ_SENT_REG(txq->id));

	return (val & MVPP21_TRANSMITTED_COUNT_MASK) >>
		MVPP21_TRANSMITTED_COUNT_OFFSET;
}

static inline void mvpp2_txq_desc_put(struct mvpp2_tx_queue *txq)
{
	if (txq->next_desc_to_proc == 0)
		txq->next_desc_to_proc = txq->last_desc - 1;
	else
		txq->next_desc_to_proc--;
}


static inline void mvpp2_txq_sent_counter_clear(void *arg)
{
	struct mvpp2_port *port = arg;
	int queue;

	for (queue = 0; queue < mvpp2_txq_number; queue++) {
		int id = port->txqs[queue]->id;

		mvpp2_read(port->priv, MVPP21_TXQ_SENT_REG(id));
	}
}

static inline struct sk_buff* mvpp21_rxdesc_cookie_get(struct mvpp2_rx_desc *rx_desc) {
	return((struct sk_buff*)((uintptr_t)rx_desc->u.pp21.buf_cookie));
}

static inline dma_addr_t mvpp21_rxdesc_phys_addr_get(struct mvpp2_rx_desc *rx_desc) {
	return((dma_addr_t)rx_desc->u.pp21.buf_phys_addr);
}

/*YuvalC: Below functions are intended to support both aarch64 & aarch32 */
static inline struct sk_buff* mvpp22_rxdesc_cookie_get(struct mvpp2_rx_desc *rx_desc) {
	return((struct sk_buff*)((uintptr_t)
		(rx_desc->u.pp22.buf_cookie_bm_qset_cls_info & DMA_BIT_MASK(40))));
}

static inline dma_addr_t mvpp22_rxdesc_phys_addr_get(struct mvpp2_rx_desc *rx_desc) {
	return((dma_addr_t)
		(rx_desc->u.pp22.buf_phys_addr_key_hash & DMA_BIT_MASK(40)));
}

static inline struct sk_buff* mvpp21_txdesc_cookie_get(struct mvpp2_tx_desc *tx_desc) {
	return((struct sk_buff*)((uintptr_t)tx_desc->u.pp21.buf_cookie));
}

static inline dma_addr_t mvpp21_txdesc_phys_addr_get(struct mvpp2_tx_desc *tx_desc) {
	return((dma_addr_t)tx_desc->u.pp21.buf_phys_addr);
}

static inline struct sk_buff* mvpp22_txdesc_cookie_get(struct mvpp2_tx_desc *tx_desc) {
	return((struct sk_buff*)((uintptr_t)
		(tx_desc->u.pp22.buf_cookie_bm_qset_hw_cmd3 & DMA_BIT_MASK(40))));
}

static inline dma_addr_t mvpp22_txdesc_phys_addr_get(struct mvpp2_tx_desc *tx_desc) {
	return((dma_addr_t)
		(tx_desc->u.pp22.buf_phys_addr_hw_cmd2 & DMA_BIT_MASK(40)));
}

static inline dma_addr_t mvpp2x_txdesc_phys_addr_get(
	enum mvppv2_version pp2_ver, struct mvpp2_tx_desc *tx_desc) {


	if (pp2_ver == PPV21) {
		return(mvpp21_txdesc_phys_addr_get(tx_desc));
	}
	return(mvpp22_txdesc_phys_addr_get(tx_desc));
}


static inline void mvpp21_txdesc_phys_addr_set(dma_addr_t phys_addr,
	struct mvpp2_tx_desc *tx_desc) {
	tx_desc->u.pp21.buf_phys_addr = phys_addr;
}

static inline void mvpp22_txdesc_phys_addr_set(dma_addr_t phys_addr,
	struct mvpp2_tx_desc *tx_desc) {

	u64 *buf_phys_addr_p = &tx_desc->u.pp22.buf_phys_addr_hw_cmd2;

#ifdef 	CONFIG_ARCH_DMA_ADDR_T_64BIT
	*buf_phys_addr_p &= ~(DMA_BIT_MASK(40));
	*buf_phys_addr_p |= phys_addr & DMA_BIT_MASK(40);
#else
	*((dma_addr_t *)buf_phys_addr_p) = phys_addr;
	*((u8 *)buf_phys_addr_p + sizeof(dma_addr_t)) = 0; //5th byte
#endif
}

static inline void mvpp2x_txdesc_phys_addr_set(enum mvppv2_version pp2_ver,
	dma_addr_t phys_addr, struct mvpp2_tx_desc *tx_desc) {

	if (pp2_ver == PPV21) {
		mvpp21_txdesc_phys_addr_set(phys_addr, tx_desc);
	}  else {
		mvpp22_txdesc_phys_addr_set(phys_addr, tx_desc);
	}
}


int mvpp2_prs_default_init(struct platform_device *pdev,
				  struct mvpp2 *priv);
void mvpp2_prs_mac_promisc_set(struct mvpp2 *priv, int port, bool add);
void mvpp2_prs_mac_multi_set(struct mvpp2 *priv, int port, int index,
				    bool add);
int mvpp2_prs_mac_da_accept(struct mvpp2 *priv, int port,
				   const u8 *da, bool add);
int mvpp2_prs_def_flow(struct mvpp2_port *port);
void mvpp2_prs_mcast_del_all(struct mvpp2 *priv, int port);
int mvpp2_prs_tag_mode_set(struct mvpp2 *priv, int port, int type);
int mvpp2_prs_update_mac_da(struct net_device *dev, const u8 *da);

void mvpp2_cls_init(struct mvpp2 *priv);
void mvpp2_cls_port_config(struct mvpp2_port *port);
void mvpp2_cls_oversize_rxq_set(struct mvpp2_port *port);

void mvpp2_txp_max_tx_size_set(struct mvpp2_port *port);
void mvpp2_gmac_max_rx_size_set(struct mvpp2_port *port);

int mvpp2_txq_pend_desc_num_get(struct mvpp2_port *port,
				       struct mvpp2_tx_queue *txq);
u32 mvpp2_txq_desc_csum(int l3_offs, int l3_proto,
			       int ip_hdr_len, int l4_proto);
struct mvpp2_tx_desc * mvpp2_txq_next_desc_get(struct mvpp2_tx_queue *txq);
int mvpp2_txq_alloc_reserved_desc(struct mvpp2 *priv,
					 struct mvpp2_tx_queue *txq, int num);
void mvpp2_aggr_txq_pend_desc_add(struct mvpp2_port *port, int pending);
int mvpp2_aggr_desc_num_check(struct mvpp2 *priv,
				     struct mvpp2_tx_queue *aggr_txq, int num);
void mvpp2_rxq_offset_set(struct mvpp2_port *port,
				 int prxq, int offset);
void mvpp2_bm_bufs_free(struct mvpp2 *priv, struct mvpp2_bm_pool *bm_pool);
void mvpp2_bm_pool_bufsize_set(struct mvpp2 *priv,
				      struct mvpp2_bm_pool *bm_pool,
				      int buf_size);
void mvpp2_pool_refill(struct mvpp2 *priv, u32 pool,
			      dma_addr_t phys_addr, struct sk_buff *cookie);

void mvpp2_rxq_long_pool_set(struct mvpp2_port *port,
				    int lrxq, int long_pool);
void mvpp2_rxq_short_pool_set(struct mvpp2_port *port,
				     int lrxq, int short_pool);

void mvpp2_port_mii_set(struct mvpp2_port *port);
void mvpp2_port_fc_adv_enable(struct mvpp2_port *port);
void mvpp2_port_enable(struct mvpp2_port *port);
void mvpp2_port_disable(struct mvpp2_port *port);

void mvpp2_ingress_enable(struct mvpp2_port *port);
void mvpp2_ingress_disable(struct mvpp2_port *port);
void mvpp2_egress_enable(struct mvpp2_port *port);
void mvpp2_egress_disable(struct mvpp2_port *port);


void mvpp2_port_periodic_xon_disable(struct mvpp2_port *port);
void mvpp2_port_loopback_set(struct mvpp2_port *port);
void mvpp2_port_reset(struct mvpp2_port *port);

void mvpp2_rx_pkts_coal_set(struct mvpp2_port *port,
				   struct mvpp2_rx_queue *rxq, u32 pkts);
void mvpp2_rx_time_coal_set(struct mvpp2_port *port,
				   struct mvpp2_rx_queue *rxq, u32 usec);
void mvpp2_tx_done_pkts_coal_set(void *arg);
void mvpp2_cause_error(struct net_device *dev, int cause);
void mvpp2_rx_error(struct mvpp2_port *port,
			   struct mvpp2_rx_desc *rx_desc);
void mvpp2_rx_csum(struct mvpp2_port *port, u32 status,
			  struct sk_buff *skb);
void mvpp2_get_mac_address(struct mvpp2_port *port, unsigned char *addr);



#endif /* _MVPP2_HW_H_ */
