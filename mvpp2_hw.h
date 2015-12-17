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
#include <linux/printk.h>

#include <linux/platform_device.h>

#if 1
void mvpp2_write(struct mvpp2_hw *hw, u32 offset, u32 data);
u32 mvpp2_read(struct mvpp2_hw *hw, u32 offset);

#else
static inline void mvpp2_write(struct mvpp2_hw *hw, u32 offset, u32 data)
{
#if 1
	if (smp_processor_id() != 0)
		pr_emerg_once("Received mvpp2_write(%d) from CPU1 !!\n", offset);
#endif
	pr_info("mvpp2_write(%p) \n", hw->cpu_base[smp_processor_id()] + offset);
	writel(data, hw->cpu_base[smp_processor_id()] + offset);
}

static inline u32 mvpp2_read(struct mvpp2_hw *hw, u32 offset)
{
#if 1
	if (smp_processor_id() != 0)
		pr_emerg_once("Received mvpp2_read(%d) from CPU1 !!\n", offset);
#endif
	pr_info("mvpp2_read(%p) \n", hw->cpu_base[smp_processor_id()] + offset);

	return readl(hw->cpu_base[smp_processor_id()] + offset);
}
#endif
static inline void mvpp22_thread_write(struct mvpp2_hw *hw, u32 sw_thread,
	u32 offset, u32 data)
{
	writel(data, hw->base + sw_thread*MVPP2_ADDR_SPACE_SIZE + offset);
}

static inline u32 mvpp22_thread_read(struct mvpp2_hw *hw, u32 sw_thread,
	u32 offset)
{
	return readl(hw->base + sw_thread*MVPP2_ADDR_SPACE_SIZE + offset);
}


static inline void mvpp21_isr_rx_group_write(struct mvpp2_hw *hw, int port,
		int num_rx_queues)
{
	mvpp2_write(hw, MVPP21_ISR_RXQ_GROUP_REG(port), num_rx_queues);
}


static inline void mvpp22_isr_rx_group_write(struct mvpp2_hw *hw, int port,
		int sub_group, int start_queue, int num_rx_queues)
{

	int val;

	val = (port << MVPP22_ISR_RXQ_GROUP_INDEX_GROUP_OFFSET) | sub_group;
	mvpp2_write(hw, MVPP22_ISR_RXQ_GROUP_INDEX_REG, val);
	val = (num_rx_queues << MVPP22_ISR_RXQ_SUB_GROUP_SIZE_OFFSET) | start_queue;
	mvpp2_write(hw, MVPP22_ISR_RXQ_SUB_GROUP_CONFIG_REG, val);

}


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
	u32 val = mvpp2_read(&(port->priv->hw), MVPP2_RXQ_STATUS_REG(rxq_id));

	return val & MVPP2_RXQ_OCCUPIED_MASK;
}


/* Get number of Rx descriptors occupied by received packets */
static inline int mvpp2_rxq_free(struct mvpp2_port *port, int rxq_id)
{
	u32 val = mvpp2_read(&(port->priv->hw), MVPP2_RXQ_STATUS_REG(rxq_id));

	return val & MVPP2_RXQ_NON_OCCUPIED_MASK;
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

	mvpp2_write(&(port->priv->hw), MVPP2_RXQ_STATUS_UPDATE_REG(rxq_id), val);
}

/* Get pointer to next RX descriptor to be processed by SW */
static inline struct mvpp2_rx_desc *
mvpp2_rxq_next_desc_get(struct mvpp2_rx_queue *rxq)
{
	int rx_desc = rxq->next_desc_to_proc;

	rxq->next_desc_to_proc = MVPP2_QUEUE_NEXT_DESC(rxq, rx_desc);
	prefetch(rxq->first_desc + rxq->next_desc_to_proc);
	return (rxq->first_desc + rx_desc);
}

/* Mask the current CPU's Rx/Tx interrupts */
static inline void mvpp2_interrupts_mask(void *arg)
{
	struct mvpp2_port *port = arg;

	mvpp2_write(&(port->priv->hw), MVPP2_ISR_RX_TX_MASK_REG(port->id), 0);
}

/* Unmask the current CPU's Rx/Tx interrupts */
static inline void mvpp2_interrupts_unmask(void *arg)
{
	struct mvpp2_port *port = arg;
	u32 val;

	val = MVPP2_CAUSE_MISC_SUM_MASK | MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK;
	if (port->priv->pp2xdata->interrupt_tx_done == true)
		val |= MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK;

	mvpp2_write(&(port->priv->hw), MVPP2_ISR_RX_TX_MASK_REG(port->id), val);
}

static inline void mvpp2_shared_thread_interrupts_mask(struct mvpp2_port *port)
{
	struct queue_vector *q_vec = &port->q_vector[0];
	int i;

	if (port->priv->pp2xdata->multi_addr_space == false)
		return;

	for (i=0;i<port->num_qvector;i++) {
		if (q_vec[i].qv_type == MVPP2_SHARED)
			mvpp22_thread_write(&port->priv->hw, q_vec[i].sw_thread_id,
			MVPP2_ISR_RX_TX_MASK_REG(port->id), 0);
	}
}

/* Unmask the shared CPU's Rx interrupts */
static inline void mvpp2_shared_thread_interrupts_unmask(struct mvpp2_port *port)
{
	struct queue_vector *q_vec = &port->q_vector[0];
	int i;

	if (port->priv->pp2xdata->multi_addr_space == false)
		return;

	for (i=0;i<port->num_qvector;i++) {
		if (q_vec[i].qv_type == MVPP2_SHARED)
			mvpp22_thread_write(&port->priv->hw, q_vec[i].sw_thread_id,
			MVPP2_ISR_RX_TX_MASK_REG(port->id), MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK);
	}
}


static inline struct mvpp2_rx_queue *mvpp2_get_rx_queue(struct mvpp2_port *port,
							u32 cause)
{
	int rx_queue = fls(cause) - 1;
	return port->rxqs[rx_queue];
}

static inline struct mvpp2_tx_queue *mvpp2_get_tx_queue(struct mvpp2_port *port,
	u32 cause)
{
	int tx_queue = fls(cause) - 1;
	return port->txqs[tx_queue];
}


static inline struct sk_buff *mvpp2_bm_virt_addr_get(struct mvpp2_hw *hw,
						u32 pool)
{
	uintptr_t val = 0;

	mvpp2_read(hw, MVPP2_BM_PHY_ALLOC_REG(pool));
/*TODO: Validate this is  correct CONFIG_XXX for (sk_buff *),   it is a kmem_cache address (YuvalC).*/
#ifdef CONFIG_PHYS_ADDR_T_64BIT
	val = mvpp2_read(hw, MVPP22_BM_PHY_VIRT_HIGH_ALLOC_REG);
	val &= MVPP22_BM_VIRT_HIGH_ALLOC_MASK;
	val <<= (32 - MVPP22_BM_VIRT_HIGH_ALLOC_OFFSET);
#endif
	val |= mvpp2_read(hw, MVPP2_BM_VIRT_ALLOC_REG);
	return((struct sk_buff *)val);
}


static inline void mvpp2_bm_hw_pool_create(struct mvpp2_hw *hw,
			u32 pool, dma_addr_t pool_addr, int size)
{
	u32 val;

	mvpp2_write(hw, MVPP2_BM_POOL_BASE_ADDR_REG(pool), lower_32_bits(pool_addr));
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	mvpp2_write(hw, MVPP22_BM_POOL_BASE_ADDR_HIGH_REG,
	(upper_32_bits(pool_addr)& MVPP22_ADDR_HIGH_MASK));
#endif
	mvpp2_write(hw, MVPP2_BM_POOL_SIZE_REG(pool), size);

	val = mvpp2_read(hw, MVPP2_BM_POOL_CTRL_REG(pool));
	val |= MVPP2_BM_START_MASK;
	mvpp2_write(hw, MVPP2_BM_POOL_CTRL_REG(pool), val);
}



/* Release buffer to BM */
static inline void mvpp2_bm_pool_put(struct mvpp2_hw *hw, u32 pool,
				     dma_addr_t buf_phys_addr, struct sk_buff *buf_virt_addr)
{

/*TODO: Validate this is  correct CONFIG_XXX for (sk_buff *),   it is a kmem_cache address (YuvalC).*/
#ifdef CONFIG_64BIT /*CONFIG_PHYS_ADDR_T_64BIT*/
	u32 val = 0;
	val = (upper_32_bits((uintptr_t)buf_virt_addr) & MVPP22_ADDR_HIGH_MASK) << MVPP22_BM_VIRT_HIGH_RLS_OFFST;
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	val |= (upper_32_bits(buf_phys_addr) & MVPP22_ADDR_HIGH_MASK) << MVPP22_BM_PHY_HIGH_RLS_OFFSET;

#endif
	mvpp2_write(hw, MVPP22_BM_PHY_VIRT_HIGH_RLS_REG, val);
#endif

	mvpp2_write(hw, MVPP2_BM_VIRT_RLS_REG, lower_32_bits((uintptr_t)buf_virt_addr));
	mvpp2_write(hw, MVPP2_BM_PHY_RLS_REG(pool), lower_32_bits(buf_phys_addr));

}

/* Release multicast buffer */
static inline void mvpp2_bm_pool_mc_put(struct mvpp2_port *port, int pool,
				 u32 buf_phys_addr, u32 buf_virt_addr,
				 int mc_id)
{
	u32 val = 0;

	val |= (mc_id & MVPP21_BM_MC_ID_MASK);
	mvpp2_write(&(port->priv->hw), MVPP21_BM_MC_RLS_REG, val);
	/*TODO : YuvalC, this is just workaround to compile. Need to handle mvpp2_buff_hdr_rx().*/
	mvpp2_bm_pool_put(&(port->priv->hw), pool,
			  (dma_addr_t)(buf_phys_addr | MVPP2_BM_PHY_RLS_MC_BUFF_MASK),
			  (struct sk_buff *)(buf_virt_addr));
}

static inline void mvpp2_port_interrupts_enable(struct mvpp2_port *port)
{
	int sw_thread_mask=0, i;
	struct queue_vector *q_vec = &port->q_vector[0];

	for (i=0;i<port->num_qvector;i++)
		sw_thread_mask |= q_vec[i].sw_thread_mask;
#ifndef CONFIG_MV_PP2_FPGA
	mvpp2_write(&port->priv->hw, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_ENABLE_INTERRUPT(sw_thread_mask));
#endif
}

static inline void mvpp2_port_interrupts_disable(struct mvpp2_port *port)
{
	int sw_thread_mask=0, i;
	struct queue_vector *q_vec = &port->q_vector[0];

	for (i=0;i<port->num_qvector;i++)
		sw_thread_mask |= q_vec[i].sw_thread_mask;

	mvpp2_write(&port->priv->hw, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_DISABLE_INTERRUPT(sw_thread_mask));
}


static inline void mvpp2_qvector_interrupt_enable(struct queue_vector *q_vec)
{
#ifndef CONFIG_MV_PP2_FPGA
	struct mvpp2_port *port = q_vec->parent;

	mvpp2_write(&port->priv->hw, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_ENABLE_INTERRUPT(q_vec->sw_thread_mask));
#endif
}

static inline void mvpp2_qvector_interrupt_disable(struct queue_vector *q_vec)
{
	struct mvpp2_port *port = q_vec->parent;

	mvpp2_write(&port->priv->hw, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_DISABLE_INTERRUPT(q_vec->sw_thread_mask));

}

static inline int mvpp2_txq_sent_desc_proc(struct mvpp2_port *port,
					   struct mvpp2_tx_queue *txq)
{
	u32 val;

	/* Reading status reg resets transmitted descriptor counter */
	val = mvpp2_read(&(port->priv->hw), MVPP21_TXQ_SENT_REG(txq->id));

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

	for (queue = 0; queue < port->num_tx_queues; queue++) {
		int id = port->txqs[queue]->id;
		if (port->priv->pp2_version == PPV21)
			mvpp2_read(&(port->priv->hw), MVPP21_TXQ_SENT_REG(id));
		else
			mvpp2_read(&(port->priv->hw), MVPP22_TXQ_SENT_REG(id));
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
	*((u8 *)buf_phys_addr_p + sizeof(dma_addr_t)) = 0; /*5th byte*/

	//pr_crit("phys_addr=%x, buf_phys_addr_hw_cmd2=%d\n", phys_addr, tx_desc->u.pp22.buf_phys_addr_hw_cmd2);
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

int mvpp2_prs_hw_read(struct mvpp2_hw *hw, struct mvpp2_prs_entry *pe);

int mvpp2_prs_default_init(struct platform_device *pdev,
				  struct mvpp2_hw *hw);
void mvpp2_prs_mac_promisc_set(struct mvpp2_hw *hw, int port, bool add);
void mvpp2_prs_mac_multi_set(struct mvpp2_hw *hw, int port, int index,
				    bool add);
int mvpp2_prs_mac_da_accept(struct mvpp2_hw *hw, int port,
				   const u8 *da, bool add);
int mvpp2_prs_def_flow(struct mvpp2_port *port);
int mvpp2_prs_flow_set(struct mvpp2_port *port);
void mvpp2_prs_mcast_del_all(struct mvpp2_hw *hw, int port);
int mvpp2_prs_tag_mode_set(struct mvpp2_hw *hw, int port, int type);
int mvpp2_prs_update_mac_da(struct net_device *dev, const u8 *da);
void mvpp2_prs_flow_id_attr_init(void);
int mvpp2_prs_flow_id_attr_get(int flow_id);

int mvpp2_cls_init(struct platform_device *pdev, struct mvpp2_hw *hw);
void mvpp2_cls_port_config(struct mvpp2_port *port);
void mvpp2_cls_config(struct mvpp2_hw *hw);
void mvpp2_cls_oversize_rxq_set(struct mvpp2_port *port);
void mvpp2_cls_lookup_read(struct mvpp2_hw *hw, int lkpid, int way, struct mvpp2_cls_lookup_entry *le);
void mvpp2_cls_flow_tbl_temp_copy(struct mvpp2_hw *hw, int lkpid, int *temp_flow_idx);
void mvpp2_cls_lkp_flow_set(struct mvpp2_hw *hw, int lkpid, int way, int flow_idx);
void mvpp2_cls_flow_port_add(struct mvpp2_hw *hw, int index, int port_id);
void mvpp2_cls_flow_port_del(struct mvpp2_hw *hw, int index, int port_id);

void mvpp2_txp_max_tx_size_set(struct mvpp2_port *port);
void mvpp2_tx_done_time_coal_set(struct mvpp2_port *port, u32 usec);
void mvpp2_gmac_max_rx_size_set(struct mvpp2_port *port);

int mvpp2_txq_pend_desc_num_get(struct mvpp2_port *port,
				       struct mvpp2_tx_queue *txq);
u32 mvpp2_txq_desc_csum(int l3_offs, int l3_proto,
			       int ip_hdr_len, int l4_proto);
struct mvpp2_tx_desc * mvpp2_txq_next_desc_get(struct mvpp2_aggr_tx_queue *aggr_txq);
int mvpp2_txq_alloc_reserved_desc(struct mvpp2 *priv,
					 struct mvpp2_tx_queue *txq, int num);
void mvpp2_aggr_txq_pend_desc_add(struct mvpp2_port *port, int pending);
int mvpp2_aggr_desc_num_read(struct mvpp2 *priv, int cpu);
int mvpp2_aggr_desc_num_check(struct mvpp2 *priv,
				     struct mvpp2_aggr_tx_queue *aggr_txq, int num);
void mvpp2_rxq_offset_set(struct mvpp2_port *port,
				 int prxq, int offset);
void mvpp2_bm_pool_bufsize_set(struct mvpp2_hw *hw,
				      struct mvpp2_bm_pool *bm_pool,
				      int buf_size);
void mvpp2_pool_refill(struct mvpp2 *priv, u32 pool,
			      dma_addr_t phys_addr, struct sk_buff *cookie);

void mvpp21_rxq_long_pool_set(struct mvpp2_hw *hw,
				     int prxq, int long_pool);
void mvpp21_rxq_short_pool_set(struct mvpp2_hw *hw,
				     int prxq, int short_pool);

void mvpp22_rxq_long_pool_set(struct mvpp2_hw *hw,
				     int prxq, int long_pool);
void mvpp22_rxq_short_pool_set(struct mvpp2_hw *hw,
				     int prxq, int short_pool);


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

int mvpp2_c2_init(struct platform_device *pdev, struct mvpp2_hw *hw);

int mvpp2_prs_sw_sram_shift_set(struct mvpp2_prs_entry *pe, int shift, unsigned int op);
int mvpp2_prs_sw_sram_shift_get(struct mvpp2_prs_entry *pe, int *shift);
int mvpp2_prs_sw_sram_next_lu_get(struct mvpp2_prs_entry *pe, unsigned int *lu);
int mvpp2_prs_sram_bit_get(struct mvpp2_prs_entry *pe, int bitNum, unsigned int *bit);
int mvpp2_prs_sw_sram_lu_done_get(struct mvpp2_prs_entry *pe, unsigned int *bit);
int mvpp2_prs_sw_sram_flowid_gen_get(struct mvpp2_prs_entry *pe, unsigned int *bit);
int mvpp2_prs_sw_sram_ri_get(struct mvpp2_prs_entry *pe, unsigned int *bits, unsigned int *enable);
int mvpp2_prs_sw_sram_ai_get(struct mvpp2_prs_entry *pe, unsigned int *bits, unsigned int *enable);
int mvpp2_prs_sw_sram_offset_set(struct mvpp2_prs_entry *pe, unsigned int type, int offset, unsigned int op);
int mvpp2_prs_sw_dump(struct mvpp2_prs_entry *pe);
int mvpp2_prs_hw_dump(struct mvpp2_hw *hw);
int mvpp2_prs_hw_regs_dump(struct mvpp2_hw *hw);
int mvpp2_prs_hw_hits_dump(struct mvpp2_hw *hw);
void mvpp2_prs_hw_port_init(struct mvpp2_hw *hw, int port, int lu_first, int lu_max, int offset);
void mvpp2_prs_sw_clear(struct mvpp2_prs_entry *pe);
void mvpp2_prs_hw_inv(struct mvpp2_hw *hw, int index);
void mvpp2_prs_tcam_lu_set(struct mvpp2_prs_entry *pe, unsigned int lu);
void mvpp2_prs_tcam_port_set(struct mvpp2_prs_entry *pe, unsigned int port, bool add);
void mvpp2_prs_tcam_port_map_set(struct mvpp2_prs_entry *pe, unsigned int ports);
void mvpp2_prs_tcam_data_byte_set(struct mvpp2_prs_entry *pe, unsigned int offs, unsigned char byte, unsigned char enable);
void mvpp2_prs_tcam_ai_update(struct mvpp2_prs_entry *pe, unsigned int bits, unsigned int enable);
void mvpp2_prs_sram_ri_update(struct mvpp2_prs_entry *pe, unsigned int bits, unsigned int mask);
void mvpp2_prs_sram_ai_update(struct mvpp2_prs_entry *pe, unsigned int bits, unsigned int mask);
void mvpp2_prs_sram_next_lu_set(struct mvpp2_prs_entry *pe, unsigned int lu);
void mvpp2_prs_sw_sram_lu_done_set(struct mvpp2_prs_entry *pe);
void mvpp2_prs_sw_sram_lu_done_clear(struct mvpp2_prs_entry *pe);
void mvpp2_prs_sw_sram_flowid_set(struct mvpp2_prs_entry *pe);
void mvpp2_prs_sw_sram_flowid_clear(struct mvpp2_prs_entry *pe);
int mvpp2_prs_hw_write(struct mvpp2_hw *hw, struct mvpp2_prs_entry *pe);
int mvpp2_cls_hw_lkp_read(struct mvpp2_hw * hw, int lkpid, int way, struct mvpp2_cls_lookup_entry *fe);
int mvpp2_cls_hw_lkp_write(struct mvpp2_hw * hw, int lkpid, int way, struct mvpp2_cls_lookup_entry *fe);
int mvpp2_cls_lkp_port_way_set(struct mvpp2_hw *hw, int port, int way);
int mvpp2_cls_hw_lkp_print(struct mvpp2_hw * hw, int lkpid, int way);
int mvpp2_cls_sw_lkp_rxq_get(struct mvpp2_cls_lookup_entry *lkp, int *rxq);
int mvpp2_cls_sw_lkp_rxq_set(struct mvpp2_cls_lookup_entry *fe, int rxq);
int mvpp2_cls_sw_lkp_en_get(struct mvpp2_cls_lookup_entry *lkp, int *en);
int mvpp2_cls_sw_lkp_en_set(struct mvpp2_cls_lookup_entry *lkp, int en);
int mvpp2_cls_sw_lkp_flow_get(struct mvpp2_cls_lookup_entry *lkp, int *flow_idx);
int mvpp2_cls_sw_lkp_flow_set(struct mvpp2_cls_lookup_entry *lkp, int flow_idx);
int mvpp2_cls_sw_lkp_mod_get(struct mvpp2_cls_lookup_entry *lkp, int *mod_base);
int mvpp2_cls_sw_lkp_mod_set(struct mvpp2_cls_lookup_entry *lkp, int mod_base);
int mvpp2_cls_hw_flow_read(struct mvpp2_hw * hw, int index, struct mvpp2_cls_flow_entry *fe);
int mvpp2_cls_sw_flow_dump(struct mvpp2_cls_flow_entry *fe);
int mvpp2_cls_hw_regs_dump(struct mvpp2_hw * hw);
int mvpp2_cls_hw_lkp_hit_get(struct mvpp2_hw * hw, int lkpid, int way,  unsigned int *cnt);
int mvpp2_cls_hw_flow_dump(struct mvpp2_hw * hw);
int mvpp2_cls_hw_flow_hits_dump(struct mvpp2_hw * hw);
void mvpp2_cls_flow_write(struct mvpp2_hw *hw, struct mvpp2_cls_flow_entry *fe);
int mvpp2_cls_sw_flow_port_set(struct mvpp2_cls_flow_entry *fe, int type, int portid);
int mvpp2_cls_sw_flow_hek_num_set(struct mvpp2_cls_flow_entry *fe, int num_of_fields);
int mvpp2_cls_sw_flow_hek_set(struct mvpp2_cls_flow_entry *fe, int field_index, int field_id);
int mvpp2_cls_sw_flow_portid_select(struct mvpp2_cls_flow_entry *fe, int from);
int mvpp2_cls_sw_flow_pppoe_set(struct mvpp2_cls_flow_entry *fe, int mode);
int mvpp2_cls_sw_flow_vlan_set(struct mvpp2_cls_flow_entry *fe, int mode);
int mvpp2_cls_sw_flow_macme_set(struct mvpp2_cls_flow_entry *fe, int mode);
int mvpp2_cls_sw_flow_udf7_set(struct mvpp2_cls_flow_entry *fe, int mode);
int mvpp2_cls_sw_flow_seq_ctrl_set(struct mvpp2_cls_flow_entry *fe, int mode);
int mvpp2_cls_sw_flow_engine_set(struct mvpp2_cls_flow_entry *fe, int engine, int is_last);
int mvpp2_cls_sw_flow_extra_set(struct mvpp2_cls_flow_entry *fe, int type, int prio);
int mvpp2_cls_hw_lkp_hits_dump(struct mvpp2_hw * hw);
int mvpp2_cls_sw_lkp_dump(struct mvpp2_cls_lookup_entry *lkp);
int mvpp2_cls_hw_lkp_dump(struct mvpp2_hw * hw);
int mvpp2_cls_hw_udf_set(struct mvpp2_hw *hw, int udf_no, int offs_id, int offs_bits, int size_bits);
int mvpp2_cls_c2_qos_hw_read(struct mvpp2_hw *hw, int tbl_id, int tbl_sel, int tbl_line, struct mvpp2_cls_c2_qos_entry *qos);
int mvpp2_cls_c2_qos_hw_write(struct mvpp2_hw *hw, struct mvpp2_cls_c2_qos_entry *qos);
int mvpp2_cls_c2_qos_dscp_hw_dump(struct mvpp2_hw *hw);
int mvpp2_cls_c2_qos_prio_hw_dump(struct mvpp2_hw *hw);
int mvpp2_cls_c2_qos_tbl_set(struct mvpp2_cls_c2_entry *c2, int tbl_id, int tbl_sel);
int mvpp2_cls_c2_hw_write(struct mvpp2_hw *hw, int index, struct mvpp2_cls_c2_entry *c2);
int mvpp2_cls_c2_hw_read(struct mvpp2_hw *hw, int index, struct mvpp2_cls_c2_entry *c2);
int mvpp2_cls_c2_sw_words_dump(struct mvpp2_cls_c2_entry *c2);
int mvpp2_cls_c2_sw_dump(struct mvpp2_cls_c2_entry *c2);
int mvpp2_cls_c2_hw_dump(struct mvpp2_hw *hw);
int mvpp2_cls_c2_hit_cntr_clear_all(struct mvpp2_hw *hw);
int mvpp2_cls_c2_hit_cntr_read(struct mvpp2_hw *hw, int index, u32 *cntr);
int mvpp2_cls_c2_hit_cntr_dump(struct mvpp2_hw *hw);
int mvpp2_cls_c2_regs_dump(struct mvpp2_hw *hw);
int mvpp2_cls_c2_rule_set(struct mvpp2_port *port, u8 start_queue);
u8 mvpp2_cls_c2_rule_queue_get(struct mvpp2_hw *hw, u32 rule_idx);
void mvpp2_cls_c2_rule_queue_set(struct mvpp2_hw *hw, u32 rule_idx, u8 queue);
u8 mvpp2_cls_c2_pbit_tbl_queue_get(struct mvpp2_hw *hw, u8 tbl_id, u8 tbl_line);
void mvpp2_cls_c2_pbit_tbl_queue_set(struct mvpp2_hw *hw, u8 tbl_id, u8 tbl_line, u8 queue);
int mvpp2_cls_c2_hw_inv(struct mvpp2_hw *hw, int index);
void mvpp2_cls_c2_hw_inv_all(struct mvpp2_hw *hw);
int mvpp2_cls_c2_tcam_byte_set(struct mvpp2_cls_c2_entry *c2, unsigned int offs,
					unsigned char byte, unsigned char enable);
int mvpp2_cls_c2_qos_queue_set(struct mvpp2_cls_c2_qos_entry *qos, u8 queue);
int mvpp2_cls_c2_color_set(struct mvpp2_cls_c2_entry *c2, int cmd, int from);
int mvpp2_cls_c2_prio_set(struct mvpp2_cls_c2_entry *c2, int cmd, int prio, int from);
int mvpp2_cls_c2_dscp_set(struct mvpp2_cls_c2_entry *c2, int cmd, int dscp, int from);
int mvpp2_cls_c2_queue_low_set(struct mvpp2_cls_c2_entry *c2, int cmd, int queue, int from);
int mvpp2_cls_c2_queue_high_set(struct mvpp2_cls_c2_entry *c2, int cmd, int queue, int from);
int mvpp2_cls_c2_forward_set(struct mvpp2_cls_c2_entry *c2, int cmd);
int mvpp2_cls_c2_rss_set(struct mvpp2_cls_c2_entry *c2, int cmd, int rss_en);
int mvpp2_cls_c2_flow_id_en(struct mvpp2_cls_c2_entry *c2, int flowid_en);

int mvpp22_rss_tbl_entry_set(struct mvpp2_hw *hw, struct mvpp22_rss_entry *rss);
int mvpp22_rss_tbl_entry_get(struct mvpp2_hw *hw, struct mvpp22_rss_entry *rss);

int mvpp22_rss_rxq_set(struct mvpp2_port *port, u32 cos_width);

void mvpp22_rss_c2_enable(struct mvpp2_port *port, bool en);
int mvpp22_rss_hw_dump(struct mvpp2_hw *hw);

#endif /* _MVPP2_HW_H_ */
