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

#ifndef _MVPP2_H_
#define _MVPP2_H_

#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include "mvpp2_hw_type.h"




#define MVPP2_DRIVER_NAME "mvpp2"
#define MVPP2_DRIVER_VERSION "1.0"


#define PFX			MVPP2_DRIVER_NAME ": "
//#define CONFIG_MVPP2_DEBUG
#ifdef CONFIG_MVPP2_DEBUG
#define DEBUG	1
#define VERBOSE	1
#else
#define DEBUG	0
#define VERBOSE	0
#endif /*CONFIG_MVPP2_DEBUG*/

#if VERBOSE
#define DBG_MSG(fmt, args...)	printk(KERN_CRIT PFX fmt, ## args)
#else
#if DEBUG
#define DBG_MSG(fmt, args...)	printk(KERN_DEBUG PFX fmt, ## args)
#else
#define DBG_MSG(fmt, args...)	while (0) printk(fmt, ## args)
#endif /*DEBUG*/
#endif /*VERBOSE*/


/* Descriptor ring Macros */
#define MVPP2_QUEUE_NEXT_DESC(q, index) \
	(((index) < (q)->last_desc) ? ((index) + 1) : 0)

/* Various constants */
#define MVPP2_MAX_CPUS		4
#define MVPP2_MAX_SHARED	1


/* Coalescing */
#define MVPP2_TXDONE_COAL_PKTS		15
#define MVPP2_TXDONE_HRTIMER_PERIOD_NS	1000000UL
#define MVPP2_TXDONE_COAL_USEC		500

#define MVPP2_RX_COAL_PKTS		32
#define MVPP2_RX_COAL_USEC		100

/* BM constants */
#define MVPP2_BM_POOLS_NUM		8
#define MVPP2_BM_POOL_SIZE_MAX		(16*1024 - MVPP2_BM_POOL_PTR_ALIGN/4)
#define MVPP2_BM_POOL_PTR_ALIGN		128


#define MVPP2_BM_SHORT_BUF_NUM		2048
#define MVPP2_BM_LONG_BUF_NUM		1024
#define MVPP2_BM_JUMBO_BUF_NUM		512


#define MVPP2_ALL_BUFS			0


enum mvppv2_version {
	PPV21 = 21,
	PPV22
};

enum mvpp2_queue_vector_type {
	MVPP2_SHARED,
	MVPP2_PRIVATE
};

enum mvpp2_queue_distribution_mode{
	/* All queues are shared.
   	PPv2.1 – this is the only supported mode.
  	PPv2.2 – Requires (N+1) interrupts. All rx_queues are configured on the additional interrupt. */
	MVPP2_QDIST_SINGLE_MODE,
	MVPP2_QDIST_MULTI_MODE  /* PPv2.2 only requires N interrupts. */
};



/* Per-CPU Tx queue control */
struct mvpp2_txq_pcpu {
	int cpu;

	/* Number of Tx DMA descriptors in the descriptor ring */
	int size;

	/* Number of currently used Tx DMA descriptor in the
	 * descriptor ring
	 */
	int count;

	/* Number of Tx DMA descriptors reserved for each CPU */
	int reserved_num;

	/* Array of transmitted skb */
	struct sk_buff **tx_skb;

	/* Array of transmitted buffers' physical addresses */
	dma_addr_t *tx_buffs;

	/* Index of last TX DMA descriptor that was inserted */
	int txq_put_index;

	/* Index of the TX DMA descriptor to be cleaned up */
	int txq_get_index;
};


struct mvpp2_tx_queue {
	/* Physical number of this Tx queue */
	u8 id;

	/* Logical number of this Tx queue */
	u8 log_id;

	/* Number of Tx DMA descriptors in the descriptor ring */
	int size;

	/* Per-CPU control of physical Tx queues */
	struct mvpp2_txq_pcpu __percpu *pcpu;

	u32 pkts_coal;

	/* Virtual address of thex Tx DMA descriptors array */
	struct mvpp2_tx_desc *descs;

	/* DMA address of the Tx DMA descriptors array */
	dma_addr_t descs_phys;

	/* Index of the last Tx DMA descriptor */
	int last_desc;

	/* Index of the next Tx DMA descriptor to process */
	int next_desc_to_proc;
};


struct mvpp2_aggr_tx_queue {
	/* Physical number of this Tx queue */
	u8 id;

	/* Number of Tx DMA descriptors in the descriptor ring */
	int size;

	/* Number of currently used Tx DMA descriptor in the descriptor ring */
	int count;

	/* Virtual address of thex Tx DMA descriptors array */
	struct mvpp2_tx_desc *descs;

	/* DMA address of the Tx DMA descriptors array */
	dma_addr_t descs_phys;

	/* Index of the last Tx DMA descriptor */
	int last_desc;

	/* Index of the next Tx DMA descriptor to process */
	int next_desc_to_proc;
};


struct mvpp2_rx_queue {
	/* RX queue number, in the range 0-31 for physical RXQs */
	u8 id;

	/* Port's logic RXQ number to which physical RXQ is mapped */
	int log_id;

	/* Num of rx descriptors in the rx descriptor ring */
	int size;

	u32 pkts_coal;
	u32 time_coal;

	/* Virtual address of the RX DMA descriptors array */
	struct mvpp2_rx_desc *descs;

	/* DMA address of the RX DMA descriptors array */
	dma_addr_t descs_phys;

	/* Index of the last RX DMA descriptor */
	int last_desc;

	/* Index of the next RX DMA descriptor to process */
	int next_desc_to_proc;

	/* ID of port to which physical RXQ is mapped */
	int port;

};


struct mvpp2_hw {

	/* Shared registers' base addresses */
	void __iomem *base;/* PPV22 base_address as received in devm_ioremap_resource().*/
	void __iomem *lms_base;
	void __iomem *cpu_base[MVPP2_MAX_CPUS];
/* ppv22_base_address for each CPU.
    PPv2.2 - cpu_base[x] = base + cpu_index[smp_processor_id]*MV_PP2_SPACE_64K, for non-participating CPU it is NULL.
    PPv2.1 cpu_base[x] = base */
	/* Common clocks */
	struct clk *pp_clk;
	struct clk *gop_clk;
	u32 tclk;

	/* PRS shadow table */
	struct mvpp2_prs_shadow *prs_shadow;
	/* PRS auxiliary table for double vlan entries control */
	bool *prs_double_vlans;
};

struct mvpp2_cos {
	u8 cos_classifier;    /* CoS based on VLAN or DSCP */
	u8 num_cos_queues;     /* number of queue to do CoS */
	u8 default_cos;       /* Default CoS value for non-IP or non-VLAN */
	u8 reserved;
	u32 pri_map;          /* 32 bits, each nibble maps a cos_value(0~7) to a queue.*/
};

struct mvpp2_rss {
	enum mvpp2_queue_distribution_mode queue_mode;//single/multi mode
	u8 rss_mode; /*UDP packet */
	u8 dflt_cpu; /*non-IP packet */
	u8 reserved;
};



struct mvpp2_param_config {
	struct mvpp2_cos cos_cfg;
	struct mvpp2_rss rss_cfg;
	u8 first_bm_pool;
	bool jumbo_pool; /* pp2 always supports 2 pools : short=MV_DEF_256, long=MV_DEF_2K. Param defines option to have additional pool, jumbo=MV_DEF_10K.*/
	u8 first_sw_thread; /* The index of the first PPv2.2 sub-address space for this NET_INSTANCE.*/
	u8 cell_index; /* The cell_index of the PPv22 (could be 0,1, set according to dtsi) */
	enum mvpp2_queue_distribution_mode queue_mode;
};


/* Shared Packet Processor resources */
struct mvpp2 {

	enum mvppv2_version pp2_version; /* Redundant, consider to delete. (prevents extra pointer lookup from mvpp2x_platform_data) */

	struct	mvpp2_hw hw;
	const struct mvpp2x_platform_data *pp2xdata;

	u16 cpu_map; /* Bitmap of the participating cpu's */


	struct mvpp2_param_config pp2_cfg;

	/* List of pointers to port structures */
	struct mvpp2_port **port_list;

	/* Aggregated TXQs */
	struct mvpp2_aggr_tx_queue *aggr_txqs;

	/* BM pools */
	struct mvpp2_bm_pool *bm_pools;
};

struct mvpp2_pcpu_stats {
	struct	u64_stats_sync syncp;
	u64	rx_packets;
	u64	rx_bytes;
	u64	tx_packets;
	u64	tx_bytes;
};

/* Per-CPU port control */
struct mvpp2_port_pcpu {
	struct hrtimer tx_done_timer;
	bool timer_scheduled;
	/* Tasklet for egress finalization */
	struct tasklet_struct tx_done_tasklet;
};
struct queue_vector {
	unsigned int irq;
	struct napi_struct napi;
	enum mvpp2_queue_vector_type qv_type;
	u16 sw_thread_id; /* address_space index used to retrieve interrupt_cause */
	u16 sw_thread_mask; /* Mask for Interrupt PORT_ENABLE Register */
	u8 first_rx_queue; /* Relative to port */
	u8 num_rx_queues;
	u32 pending_cause_rx; /* mask in absolute port_queues, not relative as in Ethernet Occupied Interrupt Cause (EthOccIC)) */
	struct mvpp2_port * parent;
};


struct mvpp2_port {
	u8 id;

	u8 num_irqs;
	u32 *of_irqs;

	struct mvpp2 *priv;

	/* Per-port registers' base address */
	void __iomem *base;


	struct mvpp2_rx_queue **rxqs; /*Each Port has up tp 32 rxq_queues.*/
	struct mvpp2_tx_queue **txqs;
	struct net_device *dev;

	int pkt_size; /* pkt_size determines which is pool_long: jumbo_pool or regular long_pool. */


	/* Per-CPU port control */
	struct mvpp2_port_pcpu __percpu *pcpu;
	/* Flags */
	unsigned long flags;

	u16 tx_ring_size;
	u16 rx_ring_size;

	u32 tx_time_coal;
	struct mvpp2_pcpu_stats __percpu *stats;

	struct phy_device *phy_dev;
	phy_interface_t phy_interface;
	struct device_node *phy_node;
	unsigned int link;
	unsigned int duplex;
	unsigned int speed;

	struct mvpp2_bm_pool *pool_long; /* Pointer to the pool_id (long or jumbo) */
	struct mvpp2_bm_pool *pool_short; /* Pointer to the short pool_id */

	/* Index of port's first physical RXQ */
	u8 first_rxq;

	/* port's  number of rx_queues */
	u8 num_rx_queues;
	/* port's  number of tx_queues */
	u8 num_tx_queues;

	u32 num_qvector;
	/* q_vector is the parameter that will be passed to mv_pp2_isr(int irq, void *dev_id=q_vector)  */
	struct queue_vector q_vector[MVPP2_MAX_CPUS+MVPP2_MAX_SHARED];
};

struct mvpp2x_platform_data {
	enum mvppv2_version pp2x_ver;
	u8 pp2x_max_port_rxqs;
	u8 num_port_irq;
	bool multi_addr_space;
	bool interrupt_tx_done;
	bool multi_hw_instance;
	void (*mvpp2x_rxq_short_pool_set)(struct mvpp2_hw *, int, int);
	void (*mvpp2x_rxq_long_pool_set)(struct mvpp2_hw *, int, int);
	void (*mvpp2x_port_queue_vectors_init)(struct mvpp2_port *);
	void (*mvpp2x_port_isr_rx_group_cfg)(struct mvpp2_port *);
};


int mvpp2_check_ringparam_valid(struct net_device *dev,
				       struct ethtool_ringparam *ring);
void mvpp2_start_dev(struct mvpp2_port *port);
void mvpp2_stop_dev(struct mvpp2_port *port);
void mvpp2_cleanup_rxqs(struct mvpp2_port *port);
int mvpp2_setup_rxqs(struct mvpp2_port *port);
int mvpp2_setup_txqs(struct mvpp2_port *port);
void mvpp2_cleanup_txqs(struct mvpp2_port *port);


void mvpp2_set_ethtool_ops(struct net_device *netdev);

#endif /*_MVPP2_H_*/

