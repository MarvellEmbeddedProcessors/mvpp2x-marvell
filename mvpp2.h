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


/* Number of RXQs used by single port */
extern int mvpp2_rxq_number;
/* Number of TXQs used by single port */
extern int mvpp2_txq_number;


/* Descriptor ring Macros */
#define MVPP2_QUEUE_NEXT_DESC(q, index) \
	(((index) < (q)->last_desc) ? ((index) + 1) : 0)

/* Various constants */
#define MVPP2_MAX_CPUS		4
#define MVPP2_MAX_SHARED	1


/* Coalescing */
#define MVPP2_TXDONE_COAL_PKTS_THRESH	15
#define MVPP2_RX_COAL_PKTS		32
#define MVPP2_RX_COAL_USEC		100

/* BM constants */
#define MVPP2_BM_POOLS_NUM		8
#define MVPP2_BM_LONG_BUF_NUM		1024
#define MVPP2_BM_SHORT_BUF_NUM		2048
#define MVPP2_BM_POOL_SIZE_MAX		(16*1024 - MVPP2_BM_POOL_PTR_ALIGN/4)
#define MVPP2_BM_POOL_PTR_ALIGN		128
#define MVPP2_BM_SWF_LONG_POOL(port)	((port > 2) ? 2 : port)
#define MVPP2_BM_SWF_SHORT_POOL		3



enum mvppv2_version {
	PPV21 = 1,
	PPV22
};

enum mvpp2_queue_vector_type {
	MVPP2_SHARED, 
	MVPP2_PRIVATE
};

enum queue_distribution_mode{
	SINGLE_MODE,  
	/* All queues are shared. 
   	PPv2.1 – this is the only supported mode. 
  	PPv2.2 – Requires (N+1) interrupts. All rx_queues are configured on the additional interrupt. */
	MULTI_MODE  // PPv2.2 only requires N interrupts.
};


struct mvpp2x_platform_data {
	u8 pp2x_ver;
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
	u32 time_coal;	

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
	void __iomem *base;// PPV22 base_address as received in devm_ioremap_resource().
	void __iomem *lms_base; 
	void __iomem *cpu_base[MVPP2_MAX_CPUS]; 
	void __iomem *shared_base[MVPP2_MAX_SHARED]; 
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
	u8 cos_classifier;    //CoS based on VLAN or DSCP
	u8 num_cos_queues;     //number of queue to do CoS
	u8 default_cos;       //Default CoS value for non-IP or non-VLAN
	u8 reserved;
	u32 pri_map;          //32 bits, each nibble maps a cos_value(0~7) to a queue.
};

struct mvpp2_rss {
	u8 queue_mode ;//single/multi mode
	u8 rss_mode;//UDP packet
	u8 dflt_cpu;//non-IP packet
	u8 reserved;	
};



struct mvpp2_param_config {
	struct mvpp2_cos cos_cfg;
	struct mvpp2_rss rss_cfg;
	u8 first_bm_pool;
	bool jumbo_pool; // pp2 always supports 2 pools : short=MV_DEF_256, long=MV_DEF_2K. Param defines option to have additional pool, jumbo=MV_DEF_10K.	
	u8 first_sw_thread; // The index of the first PPv2.2 sub-address space for this NET_INSTANCE.	
	u8 cell_index; // The cell_index of the PPv22 (could be 0,1, set according to dtsi)
	enum queue_distribution_mode queue_mode;
};


/* Shared Packet Processor resources */
struct mvpp2 {
	
	enum mvppv2_version pp2_version; //Redundant, consider to delete. (prevents extra pointer lookup from mvpp2x_platform_data)

	struct	mvpp2_hw hw;
	const struct mvpp2x_platform_data *pp2xdata;

	u8 cpu_map;

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

struct queue_vector {
	int irq;
	struct napi_struct napi;
	enum mvpp2_queue_vector_type qv_type;
	u16 sw_thread_id;
/* ppv22: for qv_type=shared, this is the shared sw_thr_id.
               for qv_type=private, this is the cpu’s private sw_thr_id. 
    ppv21: sw_thread_id=0.*/
	u16 sw_thread_mask;
/* ppv22: sw_thread_mask = (1<<sw_thread_id).
    ppv21: sw_thread_mask = for_each_present_cpu(); */
	u8 first_rx_queue;
	u8 num_rx_queues;
	u32 pending_cause_rx; /* mask is in absolute queues, not relative queues, 
	                                       (unlike Ethernet Occupied Interrupt Cause (EthOccIC)) */
	struct mvpp2_port * parent_port;
};


struct mvpp2_port {
	u8 id;

	int irq;

	struct mvpp2 *priv;

	/* Per-port registers' base address */
	void __iomem *base;

	struct mvpp2_rx_queue **rxqs; //Each Port has up tp 32 rxq_queues.
	struct mvpp2_tx_queue **txqs;
	struct net_device *dev;

	int pkt_size; //pkt_size determines which is pool_long: jumbo_pool or regular long_pool.

	u32 pending_cause_rx;
	struct napi_struct napi;

	/* Flags */
	unsigned long flags;

	u16 tx_ring_size;
	u16 rx_ring_size;
	struct mvpp2_pcpu_stats __percpu *stats;

	struct phy_device *phy_dev;
	phy_interface_t phy_interface;
	struct device_node *phy_node;
	unsigned int link;
	unsigned int duplex;
	unsigned int speed;

	struct mvpp2_bm_pool *pool_long; //Pointer to the pool_id (long or jumbo)
	struct mvpp2_bm_pool *pool_short; //Pointer to the short pool_id

	/* Index of first port's physical RXQ */
	u8 first_rxq;


	/* First MAX_CPUs are for private_queues, last MAX_SHARED are for shared , if exist.
	    q_vector is the parameter that will be passed to mv_pp2_isr(int irq, void *dev_id=q_vector)  */ 
	struct queue_vector q_vector[MVPP2_MAX_CPUS+MVPP2_MAX_SHARED]; 
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

