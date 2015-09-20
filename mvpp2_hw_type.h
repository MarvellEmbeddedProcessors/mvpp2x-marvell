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

#ifndef _MVPP2_HW_TYPE_H_
#define _MVPP2_HW_TYPE_H_

#include <linux/kernel.h>
#include <linux/skbuff.h>



/* RX Fifo Registers */
#define MVPP2_RX_DATA_FIFO_SIZE_REG(port)	(0x00 + 4 * (port))
#define MVPP2_RX_ATTR_FIFO_SIZE_REG(port)	(0x20 + 4 * (port))
#define MVPP2_RX_MIN_PKT_SIZE_REG		0x60
#define MVPP2_RX_FIFO_INIT_REG			0x64

/* RX DMA Top Registers */
#define MVPP2_RX_CTRL_REG(port)			(0x140 + 4 * (port))
#define MVPP2_RX_LOW_LATENCY_PKT_SIZE(s)	(((s) & 0xfff) << 16)
#define MVPP2_RX_USE_PSEUDO_FOR_CSUM_MASK	BIT(31)
#define MVPP2_POOL_BUF_SIZE_REG(pool)		(0x180 + 4 * (pool))
#define MVPP2_POOL_BUF_SIZE_OFFSET		5
#define MVPP2_RXQ_CONFIG_REG(rxq)		(0x800 + 4 * (rxq))
#define MVPP2_SNOOP_PKT_SIZE_MASK		0x1ff
#define MVPP2_SNOOP_BUF_HDR_MASK		BIT(9)
#define MVPP2_RXQ_POOL_SHORT_OFFS		20
#define MVPP2_RXQ_POOL_SHORT_MASK		0x700000
#define MVPP2_RXQ_POOL_LONG_OFFS		24
#define MVPP2_RXQ_POOL_LONG_MASK		0x7000000
#define MVPP2_RXQ_PACKET_OFFSET_OFFS		28
#define MVPP2_RXQ_PACKET_OFFSET_MASK		0x70000000
#define MVPP2_RXQ_DISABLE_MASK			BIT(31)

/* Parser Registers */
#define MVPP2_PRS_INIT_LOOKUP_REG		0x1000
#define MVPP2_PRS_PORT_LU_MAX			0xf
#define MVPP2_PRS_PORT_LU_MASK(port)		(0xff << ((port) * 4))
#define MVPP2_PRS_PORT_LU_VAL(port, val)	((val) << ((port) * 4))
#define MVPP2_PRS_INIT_OFFS_REG(port)		(0x1004 + ((port) & 4))
#define MVPP2_PRS_INIT_OFF_MASK(port)		(0x3f << (((port) % 4) * 8))
#define MVPP2_PRS_INIT_OFF_VAL(port, val)	((val) << (((port) % 4) * 8))
#define MVPP2_PRS_MAX_LOOP_REG(port)		(0x100c + ((port) & 4))
#define MVPP2_PRS_MAX_LOOP_MASK(port)		(0xff << (((port) % 4) * 8))
#define MVPP2_PRS_MAX_LOOP_VAL(port, val)	((val) << (((port) % 4) * 8))
#define MVPP2_PRS_TCAM_IDX_REG			0x1100
#define MVPP2_PRS_TCAM_DATA_REG(idx)		(0x1104 + (idx) * 4)
#define MVPP2_PRS_TCAM_INV_MASK			BIT(31)
#define MVPP2_PRS_SRAM_IDX_REG			0x1200
#define MVPP2_PRS_SRAM_DATA_REG(idx)		(0x1204 + (idx) * 4)
#define MVPP2_PRS_TCAM_CTRL_REG			0x1230
#define MVPP2_PRS_TCAM_EN_MASK			BIT(0)

/* Classifier Registers */
#define MVPP2_CLS_MODE_REG			0x1800
#define MVPP2_CLS_MODE_ACTIVE_MASK		BIT(0)
#define MVPP2_CLS_PORT_WAY_REG			0x1810
#define MVPP2_CLS_PORT_WAY_MASK(port)		(1 << (port))
#define MVPP2_CLS_LKP_INDEX_REG			0x1814
#define MVPP2_CLS_LKP_INDEX_WAY_OFFS		6
#define MVPP2_CLS_LKP_TBL_REG			0x1818
#define MVPP2_CLS_LKP_TBL_RXQ_MASK		0xff
#define MVPP2_CLS_LKP_TBL_LOOKUP_EN_MASK	BIT(25)
#define MVPP2_CLS_FLOW_INDEX_REG		0x1820
#define MVPP2_CLS_FLOW_TBL0_REG			0x1824
#define MVPP2_CLS_FLOW_TBL1_REG			0x1828
#define MVPP2_CLS_FLOW_TBL2_REG			0x182c
#define MVPP2_CLS_OVERSIZE_RXQ_LOW_REG(port)	(0x1980 + ((port) * 4))
#define MVPP2_CLS_OVERSIZE_RXQ_LOW_BITS		3
#define MVPP2_CLS_OVERSIZE_RXQ_LOW_MASK		0x7
#define MVPP2_CLS_SWFWD_P2HQ_REG(port)		(0x19b0 + ((port) * 4))
#define MVPP2_CLS_SWFWD_PCTRL_REG		0x19d0
#define MVPP2_CLS_SWFWD_PCTRL_MASK(port)	(1 << (port))

/* Descriptor Manager Top Registers */
#define MVPP2_RXQ_NUM_REG			0x2040
#define MVPP2_RXQ_DESC_ADDR_REG			0x2044
#define MVPP2_RXQ_DESC_SIZE_REG			0x2048
#define MVPP2_RXQ_DESC_SIZE_MASK		0x3ff0
#define MVPP2_RXQ_STATUS_UPDATE_REG(rxq)	(0x3000 + 4 * (rxq))
#define MVPP2_RXQ_NUM_PROCESSED_OFFSET		0
#define MVPP2_RXQ_NUM_NEW_OFFSET		16
#define MVPP2_RXQ_STATUS_REG(rxq)		(0x3400 + 4 * (rxq))
#define MVPP2_RXQ_OCCUPIED_MASK			0x3fff
#define MVPP2_RXQ_NON_OCCUPIED_OFFSET		16
#define MVPP2_RXQ_NON_OCCUPIED_MASK		0x3fff0000
#define MVPP2_RXQ_THRESH_REG			0x204c
#define MVPP2_OCCUPIED_THRESH_OFFSET		0
#define MVPP2_OCCUPIED_THRESH_MASK		0x3fff
#define MVPP2_RXQ_INDEX_REG			0x2050
#define MVPP2_TXQ_NUM_REG			0x2080
#define MVPP2_TXQ_DESC_ADDR_REG			0x2084
#define MVPP2_TXQ_DESC_SIZE_REG			0x2088
#define MVPP2_TXQ_DESC_SIZE_MASK		0x3ff0
#define MVPP2_AGGR_TXQ_UPDATE_REG		0x2090
#define MVPP2_TXQ_THRESH_REG			0x2094
#define MVPP2_TRANSMITTED_THRESH_OFFSET		16
#define MVPP2_TRANSMITTED_THRESH_MASK		0x3fff0000
#define MVPP2_TXQ_INDEX_REG			0x2098
#define MVPP2_TXQ_PREF_BUF_REG			0x209c
#define MVPP2_PREF_BUF_PTR(desc)		((desc) & 0xfff)
#define MVPP2_PREF_BUF_SIZE_4			(BIT(12) | BIT(13))
#define MVPP2_PREF_BUF_SIZE_16			(BIT(12) | BIT(14))
#define MVPP2_PREF_BUF_THRESH(val)		((val) << 17)
#define MVPP2_TXQ_DRAIN_EN_MASK			BIT(31)
#define MVPP2_TXQ_PENDING_REG			0x20a0
#define MVPP2_TXQ_PENDING_MASK			0x3fff
#define MVPP2_TXQ_INT_STATUS_REG		0x20a4
#define MVPP2_TXQ_SENT_REG(txq)			(0x3c00 + 4 * (txq))
#define MVPP2_TRANSMITTED_COUNT_OFFSET		16
#define MVPP2_TRANSMITTED_COUNT_MASK		0x3fff0000
#define MVPP2_TXQ_RSVD_REQ_REG			0x20b0
#define MVPP2_TXQ_RSVD_REQ_Q_OFFSET		16
#define MVPP2_TXQ_RSVD_RSLT_REG			0x20b4
#define MVPP2_TXQ_RSVD_RSLT_MASK		0x3fff
#define MVPP2_TXQ_RSVD_CLR_REG			0x20b8
#define MVPP2_TXQ_RSVD_CLR_OFFSET		16
#define MVPP2_AGGR_TXQ_DESC_ADDR_REG(cpu)	(0x2100 + 4 * (cpu))
#define MVPP2_AGGR_TXQ_DESC_SIZE_REG(cpu)	(0x2140 + 4 * (cpu))
#define MVPP2_AGGR_TXQ_DESC_SIZE_MASK		0x3ff0
#define MVPP2_AGGR_TXQ_STATUS_REG(cpu)		(0x2180 + 4 * (cpu))
#define MVPP2_AGGR_TXQ_PENDING_MASK		0x3fff
#define MVPP2_AGGR_TXQ_INDEX_REG(cpu)		(0x21c0 + 4 * (cpu))

/* MBUS bridge registers */
#define MVPP2_WIN_BASE(w)			(0x4000 + ((w) << 2))
#define MVPP2_WIN_SIZE(w)			(0x4020 + ((w) << 2))
#define MVPP2_WIN_REMAP(w)			(0x4040 + ((w) << 2))
#define MVPP2_BASE_ADDR_ENABLE			0x4060

/* Interrupt Cause and Mask registers */
#define MVPP2_ISR_RX_THRESHOLD_REG(rxq)		(0x5200 + 4 * (rxq))
#define MVPP2_ISR_RXQ_GROUP_REG(rxq)		(0x5400 + 4 * (rxq))
#define MVPP2_ISR_ENABLE_REG(port)		(0x5420 + 4 * (port))
#define MVPP2_ISR_ENABLE_INTERRUPT(mask)	((mask) & 0xffff)
#define MVPP2_ISR_DISABLE_INTERRUPT(mask)	(((mask) << 16) & 0xffff0000)
#define MVPP2_ISR_RX_TX_CAUSE_REG(port)		(0x5480 + 4 * (port))
#define MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK	0xffff
#define MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK	0xff0000
#define MVPP2_CAUSE_RX_FIFO_OVERRUN_MASK	BIT(24)
#define MVPP2_CAUSE_FCS_ERR_MASK		BIT(25)
#define MVPP2_CAUSE_TX_FIFO_UNDERRUN_MASK	BIT(26)
#define MVPP2_CAUSE_TX_EXCEPTION_SUM_MASK	BIT(29)
#define MVPP2_CAUSE_RX_EXCEPTION_SUM_MASK	BIT(30)
#define MVPP2_CAUSE_MISC_SUM_MASK		BIT(31)
#define MVPP2_ISR_RX_TX_MASK_REG(port)		(0x54a0 + 4 * (port))
#define MVPP2_ISR_PON_RX_TX_MASK_REG		0x54bc
#define MVPP2_PON_CAUSE_RXQ_OCCUP_DESC_ALL_MASK	0xffff
#define MVPP2_PON_CAUSE_TXP_OCCUP_DESC_ALL_MASK	0x3fc00000
#define MVPP2_PON_CAUSE_MISC_SUM_MASK		BIT(31)
#define MVPP2_ISR_MISC_CAUSE_REG		0x55b0

/* Buffer Manager registers */
#define MVPP2_BM_POOL_BASE_REG(pool)		(0x6000 + ((pool) * 4))
#define MVPP2_BM_POOL_BASE_ADDR_MASK		0xfffff80
#define MVPP2_BM_POOL_SIZE_REG(pool)		(0x6040 + ((pool) * 4))
#define MVPP2_BM_POOL_SIZE_MASK			0xfff0
#define MVPP2_BM_POOL_READ_PTR_REG(pool)	(0x6080 + ((pool) * 4))
#define MVPP2_BM_POOL_GET_READ_PTR_MASK		0xfff0
#define MVPP2_BM_POOL_PTRS_NUM_REG(pool)	(0x60c0 + ((pool) * 4))
#define MVPP2_BM_POOL_PTRS_NUM_MASK		0xfff0
#define MVPP2_BM_BPPI_READ_PTR_REG(pool)	(0x6100 + ((pool) * 4))
#define MVPP2_BM_BPPI_PTRS_NUM_REG(pool)	(0x6140 + ((pool) * 4))
#define MVPP2_BM_BPPI_PTR_NUM_MASK		0x7ff
#define MVPP2_BM_BPPI_PREFETCH_FULL_MASK	BIT(16)
#define MVPP2_BM_POOL_CTRL_REG(pool)		(0x6200 + ((pool) * 4))
#define MVPP2_BM_START_MASK			BIT(0)
#define MVPP2_BM_STOP_MASK			BIT(1)
#define MVPP2_BM_STATE_MASK			BIT(4)
#define MVPP2_BM_LOW_THRESH_OFFS		8
#define MVPP2_BM_LOW_THRESH_MASK		0x7f00
#define MVPP2_BM_LOW_THRESH_VALUE(val)		((val) << \
						MVPP2_BM_LOW_THRESH_OFFS)
#define MVPP2_BM_HIGH_THRESH_OFFS		16
#define MVPP2_BM_HIGH_THRESH_MASK		0x7f0000
#define MVPP2_BM_HIGH_THRESH_VALUE(val)		((val) << \
						MVPP2_BM_HIGH_THRESH_OFFS)
#define MVPP2_BM_INTR_CAUSE_REG(pool)		(0x6240 + ((pool) * 4))
#define MVPP2_BM_RELEASED_DELAY_MASK		BIT(0)
#define MVPP2_BM_ALLOC_FAILED_MASK		BIT(1)
#define MVPP2_BM_BPPE_EMPTY_MASK		BIT(2)
#define MVPP2_BM_BPPE_FULL_MASK			BIT(3)
#define MVPP2_BM_AVAILABLE_BP_LOW_MASK		BIT(4)
#define MVPP2_BM_INTR_MASK_REG(pool)		(0x6280 + ((pool) * 4))
#define MVPP2_BM_PHY_ALLOC_REG(pool)		(0x6400 + ((pool) * 4))
#define MVPP2_BM_PHY_ALLOC_GRNTD_MASK		BIT(0)
#define MVPP2_BM_VIRT_ALLOC_REG			0x6440
#define MVPP2_BM_PHY_RLS_REG(pool)		(0x6480 + ((pool) * 4))
#define MVPP2_BM_PHY_RLS_MC_BUFF_MASK		BIT(0)
#define MVPP2_BM_PHY_RLS_PRIO_EN_MASK		BIT(1)
#define MVPP2_BM_PHY_RLS_GRNTD_MASK		BIT(2)
#define MVPP2_BM_VIRT_RLS_REG			0x64c0
#define MVPP2_BM_MC_RLS_REG			0x64c4
#define MVPP2_BM_MC_ID_MASK			0xfff
#define MVPP2_BM_FORCE_RELEASE_MASK		BIT(12)

/* TX Scheduler registers */
#define MVPP2_TXP_SCHED_PORT_INDEX_REG		0x8000
#define MVPP2_TXP_SCHED_Q_CMD_REG		0x8004
#define MVPP2_TXP_SCHED_ENQ_MASK		0xff
#define MVPP2_TXP_SCHED_DISQ_OFFSET		8
#define MVPP2_TXP_SCHED_CMD_1_REG		0x8010
#define MVPP2_TXP_SCHED_PERIOD_REG		0x8018
#define MVPP2_TXP_SCHED_MTU_REG			0x801c
#define MVPP2_TXP_MTU_MAX			0x7FFFF
#define MVPP2_TXP_SCHED_REFILL_REG		0x8020
#define MVPP2_TXP_REFILL_TOKENS_ALL_MASK	0x7ffff
#define MVPP2_TXP_REFILL_PERIOD_ALL_MASK	0x3ff00000
#define MVPP2_TXP_REFILL_PERIOD_MASK(v)		((v) << 20)
#define MVPP2_TXP_SCHED_TOKEN_SIZE_REG		0x8024
#define MVPP2_TXP_TOKEN_SIZE_MAX		0xffffffff
#define MVPP2_TXQ_SCHED_REFILL_REG(q)		(0x8040 + ((q) << 2))
#define MVPP2_TXQ_REFILL_TOKENS_ALL_MASK	0x7ffff
#define MVPP2_TXQ_REFILL_PERIOD_ALL_MASK	0x3ff00000
#define MVPP2_TXQ_REFILL_PERIOD_MASK(v)		((v) << 20)
#define MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(q)	(0x8060 + ((q) << 2))
#define MVPP2_TXQ_TOKEN_SIZE_MAX		0x7fffffff
#define MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(q)	(0x8080 + ((q) << 2))
#define MVPP2_TXQ_TOKEN_CNTR_MAX		0xffffffff

/* TX general registers */
#define MVPP2_TX_SNOOP_REG			0x8800
#define MVPP2_TX_PORT_FLUSH_REG			0x8810
#define MVPP2_TX_PORT_FLUSH_MASK(port)		(1 << (port))

/* LMS registers */
#define MVPP2_SRC_ADDR_MIDDLE			0x24
#define MVPP2_SRC_ADDR_HIGH			0x28
#define MVPP2_PHY_AN_CFG0_REG			0x34
#define MVPP2_PHY_AN_STOP_SMI0_MASK		BIT(7)
#define MVPP2_MIB_COUNTERS_BASE(port)		(0x1000 + ((port) >> 1) * \
						0x400 + (port) * 0x400)
#define MVPP2_MIB_LATE_COLLISION		0x7c
#define MVPP2_ISR_SUM_MASK_REG			0x220c
#define MVPP2_MNG_EXTENDED_GLOBAL_CTRL_REG	0x305c
#define MVPP2_EXT_GLOBAL_CTRL_DEFAULT		0x27

/* Per-port registers */
#define MVPP2_GMAC_CTRL_0_REG			0x0
#define MVPP2_GMAC_PORT_EN_MASK			BIT(0)
#define MVPP2_GMAC_MAX_RX_SIZE_OFFS		2
#define MVPP2_GMAC_MAX_RX_SIZE_MASK		0x7ffc
#define MVPP2_GMAC_MIB_CNTR_EN_MASK		BIT(15)
#define MVPP2_GMAC_CTRL_1_REG			0x4
#define MVPP2_GMAC_PERIODIC_XON_EN_MASK		BIT(1)
#define MVPP2_GMAC_GMII_LB_EN_MASK		BIT(5)
#define MVPP2_GMAC_PCS_LB_EN_BIT		6
#define MVPP2_GMAC_PCS_LB_EN_MASK		BIT(6)
#define MVPP2_GMAC_SA_LOW_OFFS			7
#define MVPP2_GMAC_CTRL_2_REG			0x8
#define MVPP2_GMAC_INBAND_AN_MASK		BIT(0)
#define MVPP2_GMAC_PCS_ENABLE_MASK		BIT(3)
#define MVPP2_GMAC_PORT_RGMII_MASK		BIT(4)
#define MVPP2_GMAC_PORT_RESET_MASK		BIT(6)
#define MVPP2_GMAC_AUTONEG_CONFIG		0xc
#define MVPP2_GMAC_FORCE_LINK_DOWN		BIT(0)
#define MVPP2_GMAC_FORCE_LINK_PASS		BIT(1)
#define MVPP2_GMAC_CONFIG_MII_SPEED		BIT(5)
#define MVPP2_GMAC_CONFIG_GMII_SPEED		BIT(6)
#define MVPP2_GMAC_AN_SPEED_EN			BIT(7)
#define MVPP2_GMAC_FC_ADV_EN			BIT(9)
#define MVPP2_GMAC_CONFIG_FULL_DUPLEX		BIT(12)
#define MVPP2_GMAC_AN_DUPLEX_EN			BIT(13)
#define MVPP2_GMAC_PORT_FIFO_CFG_1_REG		0x1c
#define MVPP2_GMAC_TX_FIFO_MIN_TH_OFFS	6
#define MVPP2_GMAC_TX_FIFO_MIN_TH_ALL_MASK	0x1fc0
#define MVPP2_GMAC_TX_FIFO_MIN_TH_MASK(v)	(((v) << 6) & \
						MVPP2_GMAC_TX_FIFO_MIN_TH_ALL_MASK)

#define MVPP2_CAUSE_TXQ_SENT_DESC_ALL_MASK	0xff


/* The two bytes Marvell header. Either contains a special value used
 * by Marvell switches when a specific hardware mode is enabled (not
 * supported by this driver) or is filled automatically by zeroes on
 * the RX side. Those two bytes being at the front of the Ethernet
 * header, they allow to have the IP header aligned on a 4 bytes
 * boundary automatically: the hardware skips those two bytes on its
 * own.
 */
#define MVPP2_MH_SIZE			2
#define MVPP2_ETH_TYPE_LEN		2
#define MVPP2_PPPOE_HDR_SIZE		8
#define MVPP2_VLAN_TAG_LEN		4

/* Lbtd 802.3 type */
#define MVPP2_IP_LBDT_TYPE		0xfffa

#define MVPP2_CPU_D_CACHE_LINE_SIZE	32
#define MVPP2_TX_CSUM_MAX_SIZE		9800

/* Timeout constants */
#define MVPP2_TX_DISABLE_TIMEOUT_MSEC	1000
#define MVPP2_TX_PENDING_TIMEOUT_MSEC	1000

#define MVPP2_TX_MTU_MAX		0x7ffff

/* Maximum number of T-CONTs of PON port */
#define MVPP2_MAX_TCONT			16

/* Maximum number of supported ports */
#define MVPP2_MAX_PORTS			4

/* Maximum number of TXQs used by single port */
#define MVPP2_MAX_TXQ			8

/* Maximum number of RXQs used by single port */
#define MVPP2_MAX_RXQ			8

/* Dfault number of RXQs in use */
#define MVPP2_DEFAULT_RXQ		4

/* Total number of RXQs available to all ports */
#define MVPP2_RXQ_TOTAL_NUM		(MVPP2_MAX_PORTS * MVPP2_MAX_RXQ)

/* Max number of Rx descriptors */
#define MVPP2_MAX_RXD			128

/* Max number of Tx descriptors */
#define MVPP2_MAX_TXD			1024

/* Amount of Tx descriptors that can be reserved at once by CPU */
#define MVPP2_CPU_DESC_CHUNK		64

/* Max number of Tx descriptors in each aggregated queue */
#define MVPP2_AGGR_TXQ_SIZE		256

/* Descriptor aligned size */
#define MVPP2_DESC_ALIGNED_SIZE		32

/* Descriptor alignment mask */
#define MVPP2_TX_DESC_ALIGN		(MVPP2_DESC_ALIGNED_SIZE - 1)

/* RX FIFO constants */
#define MVPP2_RX_FIFO_PORT_DATA_SIZE	0x2000
#define MVPP2_RX_FIFO_PORT_ATTR_SIZE	0x80
#define MVPP2_RX_FIFO_PORT_MIN_PKT	0x80

/* RX buffer constants */
#define MVPP2_SKB_SHINFO_SIZE \
	SKB_DATA_ALIGN(sizeof(struct skb_shared_info))

#define MVPP2_RX_PKT_SIZE(mtu) \
	ALIGN((mtu) + MVPP2_MH_SIZE + MVPP2_VLAN_TAG_LEN + \
	      ETH_HLEN + ETH_FCS_LEN, MVPP2_CPU_D_CACHE_LINE_SIZE)

#define MVPP2_RX_BUF_SIZE(pkt_size)	((pkt_size) + NET_SKB_PAD)
#define MVPP2_RX_TOTAL_SIZE(buf_size)	((buf_size) + MVPP2_SKB_SHINFO_SIZE)
#define MVPP2_RX_MAX_PKT_SIZE(total_size) \
	((total_size) - NET_SKB_PAD - MVPP2_SKB_SHINFO_SIZE)

#define MVPP2_BIT_TO_BYTE(bit)		((bit) / 8)

/* IPv6 max L3 address size */
#define MVPP2_MAX_L3_ADDR_SIZE		16

/* Port flags */
#define MVPP2_F_LOOPBACK		BIT(0)

/* Marvell tag types */
enum mvpp2_tag_type {
	MVPP2_TAG_TYPE_NONE = 0,
	MVPP2_TAG_TYPE_MH   = 1,
	MVPP2_TAG_TYPE_DSA  = 2,
	MVPP2_TAG_TYPE_EDSA = 3,
	MVPP2_TAG_TYPE_VLAN = 4,
	MVPP2_TAG_TYPE_LAST = 5
};

/* Parser constants */
#define MVPP2_PRS_TCAM_SRAM_SIZE	256
#define MVPP2_PRS_TCAM_WORDS		6
#define MVPP2_PRS_SRAM_WORDS		4
#define MVPP2_PRS_FLOW_ID_SIZE		64
#define MVPP2_PRS_FLOW_ID_MASK		0x3f
#define MVPP2_PRS_TCAM_ENTRY_INVALID	1
#define MVPP2_PRS_TCAM_DSA_TAGGED_BIT	BIT(5)
#define MVPP2_PRS_IPV4_HEAD		0x40
#define MVPP2_PRS_IPV4_HEAD_MASK	0xf0
#define MVPP2_PRS_IPV4_MC		0xe0
#define MVPP2_PRS_IPV4_MC_MASK		0xf0
#define MVPP2_PRS_IPV4_BC_MASK		0xff
#define MVPP2_PRS_IPV4_IHL		0x5
#define MVPP2_PRS_IPV4_IHL_MASK		0xf
#define MVPP2_PRS_IPV6_MC		0xff
#define MVPP2_PRS_IPV6_MC_MASK		0xff
#define MVPP2_PRS_IPV6_HOP_MASK		0xff
#define MVPP2_PRS_TCAM_PROTO_MASK	0xff
#define MVPP2_PRS_TCAM_PROTO_MASK_L	0x3f
#define MVPP2_PRS_DBL_VLANS_MAX		100

/* Tcam structure:
 * - lookup ID - 4 bits
 * - port ID - 1 byte
 * - additional information - 1 byte
 * - header data - 8 bytes
 * The fields are represented by MVPP2_PRS_TCAM_DATA_REG(5)->(0).
 */
#define MVPP2_PRS_AI_BITS			8
#define MVPP2_PRS_PORT_MASK			0xff
#define MVPP2_PRS_LU_MASK			0xf
#define MVPP2_PRS_TCAM_DATA_BYTE(offs)		\
				    (((offs) - ((offs) % 2)) * 2 + ((offs) % 2))
#define MVPP2_PRS_TCAM_DATA_BYTE_EN(offs)	\
					      (((offs) * 2) - ((offs) % 2)  + 2)
#define MVPP2_PRS_TCAM_AI_BYTE			16
#define MVPP2_PRS_TCAM_PORT_BYTE		17
#define MVPP2_PRS_TCAM_LU_BYTE			20
#define MVPP2_PRS_TCAM_EN_OFFS(offs)		((offs) + 2)
#define MVPP2_PRS_TCAM_INV_WORD			5
/* Tcam entries ID */
#define MVPP2_PE_DROP_ALL		0
#define MVPP2_PE_FIRST_FREE_TID		1
#define MVPP2_PE_LAST_FREE_TID		(MVPP2_PRS_TCAM_SRAM_SIZE - 31)
#define MVPP2_PE_IP6_EXT_PROTO_UN	(MVPP2_PRS_TCAM_SRAM_SIZE - 30)
#define MVPP2_PE_MAC_MC_IP6		(MVPP2_PRS_TCAM_SRAM_SIZE - 29)
#define MVPP2_PE_IP6_ADDR_UN		(MVPP2_PRS_TCAM_SRAM_SIZE - 28)
#define MVPP2_PE_IP4_ADDR_UN		(MVPP2_PRS_TCAM_SRAM_SIZE - 27)
#define MVPP2_PE_LAST_DEFAULT_FLOW	(MVPP2_PRS_TCAM_SRAM_SIZE - 26)
#define MVPP2_PE_FIRST_DEFAULT_FLOW	(MVPP2_PRS_TCAM_SRAM_SIZE - 19)
#define MVPP2_PE_EDSA_TAGGED		(MVPP2_PRS_TCAM_SRAM_SIZE - 18)
#define MVPP2_PE_EDSA_UNTAGGED		(MVPP2_PRS_TCAM_SRAM_SIZE - 17)
#define MVPP2_PE_DSA_TAGGED		(MVPP2_PRS_TCAM_SRAM_SIZE - 16)
#define MVPP2_PE_DSA_UNTAGGED		(MVPP2_PRS_TCAM_SRAM_SIZE - 15)
#define MVPP2_PE_ETYPE_EDSA_TAGGED	(MVPP2_PRS_TCAM_SRAM_SIZE - 14)
#define MVPP2_PE_ETYPE_EDSA_UNTAGGED	(MVPP2_PRS_TCAM_SRAM_SIZE - 13)
#define MVPP2_PE_ETYPE_DSA_TAGGED	(MVPP2_PRS_TCAM_SRAM_SIZE - 12)
#define MVPP2_PE_ETYPE_DSA_UNTAGGED	(MVPP2_PRS_TCAM_SRAM_SIZE - 11)
#define MVPP2_PE_MH_DEFAULT		(MVPP2_PRS_TCAM_SRAM_SIZE - 10)
#define MVPP2_PE_DSA_DEFAULT		(MVPP2_PRS_TCAM_SRAM_SIZE - 9)
#define MVPP2_PE_IP6_PROTO_UN		(MVPP2_PRS_TCAM_SRAM_SIZE - 8)
#define MVPP2_PE_IP4_PROTO_UN		(MVPP2_PRS_TCAM_SRAM_SIZE - 7)
#define MVPP2_PE_ETH_TYPE_UN		(MVPP2_PRS_TCAM_SRAM_SIZE - 6)
#define MVPP2_PE_VLAN_DBL		(MVPP2_PRS_TCAM_SRAM_SIZE - 5)
#define MVPP2_PE_VLAN_NONE		(MVPP2_PRS_TCAM_SRAM_SIZE - 4)
#define MVPP2_PE_MAC_MC_ALL		(MVPP2_PRS_TCAM_SRAM_SIZE - 3)
#define MVPP2_PE_MAC_PROMISCUOUS	(MVPP2_PRS_TCAM_SRAM_SIZE - 2)
#define MVPP2_PE_MAC_NON_PROMISCUOUS	(MVPP2_PRS_TCAM_SRAM_SIZE - 1)

/* Sram structure
 * The fields are represented by MVPP2_PRS_TCAM_DATA_REG(3)->(0).
 */
#define MVPP2_PRS_SRAM_RI_OFFS			0
#define MVPP2_PRS_SRAM_RI_WORD			0
#define MVPP2_PRS_SRAM_RI_CTRL_OFFS		32
#define MVPP2_PRS_SRAM_RI_CTRL_WORD		1
#define MVPP2_PRS_SRAM_RI_CTRL_BITS		32
#define MVPP2_PRS_SRAM_SHIFT_OFFS		64
#define MVPP2_PRS_SRAM_SHIFT_SIGN_BIT		72
#define MVPP2_PRS_SRAM_UDF_OFFS			73
#define MVPP2_PRS_SRAM_UDF_BITS			8
#define MVPP2_PRS_SRAM_UDF_MASK			0xff
#define MVPP2_PRS_SRAM_UDF_SIGN_BIT		81
#define MVPP2_PRS_SRAM_UDF_TYPE_OFFS		82
#define MVPP2_PRS_SRAM_UDF_TYPE_MASK		0x7
#define MVPP2_PRS_SRAM_UDF_TYPE_L3		1
#define MVPP2_PRS_SRAM_UDF_TYPE_L4		4
#define MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS	85
#define MVPP2_PRS_SRAM_OP_SEL_SHIFT_MASK	0x3
#define MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD		1
#define MVPP2_PRS_SRAM_OP_SEL_SHIFT_IP4_ADD	2
#define MVPP2_PRS_SRAM_OP_SEL_SHIFT_IP6_ADD	3
#define MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS		87
#define MVPP2_PRS_SRAM_OP_SEL_UDF_BITS		2
#define MVPP2_PRS_SRAM_OP_SEL_UDF_MASK		0x3
#define MVPP2_PRS_SRAM_OP_SEL_UDF_ADD		0
#define MVPP2_PRS_SRAM_OP_SEL_UDF_IP4_ADD	2
#define MVPP2_PRS_SRAM_OP_SEL_UDF_IP6_ADD	3
#define MVPP2_PRS_SRAM_OP_SEL_BASE_OFFS		89
#define MVPP2_PRS_SRAM_AI_OFFS			90
#define MVPP2_PRS_SRAM_AI_CTRL_OFFS		98
#define MVPP2_PRS_SRAM_AI_CTRL_BITS		8
#define MVPP2_PRS_SRAM_AI_MASK			0xff
#define MVPP2_PRS_SRAM_NEXT_LU_OFFS		106
#define MVPP2_PRS_SRAM_NEXT_LU_MASK		0xf
#define MVPP2_PRS_SRAM_LU_DONE_BIT		110
#define MVPP2_PRS_SRAM_LU_GEN_BIT		111

/* Sram result info bits assignment */
#define MVPP2_PRS_RI_MAC_ME_MASK		0x1
#define MVPP2_PRS_RI_DSA_MASK			0x2
#define MVPP2_PRS_RI_VLAN_MASK			0xc
#define MVPP2_PRS_RI_VLAN_NONE			~(BIT(2) | BIT(3))
#define MVPP2_PRS_RI_VLAN_SINGLE		BIT(2)
#define MVPP2_PRS_RI_VLAN_DOUBLE		BIT(3)
#define MVPP2_PRS_RI_VLAN_TRIPLE		(BIT(2) | BIT(3))
#define MVPP2_PRS_RI_CPU_CODE_MASK		0x70
#define MVPP2_PRS_RI_CPU_CODE_RX_SPEC		BIT(4)
#define MVPP2_PRS_RI_L2_CAST_MASK		0x600
#define MVPP2_PRS_RI_L2_UCAST			~(BIT(9) | BIT(10))
#define MVPP2_PRS_RI_L2_MCAST			BIT(9)
#define MVPP2_PRS_RI_L2_BCAST			BIT(10)
#define MVPP2_PRS_RI_PPPOE_MASK			0x800
#define MVPP2_PRS_RI_L3_PROTO_MASK		0x7000
#define MVPP2_PRS_RI_L3_UN			~(BIT(12) | BIT(13) | BIT(14))
#define MVPP2_PRS_RI_L3_IP4			BIT(12)
#define MVPP2_PRS_RI_L3_IP4_OPT			BIT(13)
#define MVPP2_PRS_RI_L3_IP4_OTHER		(BIT(12) | BIT(13))
#define MVPP2_PRS_RI_L3_IP6			BIT(14)
#define MVPP2_PRS_RI_L3_IP6_EXT			(BIT(12) | BIT(14))
#define MVPP2_PRS_RI_L3_ARP			(BIT(13) | BIT(14))
#define MVPP2_PRS_RI_L3_ADDR_MASK		0x18000
#define MVPP2_PRS_RI_L3_UCAST			~(BIT(15) | BIT(16))
#define MVPP2_PRS_RI_L3_MCAST			BIT(15)
#define MVPP2_PRS_RI_L3_BCAST			(BIT(15) | BIT(16))
#define MVPP2_PRS_RI_IP_FRAG_MASK		0x20000
#define MVPP2_PRS_RI_UDF3_MASK			0x300000
#define MVPP2_PRS_RI_UDF3_RX_SPECIAL		BIT(21)
#define MVPP2_PRS_RI_L4_PROTO_MASK		0x1c00000
#define MVPP2_PRS_RI_L4_TCP			BIT(22)
#define MVPP2_PRS_RI_L4_UDP			BIT(23)
#define MVPP2_PRS_RI_L4_OTHER			(BIT(22) | BIT(23))
#define MVPP2_PRS_RI_UDF7_MASK			0x60000000
#define MVPP2_PRS_RI_UDF7_IP6_LITE		BIT(29)
#define MVPP2_PRS_RI_DROP_MASK			0x80000000

/* Sram additional info bits assignment */
#define MVPP2_PRS_IPV4_DIP_AI_BIT		BIT(0)
#define MVPP2_PRS_IPV6_NO_EXT_AI_BIT		BIT(0)
#define MVPP2_PRS_IPV6_EXT_AI_BIT		BIT(1)
#define MVPP2_PRS_IPV6_EXT_AH_AI_BIT		BIT(2)
#define MVPP2_PRS_IPV6_EXT_AH_LEN_AI_BIT	BIT(3)
#define MVPP2_PRS_IPV6_EXT_AH_L4_AI_BIT		BIT(4)
#define MVPP2_PRS_SINGLE_VLAN_AI		0
#define MVPP2_PRS_DBL_VLAN_AI_BIT		BIT(7)

/* DSA/EDSA type */
#define MVPP2_PRS_TAGGED		true
#define MVPP2_PRS_UNTAGGED		false
#define MVPP2_PRS_EDSA			true
#define MVPP2_PRS_DSA			false

/* MAC entries, shadow udf */
enum mvpp2_prs_udf {
	MVPP2_PRS_UDF_MAC_DEF,
	MVPP2_PRS_UDF_MAC_RANGE,
	MVPP2_PRS_UDF_L2_DEF,
	MVPP2_PRS_UDF_L2_DEF_COPY,
	MVPP2_PRS_UDF_L2_USER,
};

/* Lookup ID */
enum mvpp2_prs_lookup {
	MVPP2_PRS_LU_MH,
	MVPP2_PRS_LU_MAC,
	MVPP2_PRS_LU_DSA,
	MVPP2_PRS_LU_VLAN,
	MVPP2_PRS_LU_L2,
	MVPP2_PRS_LU_PPPOE,
	MVPP2_PRS_LU_IP4,
	MVPP2_PRS_LU_IP6,
	MVPP2_PRS_LU_FLOWS,
	MVPP2_PRS_LU_LAST,
};

/* L3 cast enum */
enum mvpp2_prs_l3_cast {
	MVPP2_PRS_L3_UNI_CAST,
	MVPP2_PRS_L3_MULTI_CAST,
	MVPP2_PRS_L3_BROAD_CAST
};

/* Classifier constants */
#define MVPP2_CLS_FLOWS_TBL_SIZE	512
#define MVPP2_CLS_FLOWS_TBL_DATA_WORDS	3
#define MVPP2_CLS_LKP_TBL_SIZE		64



/* BM cookie (32 bits) definition */
#define MVPP2_BM_COOKIE_POOL_OFFS	8
#define MVPP2_BM_COOKIE_CPU_OFFS	24

/* BM short pool packet size
 * These value assure that for SWF the total number
 * of bytes allocated for each buffer will be 512
 */
#define MVPP2_BM_SHORT_PKT_SIZE		MVPP2_RX_MAX_PKT_SIZE(512)

enum mvpp2_bm_type {
	MVPP2_BM_FREE,
	MVPP2_BM_SWF_LONG,
	MVPP2_BM_SWF_SHORT
};


/* The mvpp2_tx_desc and mvpp2_rx_desc structures describe the
 * layout of the transmit and reception DMA descriptors, and their
 * layout is therefore defined by the hardware design
 */

#define MVPP2_TXD_L3_OFF_SHIFT		0
#define MVPP2_TXD_IP_HLEN_SHIFT		8
#define MVPP2_TXD_L4_CSUM_FRAG		BIT(13)
#define MVPP2_TXD_L4_CSUM_NOT		BIT(14)
#define MVPP2_TXD_IP_CSUM_DISABLE	BIT(15)
#define MVPP2_TXD_PADDING_DISABLE	BIT(23)
#define MVPP2_TXD_L4_UDP		BIT(24)
#define MVPP2_TXD_L3_IP6		BIT(26)
#define MVPP2_TXD_L_DESC		BIT(28)
#define MVPP2_TXD_F_DESC		BIT(29)

#define MVPP2_RXD_ERR_SUMMARY		BIT(15)
#define MVPP2_RXD_ERR_CODE_MASK		(BIT(13) | BIT(14))
#define MVPP2_RXD_ERR_CRC		0x0
#define MVPP2_RXD_ERR_OVERRUN		BIT(13)
#define MVPP2_RXD_ERR_RESOURCE		(BIT(13) | BIT(14))
#define MVPP2_RXD_BM_POOL_ID_OFFS	16
#define MVPP2_RXD_BM_POOL_ID_MASK	(BIT(16) | BIT(17) | BIT(18))
#define MVPP2_RXD_HWF_SYNC		BIT(21)
#define MVPP2_RXD_L4_CSUM_OK		BIT(22)
#define MVPP2_RXD_IP4_HEADER_ERR	BIT(24)
#define MVPP2_RXD_L4_TCP		BIT(25)
#define MVPP2_RXD_L4_UDP		BIT(26)
#define MVPP2_RXD_L3_IP4		BIT(28)
#define MVPP2_RXD_L3_IP6		BIT(30)
#define MVPP2_RXD_BUF_HDR		BIT(31)

struct mvpp2_tx_desc {
	u32 command;		/* Options used by HW for packet transmitting.*/
	u8  packet_offset;	/* the offset from the buffer beginning	*/
	u8  phys_txq;		/* destination queue ID			*/
	u16 data_size;		/* data size of transmitted packet in bytes */
	u32 buf_phys_addr;	/* physical addr of transmitted buffer	*/
	u32 buf_cookie;		/* cookie for access to TX buffer in tx path */
	u32 reserved1[3];	/* hw_cmd (for future use, BM, PON, PNC) */
	u32 reserved2;		/* reserved (for future use)		*/
};

struct mvpp2_rx_desc {
	u32 status;		/* info about received packet		*/
	u16 reserved1;		/* parser_info (for future use, PnC)	*/
	u16 data_size;		/* size of received packet in bytes	*/
	u32 buf_phys_addr;	/* physical address of the buffer	*/
	u32 buf_cookie;		/* cookie for access to RX buffer in rx path */
	u16 reserved2;		/* gem_port_id (for future use, PON)	*/
	u16 reserved3;		/* csum_l4 (for future use, PnC)	*/
	u8  reserved4;		/* bm_qset (for future use, BM)		*/
	u8  reserved5;
	u16 reserved6;		/* classify_info (for future use, PnC)	*/
	u32 reserved7;		/* flow_id (for future use, PnC) */
	u32 reserved8;
};


union mvpp2_prs_tcam_entry {
	u32 word[MVPP2_PRS_TCAM_WORDS];
	u8  byte[MVPP2_PRS_TCAM_WORDS * 4];
};

union mvpp2_prs_sram_entry {
	u32 word[MVPP2_PRS_SRAM_WORDS];
	u8  byte[MVPP2_PRS_SRAM_WORDS * 4];
};

struct mvpp2_prs_entry {
	u32 index;
	union mvpp2_prs_tcam_entry tcam;
	union mvpp2_prs_sram_entry sram;
};

struct mvpp2_prs_shadow {
	bool valid;
	bool finish;

	/* Lookup ID */
	int lu;

	/* User defined offset */
	int udf;

	/* Result info */
	u32 ri;
	u32 ri_mask;
};

struct mvpp2_cls_flow_entry {
	u32 index;
	u32 data[MVPP2_CLS_FLOWS_TBL_DATA_WORDS];
};

struct mvpp2_cls_lookup_entry {
	u32 lkpid;
	u32 way;
	u32 data;
};

struct mvpp2_bm_pool {
	/* Pool number in the range 0-7 */
	int id;
	enum mvpp2_bm_type type;

	/* Buffer Pointers Pool External (BPPE) size */
	int size;
	/* Number of buffers for this pool */
	int buf_num;
	/* Pool buffer size */
	int buf_size;
	/* Packet size */
	int pkt_size;

	/* BPPE virtual base address */
	u32 *virt_addr;
	/* BPPE physical base address */
	dma_addr_t phys_addr;

	/* Ports using BM pool */
	u32 port_map;

	/* Occupied buffers indicator */
	atomic_t in_use;
	int in_use_thresh;

	spinlock_t lock;
};

struct mvpp2_buff_hdr {
	u32 next_buff_phys_addr;
	u32 next_buff_virt_addr;
	u16 byte_count;
	u16 info;
	u8  reserved1;		/* bm_qset (for future use, BM)		*/
};

/* Buffer header info bits */
#define MVPP2_B_HDR_INFO_MC_ID_MASK	0xfff
#define MVPP2_B_HDR_INFO_MC_ID(info)	((info) & MVPP2_B_HDR_INFO_MC_ID_MASK)
#define MVPP2_B_HDR_INFO_LAST_OFFS	12
#define MVPP2_B_HDR_INFO_LAST_MASK	BIT(12)
#define MVPP2_B_HDR_INFO_IS_LAST(info) \
	   ((info & MVPP2_B_HDR_INFO_LAST_MASK) >> MVPP2_B_HDR_INFO_LAST_OFFS)

#endif /*_MVPP2_HW_TYPE_H_*/

