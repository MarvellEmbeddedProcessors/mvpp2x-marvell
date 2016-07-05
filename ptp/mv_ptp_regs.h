/*
* ***************************************************************************
* Copyright (C) 2016 Marvell International Ltd.
* ***************************************************************************
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
* ***************************************************************************
*/

#ifndef _mv_ptp_regs_h_
#define _mv_ptp_regs_h_

/* Low/Hw level REGISTER definition files mv_ptp_regs.h and mv_tai_regs.h
 * are used for TAI and PTP HW register access in KERNEL,
 * but the same hw-units are memory-mapped over UIO device
 * for USER-Space PTP application (work like a PCI-card application)
 * The PTP and TAI hw-units have the same structure, logic and mem-offsets
 * for different platforms.
 * That is the reason these .H have #ifdef:
 *  #ifdef __KERNEL__
 *  #ifdef ARMADA_390
 * and also Reg-BASE onto whole 4k page
 */

#ifdef __KERNEL__
/* includes */
#ifdef ARMADA_390
#include "common/mv_hw_if.h"
#include "gop/a390_mg_if.h"
#else
#include <linux/io.h>
#endif
#else
/* This "mv_*_regs.h is also included in User-space UIO */
#endif

#ifdef ARMADA_390
/* unit offset (there are TAI & PTP which are the one same HW) */
#define MV_PP3_PTP_TAI_UNIT_OFFSET		0x03180000
#define MV_PTP_UNIT_OFFSET	MV_PP3_PTP_TAI_UNIT_OFFSET
#define MV_PTP_VALID_PORTS	0x0f /* bit-map ports 0,1,2,3 < MV_EMAC_NUM */
#else
/* port0: 0x130800..0874; port1:0x131800..1874; 0x132800..2874; 0x133800..3874
 * mapped over __iomem  mv_ptp_base
*/
#define MV_PTP_UNIT_OFFSET	0x130000 /* for debug address-print only */
#define MV_PTP_VALID_PORTS	0x0d /* bit-map ports 0,-,2,3 < MV_EMAC_NUM */
#endif

#define MV_PTP_PORT_IS_VALID(port)	((1 << port) & MV_PTP_VALID_PORTS)
#define MV_PTP_PORT_BASE(port)	(port * 0x1000)

/***** {{ Common Platform independent Register access definitions ************/
/* Ptp Interrupt Cause (Timestamp status) */
#define MV_PTP_INT_STATUS_TS_REG(port)	(port * 0x1000 + 0x0800)
#define MV_PTP_INT_STATUS_TS_Q1_FULL_OFFS		8
#define MV_PTP_INT_STATUS_TS_Q0_FULL_OFFS		7
#define MV_PTP_INT_STATUS_TS_Q1_NEW_OFFS		6
#define MV_PTP_INT_STATUS_TS_Q0_NEW_OFFS		5
#define MV_PTP_INT_STATUS_TS_Q1_RX_WR_ERR_OFFS		4
#define MV_PTP_INT_STATUS_TS_Q0_RX_RD_ERR_OFFS		3
#define MV_PTP_INT_STATUS_TS_Q1_RX_FULL_OFFS		2
#define MV_PTP_INT_STATUS_TS_Q0_RX_FULL_OFFS		1

#define MV_PTP_INT_STATUS_TS_Q1_FULL_MASK      (1 << MV_PTP_INT_STATUS_TS_Q1_FULL_OFFS)
#define MV_PTP_INT_STATUS_TS_Q0_FULL_MASK      (1 << MV_PTP_INT_STATUS_TS_Q0_FULL_OFFS)
#define MV_PTP_INT_STATUS_TS_Q1_NEW_MASK       (1 << MV_PTP_INT_STATUS_TS_Q1_NEW_OFFS)
#define MV_PTP_INT_STATUS_TS_Q0_NEW_MASK       (1 << MV_PTP_INT_STATUS_TS_Q0_NEW_OFFS)
#define MV_PTP_INT_STATUS_TS_Q1_RX_WR_ERR_MASK (1 << MV_PTP_INT_STATUS_TS_Q1_RX_WR_ERR_OFFS)
#define MV_PTP_INT_STATUS_TS_Q0_RX_RD_ERR_MASK (1 << MV_PTP_INT_STATUS_TS_Q0_RX_RD_ERR_OFFS)
#define MV_PTP_INT_STATUS_TS_Q1_RX_FULL_MASK   (1 << MV_PTP_INT_STATUS_TS_Q1_RX_FULL_OFFS)
#define MV_PTP_INT_STATUS_TS_Q0_RX_FULL_MASK   (1 << MV_PTP_INT_STATUS_TS_Q0_RX_FULL_OFFS)

/* Tai Interrupt Cause */
#define MV_PTP_INTR_CAUSE_REG(port)	(port * 0x1000 + 0x0000)
/* Tai Interrupt Mask */
#define MV_PTP_INTR_MASK_REG(port)	(port * 0x1000 + 0x0004)

/* Ptp General Control */
#define MV_PTP_GENERAL_CTRL_REG(port)			(port * 0x1000 + 0x0808)
#define MV_PTP_GENERAL_CTRL_PTP_UNIT_ENABLE_MASK		(0x0001 << 0)
#define MV_PTP_GENERAL_CTRL_PTP_RESET_TX_MASK			(0x0001 << 1)
#define MV_PTP_GENERAL_CTRL_INTERFACE_WIDTH_SELECT_MASK	(0x0003 << 2)
#define MV_PTP_GENERAL_CTRL_CLEAR_COUNTERS_MASK		(0x0001 << 4)
#define MV_PTP_GENERAL_CTRL_TAI_SELECT_MASK			(0x0001 << 5)
#define MV_PTP_GENERAL_CTRL_TS_QUEUE_OVER_WRITE_ENABLE_MASK	(0x0001 << 6)
#define MV_PTP_GENERAL_CTRL_TAI_ACK_DELAY_MASK			(0x001f << 7)
#define MV_PTP_GENERAL_CTRL_TAI_ACK_DELAY_TX_MASK		(0x0007 << 7)
#define MV_PTP_GENERAL_CTRL_TAI_ACK_DELAY_RX_MASK		(0x0007 << 10)
#define MV_PTP_GENERAL_CTRL_PTP_RESET_RX_MASK			(0x0001 << 13)
#define MV_PTP_GENERAL_CTRL_RX_HW_OVERRUN_RST_ENA_MASK		(0x0001 << 14)

#define MV_PTP_GENERAL_CTRL_PTP_RESET_MASK \
	(MV_PTP_GENERAL_CTRL_PTP_RESET_TX_MASK | MV_PTP_GENERAL_CTRL_PTP_RESET_RX_MASK)


/* Ptp Tx Timestamp Queue0 Reg0 */
#define MV_PTP_TX_TIMESTAMP_QUEUE0_REG0_REG(port)	(port * 0x1000 + 0x080c)
#define MV_PTP_TX_TIMESTAMP_QUEUE0_REG0_PTP_TX_TIMESTAMP_QUEUE0_VALID_MASK \
		(0x00000001 << 0)
#define MV_PTP_TX_TIMESTAMP_QUEUE0_REG0_QUEUE_ID_MASK			(0x03ff << 1)
#define MV_PTP_TX_TIMESTAMP_QUEUE0_REG0_TAI_SELECT_MASK		(0x0001 << 11)
#define MV_PTP_TX_TIMESTAMP_QUEUE0_REG0_TOD_UPDATE_FLAG_MASK	(0x0001 << 12)
#define MV_PTP_TX_TIMESTAMP_QUEUE0_REG0_TIMESTAMP_BITS_0_2_MASK	(0x0007 << 13)

/* Ptp Tx Timestamp Queue0 Reg1 */
#define MV_PTP_TX_TIMESTAMP_QUEUE0_REG1_REG(port)	(port * 0x1000 + 0x0810)
#define MV_PTP_TX_TIMESTAMP_QUEUE0_REG1_TIMESTAMP_BITS_3_18_MASK (0x0000ffff << 0)

/* Ptp Tx Timestamp Queue0 Reg2 */
#define MV_PTP_TX_TIMESTAMP_QUEUE0_REG2_REG(port)	(port * 0x1000 + 0x0814)
#define MV_PTP_TX_TIMESTAMP_QUEUE0_REG2_TIMESTAMP_BITS_19_31_MASK (0x00001fff << 0)

/* Ptp Tx Timestamp Queue1 Reg0 */
#define MV_PTP_TX_TIMESTAMP_QUEUE1_REG0_REG(port)	(port * 0x1000 + 0x0818)
#define MV_PTP_TX_TIMESTAMP_QUEUE1_REG0_PTP_TX_TIMESTAMP_QUEUE1_VALID_MASK \
		(0x00000001 << 0)
#define MV_PTP_TX_TIMESTAMP_QUEUE1_REG0_QUEUE_ID_MASK			(0x03ff << 1)
#define MV_PTP_TX_TIMESTAMP_QUEUE1_REG0_TAI_SELECT_MASK			(0x0001 << 11)
#define MV_PTP_TX_TIMESTAMP_QUEUE1_REG0_TOD_UPDATE_FLAG_MASK	(0x0001 << 12)
#define MV_PTP_TX_TIMESTAMP_QUEUE1_REG0_TIMESTAMP_BITS_0_2_MASK	(0x0007 << 13)

/* Ptp Tx Timestamp Queue1 Reg1 */
#define MV_PTP_TX_TIMESTAMP_QUEUE1_REG1_REG(port)	(port * 0x1000 + 0x081c)
#define MV_PTP_TX_TIMESTAMP_QUEUE1_REG1_TIMESTAMP_BITS_3_18_MASK (0x0000ffff << 0)

/* Ptp Tx Timestamp Queue1 Reg2 */
#define MV_PTP_TX_TIMESTAMP_QUEUE1_REG2_REG(port)	(port * 0x1000 + 0x0820)
#define MV_PTP_TX_TIMESTAMP_QUEUE1_REG2_TIMESTAMP_BITS_19_31_MASK (0x00001fff << 0)


/* Total Ptp Packets Counter */
#define MV_PTP_TOTAL_PTP_PCKTS_CNTR_REG(port)	(port * 0x1000 + 0x0824)
#define MV_PTP_TOTAL_PTP_PCKTS_CNTR_TOTAL_PTP_PACKETS_COUNTER_MASK	0x00ff

/* Ptpv1 Packet Counter */
#define MV_PTP_PTPV1_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0828)
#define MV_PTP_PTPV1_PCKT_CNTR_PTPV1_PACKET_COUNTER_OFFS		0
#define MV_PTP_PTPV1_PCKT_CNTR_PTPV1_PACKET_COUNTER_MASK	0x00ff

/* Ptpv2 Packet Counter */
#define MV_PTP_PTPV2_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x082c)
#define MV_PTP_PTPV2_PCKT_CNTR_PTPV2_PACKET_COUNTER_MASK	0x00ff

/* Y1731 Packet Counter */
#define MV_PTP_Y1731_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0830)
#define MV_PTP_Y1731_PCKT_CNTR_Y1731_PACKET_COUNTER_MASK	0x00ff

/* Ntpts Packet Counter */
#define MV_PTP_NTPTS_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0834)
#define MV_PTP_NTPTS_PCKT_CNTR_NTPTS_PACKET_COUNTER_MASK	0x00ff
	
/* Ntpreceive Packet Counter */
#define MV_PTP_NTPRECEIVE_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0838)
#define MV_PTP_NTPRECEIVE_PCKT_CNTR_NTPRX_PACKET_COUNTER_MASK	0x00ff

/* Ntptransmit Packet Counter */
#define MV_PTP_NTPTRANSMIT_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x083c)
#define MV_PTP_NTPTRANSMIT_PCKT_CNTR_NTPTX_PACKET_COUNTER_MASK	0x00ff

/* Wamp Packet Counter */
#define MV_PTP_WAMP_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0840)
#define MV_PTP_WAMP_PCKT_CNTR_WAMP_PACKET_COUNTER_MASK	0x00ff

/* None Action Packet Counter */
#define MV_PTP_NONE_ACTION_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0844)
#define MV_PTP_NONE_ACTION_PCKT_CNTR_NONE_ACTION_PACKET_COUNTER_MASK	0x00ff

/* Forward Action Packet Counter */
#define MV_PTP_FORWARD_ACTION_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0848)
#define MV_PTP_FORWARD_ACTION_PCKT_CNTR_FORWARD_ACTION_PACKET_COUNTER_MASK	0x00ff

/* Drop Action Packet Counter */
#define MV_PTP_DROP_ACTION_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x084c)
#define MV_PTP_DROP_ACTION_PCKT_CNTR_DROP_ACTION_PACKET_COUNTER_OFFS		0
#define MV_PTP_DROP_ACTION_PCKT_CNTR_DROP_ACTION_PACKET_COUNTER_MASK	0x00ff

/* Capture Action Packet Counter */
#define MV_PTP_CAPTURE_ACTION_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0850)
#define MV_PTP_CAPTURE_ACTION_PCKT_CNTR_CAPTURE_ACTION_PACKET_COUNTER_MASK	0x00ff

/* Addtime Action Packet Counter */
#define MV_PTP_ADDTIME_ACTION_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0854)
#define MV_PTP_ADDTIME_ACTION_PCKT_CNTR_ADDTIME_ACTION_PACKET_COUNTER_MASK	0x00ff

/* Addcorrectedtime Action Packet Counter */
#define MV_PTP_ADDCORRECTEDTIME_ACTION_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0858)
#define MV_PTP_ADDCORRECTEDTIME_ACTION_PACKET_COUNTER_MASK	0x00ff

/* Captureaddtime Action Packet Counter */
#define MV_PTP_CAPTUREADDTIME_ACTION_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x085c)
#define MV_PTP_CAPTUREADDTIME_ACTION_PACKET_COUNTER_MASK	0x00ff

/* Captureaddcorrectedtime Action Packet Counter */
#define MV_PTP_CAPTUREADDCORRECTEDTIME_ACTION_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0860)
#define MV_PTP_CAPTUREADDCORRECTEDTIME_ACTION_PACKET_COUNTER_MASK	0x00ff

/* Addingresstime Action Packet Counter */
#define MV_PTP_ADDINGRESSTIME_ACTION_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0864)
#define MV_PTP_ADDINGRESSTIME_ACTION_PACKET_COUNTER_MASK		0x00ff

/* Captureaddingresstime Action Packet Counter */
#define MV_PTP_CAPTUREADDINGRESSTIME_ACTION_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x0868)
#define MV_PTP_CAPTUREADDINGRESSTIME_ACTION_PACKET_COUNTER_MASK		0x00ff

/* Captureingresstime Action Packet Counter */
#define MV_PTP_CAPTUREINGRESSTIME_ACTION_PCKT_CNTR_REG(port)	(port * 0x1000 + 0x086c)
#define MV_PTP_CAPTUREINGRESSTIME_ACTION_PACKET_COUNTER_MASK	0x00ff


/* Ntp Ptp Offset High */
#define MV_PTP_NTP_PTP_OFFSET_HIGH_REG(port)	(port * 0x1000 + 0x0870)
#define MV_PTP_NTP_PTP_OFFSET_HIGH_PTP_NTP_OFFSET_HIGH_MASK		0x0000ffff


/* Ntp Ptp Offset Low */
#define MV_PTP_NTP_PTP_OFFSET_LOW_REG(port)	(port * 0x1000 + 0x0874)
#define MV_PTP_NTP_PTP_OFFSET_LOW_PTP_NTP_OFFSET_LOW_MASK	0x0000ffff

/***   }} Common Platform independent Register access definitions           ***/
/******************************************************************************/

#ifdef __KERNEL__
#ifdef ARMADA_390
/* Indirect mapping over mv_gop_reg_read/write */
static inline u32 mv_ptp_reg_read(u32 reg_addr)
{
	return mv_gop_reg_read(reg_addr + MV_PTP_UNIT_OFFSET);
}

static inline void mv_ptp_reg_write(u32 reg_addr, u32 reg_data)
{
	return mv_gop_reg_write(reg_addr + MV_PTP_UNIT_OFFSET, reg_data);
}
#else
/* Direct mapping over VALUE (not ptr) phys_addr_t ~ u32 or u64  */
extern phys_addr_t mv_ptp_base;

static inline u32 mv_ptp_reg_read(u32 reg_addr)
{
	unsigned long *reg_ptr = (unsigned long *)(mv_ptp_base + reg_addr);

	return readl(reg_ptr);
}

static inline void mv_ptp_reg_write(u32 reg_addr, u32 reg_data)
{
	unsigned long *reg_ptr = (unsigned long *)(mv_ptp_base + reg_addr);

	writel(reg_data, reg_ptr);
}
#endif/*ARMADA_390*/

static inline void mv_ptp_reg_print(char *reg_name, u32 reg)
{
	pr_info(" %-42s: 0x%x = %08x\n", reg_name,
		MV_PTP_UNIT_OFFSET + reg, mv_ptp_reg_read(reg));
}

#endif/*__KERNEL__*/
#endif /* _mv_ptp_regs_h_ */
