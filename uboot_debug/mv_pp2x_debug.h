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

#include <common.h>
#include <net.h>
#include <netdev.h>
#include <config.h>
#include <malloc.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <phy.h>
#include <miiphy.h>
#include <watchdog.h>
#include <linux/compat.h>
#include <linux/mbus.h>
#include <fdtdec.h>
#include <asm/arch-mvebu/fdt.h>
#include <pci.h>

#define MVPP2_PRS_TCAM_ENTRY_VALID	0
/*PPv2.1 MASS 3.20 new feature */
#define MVPP2_PRS_TCAM_HIT_IDX_REG		0x1240
/*-------------------------------------------------------------------------------*/
/*PPv2.1 MASS 3.20 new feature */
#define MVPP2_PRS_TCAM_HIT_CNT_REG		0x1244
#define MVPP2_PRS_TCAM_HIT_CNT_BITS		16
#define MVPP2_PRS_TCAM_HIT_CNT_OFFS		0
#define MVPP2_PRS_TCAM_HIT_CNT_MASK		\
	(((1 << MVPP2_PRS_TCAM_HIT_CNT_BITS) - 1) << MVPP2_PRS_TCAM_HIT_CNT_OFFS)

#define PRS_SRAM_FMT					"%4.4x %8.8x %8.8x %8.8x"
#define PRS_SRAM_VAL(p)					p[3] & 0xFFFF, p[2], p[1], p[0]

#define MVPP2_CNT_IDX_REG			0x7040
/* LKP counters index */
#define MVPP2_CNT_IDX_LKP(lkp, way)		((way) << 6 | (lkp))
/* Flow counters index */
#define MVPP2_CNT_IDX_FLOW(index)		(index)
/* TX counters index */
#define MVPP2_CNT_IDX_TX(port, txq)		(((16+port) << 3) | (txq))
/* Classifier Registers */
#define MVPP2_CLS_MODE_REG			0x1800
#define MVPP2_CLS_MODE_ACTIVE_MASK		BIT(0)
#define MVPP2_CLS_PORT_WAY_REG			0x1810
#define MVPP2_CLS_PORT_WAY_MASK(port)		(1 << (port))
#define MVPP2_CLS_LKP_INDEX_REG			0x1814
#define MVPP2_CLS_LKP_INDEX_WAY_OFFS		6
#define MVPP2_CLS_LKP_INDEX_LKP_OFFS		0
#define MVPP2_CLS_LKP_TBL_REG			0x1818
#define MVPP2_CLS_LKP_TBL_RXQ_MASK		0xff
#define MVPP2_CLS_LKP_TBL_LOOKUP_EN_MASK	BIT(25)
#define MVPP22_CLS_LKP_TBL_SEL_REG		0x181c
#define MVPP22_CLS_LKP_TBL_SEL_CDT_MASK		BIT(0)
#define MVPP22_CLS_LKP_TBL_SEL_FDT_MASK		BIT(1)
#define MVPP2_CLS_FLOW_INDEX_REG		0x1820
#define MVPP2_CLS_FLOW_TBL0_REG			0x1824
#define MVPP2_CLS_FLOW_TBL1_REG			0x1828
#define MVPP2_CLS_FLOW_TBL2_REG			0x182c

/* lkpid table structure	*/
#define MVPP2_FLOWID_RXQ		0
#define MVPP2_FLOWID_RXQ_BITS		8
#define MVPP2_FLOWID_RXQ_MASK		(((1 << MVPP2_FLOWID_RXQ_BITS) - 1) << MVPP2_FLOWID_RXQ)

#define MVPP2_FLOWID_MODE		8
#define MVPP2_FLOWID_MODE_BITS		8
#define MVPP2_FLOWID_MODE_MASK		(((1 << MVPP2_FLOWID_MODE_BITS) - 1) << MVPP2_FLOWID_MODE)
#define MVPP2_FLOWID_MODE_MAX		((1 << MVPP2_FLOWID_MODE_BITS) - 1)

#define MVPP2_FLOWID_FLOW		16
#define MVPP2_FLOWID_FLOW_BITS		9
#define MVPP2_FLOWID_FLOW_MASK		(((1 << MVPP2_FLOWID_FLOW_BITS) - 1) << MVPP2_FLOWID_FLOW)

#define MVPP2_FLOWID_EN			25 /*one bit */
#define MVPP2_FLOWID_EN_MASK		(1 << MVPP2_FLOWID_EN)

#define MVPP2_CLS_LKP_TBL_HIT_REG		0x7700

/*************/
/*   MIB  REGS    */
/*************/

#define MV_MIB_PORT_OFFSET(port)		((port) * 0x400)
#define MV_MIB_COUNTERS_BASE(port)		(MV_MIB_PORT_OFFSET(port))

/* GMAC_MIB Counters register definitions */
#define MV_MIB_GOOD_OCTETS_RECEIVED_LOW		0x0
#define MV_MIB_GOOD_OCTETS_RECEIVED_HIGH	0x4
#define MV_MIB_BAD_OCTETS_RECEIVED		0x8
#define MV_MIB_CRC_ERRORS_SENT			0xc
#define MV_MIB_UNICAST_FRAMES_RECEIVED		0x10
/* Reserved					0x14 */
#define MV_MIB_BROADCAST_FRAMES_RECEIVED	0x18
#define MV_MIB_MULTICAST_FRAMES_RECEIVED	0x1c
#define MV_MIB_FRAMES_64_OCTETS			0x20
#define MV_MIB_FRAMES_65_TO_127_OCTETS		0x24
#define MV_MIB_FRAMES_128_TO_255_OCTETS		0x28
#define MV_MIB_FRAMES_256_TO_511_OCTETS		0x2c
#define MV_MIB_FRAMES_512_TO_1023_OCTETS	0x30
#define MV_MIB_FRAMES_1024_TO_MAX_OCTETS	0x34
#define MV_MIB_GOOD_OCTETS_SENT_LOW		0x38
#define MV_MIB_GOOD_OCTETS_SENT_HIGH		0x3c
#define MV_MIB_UNICAST_FRAMES_SENT		0x40
/* Reserved					0x44 */
#define MV_MIB_MULTICAST_FRAMES_SENT		0x48
#define MV_MIB_BROADCAST_FRAMES_SENT		0x4c
/* Reserved					0x50 */
#define MV_MIB_FC_SENT				0x54
#define MV_MIB_FC_RECEIVED			0x58
#define MV_MIB_RX_FIFO_OVERRUN			0x5c
#define MV_MIB_UNDERSIZE_RECEIVED		0x60
#define MV_MIB_FRAGMENTS_RECEIVED		0x64
#define MV_MIB_OVERSIZE_RECEIVED		0x68
#define MV_MIB_JABBER_RECEIVED			0x6c
#define MV_MIB_MAC_RECEIVE_ERROR		0x70
#define MV_MIB_BAD_CRC_EVENT			0x74
#define MV_MIB_COLLISION			0x78
#define MV_MIB_LATE_COLLISION			0x7c

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

/* The mvpp2_tx_desc and mvpp2_rx_desc structures describe the
 * layout of the transmit and reception DMA descriptors, and their
 * layout is therefore defined by the hardware design
 */

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
/* Sub fields of "parserInfo" field */
#define MVPP2_RXD_LKP_ID_OFFS		0
#define MVPP2_RXD_LKP_ID_BITS		6
#define MVPP2_RXD_LKP_ID_MASK		(((1 << MVPP2_RXD_LKP_ID_BITS) - 1) << MVPP2_RXD_LKP_ID_OFFS)
#define MVPP2_RXD_CPU_CODE_OFFS		6
#define MVPP2_RXD_CPU_CODE_BITS		3
#define MVPP2_RXD_CPU_CODE_MASK		(((1 << MVPP2_RXD_CPU_CODE_BITS) - 1) << MVPP2_RXD_CPU_CODE_OFFS)
#define MVPP2_RXD_PPPOE_BIT		9
#define MVPP2_RXD_PPPOE_MASK		(1 << MVPP2_RXD_PPPOE_BIT)
#define MVPP2_RXD_L3_CAST_OFFS		10
#define MVPP2_RXD_L3_CAST_BITS		2
#define MVPP2_RXD_L3_CAST_MASK		(((1 << MVPP2_RXD_L3_CAST_BITS) - 1) << MVPP2_RXD_L3_CAST_OFFS)
#define MVPP2_RXD_L2_CAST_OFFS		12
#define MVPP2_RXD_L2_CAST_BITS		2
#define MVPP2_RXD_L2_CAST_MASK		(((1 << MVPP2_RXD_L2_CAST_BITS) - 1) << MVPP2_RXD_L2_CAST_OFFS)
#define MVPP2_RXD_VLAN_INFO_OFFS	14
#define MVPP2_RXD_VLAN_INFO_BITS	2
#define MVPP2_RXD_VLAN_INFO_MASK	(((1 << MVPP2_RXD_VLAN_INFO_BITS) - 1) << MVPP2_RXD_VLAN_INFO_OFFS)
/* Bits of "bmQset" field */
#define MVPP2_RXD_BUFF_QSET_NUM_OFFS	0
#define MVPP2_RXD_BUFF_QSET_NUM_MASK	(0x7f << MVPP2_RXD_BUFF_QSET_NUM_OFFS)
#define MVPP2_RXD_BUFF_TYPE_OFFS	7
#define MVPP2_RXD_BUFF_TYPE_MASK	(0x1 << MVPP2_RXD_BUFF_TYPE_OFFS)
/* Bits of "status" field */
#define MVPP2_RXD_L3_OFFSET_OFFS	0
#define MVPP2_RXD_L3_OFFSET_MASK	(0x7F << MVPP2_RXD_L3_OFFSET_OFFS)
#define MVPP2_RXD_IP_HLEN_OFFS		8
#define MVPP2_RXD_IP_HLEN_MASK		(0x1F << MVPP2_RXD_IP_HLEN_OFFS)
#define MVPP2_RXD_ES_BIT		15
#define MVPP2_RXD_ES_MASK		(1 << MVPP2_RXD_ES_BIT)
#define MVPP2_RXD_HWF_SYNC_BIT		21
#define MVPP2_RXD_HWF_SYNC_MASK		(1 << MVPP2_RXD_HWF_SYNC_BIT)
#define MVPP2_RXD_L4_CHK_OK_BIT		22
#define MVPP2_RXD_L4_CHK_OK_MASK	(1 << MVPP2_RXD_L4_CHK_OK_BIT)
#define MVPP2_RXD_IP_FRAG_BIT		23
#define MVPP2_RXD_IP_FRAG_MASK		(1 << MVPP2_RXD_IP_FRAG_BIT)
#define MVPP2_RXD_IP4_HEADER_ERR_BIT	24
#define MVPP2_RXD_IP4_HEADER_ERR_MASK	(1 << MVPP2_RXD_IP4_HEADER_ERR_BIT)
#define MVPP2_RXD_L4_OFFS		25
#define MVPP2_RXD_L4_MASK		(7 << MVPP2_RXD_L4_OFFS)
/* Value 0 - N/A, 3-7 - User Defined */
#define MVPP2_RXD_L3_OFFS		28
#define MVPP2_RXD_L3_MASK		(7 << MVPP2_RXD_L3_OFFS)
/* Value 0 - N/A, 6-7 - User Defined */
#define MVPP2_RXD_L3_IP4_OPT		(2 << MVPP2_RXD_L3_OFFS)
#define MVPP2_RXD_L3_IP4_OTHER		(3 << MVPP2_RXD_L3_OFFS)
#define MVPP2_RXD_L3_IP6_EXT		(5 << MVPP2_RXD_L3_OFFS)
#define MVPP2_RXD_BUF_HDR_BIT		31
#define MVPP2_RXD_BUF_HDR_MASK		(1 << MVPP2_RXD_BUF_HDR_BIT)
/* status field MACROs */
#define MVPP2_RXD_L3_IS_IP4(status)		(((status) & MVPP2_RXD_L3_MASK) == MVPP2_RXD_L3_IP4)
#define MVPP2_RXD_L3_IS_IP4_OPT(status)		(((status) & MVPP2_RXD_L3_MASK) == MVPP2_RXD_L3_IP4_OPT)
#define MVPP2_RXD_L3_IS_IP4_OTHER(status)	(((status) & MVPP2_RXD_L3_MASK) == MVPP2_RXD_L3_IP4_OTHER)
#define MVPP2_RXD_L3_IS_IP6(status)		(((status) & MVPP2_RXD_L3_MASK) == MVPP2_RXD_L3_IP6)
#define MVPP2_RXD_L3_IS_IP6_EXT(status)		(((status) & MVPP2_RXD_L3_MASK) == MVPP2_RXD_L3_IP6_EXT)
#define MVPP2_RXD_L4_IS_UDP(status)		(((status) & MVPP2_RXD_L4_MASK) == MVPP2_RXD_L4_UDP)
#define MVPP2_RXD_L4_IS_TCP(status)		(((status) & MVPP2_RXD_L4_MASK) == MVPP2_RXD_L4_TCP)
#define MVPP2_RXD_IP4_HDR_ERR(status)		((status) & MVPP2_RXD_IP4_HEADER_ERR_MASK)
#define MVPP2_RXD_IP4_FRG(status)		((status) & MVPP2_RXD_IP_FRAG_MASK)
#define MVPP2_RXD_L4_CHK_OK(status)		((status) & MVPP2_RXD_L4_CHK_OK_MASK)

struct dummy_packet {
	u32 mac_addr[3];
	u16 type;
	u8 data_dummy[80];

};

void read_mibs(int port_id);

