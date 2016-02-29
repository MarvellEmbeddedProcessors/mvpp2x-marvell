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

#include "mv_pp2x.h"
#include "mv_pp2x_debug.h"

void read_mibs(int port_id)
{
	int val = 0;
	void *addr;
	void *gop_110_xmib_base = (void *)(CPN110_BASE_ADDR + 0x129000);
	printf("Start:\n");
	printf("\n[Rx]\n");
	addr = gop_110_xmib_base + port_id * 0x100 + MV_MIB_GOOD_OCTETS_RECEIVED_LOW;
	val   = readl(addr);
	printf("GOOD_OCTETS_RECEIVED			=%d\n", val);

	addr = gop_110_xmib_base + port_id * 0x100 + MV_MIB_BAD_OCTETS_RECEIVED;
	val   = readl(addr);
	printf("BAD_OCTETS_RECEIVED		        =%d\n", val);

	addr = gop_110_xmib_base + port_id * 0x100 + MV_MIB_UNICAST_FRAMES_RECEIVED;
	printf("UNCAST_FRAMES_RECEIVED	        =%d\n", val);

	addr = gop_110_xmib_base + port_id * 0x100 + MV_MIB_BROADCAST_FRAMES_RECEIVED;
	val  = readl(addr);
	printf("BROADCAST_FRAMES_RECEIVED       =%d\n", val);

	addr = gop_110_xmib_base + port_id * 0x100 + MV_MIB_MULTICAST_FRAMES_RECEIVED;
	val  = readl(addr);
	printf("MULTICAST_FRAMES_RECEIVED		=%d\n", val);

	printf("\n[Tx]\n");
	addr = gop_110_xmib_base + port_id * 0x100 + MV_MIB_GOOD_OCTETS_SENT_LOW;
	val  = readl(addr);
	printf("GOOD_OCTETS_SENT                =%d\n", val);

	addr = gop_110_xmib_base + port_id * 0x100 + MV_MIB_UNICAST_FRAMES_SENT;
	val  = readl(addr);
	printf("UNICAST_FRAMES_SENT		        =%d\n", val);

	addr = gop_110_xmib_base + port_id * 0x100 + MV_MIB_MULTICAST_FRAMES_SENT;
	val  = readl(addr);
	printf("MULTICAST_FRAMES_SENT           =%d\n", val);

	addr = gop_110_xmib_base + port_id * 0x100 + MV_MIB_BROADCAST_FRAMES_SENT;
	val  = readl(addr);
	printf("BROADCAST_FRAMES_SENT           =%d\n", val);

	addr = gop_110_xmib_base + port_id * 0x100 + MV_MIB_CRC_ERRORS_SENT;
	val  = readl(addr);
	printf("CRC_ERRORS_SENT			        =%d\n", val);

	addr = gop_110_xmib_base + port_id * 0x100 + MV_MIB_LATE_COLLISION;
	val  = readl(addr);
	printf("LATE_COLLISION			        =%d\n", val);

}

int mv_pp2x_prs_sram_bit_get(struct mv_pp2x_prs_entry *pe, int bitNum, unsigned int *bit)
{
	*bit = pe->sram.byte[MVPP2_PRS_SRAM_BIT_TO_BYTE(bitNum)]  & (1 << (bitNum % 8));
	*bit = (*bit) >> (bitNum % 8);
	return 0;
}

int mv_pp2x_prs_sw_sram_lu_done_get(struct mv_pp2x_prs_entry *pe, unsigned int *bit)
{
	return mv_pp2x_prs_sram_bit_get(pe, MVPP2_PRS_SRAM_LU_DONE_BIT, bit);
}

int mv_pp2x_prs_sw_sram_flowid_gen_get(struct mv_pp2x_prs_entry *pe, unsigned int *bit)
{
	return mv_pp2x_prs_sram_bit_get(pe, MVPP2_PRS_SRAM_LU_GEN_BIT, bit);

}

int mv_pp2x_prs_sw_sram_offset_get(struct mv_pp2x_prs_entry *pe, unsigned int *type, int *offset, unsigned int *op)
{
	int sign;

	*type = pe->sram.byte[MVPP2_PRS_SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_UDF_TYPE_OFFS)] >> (MVPP2_PRS_SRAM_UDF_TYPE_OFFS % 8);
	*type &= MVPP2_PRS_SRAM_UDF_TYPE_MASK;


	*offset = (pe->sram.byte[MVPP2_PRS_SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_UDF_OFFS)] >> (MVPP2_PRS_SRAM_UDF_OFFS % 8)) & 0x7f;
	*offset |= (pe->sram.byte[MVPP2_PRS_SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_UDF_OFFS + MVPP2_PRS_SRAM_UDF_OFFS)] <<
			(8 - (MVPP2_PRS_SRAM_UDF_OFFS % 8))) & 0x80;

	*op = (pe->sram.byte[MVPP2_PRS_SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS)] >> (MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS % 8)) & 0x7;
	*op |= (pe->sram.byte[MVPP2_PRS_SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS + MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS)] <<
			(8 - (MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS % 8))) & 0x18;

	/* if signed bit is tes */
	sign = pe->sram.byte[MVPP2_PRS_SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_UDF_SIGN_BIT)] & (1 << (MVPP2_PRS_SRAM_UDF_SIGN_BIT % 8));
	if (sign != 0)
		*offset = 1-(*offset);

	return 0;
}

int mv_pp2x_prs_sw_sram_next_lu_get(struct mv_pp2x_prs_entry *pe, unsigned int *lu)
{
	*lu = pe->sram.byte[MVPP2_PRS_SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_NEXT_LU_OFFS)];
	*lu = ((*lu) >> MVPP2_PRS_SRAM_NEXT_LU_OFFS % 8);
	*lu &= MVPP2_PRS_SRAM_NEXT_LU_MASK;
	return 0;
}

/* shift to (current offset + shift) */
int mv_pp2x_prs_sw_sram_shift_set(struct mv_pp2x_prs_entry *pe, int shift, unsigned int op)
{

	/* Set sign */
	if (shift < 0) {
		pe->sram.byte[MVPP2_PRS_SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_SHIFT_SIGN_BIT)] |=
									(1 << (MVPP2_PRS_SRAM_SHIFT_SIGN_BIT % 8));
		shift = 0 - shift;
	} else
		pe->sram.byte[SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_SHIFT_SIGN_BIT)] &=
									(~(1 << (MVPP2_PRS_SRAM_SHIFT_SIGN_BIT % 8)));

	/* Set offset */
	pe->sram.byte[SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_SHIFT_OFFS)] = (unsigned char)shift;

	/* Reset and Set operation */
	pe->sram.byte[SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS)] &=
		~(MVPP2_PRS_SRAM_OP_SEL_SHIFT_MASK << (MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS % 8));

	pe->sram.byte[SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS)] |=
									(op << (MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS % 8));

	/* Set base offset as current */
	pe->sram.byte[SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_OP_SEL_BASE_OFFS)] &=
									(~(1 << (MVPP2_PRS_SRAM_OP_SEL_BASE_OFFS % 8)));

	return 0;
}

int mv_pp2x_prs_sw_sram_shift_get(struct mv_pp2x_prs_entry *pe, int *shift)
{
	int sign;

	sign = pe->sram.byte[MVPP2_PRS_SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_SHIFT_SIGN_BIT)] & (1 << (MVPP2_PRS_SRAM_SHIFT_SIGN_BIT % 8));
	*shift = ((int)(pe->sram.byte[MVPP2_PRS_SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_SHIFT_OFFS)])) & MVPP2_PRS_SRAM_SHIFT_MASK;

	if (sign == 1)
		*shift *= -1;

	return 0;
}

/* return RI and RI_UPDATE */
int mv_pp2x_prs_sw_sram_ri_get(struct mv_pp2x_prs_entry *pe, unsigned int *bits, unsigned int *enable)
{

	*bits = pe->sram.word[MVPP2_PRS_SRAM_RI_OFFS/32];
	*enable = pe->sram.word[MVPP2_PRS_SRAM_RI_CTRL_OFFS/32];
	return 0;
}

int mv_pp2x_prs_sw_sram_ai_get(struct mv_pp2x_prs_entry *pe, unsigned int *bits, unsigned int *enable)
{

	*bits = (pe->sram.byte[SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_AI_OFFS)] >> (MVPP2_PRS_SRAM_AI_OFFS % 8)) |
		(pe->sram.byte[SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_AI_OFFS+MVPP2_PRS_SRAM_AI_CTRL_BITS)] << (8 - (MVPP2_PRS_SRAM_AI_OFFS % 8)));

	*enable = (pe->sram.byte[SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_AI_CTRL_OFFS)] >> (MVPP2_PRS_SRAM_AI_CTRL_OFFS % 8)) |
			(pe->sram.byte[SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_AI_CTRL_OFFS+MVPP2_PRS_SRAM_AI_CTRL_BITS)] <<
				(8 - (MVPP2_PRS_SRAM_AI_CTRL_OFFS % 8)));

	*bits &= MVPP2_PRS_SRAM_AI_MASK;
	*enable &= MVPP2_PRS_SRAM_AI_MASK;

	return 0;
}

static int mv_pp2x_prs_hw_tcam_cnt_dump(struct mv_pp2x *pp2, int tid, unsigned int *cnt)
{
	unsigned int regVal;

	/* write index */
	mv_pp2x_write(pp2, MVPP2_PRS_TCAM_HIT_IDX_REG, tid);

	regVal = mv_pp2x_read(pp2, MVPP2_PRS_TCAM_HIT_CNT_REG);
	regVal &= MVPP2_PRS_TCAM_HIT_CNT_MASK;

	if (cnt)
		*cnt = regVal;
	else
		printf("HIT COUNTER: %d\n", regVal);

	return 0;
}


static int mv_pp2x_prs_sw_sram_ri_dump(struct mv_pp2x_prs_entry *pe)
{
	unsigned int data, mask;
	int i, bitsOffs = 0;
	char bits[100];


	mv_pp2x_prs_sw_sram_ri_get(pe, &data, &mask);
	if (mask == 0)
		return 0;

	printf("\n       ");

	printf("S_RI=");
	for (i = (MVPP2_PRS_SRAM_RI_CTRL_BITS-1); i > -1 ; i--)
		if (mask & (1 << i)) {
			printf("%d", ((data & (1 << i)) != 0));
			bitsOffs += sprintf(bits + bitsOffs, "%d:", i);
		} else
			printf("x");

	bits[bitsOffs] = '\0';
	printf(" %s", bits);

	return 0;
}

static int mv_pp2x_prs_sw_sram_ai_dump(struct mv_pp2x_prs_entry *pe)
{
	int i, bitsOffs = 0;
	unsigned int data, mask;
	char bits[30];

	mv_pp2x_prs_sw_sram_ai_get(pe, &data, &mask);

	if (mask == 0)
		return 0;

	printf("\n       ");

	printf("S_AI=");
	for (i = (MVPP2_PRS_SRAM_AI_CTRL_BITS-1); i > -1 ; i--)
		if (mask & (1 << i)) {
			printf("%d", ((data & (1 << i)) != 0));
			bitsOffs += sprintf(bits + bitsOffs, "%d:", i);
		} else
			printf("x");
	bits[bitsOffs] = '\0';
	printf(" %s", bits);
	return 0;
}

int mv_pp2x_prs_sw_dump(struct mv_pp2x_prs_entry *pe)
{
	u32 op, type, lu, done, flowid;
	int	shift, offset, i;

	/* hw entry id */
	printf("[%4d] ", pe->index);

	i = MVPP2_PRS_TCAM_WORDS - 1;
	printf("%1.1x ", pe->tcam.word[i--] & 0xF);

	while (i >= 0)
		printf("%4.4x ", (pe->tcam.word[i--]) & 0xFFFF);

	printf("| ");

	printf(PRS_SRAM_FMT, PRS_SRAM_VAL(pe->sram.word));

	printf("\n       ");

	i = MVPP2_PRS_TCAM_WORDS - 1;
	printf("%1.1x ", (pe->tcam.word[i--] >> 16) & 0xF);

	while (i >= 0)
		printf("%4.4x ", ((pe->tcam.word[i--]) >> 16)  & 0xFFFF);

	printf("| ");

	mv_pp2x_prs_sw_sram_shift_get(pe, &shift);
	printf("SH=%d ", shift);

	mv_pp2x_prs_sw_sram_offset_get(pe, &type, &offset, &op);
	if (offset != 0 || ((op >> MVPP2_PRS_SRAM_OP_SEL_SHIFT_BITS) != 0))
		printf("UDFT=%u UDFO=%d ", type, offset);

	printf("op=%u ", op);

	mv_pp2x_prs_sw_sram_next_lu_get(pe, &lu);
	printf("LU=%u ", lu);

	mv_pp2x_prs_sw_sram_lu_done_get(pe, &done);
	printf("%s ", done ? "DONE" : "N_DONE");

	/*flow id generation bit*/
	mv_pp2x_prs_sw_sram_flowid_gen_get(pe, &flowid);
	printf("%s ", flowid ? "FIDG" : "N_FIDG");

	(pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] & MVPP2_PRS_TCAM_INV_MASK) ? printf(" [inv]") : 0;

	if (mv_pp2x_prs_sw_sram_ri_dump(pe))
		return -1;

	if (mv_pp2x_prs_sw_sram_ai_dump(pe))
		return -1;

	printf("\n");

	return 0;

}

int mv_pp2x_prs_hw_dump(struct mv_pp2x *pp2)
{
	int index;
	struct mv_pp2x_prs_entry pe;


	printf("%s\n", __func__);

	for (index = 0; index < MVPP2_PRS_TCAM_SRAM_SIZE; index++) {
		pe.index = index;
		mv_pp2x_prs_hw_read(pp2, &pe);
		if ((pe.tcam.word[MVPP2_PRS_TCAM_INV_WORD] & MVPP2_PRS_TCAM_INV_MASK) == MVPP2_PRS_TCAM_ENTRY_VALID) {
			mv_pp2x_prs_sw_dump(&pe);
			mv_pp2x_prs_hw_tcam_cnt_dump(pp2, index, NULL);
			printf("-------------------------------------------------------------------------\n");
		}
	}

	return 0;
}

int mv_pp2x_cls_hw_lkp_read(struct mv_pp2x *pp2, int lkpid, int way,
	struct mv_pp2x_cls_lkp_entry *fe)
{
	unsigned int regVal = 0;

	/* write index reg */
	regVal = (way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) | (lkpid << MVPP2_CLS_LKP_INDEX_LKP_OFFS);
	mv_pp2x_write(pp2, MVPP2_CLS_LKP_INDEX_REG, regVal);

	fe->way = way;
	fe->lkpid = lkpid;

	fe->data = mv_pp2x_read(pp2, MVPP2_CLS_LKP_TBL_REG);

	return 0;
}

int mv_pp2x_cls_sw_lkp_rxq_get(struct mv_pp2x_cls_lkp_entry *lkp, int *rxq)
{
	*rxq =  (lkp->data & MVPP2_FLOWID_RXQ_MASK) >> MVPP2_FLOWID_RXQ;
	return 0;
}

int mv_pp2x_cls_sw_lkp_en_get(struct mv_pp2x_cls_lkp_entry *lkp, int *en)
{
	*en = (lkp->data & MVPP2_FLOWID_EN_MASK) >> MVPP2_FLOWID_EN;
	return 0;
}

int mv_pp2x_cls_sw_lkp_flow_get(struct mv_pp2x_cls_lkp_entry *lkp, int *flow_idx)
{
	*flow_idx = (lkp->data & MVPP2_FLOWID_FLOW_MASK) >> MVPP2_FLOWID_FLOW;
	return 0;
}

int mv_pp2x_cls_sw_lkp_mod_get(struct mv_pp2x_cls_lkp_entry *lkp, int *mod_base)
{
	*mod_base = (lkp->data & MVPP2_FLOWID_MODE_MASK) >> MVPP2_FLOWID_MODE;
	return 0;
}

int mv_pp2x_cls_hw_lkp_hit_get(struct mv_pp2x *pp2, int lkpid, int way,  unsigned int *cnt)
{

	/*set index */
	mv_pp2x_write(pp2, MVPP2_CNT_IDX_REG, MVPP2_CNT_IDX_LKP(lkpid, way));

	if (cnt)
		*cnt = mv_pp2x_read(pp2, MVPP2_CLS_LKP_TBL_HIT_REG);
	else
		printf("HITS: %d\n", mv_pp2x_read(pp2, MVPP2_CLS_LKP_TBL_HIT_REG));

	return 0;
}

int mv_pp2x_cls_hw_lkp_dump(struct mv_pp2x *pp2)
{
	int index, way, int32bit;
	unsigned int uint32bit;

	struct mv_pp2x_cls_lkp_entry lkp;


	printf("< ID  WAY >:	RXQ	EN	FLOW	MODE_BASE  HITS\n");
	for (index = 0; index < 4 ; index++)
		for (way = 0; way < 2 ; way++)	{
			mv_pp2x_cls_hw_lkp_read(pp2, index, way, &lkp);
			printf(" 0x%2.2x  %1.1d\t", lkp.lkpid, lkp.way);
			mv_pp2x_cls_sw_lkp_rxq_get(&lkp, &int32bit);
			printf("0x%2.2x\t", int32bit);
			mv_pp2x_cls_sw_lkp_en_get(&lkp, &int32bit);
			printf("%1.1d\t", int32bit);
			mv_pp2x_cls_sw_lkp_flow_get(&lkp, &int32bit);
			printf("0x%3.3x\t", int32bit);
			mv_pp2x_cls_sw_lkp_mod_get(&lkp, &int32bit);
			printf(" 0x%2.2x\t", int32bit);
			mv_pp2x_cls_hw_lkp_hit_get(pp2, index, way, &uint32bit);
			printf(" 0x%8.8x\n", uint32bit);
			printf("\n");

		}
	return 0;
}

static char *mv_pp2x_prs_l2_info_str(unsigned int l2_info)
{
	switch (l2_info << MVPP2_PRS_RI_L2_CAST_OFFS) {
	case MVPP2_PRS_RI_L2_UCAST:
		return "Ucast";
	case MVPP2_PRS_RI_L2_MCAST:
		return "Mcast";
	case MVPP2_PRS_RI_L2_BCAST:
		return "Bcast";
	default:
		return "Unknown";
	}
	return NULL;
}

static char *mv_pp2x_prs_vlan_info_str(unsigned int vlan_info)
{
	switch (vlan_info << MVPP2_PRS_RI_VLAN_OFFS) {
	case MVPP2_PRS_RI_VLAN_NONE:
		return "None";
	case MVPP2_PRS_RI_VLAN_SINGLE:
		return "Single";
	case MVPP2_PRS_RI_VLAN_DOUBLE:
		return "Double";
	case MVPP2_PRS_RI_VLAN_TRIPLE:
		return "Triple";
	default:
		return "Unknown";
	}
	return NULL;
}
void mv_pp2x_rx_desc_print(struct mv_pp2x_rx_desc *desc)
{
	int i;
	u32 *words = (u32 *) desc;

	printf("RX desc - %p: ", desc);
	for (i = 0; i < 8; i++)
		printf("%8.8x ", *words++);
	printf("\n");

	printf("pkt_size=%d, L3_offs=%d, IP_hlen=%d, ",
	       desc->data_size,
	       (desc->status & MVPP2_RXD_L3_OFFSET_MASK) >> MVPP2_RXD_L3_OFFSET_OFFS,
	       (desc->status & MVPP2_RXD_IP_HLEN_MASK) >> MVPP2_RXD_IP_HLEN_OFFS);

	printf("L2=%s, ",
		mv_pp2x_prs_l2_info_str((desc->rsrvd_parser & MVPP2_RXD_L2_CAST_MASK) >> MVPP2_RXD_L2_CAST_OFFS));

	printf("VLAN=");
	printf("%s, ",
		mv_pp2x_prs_vlan_info_str((desc->rsrvd_parser & MVPP2_RXD_VLAN_INFO_MASK) >> MVPP2_RXD_VLAN_INFO_OFFS));

	printf("L3=");
	if (MVPP2_RXD_L3_IS_IP4(desc->status))
		printf("IPv4 (hdr=%s), ", MVPP2_RXD_IP4_HDR_ERR(desc->status) ? "bad" : "ok");
	else if (MVPP2_RXD_L3_IS_IP4_OPT(desc->status))
		printf("IPv4 Options (hdr=%s), ", MVPP2_RXD_IP4_HDR_ERR(desc->status) ? "bad" : "ok");
	else if (MVPP2_RXD_L3_IS_IP4_OTHER(desc->status))
		printf("IPv4 Other (hdr=%s), ", MVPP2_RXD_IP4_HDR_ERR(desc->status) ? "bad" : "ok");
	else if (MVPP2_RXD_L3_IS_IP6(desc->status))
		printf("IPv6, ");
	else if (MVPP2_RXD_L3_IS_IP6_EXT(desc->status))
		printf("IPv6 Ext, ");
	else
		printf("Unknown, ");

	if (desc->status & MVPP2_RXD_IP_FRAG_MASK)
		printf("Frag, ");

	printf("L4=");
	if (MVPP2_RXD_L4_IS_TCP(desc->status))
		printf("TCP (csum=%s)", (desc->status & MVPP2_RXD_L4_CHK_OK_MASK) ? "Ok" : "Bad");
	else if (MVPP2_RXD_L4_IS_UDP(desc->status))
		printf("UDP (csum=%s)", (desc->status & MVPP2_RXD_L4_CHK_OK_MASK) ? "Ok" : "Bad");
	else
		printf("Unknown");

	printf("\n");

	printf("Lookup_ID=0x%x, cpu_code=0x%x\n",
		(desc->rsrvd_parser & MVPP2_RXD_LKP_ID_MASK) >> MVPP2_RXD_LKP_ID_OFFS,
		(desc->rsrvd_parser & MVPP2_RXD_CPU_CODE_MASK) >> MVPP2_RXD_CPU_CODE_OFFS);


	printf("buf_phys_addr = 0x%llx\n", desc->u.pp22.buf_phys_addr_key_hash & 0xffffffffff);
	printf("buf_virt_addr = 0x%llx\n", desc->u.pp22.buf_cookie_bm_qset_cls_info & 0xffffffffff);
}

void mv_pp2x_PrintReg(struct mv_pp2x *pp2, unsigned int reg_addr, char *reg_name)
{
	printf("  %-32s: 0x%x = 0x%08x\n", reg_name, reg_addr, mv_pp2x_read(pp2, reg_addr));
}

void mv_pp2x_PhysTxqRegs(struct mv_pp2x_port *pp)
{
	int txq = pp->txqs[0].id;
	printf("\n[PPv2 TxQ registers: global txq=%d]\n", txq);

	mv_pp2x_write(pp->pp2, MVPP2_TXQ_NUM_REG, txq);
	mv_pp2x_PrintReg(pp->pp2, MVPP2_TXQ_NUM_REG, "MV_PP2_TXQ_NUM_REG");
	mv_pp2x_PrintReg(pp->pp2, MVPP2_TXQ_DESC_ADDR_LOW_REG, "MVPP2_TXQ_DESC_ADDR_LOW_REG");
	mv_pp2x_PrintReg(pp->pp2, MVPP2_TXQ_DESC_SIZE_REG, "MV_PP2_TXQ_DESC_SIZE_REG");
	mv_pp2x_PrintReg(pp->pp2, MVPP2_TXQ_DESC_HWF_SIZE_REG, "MV_PP2_TXQ_DESC_HWF_SIZE_REG");
	mv_pp2x_PrintReg(pp->pp2, MVPP2_TXQ_INDEX_REG, "MV_PP2_TXQ_INDEX_REG");
	mv_pp2x_PrintReg(pp->pp2, MVPP2_TXQ_PREF_BUF_REG, "MV_PP2_TXQ_PREF_BUF_REG");
	mv_pp2x_PrintReg(pp->pp2, MVPP2_TXQ_PENDING_REG, "MV_PP2_TXQ_PENDING_REG");
	mv_pp2x_PrintReg(pp->pp2, MVPP22_TXQ_SENT_REG(txq), "MV_PP22_TXQ_SENT_REG");
	mv_pp2x_PrintReg(pp->pp2, MVPP2_TXQ_INT_STATUS_REG, "MV_PP2_TXQ_INT_STATUS_REG");
}

void mv_pp2x_AggrTxqRegs(struct mv_pp2x *pp2, int cpu)
{
	printf("\n[PP2 Aggr TXQ registers: cpu=%d]\n", cpu);

	mv_pp2x_PrintReg(pp2, MVPP2_AGGR_TXQ_DESC_ADDR_REG(cpu), "MVPP2_AGGR_TXQ_DESC_ADDR_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_AGGR_TXQ_DESC_SIZE_REG(cpu), "MVPP2_AGGR_TXQ_DESC_SIZE_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_AGGR_TXQ_STATUS_REG(cpu), "MVPP2_AGGR_TXQ_STATUS_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_AGGR_TXQ_INDEX_REG(cpu), "MVPP2_AGGR_TXQ_INDEX_REG");
}

void mv_pp2x_TxRegs(struct mv_pp2x_port *pp)
{
	printf("\n[TX general registers]\n");

	mv_pp2x_PrintReg(pp->pp2, MVPP2_TX_SNOOP_REG, "MVPP2_TX_SNOOP_REG");
	mv_pp2x_PrintReg(pp->pp2, MVPP22_TX_FIFO_THRESH_REG(0), "MVPP22_TX_FIFO_THRESH_REG");

	mv_pp2x_PrintReg(pp->pp2, MVPP2_TX_PORT_FLUSH_REG, "MVPP2_TX_PORT_FLUSH_REG");
	mv_pp2x_PrintReg(pp->pp2, MVPP2_TXP_SCHED_PORT_INDEX_REG, "MVPP2_TXP_SCHED_PORT_INDEX_REG");
	mv_pp2x_PrintReg(pp->pp2, MVPP2_TXP_SCHED_Q_CMD_REG, "MVPP2_TXP_SCHED_Q_CMD_REG");
}

void mv_pp2x_PhysRxqRegs(struct mv_pp2x *pp2, int rxq)
{
	printf("\n[PPv2 RxQ registers: global rxq=%d]\n", rxq);

	mv_pp2x_write(pp2, MVPP2_RXQ_NUM_REG, rxq);
	mv_pp2x_PrintReg(pp2, MVPP2_RXQ_NUM_REG, "MVPP2_RXQ_NUM_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_RXQ_DESC_ADDR_REG, "MVPP2_RXQ_DESC_ADDR_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_RXQ_DESC_SIZE_REG, "MVPP2_RXQ_DESC_SIZE_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_RXQ_STATUS_REG(rxq), "MVPP2_RXQ_STATUS_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_RXQ_THRESH_REG, "MVPP2_RXQ_THRESH_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_RXQ_INDEX_REG, "MVPP2_RXQ_INDEX_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_RXQ_CONFIG_REG(rxq), "MVPP2_RXQ_CONFIG_REG");
}

void mv_pp2x_bm_pool_regs(struct mv_pp2x *pp2, int pool)
{

	printf("\n[BM pool registers: pool=%d]\n", pool);
	mv_pp2x_PrintReg(pp2, MVPP2_BM_POOL_BASE_REG(pool), "MV_BM_POOL_BASE_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_BM_POOL_SIZE_REG(pool), "MVPP2_BM_POOL_SIZE_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_BM_POOL_READ_PTR_REG(pool), "MVPP2_BM_POOL_READ_PTR_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_BM_POOL_PTRS_NUM_REG(pool), "MVPP2_BM_POOL_PTRS_NUM_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_BM_BPPI_READ_PTR_REG(pool), "MVPP2_BM_BPPI_READ_PTR_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_BM_BPPI_PTRS_NUM_REG(pool), "MVPP2_BM_BPPI_PTRS_NUM_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_BM_POOL_CTRL_REG(pool), "MVPP2_BM_POOL_CTRL_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_BM_INTR_CAUSE_REG(pool), "MVPP2_BM_INTR_CAUSE_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_BM_INTR_MASK_REG(pool), "MVPP2_BM_INTR_MASK_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_BM_DROP_CNTR_REG(pool), "MVPP2_BM_DROP_CNTR_REG");
	mv_pp2x_PrintReg(pp2, MVPP2_BM_MC_DROP_CNTR_REG(pool), "MVPP2_BM_MC_DROP_CNTR_REG");
}

void pkt_dump(u8 *pkt)
{
	int i;
	for (i = 0; i < 64; i++) {
		if (!(i % 16))
			printf("\n");
		printf("%02x", *(pkt+i));
	}
}
