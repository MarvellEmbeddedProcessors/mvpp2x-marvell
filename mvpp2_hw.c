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
#ifdef ARMADA_390
#include <linux/phy.h>
#endif
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/inetdevice.h>
#include <uapi/linux/ppp_defs.h>

#include <net/ip.h>
#include <net/ipv6.h>

#include "mvpp2.h"
#include "mvpp2_hw.h"



#if 0

#include <linux/mbus.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/cpumask.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/phy.h>
#include <linux/clk.h>
#endif

/* Utility/helper methods */






/* Parser configuration routines */

/* Update parser tcam and sram hw entries */
static int mvpp2_prs_hw_write(struct mvpp2_hw *hw, struct mvpp2_prs_entry *pe)
{
	int i;

	if (pe->index > MVPP2_PRS_TCAM_SRAM_SIZE - 1)
		return -EINVAL;

	/* Clear entry invalidation bit */
	pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] &= ~MVPP2_PRS_TCAM_INV_MASK;

	/* Write tcam index - indirect access */
	mvpp2_write(hw, MVPP2_PRS_TCAM_IDX_REG, pe->index);
	for (i = 0; i < MVPP2_PRS_TCAM_WORDS; i++)
		mvpp2_write(hw, MVPP2_PRS_TCAM_DATA_REG(i), pe->tcam.word[i]);

	/* Write sram index - indirect access */
	mvpp2_write(hw, MVPP2_PRS_SRAM_IDX_REG, pe->index);
	for (i = 0; i < MVPP2_PRS_SRAM_WORDS; i++)
		mvpp2_write(hw, MVPP2_PRS_SRAM_DATA_REG(i), pe->sram.word[i]);

	return 0;
}

/* Read tcam entry from hw */
static int mvpp2_prs_hw_read(struct mvpp2_hw *hw, struct mvpp2_prs_entry *pe)
{
	int i;

	if (pe->index > MVPP2_PRS_TCAM_SRAM_SIZE - 1)
		return -EINVAL;

	/* Write tcam index - indirect access */
	mvpp2_write(hw, MVPP2_PRS_TCAM_IDX_REG, pe->index);

	pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] = mvpp2_read(hw,
			      MVPP2_PRS_TCAM_DATA_REG(MVPP2_PRS_TCAM_INV_WORD));
	if (pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] & MVPP2_PRS_TCAM_INV_MASK)
		return MVPP2_PRS_TCAM_ENTRY_INVALID;

	for (i = 0; i < MVPP2_PRS_TCAM_WORDS; i++)
		pe->tcam.word[i] = mvpp2_read(hw, MVPP2_PRS_TCAM_DATA_REG(i));

	/* Write sram index - indirect access */
	mvpp2_write(hw, MVPP2_PRS_SRAM_IDX_REG, pe->index);
	for (i = 0; i < MVPP2_PRS_SRAM_WORDS; i++)
		pe->sram.word[i] = mvpp2_read(hw, MVPP2_PRS_SRAM_DATA_REG(i));

	return 0;
}

/* Invalidate tcam hw entry */
static void mvpp2_prs_hw_inv(struct mvpp2_hw *hw, int index)
{
	/* Write index - indirect access */
	mvpp2_write(hw, MVPP2_PRS_TCAM_IDX_REG, index);
	mvpp2_write(hw, MVPP2_PRS_TCAM_DATA_REG(MVPP2_PRS_TCAM_INV_WORD),
		    MVPP2_PRS_TCAM_INV_MASK);
}

/* Enable shadow table entry and set its lookup ID */
static void mvpp2_prs_shadow_set(struct mvpp2_hw *hw, int index, int lu)
{
	hw->prs_shadow[index].valid = true;
	hw->prs_shadow[index].lu = lu;
}

/* Update ri fields in shadow table entry */
static void mvpp2_prs_shadow_ri_set(struct mvpp2_hw *hw, int index,
				    unsigned int ri, unsigned int ri_mask)
{
	hw->prs_shadow[index].ri_mask = ri_mask;
	hw->prs_shadow[index].ri = ri;
}

/* Update lookup field in tcam sw entry */
static void mvpp2_prs_tcam_lu_set(struct mvpp2_prs_entry *pe, unsigned int lu)
{
	int enable_off = MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_LU_BYTE);

	pe->tcam.byte[MVPP2_PRS_TCAM_LU_BYTE] = lu;
	pe->tcam.byte[enable_off] = MVPP2_PRS_LU_MASK;
}

/* Update mask for single port in tcam sw entry */
static void mvpp2_prs_tcam_port_set(struct mvpp2_prs_entry *pe,
				    unsigned int port, bool add)
{
	int enable_off = MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE);

	if (add)
		pe->tcam.byte[enable_off] &= ~(1 << port);
	else
		pe->tcam.byte[enable_off] |= 1 << port;
}

/* Update port map in tcam sw entry */
static void mvpp2_prs_tcam_port_map_set(struct mvpp2_prs_entry *pe,
					unsigned int ports)
{
	unsigned char port_mask = MVPP2_PRS_PORT_MASK;
	int enable_off = MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE);

	pe->tcam.byte[MVPP2_PRS_TCAM_PORT_BYTE] = 0;
	pe->tcam.byte[enable_off] &= ~port_mask;
	pe->tcam.byte[enable_off] |= ~ports & MVPP2_PRS_PORT_MASK;
}

/* Obtain port map from tcam sw entry */
static unsigned int mvpp2_prs_tcam_port_map_get(struct mvpp2_prs_entry *pe)
{
	int enable_off = MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE);

	return ~(pe->tcam.byte[enable_off]) & MVPP2_PRS_PORT_MASK;
}

/* Set byte of data and its enable bits in tcam sw entry */
static void mvpp2_prs_tcam_data_byte_set(struct mvpp2_prs_entry *pe,
					 unsigned int offs, unsigned char byte,
					 unsigned char enable)
{
	pe->tcam.byte[MVPP2_PRS_TCAM_DATA_BYTE(offs)] = byte;
	pe->tcam.byte[MVPP2_PRS_TCAM_DATA_BYTE_EN(offs)] = enable;
}

/* Get byte of data and its enable bits from tcam sw entry */
static void mvpp2_prs_tcam_data_byte_get(struct mvpp2_prs_entry *pe,
					 unsigned int offs, unsigned char *byte,
					 unsigned char *enable)
{
	*byte = pe->tcam.byte[MVPP2_PRS_TCAM_DATA_BYTE(offs)];
	*enable = pe->tcam.byte[MVPP2_PRS_TCAM_DATA_BYTE_EN(offs)];
}

/* Compare tcam data bytes with a pattern */
static bool mvpp2_prs_tcam_data_cmp(struct mvpp2_prs_entry *pe, int offs,
				    u16 data)
{
	int off = MVPP2_PRS_TCAM_DATA_BYTE(offs);
	u16 tcam_data;

	tcam_data = (8 << pe->tcam.byte[off + 1]) | pe->tcam.byte[off];
	if (tcam_data != data)
		return false;
	return true;
}

/* Update ai bits in tcam sw entry */
static void mvpp2_prs_tcam_ai_update(struct mvpp2_prs_entry *pe,
				     unsigned int bits, unsigned int enable)
{
	int i, ai_idx = MVPP2_PRS_TCAM_AI_BYTE;

	for (i = 0; i < MVPP2_PRS_AI_BITS; i++) {

		if (!(enable & BIT(i)))
			continue;

		if (bits & BIT(i))
			pe->tcam.byte[ai_idx] |= 1 << i;
		else
			pe->tcam.byte[ai_idx] &= ~(1 << i);
	}

	pe->tcam.byte[MVPP2_PRS_TCAM_EN_OFFS(ai_idx)] |= enable;
}

/* Get ai bits from tcam sw entry */
static int mvpp2_prs_tcam_ai_get(struct mvpp2_prs_entry *pe)
{
	return pe->tcam.byte[MVPP2_PRS_TCAM_AI_BYTE];
}

/* Set ethertype in tcam sw entry */
static void mvpp2_prs_match_etype(struct mvpp2_prs_entry *pe, int offset,
				  unsigned short ethertype)
{
	mvpp2_prs_tcam_data_byte_set(pe, offset + 0, ethertype >> 8, 0xff);
	mvpp2_prs_tcam_data_byte_set(pe, offset + 1, ethertype & 0xff, 0xff);
}

/* Set bits in sram sw entry */
static void mvpp2_prs_sram_bits_set(struct mvpp2_prs_entry *pe, int bit_num,
				    int val)
{
	pe->sram.byte[MVPP2_BIT_TO_BYTE(bit_num)] |= (val << (bit_num % 8));
}

/* Clear bits in sram sw entry */
static void mvpp2_prs_sram_bits_clear(struct mvpp2_prs_entry *pe, int bit_num,
				      int val)
{
	pe->sram.byte[MVPP2_BIT_TO_BYTE(bit_num)] &= ~(val << (bit_num % 8));
}

/* Update ri bits in sram sw entry */
static void mvpp2_prs_sram_ri_update(struct mvpp2_prs_entry *pe,
				     unsigned int bits, unsigned int mask)
{
	unsigned int i;

	for (i = 0; i < MVPP2_PRS_SRAM_RI_CTRL_BITS; i++) {
		int ri_off = MVPP2_PRS_SRAM_RI_OFFS;

		if (!(mask & BIT(i)))
			continue;

		if (bits & BIT(i))
			mvpp2_prs_sram_bits_set(pe, ri_off + i, 1);
		else
			mvpp2_prs_sram_bits_clear(pe, ri_off + i, 1);

		mvpp2_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_RI_CTRL_OFFS + i, 1);
	}
}

/* Obtain ri bits from sram sw entry */
static int mvpp2_prs_sram_ri_get(struct mvpp2_prs_entry *pe)
{
	return pe->sram.word[MVPP2_PRS_SRAM_RI_WORD];
}

/* Update ai bits in sram sw entry */
static void mvpp2_prs_sram_ai_update(struct mvpp2_prs_entry *pe,
				     unsigned int bits, unsigned int mask)
{
	unsigned int i;
	int ai_off = MVPP2_PRS_SRAM_AI_OFFS;

	for (i = 0; i < MVPP2_PRS_SRAM_AI_CTRL_BITS; i++) {

		if (!(mask & BIT(i)))
			continue;

		if (bits & BIT(i))
			mvpp2_prs_sram_bits_set(pe, ai_off + i, 1);
		else
			mvpp2_prs_sram_bits_clear(pe, ai_off + i, 1);

		mvpp2_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_AI_CTRL_OFFS + i, 1);
	}
}

/* Read ai bits from sram sw entry */
static int mvpp2_prs_sram_ai_get(struct mvpp2_prs_entry *pe)
{
	u8 bits;
	int ai_off = MVPP2_BIT_TO_BYTE(MVPP2_PRS_SRAM_AI_OFFS);
	int ai_en_off = ai_off + 1;
	int ai_shift = MVPP2_PRS_SRAM_AI_OFFS % 8;

	bits = (pe->sram.byte[ai_off] >> ai_shift) |
	       (pe->sram.byte[ai_en_off] << (8 - ai_shift));

	return bits;
}

/* In sram sw entry set lookup ID field of the tcam key to be used in the next
 * lookup interation
 */
static void mvpp2_prs_sram_next_lu_set(struct mvpp2_prs_entry *pe,
				       unsigned int lu)
{
	int sram_next_off = MVPP2_PRS_SRAM_NEXT_LU_OFFS;

	mvpp2_prs_sram_bits_clear(pe, sram_next_off,
				  MVPP2_PRS_SRAM_NEXT_LU_MASK);
	mvpp2_prs_sram_bits_set(pe, sram_next_off, lu);
}

/* In the sram sw entry set sign and value of the next lookup offset
 * and the offset value generated to the classifier
 */
static void mvpp2_prs_sram_shift_set(struct mvpp2_prs_entry *pe, int shift,
				     unsigned int op)
{
	/* Set sign */
	if (shift < 0) {
		mvpp2_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_SHIFT_SIGN_BIT, 1);
		shift = 0 - shift;
	} else {
		mvpp2_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_SHIFT_SIGN_BIT, 1);
	}

	/* Set value */
	pe->sram.byte[MVPP2_BIT_TO_BYTE(MVPP2_PRS_SRAM_SHIFT_OFFS)] =
							   (unsigned char)shift;

	/* Reset and set operation */
	mvpp2_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS,
				  MVPP2_PRS_SRAM_OP_SEL_SHIFT_MASK);
	mvpp2_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS, op);

	/* Set base offset as current */
	mvpp2_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_OP_SEL_BASE_OFFS, 1);
}

/* In the sram sw entry set sign and value of the user defined offset
 * generated to the classifier
 */
static void mvpp2_prs_sram_offset_set(struct mvpp2_prs_entry *pe,
				      unsigned int type, int offset,
				      unsigned int op)
{
	/* Set sign */
	if (offset < 0) {
		mvpp2_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_UDF_SIGN_BIT, 1);
		offset = 0 - offset;
	} else {
		mvpp2_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_UDF_SIGN_BIT, 1);
	}

	/* Set value */
	mvpp2_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_UDF_OFFS,
				  MVPP2_PRS_SRAM_UDF_MASK);
	mvpp2_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_UDF_OFFS, offset);
	pe->sram.byte[MVPP2_BIT_TO_BYTE(MVPP2_PRS_SRAM_UDF_OFFS +
					MVPP2_PRS_SRAM_UDF_BITS)] &=
	      ~(MVPP2_PRS_SRAM_UDF_MASK >> (8 - (MVPP2_PRS_SRAM_UDF_OFFS % 8)));
	pe->sram.byte[MVPP2_BIT_TO_BYTE(MVPP2_PRS_SRAM_UDF_OFFS +
					MVPP2_PRS_SRAM_UDF_BITS)] |=
				(offset >> (8 - (MVPP2_PRS_SRAM_UDF_OFFS % 8)));

	/* Set offset type */
	mvpp2_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_UDF_TYPE_OFFS,
				  MVPP2_PRS_SRAM_UDF_TYPE_MASK);
	mvpp2_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_UDF_TYPE_OFFS, type);

	/* Set offset operation */
	mvpp2_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_MASK);
	mvpp2_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS, op);

	pe->sram.byte[MVPP2_BIT_TO_BYTE(MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS +
					MVPP2_PRS_SRAM_OP_SEL_UDF_BITS)] &=
					     ~(MVPP2_PRS_SRAM_OP_SEL_UDF_MASK >>
				    (8 - (MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS % 8)));

	pe->sram.byte[MVPP2_BIT_TO_BYTE(MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS +
					MVPP2_PRS_SRAM_OP_SEL_UDF_BITS)] |=
			     (op >> (8 - (MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS % 8)));

	/* Set base offset as current */
	mvpp2_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_OP_SEL_BASE_OFFS, 1);
}

/* Find parser flow entry */
static struct mvpp2_prs_entry *mvpp2_prs_flow_find(struct mvpp2_hw *hw, int flow)
{
	struct mvpp2_prs_entry *pe;
	int tid;

	pe = kzalloc(sizeof(*pe), GFP_KERNEL);
	if (!pe)
		return NULL;
	mvpp2_prs_tcam_lu_set(pe, MVPP2_PRS_LU_FLOWS);

	/* Go through the all entires with MVPP2_PRS_LU_FLOWS */
	for (tid = MVPP2_PRS_TCAM_SRAM_SIZE - 1; tid >= 0; tid--) {
		u8 bits;

		if (!hw->prs_shadow[tid].valid ||
		    hw->prs_shadow[tid].lu != MVPP2_PRS_LU_FLOWS)
			continue;

		pe->index = tid;
		mvpp2_prs_hw_read(hw, pe);
		bits = mvpp2_prs_sram_ai_get(pe);

		/* Sram store classification lookup ID in AI bits [5:0] */
		if ((bits & MVPP2_PRS_FLOW_ID_MASK) == flow)
			return pe;
	}
	kfree(pe);

	return NULL;
}

/* Return first free tcam index, seeking from start to end */
static int mvpp2_prs_tcam_first_free(struct mvpp2_hw *hw, unsigned char start,
				     unsigned char end)
{
	int tid;

	if (start > end)
		swap(start, end);

	if (end >= MVPP2_PRS_TCAM_SRAM_SIZE)
		end = MVPP2_PRS_TCAM_SRAM_SIZE - 1;

	for (tid = start; tid <= end; tid++) {
		if (!hw->prs_shadow[tid].valid)
			return tid;
	}

	return -EINVAL;
}

/* Enable/disable dropping all mac da's */
static void mvpp2_prs_mac_drop_all_set(struct mvpp2_hw *hw, int port, bool add)
{
	struct mvpp2_prs_entry pe;

	if (hw->prs_shadow[MVPP2_PE_DROP_ALL].valid) {
		/* Entry exist - update port only */
		pe.index = MVPP2_PE_DROP_ALL;
		mvpp2_prs_hw_read(hw, &pe);
	} else {
		/* Entry doesn't exist - create new */
		memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
		mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_MAC);
		pe.index = MVPP2_PE_DROP_ALL;

		/* Non-promiscuous mode for all ports - DROP unknown packets */
		mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_DROP_MASK,
					 MVPP2_PRS_RI_DROP_MASK);

		mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
		mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);

		/* Update shadow table */
		mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_MAC);

		/* Mask all ports */
		mvpp2_prs_tcam_port_map_set(&pe, 0);
	}

	/* Update port mask */
	mvpp2_prs_tcam_port_set(&pe, port, add);

	mvpp2_prs_hw_write(hw, &pe);
}

/* Set port to promiscuous mode */
void mvpp2_prs_mac_promisc_set(struct mvpp2_hw *hw, int port, bool add)
{
	struct mvpp2_prs_entry pe;

	/* Promiscous mode - Accept unknown packets */

	if (hw->prs_shadow[MVPP2_PE_MAC_PROMISCUOUS].valid) {
		/* Entry exist - update port only */
		pe.index = MVPP2_PE_MAC_PROMISCUOUS;
		mvpp2_prs_hw_read(hw, &pe);
	} else {
		/* Entry doesn't exist - create new */
		memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
		mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_MAC);
		pe.index = MVPP2_PE_MAC_PROMISCUOUS;

		/* Continue - set next lookup */
		mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_DSA);

		/* Set result info bits */
		mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L2_UCAST,
					 MVPP2_PRS_RI_L2_CAST_MASK);

		/* Shift to ethertype */
		mvpp2_prs_sram_shift_set(&pe, 2 * ETH_ALEN,
					 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

		/* Mask all ports */
		mvpp2_prs_tcam_port_map_set(&pe, 0);

		/* Update shadow table */
		mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_MAC);
	}

	/* Update port mask */
	mvpp2_prs_tcam_port_set(&pe, port, add);

	mvpp2_prs_hw_write(hw, &pe);
}

/* Accept multicast */
void mvpp2_prs_mac_multi_set(struct mvpp2_hw *hw, int port, int index,
				    bool add)
{
	struct mvpp2_prs_entry pe;
	unsigned char da_mc;

	/* Ethernet multicast address first byte is
	 * 0x01 for IPv4 and 0x33 for IPv6
	 */
	da_mc = (index == MVPP2_PE_MAC_MC_ALL) ? 0x01 : 0x33;

	if (hw->prs_shadow[index].valid) {
		/* Entry exist - update port only */
		pe.index = index;
		mvpp2_prs_hw_read(hw, &pe);
	} else {
		/* Entry doesn't exist - create new */
		memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
		mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_MAC);
		pe.index = index;

		/* Continue - set next lookup */
		mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_DSA);

		/* Set result info bits */
		mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L2_MCAST,
					 MVPP2_PRS_RI_L2_CAST_MASK);

		/* Update tcam entry data first byte */
		mvpp2_prs_tcam_data_byte_set(&pe, 0, da_mc, 0xff);

		/* Shift to ethertype */
		mvpp2_prs_sram_shift_set(&pe, 2 * ETH_ALEN,
					 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

		/* Mask all ports */
		mvpp2_prs_tcam_port_map_set(&pe, 0);

		/* Update shadow table */
		mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_MAC);
	}

	/* Update port mask */
	mvpp2_prs_tcam_port_set(&pe, port, add);

	mvpp2_prs_hw_write(hw, &pe);
}

/* Set entry for dsa packets */
static void mvpp2_prs_dsa_tag_set(struct mvpp2_hw *hw, int port, bool add,
				  bool tagged, bool extend)
{
	struct mvpp2_prs_entry pe;
	int tid, shift;

	if (extend) {
		tid = tagged ? MVPP2_PE_EDSA_TAGGED : MVPP2_PE_EDSA_UNTAGGED;
		shift = 8;
	} else {
		tid = tagged ? MVPP2_PE_DSA_TAGGED : MVPP2_PE_DSA_UNTAGGED;
		shift = 4;
	}

	if (hw->prs_shadow[tid].valid) {
		/* Entry exist - update port only */
		pe.index = tid;
		mvpp2_prs_hw_read(hw, &pe);
	} else {
		/* Entry doesn't exist - create new */
		memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
		mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_DSA);
		pe.index = tid;

		/* Shift 4 bytes if DSA tag or 8 bytes in case of EDSA tag*/
		mvpp2_prs_sram_shift_set(&pe, shift,
					 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

		/* Update shadow table */
		mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_DSA);

		if (tagged) {
			/* Set tagged bit in DSA tag */
			mvpp2_prs_tcam_data_byte_set(&pe, 0,
						     MVPP2_PRS_TCAM_DSA_TAGGED_BIT,
						     MVPP2_PRS_TCAM_DSA_TAGGED_BIT);
			/* Clear all ai bits for next iteration */
			mvpp2_prs_sram_ai_update(&pe, 0,
						 MVPP2_PRS_SRAM_AI_MASK);
			/* If packet is tagged continue check vlans */
			mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_VLAN);
		} else {
			/* Set result info bits to 'no vlans' */
			mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_VLAN_NONE,
						 MVPP2_PRS_RI_VLAN_MASK);
			mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_L2);
		}

		/* Mask all ports */
		mvpp2_prs_tcam_port_map_set(&pe, 0);
	}

	/* Update port mask */
	mvpp2_prs_tcam_port_set(&pe, port, add);

	mvpp2_prs_hw_write(hw, &pe);
}

/* Set entry for dsa ethertype */
static void mvpp2_prs_dsa_tag_ethertype_set(struct mvpp2_hw *hw, int port,
					    bool add, bool tagged, bool extend)
{
	struct mvpp2_prs_entry pe;
	int tid, shift, port_mask;

	if (extend) {
		tid = tagged ? MVPP2_PE_ETYPE_EDSA_TAGGED :
		      MVPP2_PE_ETYPE_EDSA_UNTAGGED;
		port_mask = 0;
		shift = 8;
	} else {
		tid = tagged ? MVPP2_PE_ETYPE_DSA_TAGGED :
		      MVPP2_PE_ETYPE_DSA_UNTAGGED;
		port_mask = MVPP2_PRS_PORT_MASK;
		shift = 4;
	}

	if (hw->prs_shadow[tid].valid) {
		/* Entry exist - update port only */
		pe.index = tid;
		mvpp2_prs_hw_read(hw, &pe);
	} else {
		/* Entry doesn't exist - create new */
		memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
		mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_DSA);
		pe.index = tid;

		/* Set ethertype */
		mvpp2_prs_match_etype(&pe, 0, ETH_P_EDSA);
		mvpp2_prs_match_etype(&pe, 2, 0);

		mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_DSA_MASK,
					 MVPP2_PRS_RI_DSA_MASK);
		/* Shift ethertype + 2 byte reserved + tag*/
		mvpp2_prs_sram_shift_set(&pe, 2 + MVPP2_ETH_TYPE_LEN + shift,
					 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

		/* Update shadow table */
		mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_DSA);

		if (tagged) {
			/* Set tagged bit in DSA tag */
			mvpp2_prs_tcam_data_byte_set(&pe,
						     MVPP2_ETH_TYPE_LEN + 2 + 3,
						 MVPP2_PRS_TCAM_DSA_TAGGED_BIT,
						 MVPP2_PRS_TCAM_DSA_TAGGED_BIT);
			/* Clear all ai bits for next iteration */
			mvpp2_prs_sram_ai_update(&pe, 0,
						 MVPP2_PRS_SRAM_AI_MASK);
			/* If packet is tagged continue check vlans */
			mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_VLAN);
		} else {
			/* Set result info bits to 'no vlans' */
			mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_VLAN_NONE,
						 MVPP2_PRS_RI_VLAN_MASK);
			mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_L2);
		}
		/* Mask/unmask all ports, depending on dsa type */
		mvpp2_prs_tcam_port_map_set(&pe, port_mask);
	}

	/* Update port mask */
	mvpp2_prs_tcam_port_set(&pe, port, add);

	mvpp2_prs_hw_write(hw, &pe);
}

/* Search for existing single/triple vlan entry */
static struct mvpp2_prs_entry *mvpp2_prs_vlan_find(struct mvpp2_hw *hw,
						   unsigned short tpid, int ai)
{
	struct mvpp2_prs_entry *pe;
	int tid;

	pe = kzalloc(sizeof(*pe), GFP_KERNEL);
	if (!pe)
		return NULL;
	mvpp2_prs_tcam_lu_set(pe, MVPP2_PRS_LU_VLAN);

	/* Go through the all entries with MVPP2_PRS_LU_VLAN */
	for (tid = MVPP2_PE_FIRST_FREE_TID;
	     tid <= MVPP2_PE_LAST_FREE_TID; tid++) {
		unsigned int ri_bits, ai_bits;
		bool match;

		if (!hw->prs_shadow[tid].valid ||
		    hw->prs_shadow[tid].lu != MVPP2_PRS_LU_VLAN)
			continue;

		pe->index = tid;

		mvpp2_prs_hw_read(hw, pe);
		match = mvpp2_prs_tcam_data_cmp(pe, 0, swab16(tpid));
		if (!match)
			continue;

		/* Get vlan type */
		ri_bits = mvpp2_prs_sram_ri_get(pe);
		ri_bits &= MVPP2_PRS_RI_VLAN_MASK;

		/* Get current ai value from tcam */
		ai_bits = mvpp2_prs_tcam_ai_get(pe);
		/* Clear double vlan bit */
		ai_bits &= ~MVPP2_PRS_DBL_VLAN_AI_BIT;

		if (ai != ai_bits)
			continue;

		if (ri_bits == MVPP2_PRS_RI_VLAN_SINGLE ||
		    ri_bits == MVPP2_PRS_RI_VLAN_TRIPLE)
			return pe;
	}
	kfree(pe);

	return NULL;
}

/* Add/update single/triple vlan entry */
static int mvpp2_prs_vlan_add(struct mvpp2_hw *hw, unsigned short tpid, int ai,
			      unsigned int port_map)
{
	struct mvpp2_prs_entry *pe;
	int tid_aux, tid;
	int ret = 0;

	pe = mvpp2_prs_vlan_find(hw, tpid, ai);

	if (!pe) {
		/* Create new tcam entry */
		tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_LAST_FREE_TID,
						MVPP2_PE_FIRST_FREE_TID);
		if (tid < 0)
			return tid;

		pe = kzalloc(sizeof(*pe), GFP_KERNEL);
		if (!pe)
			return -ENOMEM;

		/* Get last double vlan tid */
		for (tid_aux = MVPP2_PE_LAST_FREE_TID;
		     tid_aux >= MVPP2_PE_FIRST_FREE_TID; tid_aux--) {
			unsigned int ri_bits;

			if (!hw->prs_shadow[tid_aux].valid ||
			    hw->prs_shadow[tid_aux].lu != MVPP2_PRS_LU_VLAN)
				continue;

			pe->index = tid_aux;
			mvpp2_prs_hw_read(hw, pe);
			ri_bits = mvpp2_prs_sram_ri_get(pe);
			if ((ri_bits & MVPP2_PRS_RI_VLAN_MASK) ==
			    MVPP2_PRS_RI_VLAN_DOUBLE)
				break;
		}

		if (tid <= tid_aux) {
			ret = -EINVAL;
			goto error;
		}

		memset(pe, 0 , sizeof(struct mvpp2_prs_entry));
		mvpp2_prs_tcam_lu_set(pe, MVPP2_PRS_LU_VLAN);
		pe->index = tid;

		mvpp2_prs_match_etype(pe, 0, tpid);

		mvpp2_prs_sram_next_lu_set(pe, MVPP2_PRS_LU_L2);
		/* Shift 4 bytes - skip 1 vlan tag */
		mvpp2_prs_sram_shift_set(pe, MVPP2_VLAN_TAG_LEN,
					 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
		/* Clear all ai bits for next iteration */
		mvpp2_prs_sram_ai_update(pe, 0, MVPP2_PRS_SRAM_AI_MASK);

		if (ai == MVPP2_PRS_SINGLE_VLAN_AI) {
			mvpp2_prs_sram_ri_update(pe, MVPP2_PRS_RI_VLAN_SINGLE,
						 MVPP2_PRS_RI_VLAN_MASK);
		} else {
			ai |= MVPP2_PRS_DBL_VLAN_AI_BIT;
			mvpp2_prs_sram_ri_update(pe, MVPP2_PRS_RI_VLAN_TRIPLE,
						 MVPP2_PRS_RI_VLAN_MASK);
		}
		mvpp2_prs_tcam_ai_update(pe, ai, MVPP2_PRS_SRAM_AI_MASK);

		mvpp2_prs_shadow_set(hw, pe->index, MVPP2_PRS_LU_VLAN);
	}
	/* Update ports' mask */
	mvpp2_prs_tcam_port_map_set(pe, port_map);

	mvpp2_prs_hw_write(hw, pe);

error:
	kfree(pe);

	return ret;
}

/* Get first free double vlan ai number */
static int mvpp2_prs_double_vlan_ai_free_get(struct mvpp2_hw *hw)
{
	int i;

	for (i = 1; i < MVPP2_PRS_DBL_VLANS_MAX; i++) {
		if (!hw->prs_double_vlans[i])
			return i;
	}

	return -EINVAL;
}

/* Search for existing double vlan entry */
static struct mvpp2_prs_entry *mvpp2_prs_double_vlan_find(struct mvpp2_hw *hw,
							  unsigned short tpid1,
							  unsigned short tpid2)
{
	struct mvpp2_prs_entry *pe;
	int tid;

	pe = kzalloc(sizeof(*pe), GFP_KERNEL);
	if (!pe)
		return NULL;
	mvpp2_prs_tcam_lu_set(pe, MVPP2_PRS_LU_VLAN);

	/* Go through the all entries with MVPP2_PRS_LU_VLAN */
	for (tid = MVPP2_PE_FIRST_FREE_TID;
	     tid <= MVPP2_PE_LAST_FREE_TID; tid++) {
		unsigned int ri_mask;
		bool match;

		if (!hw->prs_shadow[tid].valid ||
		    hw->prs_shadow[tid].lu != MVPP2_PRS_LU_VLAN)
			continue;

		pe->index = tid;
		mvpp2_prs_hw_read(hw, pe);

		match = mvpp2_prs_tcam_data_cmp(pe, 0, swab16(tpid1))
			&& mvpp2_prs_tcam_data_cmp(pe, 4, swab16(tpid2));

		if (!match)
			continue;

		ri_mask = mvpp2_prs_sram_ri_get(pe) & MVPP2_PRS_RI_VLAN_MASK;
		if (ri_mask == MVPP2_PRS_RI_VLAN_DOUBLE)
			return pe;
	}
	kfree(pe);

	return NULL;
}

/* Add or update double vlan entry */
static int mvpp2_prs_double_vlan_add(struct mvpp2_hw *hw, unsigned short tpid1,
				     unsigned short tpid2,
				     unsigned int port_map)
{
	struct mvpp2_prs_entry *pe;
	int tid_aux, tid, ai, ret = 0;

	pe = mvpp2_prs_double_vlan_find(hw, tpid1, tpid2);

	if (!pe) {
		/* Create new tcam entry */
		tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
				MVPP2_PE_LAST_FREE_TID);
		if (tid < 0)
			return tid;

		pe = kzalloc(sizeof(*pe), GFP_KERNEL);
		if (!pe)
			return -ENOMEM;

		/* Set ai value for new double vlan entry */
		ai = mvpp2_prs_double_vlan_ai_free_get(hw);
		if (ai < 0) {
			ret = ai;
			goto error;
		}

		/* Get first single/triple vlan tid */
		for (tid_aux = MVPP2_PE_FIRST_FREE_TID;
		     tid_aux <= MVPP2_PE_LAST_FREE_TID; tid_aux++) {
			unsigned int ri_bits;

			if (!hw->prs_shadow[tid_aux].valid ||
			    hw->prs_shadow[tid_aux].lu != MVPP2_PRS_LU_VLAN)
				continue;

			pe->index = tid_aux;
			mvpp2_prs_hw_read(hw, pe);
			ri_bits = mvpp2_prs_sram_ri_get(pe);
			ri_bits &= MVPP2_PRS_RI_VLAN_MASK;
			if (ri_bits == MVPP2_PRS_RI_VLAN_SINGLE ||
			    ri_bits == MVPP2_PRS_RI_VLAN_TRIPLE)
				break;
		}

		if (tid >= tid_aux) {
			ret = -ERANGE;
			goto error;
		}

		memset(pe, 0, sizeof(struct mvpp2_prs_entry));
		mvpp2_prs_tcam_lu_set(pe, MVPP2_PRS_LU_VLAN);
		pe->index = tid;

		hw->prs_double_vlans[ai] = true;

		mvpp2_prs_match_etype(pe, 0, tpid1);
		mvpp2_prs_match_etype(pe, 4, tpid2);

		mvpp2_prs_sram_next_lu_set(pe, MVPP2_PRS_LU_VLAN);
		/* Shift 8 bytes - skip 2 vlan tags */
		mvpp2_prs_sram_shift_set(pe, 2 * MVPP2_VLAN_TAG_LEN,
					 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
		mvpp2_prs_sram_ri_update(pe, MVPP2_PRS_RI_VLAN_DOUBLE,
					 MVPP2_PRS_RI_VLAN_MASK);
		mvpp2_prs_sram_ai_update(pe, ai | MVPP2_PRS_DBL_VLAN_AI_BIT,
					 MVPP2_PRS_SRAM_AI_MASK);

		mvpp2_prs_shadow_set(hw, pe->index, MVPP2_PRS_LU_VLAN);
	}

	/* Update ports' mask */
	mvpp2_prs_tcam_port_map_set(pe, port_map);
	mvpp2_prs_hw_write(hw, pe);

error:
	kfree(pe);
	return ret;
}

/* IPv4 header parsing for fragmentation and L4 offset */
static int mvpp2_prs_ip4_proto(struct mvpp2_hw *hw, unsigned short proto,
			       unsigned int ri, unsigned int ri_mask)
{
	struct mvpp2_prs_entry pe;
	int tid;

	if ((proto != IPPROTO_TCP) && (proto != IPPROTO_UDP) &&
	    (proto != IPPROTO_IGMP))
		return -EINVAL;

	/* Fragmented packet */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP4);
	pe.index = tid;

	/* Set next lu to IPv4 */
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP4);
	mvpp2_prs_sram_shift_set(&pe, 12, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	/* Set L4 offset */
	mvpp2_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L4,
				  sizeof(struct iphdr) - 4,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);
	mvpp2_prs_sram_ai_update(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
				 MVPP2_PRS_IPV4_DIP_AI_BIT);
	mvpp2_prs_sram_ri_update(&pe, ri | MVPP2_PRS_RI_IP_FRAG_MASK,
				 ri_mask | MVPP2_PRS_RI_IP_FRAG_MASK);

	mvpp2_prs_tcam_data_byte_set(&pe, 5, proto, MVPP2_PRS_TCAM_PROTO_MASK);
	mvpp2_prs_tcam_ai_update(&pe, 0, MVPP2_PRS_IPV4_DIP_AI_BIT);
	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mvpp2_prs_hw_write(hw, &pe);

	/* Not fragmented packet */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	pe.index = tid;
	/* Clear ri before updating */
	pe.sram.word[MVPP2_PRS_SRAM_RI_WORD] = 0x0;
	pe.sram.word[MVPP2_PRS_SRAM_RI_CTRL_WORD] = 0x0;
	mvpp2_prs_sram_ri_update(&pe, ri, ri_mask);

	mvpp2_prs_tcam_data_byte_set(&pe, 2, 0x00, MVPP2_PRS_TCAM_PROTO_MASK_L);
	mvpp2_prs_tcam_data_byte_set(&pe, 3, 0x00, MVPP2_PRS_TCAM_PROTO_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mvpp2_prs_hw_write(hw, &pe);

	return 0;
}

/* IPv4 L3 multicast or broadcast */
static int mvpp2_prs_ip4_cast(struct mvpp2_hw *hw, unsigned short l3_cast)
{
	struct mvpp2_prs_entry pe;
	int mask, tid;

	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP4);
	pe.index = tid;

	switch (l3_cast) {
	case MVPP2_PRS_L3_MULTI_CAST:
		mvpp2_prs_tcam_data_byte_set(&pe, 0, MVPP2_PRS_IPV4_MC,
					     MVPP2_PRS_IPV4_MC_MASK);
		mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_MCAST,
					 MVPP2_PRS_RI_L3_ADDR_MASK);
		break;
	case  MVPP2_PRS_L3_BROAD_CAST:
		mask = MVPP2_PRS_IPV4_BC_MASK;
		mvpp2_prs_tcam_data_byte_set(&pe, 0, mask, mask);
		mvpp2_prs_tcam_data_byte_set(&pe, 1, mask, mask);
		mvpp2_prs_tcam_data_byte_set(&pe, 2, mask, mask);
		mvpp2_prs_tcam_data_byte_set(&pe, 3, mask, mask);
		mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_BCAST,
					 MVPP2_PRS_RI_L3_ADDR_MASK);
		break;
	default:
		return -EINVAL;
	}

	/* Finished: go to flowid generation */
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);

	mvpp2_prs_tcam_ai_update(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
				 MVPP2_PRS_IPV4_DIP_AI_BIT);
	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mvpp2_prs_hw_write(hw, &pe);

	return 0;
}

/* Set entries for protocols over IPv6  */
static int mvpp2_prs_ip6_proto(struct mvpp2_hw *hw, unsigned short proto,
			       unsigned int ri, unsigned int ri_mask)
{
	struct mvpp2_prs_entry pe;
	int tid;

	if ((proto != IPPROTO_TCP) && (proto != IPPROTO_UDP) &&
	    (proto != IPPROTO_ICMPV6) && (proto != IPPROTO_IPIP))
		return -EINVAL;

	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP6);
	pe.index = tid;

	/* Finished: go to flowid generation */
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mvpp2_prs_sram_ri_update(&pe, ri, ri_mask);
	mvpp2_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L4,
				  sizeof(struct ipv6hdr) - 6,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	mvpp2_prs_tcam_data_byte_set(&pe, 0, proto, MVPP2_PRS_TCAM_PROTO_MASK);
	mvpp2_prs_tcam_ai_update(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
				 MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Write HW */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP6);
	mvpp2_prs_hw_write(hw, &pe);

	return 0;
}

/* IPv6 L3 multicast entry */
static int mvpp2_prs_ip6_cast(struct mvpp2_hw *hw, unsigned short l3_cast)
{
	struct mvpp2_prs_entry pe;
	int tid;

	if (l3_cast != MVPP2_PRS_L3_MULTI_CAST)
		return -EINVAL;

	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP6);
	pe.index = tid;

	/* Finished: go to flowid generation */
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP6);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_MCAST,
				 MVPP2_PRS_RI_L3_ADDR_MASK);
	mvpp2_prs_sram_ai_update(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
				 MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
	/* Shift back to IPv6 NH */
	mvpp2_prs_sram_shift_set(&pe, -18, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

	mvpp2_prs_tcam_data_byte_set(&pe, 0, MVPP2_PRS_IPV6_MC,
				     MVPP2_PRS_IPV6_MC_MASK);
	mvpp2_prs_tcam_ai_update(&pe, 0, MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP6);
	mvpp2_prs_hw_write(hw, &pe);

	return 0;
}

/* Parser per-port initialization */
static void mvpp2_prs_hw_port_init(struct mvpp2_hw *hw, int port, int lu_first,
				   int lu_max, int offset)
{
	u32 val;

	/* Set lookup ID */
	val = mvpp2_read(hw, MVPP2_PRS_INIT_LOOKUP_REG);
	val &= ~MVPP2_PRS_PORT_LU_MASK(port);
	val |=  MVPP2_PRS_PORT_LU_VAL(port, lu_first);
	mvpp2_write(hw, MVPP2_PRS_INIT_LOOKUP_REG, val);

	/* Set maximum number of loops for packet received from port */
	val = mvpp2_read(hw, MVPP2_PRS_MAX_LOOP_REG(port));
	val &= ~MVPP2_PRS_MAX_LOOP_MASK(port);
	val |= MVPP2_PRS_MAX_LOOP_VAL(port, lu_max);
	mvpp2_write(hw, MVPP2_PRS_MAX_LOOP_REG(port), val);

	/* Set initial offset for packet header extraction for the first
	 * searching loop
	 */
	val = mvpp2_read(hw, MVPP2_PRS_INIT_OFFS_REG(port));
	val &= ~MVPP2_PRS_INIT_OFF_MASK(port);
	val |= MVPP2_PRS_INIT_OFF_VAL(port, offset);
	mvpp2_write(hw, MVPP2_PRS_INIT_OFFS_REG(port), val);
}

/* Default flow entries initialization for all ports */
static void mvpp2_prs_def_flow_init(struct mvpp2_hw *hw)
{
	struct mvpp2_prs_entry pe;
	int port;

	for (port = 0; port < MVPP2_MAX_PORTS; port++) {
		memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
		mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
		pe.index = MVPP2_PE_FIRST_DEFAULT_FLOW - port;

		/* Mask all ports */
		mvpp2_prs_tcam_port_map_set(&pe, 0);

		/* Set flow ID*/
		mvpp2_prs_sram_ai_update(&pe, port, MVPP2_PRS_FLOW_ID_MASK);
		mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_DONE_BIT, 1);

		/* Update shadow table and hw entry */
		mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_FLOWS);
		mvpp2_prs_hw_write(hw, &pe);
	}
}

/* Set default entry for Marvell Header field */
static void mvpp2_prs_mh_init(struct mvpp2_hw *hw)
{
	struct mvpp2_prs_entry pe;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));

	pe.index = MVPP2_PE_MH_DEFAULT;
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_MH);
	mvpp2_prs_sram_shift_set(&pe, MVPP2_MH_SIZE,
				 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_MAC);

	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_MH);
	mvpp2_prs_hw_write(hw, &pe);
}

/* Set default entires (place holder) for promiscuous, non-promiscuous and
 * multicast MAC addresses
 */
static void mvpp2_prs_mac_init(struct mvpp2_hw *hw)
{
	struct mvpp2_prs_entry pe;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));

	/* Non-promiscuous mode for all ports - DROP unknown packets */
	pe.index = MVPP2_PE_MAC_NON_PROMISCUOUS;
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_MAC);

	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_DROP_MASK,
				 MVPP2_PRS_RI_DROP_MASK);
	mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);

	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_MAC);
	mvpp2_prs_hw_write(hw, &pe);

	/* place holders only - no ports */
	mvpp2_prs_mac_drop_all_set(hw, 0, false);
	mvpp2_prs_mac_promisc_set(hw, 0, false);
	mvpp2_prs_mac_multi_set(hw, MVPP2_PE_MAC_MC_ALL, 0, false);
	mvpp2_prs_mac_multi_set(hw, MVPP2_PE_MAC_MC_IP6, 0, false);
}

/* Set default entries for various types of dsa packets */
static void mvpp2_prs_dsa_init(struct mvpp2_hw *hw)
{
	struct mvpp2_prs_entry pe;

	/* None tagged EDSA entry - place holder */
	mvpp2_prs_dsa_tag_set(hw, 0, false, MVPP2_PRS_UNTAGGED,
			      MVPP2_PRS_EDSA);

	/* Tagged EDSA entry - place holder */
	mvpp2_prs_dsa_tag_set(hw, 0, false, MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);

	/* None tagged DSA entry - place holder */
	mvpp2_prs_dsa_tag_set(hw, 0, false, MVPP2_PRS_UNTAGGED,
			      MVPP2_PRS_DSA);

	/* Tagged DSA entry - place holder */
	mvpp2_prs_dsa_tag_set(hw, 0, false, MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);

	/* None tagged EDSA ethertype entry - place holder*/
	mvpp2_prs_dsa_tag_ethertype_set(hw, 0, false,
					MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA);

	/* Tagged EDSA ethertype entry - place holder*/
	mvpp2_prs_dsa_tag_ethertype_set(hw, 0, false,
					MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);

	/* None tagged DSA ethertype entry */
	mvpp2_prs_dsa_tag_ethertype_set(hw, 0, true,
					MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA);

	/* Tagged DSA ethertype entry */
	mvpp2_prs_dsa_tag_ethertype_set(hw, 0, true,
					MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);

	/* Set default entry, in case DSA or EDSA tag not found */
	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_DSA);
	pe.index = MVPP2_PE_DSA_DEFAULT;
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_VLAN);

	/* Shift 0 bytes */
	mvpp2_prs_sram_shift_set(&pe, 0, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_MAC);

	/* Clear all sram ai bits for next iteration */
	mvpp2_prs_sram_ai_update(&pe, 0, MVPP2_PRS_SRAM_AI_MASK);

	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	mvpp2_prs_hw_write(hw, &pe);
}

/* Match basic ethertypes */
static int mvpp2_prs_etype_init(struct mvpp2_hw *hw)
{
	struct mvpp2_prs_entry pe;
	int tid;

	/* Ethertype: PPPoE */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_L2);
	pe.index = tid;

	mvpp2_prs_match_etype(&pe, 0, ETH_P_PPP_SES);

	mvpp2_prs_sram_shift_set(&pe, MVPP2_PPPOE_HDR_SIZE,
				 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_PPPOE);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_PPPOE_MASK,
				 MVPP2_PRS_RI_PPPOE_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = false;
	mvpp2_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_PPPOE_MASK,
				MVPP2_PRS_RI_PPPOE_MASK);
	mvpp2_prs_hw_write(hw, &pe);

	/* Ethertype: ARP */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_L2);
	pe.index = tid;

	mvpp2_prs_match_etype(&pe, 0, ETH_P_ARP);

	/* Generate flow in the next iteration*/
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_ARP,
				 MVPP2_PRS_RI_L3_PROTO_MASK);
	/* Set L3 offset */
	mvpp2_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				  MVPP2_ETH_TYPE_LEN,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = true;
	mvpp2_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_L3_ARP,
				MVPP2_PRS_RI_L3_PROTO_MASK);
	mvpp2_prs_hw_write(hw, &pe);

	/* Ethertype: LBTD */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_L2);
	pe.index = tid;

	mvpp2_prs_match_etype(&pe, 0, MVPP2_IP_LBDT_TYPE);

	/* Generate flow in the next iteration*/
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_CPU_CODE_RX_SPEC |
				 MVPP2_PRS_RI_UDF3_RX_SPECIAL,
				 MVPP2_PRS_RI_CPU_CODE_MASK |
				 MVPP2_PRS_RI_UDF3_MASK);
	/* Set L3 offset */
	mvpp2_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				  MVPP2_ETH_TYPE_LEN,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = true;
	mvpp2_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_CPU_CODE_RX_SPEC |
				MVPP2_PRS_RI_UDF3_RX_SPECIAL,
				MVPP2_PRS_RI_CPU_CODE_MASK |
				MVPP2_PRS_RI_UDF3_MASK);
	mvpp2_prs_hw_write(hw, &pe);

	/* Ethertype: IPv4 without options */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_L2);
	pe.index = tid;

	mvpp2_prs_match_etype(&pe, 0, ETH_P_IP);
	mvpp2_prs_tcam_data_byte_set(&pe, MVPP2_ETH_TYPE_LEN,
				     MVPP2_PRS_IPV4_HEAD | MVPP2_PRS_IPV4_IHL,
				     MVPP2_PRS_IPV4_HEAD_MASK |
				     MVPP2_PRS_IPV4_IHL_MASK);

	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP4);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_IP4,
				 MVPP2_PRS_RI_L3_PROTO_MASK);
	/* Skip eth_type + 4 bytes of IP header */
	mvpp2_prs_sram_shift_set(&pe, MVPP2_ETH_TYPE_LEN + 4,
				 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	/* Set L3 offset */
	mvpp2_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				  MVPP2_ETH_TYPE_LEN,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = false;
	mvpp2_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_L3_IP4,
				MVPP2_PRS_RI_L3_PROTO_MASK);
	mvpp2_prs_hw_write(hw, &pe);

	/* Ethertype: IPv4 with options */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	pe.index = tid;

	/* Clear tcam data before updating */
	pe.tcam.byte[MVPP2_PRS_TCAM_DATA_BYTE(MVPP2_ETH_TYPE_LEN)] = 0x0;
	pe.tcam.byte[MVPP2_PRS_TCAM_DATA_BYTE_EN(MVPP2_ETH_TYPE_LEN)] = 0x0;

	mvpp2_prs_tcam_data_byte_set(&pe, MVPP2_ETH_TYPE_LEN,
				     MVPP2_PRS_IPV4_HEAD,
				     MVPP2_PRS_IPV4_HEAD_MASK);

	/* Clear ri before updating */
	pe.sram.word[MVPP2_PRS_SRAM_RI_WORD] = 0x0;
	pe.sram.word[MVPP2_PRS_SRAM_RI_CTRL_WORD] = 0x0;
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_IP4_OPT,
				 MVPP2_PRS_RI_L3_PROTO_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = false;
	mvpp2_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_L3_IP4_OPT,
				MVPP2_PRS_RI_L3_PROTO_MASK);
	mvpp2_prs_hw_write(hw, &pe);

	/* Ethertype: IPv6 without options */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_L2);
	pe.index = tid;

	mvpp2_prs_match_etype(&pe, 0, ETH_P_IPV6);

	/* Skip DIP of IPV6 header */
	mvpp2_prs_sram_shift_set(&pe, MVPP2_ETH_TYPE_LEN + 8 +
				 MVPP2_MAX_L3_ADDR_SIZE,
				 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP6);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_IP6,
				 MVPP2_PRS_RI_L3_PROTO_MASK);
	/* Set L3 offset */
	mvpp2_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				  MVPP2_ETH_TYPE_LEN,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = false;
	mvpp2_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_L3_IP6,
				MVPP2_PRS_RI_L3_PROTO_MASK);
	mvpp2_prs_hw_write(hw, &pe);

	/* Default entry for MVPP2_PRS_LU_L2 - Unknown ethtype */
	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_L2);
	pe.index = MVPP2_PE_ETH_TYPE_UN;

	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Generate flow in the next iteration*/
	mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_UN,
				 MVPP2_PRS_RI_L3_PROTO_MASK);
	/* Set L3 offset even it's unknown L3 */
	mvpp2_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				  MVPP2_ETH_TYPE_LEN,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = true;
	mvpp2_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_L3_UN,
				MVPP2_PRS_RI_L3_PROTO_MASK);
	mvpp2_prs_hw_write(hw, &pe);

	return 0;
}

/* Configure vlan entries and detect up to 2 successive VLAN tags.
 * Possible options:
 * 0x8100, 0x88A8
 * 0x8100, 0x8100
 * 0x8100
 * 0x88A8
 */
static int mvpp2_prs_vlan_init(struct platform_device *pdev, struct mvpp2_hw *hw)
{
	struct mvpp2_prs_entry pe;
	int err;

	hw->prs_double_vlans = devm_kcalloc(&pdev->dev, sizeof(bool),
					      MVPP2_PRS_DBL_VLANS_MAX,
					      GFP_KERNEL);
	if (!hw->prs_double_vlans)
		return -ENOMEM;

	/* Double VLAN: 0x8100, 0x88A8 */
	err = mvpp2_prs_double_vlan_add(hw, ETH_P_8021Q, ETH_P_8021AD,
					MVPP2_PRS_PORT_MASK);
	if (err)
		return err;

	/* Double VLAN: 0x8100, 0x8100 */
	err = mvpp2_prs_double_vlan_add(hw, ETH_P_8021Q, ETH_P_8021Q,
					MVPP2_PRS_PORT_MASK);
	if (err)
		return err;

	/* Single VLAN: 0x88a8 */
	err = mvpp2_prs_vlan_add(hw, ETH_P_8021AD, MVPP2_PRS_SINGLE_VLAN_AI,
				 MVPP2_PRS_PORT_MASK);
	if (err)
		return err;

	/* Single VLAN: 0x8100 */
	err = mvpp2_prs_vlan_add(hw, ETH_P_8021Q, MVPP2_PRS_SINGLE_VLAN_AI,
				 MVPP2_PRS_PORT_MASK);
	if (err)
		return err;

	/* Set default double vlan entry */
	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_VLAN);
	pe.index = MVPP2_PE_VLAN_DBL;

	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_L2);
	/* Clear ai for next iterations */
	mvpp2_prs_sram_ai_update(&pe, 0, MVPP2_PRS_SRAM_AI_MASK);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_VLAN_DOUBLE,
				 MVPP2_PRS_RI_VLAN_MASK);

	mvpp2_prs_tcam_ai_update(&pe, MVPP2_PRS_DBL_VLAN_AI_BIT,
				 MVPP2_PRS_DBL_VLAN_AI_BIT);
	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_VLAN);
	mvpp2_prs_hw_write(hw, &pe);

	/* Set default vlan none entry */
	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_VLAN);
	pe.index = MVPP2_PE_VLAN_NONE;

	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_L2);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_VLAN_NONE,
				 MVPP2_PRS_RI_VLAN_MASK);

	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_VLAN);
	mvpp2_prs_hw_write(hw, &pe);

	return 0;
}

/* Set entries for PPPoE ethertype */
static int mvpp2_prs_pppoe_init(struct mvpp2_hw *hw)
{
	struct mvpp2_prs_entry pe;
	int tid;

	/* IPv4 over PPPoE with options */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_PPPOE);
	pe.index = tid;

	mvpp2_prs_match_etype(&pe, 0, PPP_IP);

	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP4);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_IP4_OPT,
				 MVPP2_PRS_RI_L3_PROTO_MASK);
	/* Skip eth_type + 4 bytes of IP header */
	mvpp2_prs_sram_shift_set(&pe, MVPP2_ETH_TYPE_LEN + 4,
				 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	/* Set L3 offset */
	mvpp2_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				  MVPP2_ETH_TYPE_LEN,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_PPPOE);
	mvpp2_prs_hw_write(hw, &pe);

	/* IPv4 over PPPoE without options */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	pe.index = tid;

	mvpp2_prs_tcam_data_byte_set(&pe, MVPP2_ETH_TYPE_LEN,
				     MVPP2_PRS_IPV4_HEAD | MVPP2_PRS_IPV4_IHL,
				     MVPP2_PRS_IPV4_HEAD_MASK |
				     MVPP2_PRS_IPV4_IHL_MASK);

	/* Clear ri before updating */
	pe.sram.word[MVPP2_PRS_SRAM_RI_WORD] = 0x0;
	pe.sram.word[MVPP2_PRS_SRAM_RI_CTRL_WORD] = 0x0;
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_IP4,
				 MVPP2_PRS_RI_L3_PROTO_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_PPPOE);
	mvpp2_prs_hw_write(hw, &pe);

	/* IPv6 over PPPoE */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_PPPOE);
	pe.index = tid;

	mvpp2_prs_match_etype(&pe, 0, PPP_IPV6);

	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP6);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_IP6,
				 MVPP2_PRS_RI_L3_PROTO_MASK);
	/* Skip eth_type + 4 bytes of IPv6 header */
	mvpp2_prs_sram_shift_set(&pe, MVPP2_ETH_TYPE_LEN + 4,
				 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	/* Set L3 offset */
	mvpp2_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				  MVPP2_ETH_TYPE_LEN,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_PPPOE);
	mvpp2_prs_hw_write(hw, &pe);

	/* Non-IP over PPPoE */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_PPPOE);
	pe.index = tid;

	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_UN,
				 MVPP2_PRS_RI_L3_PROTO_MASK);

	/* Finished: go to flowid generation */
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	/* Set L3 offset even if it's unknown L3 */
	mvpp2_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				  MVPP2_ETH_TYPE_LEN,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_PPPOE);
	mvpp2_prs_hw_write(hw, &pe);

	return 0;
}

/* Initialize entries for IPv4 */
static int mvpp2_prs_ip4_init(struct mvpp2_hw *hw)
{
	struct mvpp2_prs_entry pe;
	int err;

	/* Set entries for TCP, UDP and IGMP over IPv4 */
	err = mvpp2_prs_ip4_proto(hw, IPPROTO_TCP, MVPP2_PRS_RI_L4_TCP,
				  MVPP2_PRS_RI_L4_PROTO_MASK);
	if (err)
		return err;

	err = mvpp2_prs_ip4_proto(hw, IPPROTO_UDP, MVPP2_PRS_RI_L4_UDP,
				  MVPP2_PRS_RI_L4_PROTO_MASK);
	if (err)
		return err;

	err = mvpp2_prs_ip4_proto(hw, IPPROTO_IGMP,
				  MVPP2_PRS_RI_CPU_CODE_RX_SPEC |
				  MVPP2_PRS_RI_UDF3_RX_SPECIAL,
				  MVPP2_PRS_RI_CPU_CODE_MASK |
				  MVPP2_PRS_RI_UDF3_MASK);
	if (err)
		return err;

	/* IPv4 Broadcast */
	err = mvpp2_prs_ip4_cast(hw, MVPP2_PRS_L3_BROAD_CAST);
	if (err)
		return err;

	/* IPv4 Multicast */
	err = mvpp2_prs_ip4_cast(hw, MVPP2_PRS_L3_MULTI_CAST);
	if (err)
		return err;

	/* Default IPv4 entry for unknown protocols */
	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP4);
	pe.index = MVPP2_PE_IP4_PROTO_UN;

	/* Set next lu to IPv4 */
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP4);
	mvpp2_prs_sram_shift_set(&pe, 12, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	/* Set L4 offset */
	mvpp2_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L4,
				  sizeof(struct iphdr) - 4,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);
	mvpp2_prs_sram_ai_update(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
				 MVPP2_PRS_IPV4_DIP_AI_BIT);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L4_OTHER,
				 MVPP2_PRS_RI_L4_PROTO_MASK);

	mvpp2_prs_tcam_ai_update(&pe, 0, MVPP2_PRS_IPV4_DIP_AI_BIT);
	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mvpp2_prs_hw_write(hw, &pe);

	/* Default IPv4 entry for unicast address */
	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP4);
	pe.index = MVPP2_PE_IP4_ADDR_UN;

	/* Finished: go to flowid generation */
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_UCAST,
				 MVPP2_PRS_RI_L3_ADDR_MASK);

	mvpp2_prs_tcam_ai_update(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
				 MVPP2_PRS_IPV4_DIP_AI_BIT);
	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mvpp2_prs_hw_write(hw, &pe);

	return 0;
}

/* Initialize entries for IPv6 */
static int mvpp2_prs_ip6_init(struct mvpp2_hw *hw)
{
	struct mvpp2_prs_entry pe;
	int tid, err;

	/* Set entries for TCP, UDP and ICMP over IPv6 */
	err = mvpp2_prs_ip6_proto(hw, IPPROTO_TCP,
				  MVPP2_PRS_RI_L4_TCP,
				  MVPP2_PRS_RI_L4_PROTO_MASK);
	if (err)
		return err;

	err = mvpp2_prs_ip6_proto(hw, IPPROTO_UDP,
				  MVPP2_PRS_RI_L4_UDP,
				  MVPP2_PRS_RI_L4_PROTO_MASK);
	if (err)
		return err;

	err = mvpp2_prs_ip6_proto(hw, IPPROTO_ICMPV6,
				  MVPP2_PRS_RI_CPU_CODE_RX_SPEC |
				  MVPP2_PRS_RI_UDF3_RX_SPECIAL,
				  MVPP2_PRS_RI_CPU_CODE_MASK |
				  MVPP2_PRS_RI_UDF3_MASK);
	if (err)
		return err;

	/* IPv4 is the last header. This is similar case as 6-TCP or 17-UDP */
	/* Result Info: UDF7=1, DS lite */
	err = mvpp2_prs_ip6_proto(hw, IPPROTO_IPIP,
				  MVPP2_PRS_RI_UDF7_IP6_LITE,
				  MVPP2_PRS_RI_UDF7_MASK);
	if (err)
		return err;

	/* IPv6 multicast */
	err = mvpp2_prs_ip6_cast(hw, MVPP2_PRS_L3_MULTI_CAST);
	if (err)
		return err;

	/* Entry for checking hop limit */
	tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP6);
	pe.index = tid;

	/* Finished: go to flowid generation */
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_UN |
				 MVPP2_PRS_RI_DROP_MASK,
				 MVPP2_PRS_RI_L3_PROTO_MASK |
				 MVPP2_PRS_RI_DROP_MASK);

	mvpp2_prs_tcam_data_byte_set(&pe, 1, 0x00, MVPP2_PRS_IPV6_HOP_MASK);
	mvpp2_prs_tcam_ai_update(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
				 MVPP2_PRS_IPV6_NO_EXT_AI_BIT);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mvpp2_prs_hw_write(hw, &pe);

	/* Default IPv6 entry for unknown protocols */
	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP6);
	pe.index = MVPP2_PE_IP6_PROTO_UN;

	/* Finished: go to flowid generation */
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L4_OTHER,
				 MVPP2_PRS_RI_L4_PROTO_MASK);
	/* Set L4 offset relatively to our current place */
	mvpp2_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L4,
				  sizeof(struct ipv6hdr) - 4,
				  MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	mvpp2_prs_tcam_ai_update(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
				 MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mvpp2_prs_hw_write(hw, &pe);

	/* Default IPv6 entry for unknown ext protocols */
	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP6);
	pe.index = MVPP2_PE_IP6_EXT_PROTO_UN;

	/* Finished: go to flowid generation */
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mvpp2_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L4_OTHER,
				 MVPP2_PRS_RI_L4_PROTO_MASK);

	mvpp2_prs_tcam_ai_update(&pe, MVPP2_PRS_IPV6_EXT_AI_BIT,
				 MVPP2_PRS_IPV6_EXT_AI_BIT);
	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mvpp2_prs_hw_write(hw, &pe);

	/* Default IPv6 entry for unicast address */
	memset(&pe, 0, sizeof(struct mvpp2_prs_entry));
	mvpp2_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP6);
	pe.index = MVPP2_PE_IP6_ADDR_UN;

	/* Finished: go to IPv6 again */
	mvpp2_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP6);
	mvpp2_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_UCAST,
				 MVPP2_PRS_RI_L3_ADDR_MASK);
	mvpp2_prs_sram_ai_update(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
				 MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
	/* Shift back to IPV6 NH */
	mvpp2_prs_sram_shift_set(&pe, -18, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

	mvpp2_prs_tcam_ai_update(&pe, 0, MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
	/* Unmask all ports */
	mvpp2_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mvpp2_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP6);
	mvpp2_prs_hw_write(hw, &pe);

	return 0;
}


/* Compare MAC DA with tcam entry data */
static bool mvpp2_prs_mac_range_equals(struct mvpp2_prs_entry *pe,
				       const u8 *da, unsigned char *mask)
{
	unsigned char tcam_byte, tcam_mask;
	int index;

	for (index = 0; index < ETH_ALEN; index++) {
		mvpp2_prs_tcam_data_byte_get(pe, index, &tcam_byte, &tcam_mask);
		if (tcam_mask != mask[index])
			return false;

		if ((tcam_mask & tcam_byte) != (da[index] & mask[index]))
			return false;
	}

	return true;
}

/* Find tcam entry with matched pair <MAC DA, port> */
static struct mvpp2_prs_entry *
mvpp2_prs_mac_da_range_find(struct mvpp2_hw *hw, int pmap, const u8 *da,
			    unsigned char *mask, int udf_type)
{
	struct mvpp2_prs_entry *pe;
	int tid;

	pe = kzalloc(sizeof(*pe), GFP_KERNEL);
	if (!pe)
		return NULL;
	mvpp2_prs_tcam_lu_set(pe, MVPP2_PRS_LU_MAC);

	/* Go through the all entires with MVPP2_PRS_LU_MAC */
	for (tid = MVPP2_PE_FIRST_FREE_TID;
	     tid <= MVPP2_PE_LAST_FREE_TID; tid++) {
		unsigned int entry_pmap;

		if (!hw->prs_shadow[tid].valid ||
		    (hw->prs_shadow[tid].lu != MVPP2_PRS_LU_MAC) ||
		    (hw->prs_shadow[tid].udf != udf_type))
			continue;

		pe->index = tid;
		mvpp2_prs_hw_read(hw, pe);
		entry_pmap = mvpp2_prs_tcam_port_map_get(pe);

		if (mvpp2_prs_mac_range_equals(pe, da, mask) &&
		    entry_pmap == pmap)
			return pe;
	}
	kfree(pe);

	return NULL;
}

/* Update parser's mac da entry */
int mvpp2_prs_mac_da_accept(struct mvpp2_hw *hw, int port,
				   const u8 *da, bool add)
{
	struct mvpp2_prs_entry *pe;
	unsigned int pmap, len, ri;
	unsigned char mask[ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	int tid;

	/* Scan TCAM and see if entry with this <MAC DA, port> already exist */
	pe = mvpp2_prs_mac_da_range_find(hw, (1 << port), da, mask,
					 MVPP2_PRS_UDF_MAC_DEF);

	/* No such entry */
	if (!pe) {
		if (!add)
			return 0;

		/* Create new TCAM entry */
		/* Find first range mac entry*/
		for (tid = MVPP2_PE_FIRST_FREE_TID;
		     tid <= MVPP2_PE_LAST_FREE_TID; tid++)
			if (hw->prs_shadow[tid].valid &&
			    (hw->prs_shadow[tid].lu == MVPP2_PRS_LU_MAC) &&
			    (hw->prs_shadow[tid].udf ==
						       MVPP2_PRS_UDF_MAC_RANGE))
				break;

		/* Go through the all entries from first to last */
		tid = mvpp2_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
						tid - 1);
		if (tid < 0)
			return tid;

		pe = kzalloc(sizeof(*pe), GFP_KERNEL);
		if (!pe)
			return -1;
		mvpp2_prs_tcam_lu_set(pe, MVPP2_PRS_LU_MAC);
		pe->index = tid;

		/* Mask all ports */
		mvpp2_prs_tcam_port_map_set(pe, 0);
	}

	/* Update port mask */
	mvpp2_prs_tcam_port_set(pe, port, add);

	/* Invalidate the entry if no ports are left enabled */
	pmap = mvpp2_prs_tcam_port_map_get(pe);
	if (pmap == 0) {
		if (add) {
			kfree(pe);
			return -1;
		}
		mvpp2_prs_hw_inv(hw, pe->index);
		hw->prs_shadow[pe->index].valid = false;
		kfree(pe);
		return 0;
	}

	/* Continue - set next lookup */
	mvpp2_prs_sram_next_lu_set(pe, MVPP2_PRS_LU_DSA);

	/* Set match on DA */
	len = ETH_ALEN;
	while (len--)
		mvpp2_prs_tcam_data_byte_set(pe, len, da[len], 0xff);

	/* Set result info bits */
	if (is_broadcast_ether_addr(da))
		ri = MVPP2_PRS_RI_L2_BCAST;
	else if (is_multicast_ether_addr(da))
		ri = MVPP2_PRS_RI_L2_MCAST;
	else
		ri = MVPP2_PRS_RI_L2_UCAST | MVPP2_PRS_RI_MAC_ME_MASK;

	mvpp2_prs_sram_ri_update(pe, ri, MVPP2_PRS_RI_L2_CAST_MASK |
				 MVPP2_PRS_RI_MAC_ME_MASK);
	mvpp2_prs_shadow_ri_set(hw, pe->index, ri, MVPP2_PRS_RI_L2_CAST_MASK |
				MVPP2_PRS_RI_MAC_ME_MASK);

	/* Shift to ethertype */
	mvpp2_prs_sram_shift_set(pe, 2 * ETH_ALEN,
				 MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

	/* Update shadow table and hw entry */
	hw->prs_shadow[pe->index].udf = MVPP2_PRS_UDF_MAC_DEF;
	mvpp2_prs_shadow_set(hw, pe->index, MVPP2_PRS_LU_MAC);
	mvpp2_prs_hw_write(hw, pe);

	kfree(pe);

	return 0;
}

int mvpp2_prs_update_mac_da(struct net_device *dev, const u8 *da)
{
	struct mvpp2_port *port = netdev_priv(dev);
	int err;

	/* Remove old parser entry */
	err = mvpp2_prs_mac_da_accept(&(port->priv->hw), port->id, dev->dev_addr,
				      false);
	if (err)
		return err;

	/* Add new parser entry */
	err = mvpp2_prs_mac_da_accept(&(port->priv->hw), port->id, da, true);
	if (err)
		return err;

	/* Set addr in the device */
	ether_addr_copy(dev->dev_addr, da);

	return 0;
}

/* Delete all port's multicast simple (not range) entries */
void mvpp2_prs_mcast_del_all(struct mvpp2_hw *hw, int port)
{
	struct mvpp2_prs_entry pe;
	int index, tid;

	for (tid = MVPP2_PE_FIRST_FREE_TID;
	     tid <= MVPP2_PE_LAST_FREE_TID; tid++) {
		unsigned char da[ETH_ALEN], da_mask[ETH_ALEN];

		if (!hw->prs_shadow[tid].valid ||
		    (hw->prs_shadow[tid].lu != MVPP2_PRS_LU_MAC) ||
		    (hw->prs_shadow[tid].udf != MVPP2_PRS_UDF_MAC_DEF))
			continue;

		/* Only simple mac entries */
		pe.index = tid;
		mvpp2_prs_hw_read(hw, &pe);

		/* Read mac addr from entry */
		for (index = 0; index < ETH_ALEN; index++)
			mvpp2_prs_tcam_data_byte_get(&pe, index, &da[index],
						     &da_mask[index]);

		if (is_multicast_ether_addr(da) && !is_broadcast_ether_addr(da))
			/* Delete this entry */
			mvpp2_prs_mac_da_accept(hw, port, da, false);
	}
}

int mvpp2_prs_tag_mode_set(struct mvpp2_hw *hw, int port, int type)
{
	switch (type) {
	case MVPP2_TAG_TYPE_EDSA:
		/* Add port to EDSA entries */
		mvpp2_prs_dsa_tag_set(hw, port, true,
				      MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);
		mvpp2_prs_dsa_tag_set(hw, port, true,
				      MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA);
		/* Remove port from DSA entries */
		mvpp2_prs_dsa_tag_set(hw, port, false,
				      MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);
		mvpp2_prs_dsa_tag_set(hw, port, false,
				      MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA);
		break;

	case MVPP2_TAG_TYPE_DSA:
		/* Add port to DSA entries */
		mvpp2_prs_dsa_tag_set(hw, port, true,
				      MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);
		mvpp2_prs_dsa_tag_set(hw, port, true,
				      MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA);
		/* Remove port from EDSA entries */
		mvpp2_prs_dsa_tag_set(hw, port, false,
				      MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);
		mvpp2_prs_dsa_tag_set(hw, port, false,
				      MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA);
		break;

	case MVPP2_TAG_TYPE_MH:
	case MVPP2_TAG_TYPE_NONE:
		/* Remove port form EDSA and DSA entries */
		mvpp2_prs_dsa_tag_set(hw, port, false,
				      MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);
		mvpp2_prs_dsa_tag_set(hw, port, false,
				      MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA);
		mvpp2_prs_dsa_tag_set(hw, port, false,
				      MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);
		mvpp2_prs_dsa_tag_set(hw, port, false,
				      MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA);
		break;

	default:
		if ((type < 0) || (type > MVPP2_TAG_TYPE_EDSA))
			return -EINVAL;
	}

	return 0;
}

/* Set prs flow for the port */
int mvpp2_prs_def_flow(struct mvpp2_port *port)
{
	struct mvpp2_prs_entry *pe;
	struct mvpp2_hw *hw = &(port->priv->hw);
	int tid;

	pe = mvpp2_prs_flow_find(hw, port->id);

	/* Such entry not exist */
	if (!pe) {
		/* Go through the all entires from last to first */
		tid = mvpp2_prs_tcam_first_free(hw,
						MVPP2_PE_LAST_FREE_TID,
					       MVPP2_PE_FIRST_FREE_TID);
		if (tid < 0)
			return tid;

		pe = kzalloc(sizeof(*pe), GFP_KERNEL);
		if (!pe)
			return -ENOMEM;

		mvpp2_prs_tcam_lu_set(pe, MVPP2_PRS_LU_FLOWS);
		pe->index = tid;

		/* Set flow ID*/
		mvpp2_prs_sram_ai_update(pe, port->id, MVPP2_PRS_FLOW_ID_MASK);
		mvpp2_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_LU_DONE_BIT, 1);

		/* Update shadow table */
		mvpp2_prs_shadow_set(hw, pe->index, MVPP2_PRS_LU_FLOWS);
	}

	mvpp2_prs_tcam_port_map_set(pe, (1 << port->id));
	mvpp2_prs_hw_write(hw, pe);
	kfree(pe);

	return 0;
}

/* Classifier configuration routines */

/* Update classification flow table registers */
static void mvpp2_cls_flow_write(struct mvpp2_hw *hw,
				 struct mvpp2_cls_flow_entry *fe)
{
	mvpp2_write(hw, MVPP2_CLS_FLOW_INDEX_REG, fe->index);
	mvpp2_write(hw, MVPP2_CLS_FLOW_TBL0_REG,  fe->data[0]);
	mvpp2_write(hw, MVPP2_CLS_FLOW_TBL1_REG,  fe->data[1]);
	mvpp2_write(hw, MVPP2_CLS_FLOW_TBL2_REG,  fe->data[2]);
}

/* Update classification lookup table register */
static void mvpp2_cls_lookup_write(struct mvpp2_hw *hw,
				   struct mvpp2_cls_lookup_entry *le)
{
	u32 val;

	val = (le->way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) | le->lkpid;
	mvpp2_write(hw, MVPP2_CLS_LKP_INDEX_REG, val);
	mvpp2_write(hw, MVPP2_CLS_LKP_TBL_REG, le->data);
}

/* Classifier default initialization */
void mvpp2_cls_init(struct mvpp2_hw *hw)
{
	struct mvpp2_cls_lookup_entry le;
	struct mvpp2_cls_flow_entry fe;
	int index;

	/* Enable classifier */
	mvpp2_write(hw, MVPP2_CLS_MODE_REG, MVPP2_CLS_MODE_ACTIVE_MASK);

	/* Clear classifier flow table */
	memset(&fe.data, 0, MVPP2_CLS_FLOWS_TBL_DATA_WORDS);
	for (index = 0; index < MVPP2_CLS_FLOWS_TBL_SIZE; index++) {
		fe.index = index;
		mvpp2_cls_flow_write(hw, &fe);
	}

	/* Clear classifier lookup table */
	le.data = 0;
	for (index = 0; index < MVPP2_CLS_LKP_TBL_SIZE; index++) {
		le.lkpid = index;
		le.way = 0;
		mvpp2_cls_lookup_write(hw, &le);

		le.way = 1;
		mvpp2_cls_lookup_write(hw, &le);
	}
}

void mvpp2_cls_port_config(struct mvpp2_port *port)
{
	struct mvpp2_cls_lookup_entry le;
	struct mvpp2_hw *hw = &(port->priv->hw);
	u32 val;

	/* Set way for the port */
	val = mvpp2_read(hw, MVPP2_CLS_PORT_WAY_REG);
	val &= ~MVPP2_CLS_PORT_WAY_MASK(port->id);
	mvpp2_write(hw, MVPP2_CLS_PORT_WAY_REG, val);

	/* Pick the entry to be accessed in lookup ID decoding table
	 * according to the way and lkpid.
	 */
	le.lkpid = port->id;
	le.way = 0;
	le.data = 0;

	/* Set initial CPU queue for receiving packets */
	le.data &= ~MVPP2_CLS_LKP_TBL_RXQ_MASK;
	le.data |= port->first_rxq;

	/* Disable classification engines */
	le.data &= ~MVPP2_CLS_LKP_TBL_LOOKUP_EN_MASK;

	/* Update lookup ID table entry */
	mvpp2_cls_lookup_write(hw, &le);
}

/* Set CPU queue number for oversize packets */
void mvpp2_cls_oversize_rxq_set(struct mvpp2_port *port)
{
	u32 val;
	struct mvpp2_hw *hw = &(port->priv->hw);

	mvpp2_write(hw, MVPP2_CLS_OVERSIZE_RXQ_LOW_REG(port->id),
		    port->first_rxq & MVPP2_CLS_OVERSIZE_RXQ_LOW_MASK);

	mvpp2_write(hw, MVPP2_CLS_SWFWD_P2HQ_REG(port->id),
		    (port->first_rxq >> MVPP2_CLS_OVERSIZE_RXQ_LOW_BITS));

	val = mvpp2_read(hw, MVPP2_CLS_SWFWD_PCTRL_REG);
	val |= MVPP2_CLS_SWFWD_PCTRL_MASK(port->id);
	mvpp2_write(hw, MVPP2_CLS_SWFWD_PCTRL_REG, val);
}


void mvpp2_get_mac_address(struct mvpp2_port *port, unsigned char *addr)
{
	u32 mac_addr_l, mac_addr_m, mac_addr_h;

	mac_addr_l = readl(port->base + MVPP2_GMAC_CTRL_1_REG);
	mac_addr_m = readl(port->priv->hw.lms_base + MVPP2_SRC_ADDR_MIDDLE);
	mac_addr_h = readl(port->priv->hw.lms_base + MVPP2_SRC_ADDR_HIGH);
	addr[0] = (mac_addr_h >> 24) & 0xFF;
	addr[1] = (mac_addr_h >> 16) & 0xFF;
	addr[2] = (mac_addr_h >> 8) & 0xFF;
	addr[3] = mac_addr_h & 0xFF;
	addr[4] = mac_addr_m & 0xFF;
	addr[5] = (mac_addr_l >> MVPP2_GMAC_SA_LOW_OFFS) & 0xFF;
}

void mvpp2_cause_error(struct net_device *dev, int cause)
{
	if (cause & MVPP2_CAUSE_FCS_ERR_MASK)
		netdev_err(dev, "FCS error\n");
	if (cause & MVPP2_CAUSE_RX_FIFO_OVERRUN_MASK)
		netdev_err(dev, "rx fifo overrun error\n");
	if (cause & MVPP2_CAUSE_TX_FIFO_UNDERRUN_MASK)
		netdev_err(dev, "tx fifo underrun error\n");
}

/* Display more error info */
void mvpp2_rx_error(struct mvpp2_port *port,
			   struct mvpp2_rx_desc *rx_desc)
{
	u32 status = rx_desc->status;

	switch (status & MVPP2_RXD_ERR_CODE_MASK) {
	case MVPP2_RXD_ERR_CRC:
		netdev_err(port->dev, "bad rx status %08x (crc error), size=%d\n",
			   status, rx_desc->data_size);
		break;
	case MVPP2_RXD_ERR_OVERRUN:
		netdev_err(port->dev, "bad rx status %08x (overrun error), size=%d\n",
			   status, rx_desc->data_size);
		break;
	case MVPP2_RXD_ERR_RESOURCE:
		netdev_err(port->dev, "bad rx status %08x (resource error), size=%d\n",
			   status, rx_desc->data_size);
		break;
	}
}

/* Handle RX checksum offload */
void mvpp2_rx_csum(struct mvpp2_port *port, u32 status,
			  struct sk_buff *skb)
{
	if (((status & MVPP2_RXD_L3_IP4) &&
	     !(status & MVPP2_RXD_IP4_HEADER_ERR)) ||
	    (status & MVPP2_RXD_L3_IP6))
		if (((status & MVPP2_RXD_L4_UDP) ||
		     (status & MVPP2_RXD_L4_TCP)) &&
		     (status & MVPP2_RXD_L4_CSUM_OK)) {
			skb->csum = 0;
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			return;
		}

	skb->ip_summed = CHECKSUM_NONE;
}


/* Set the number of packets that will be received before Rx interrupt
 * will be generated by HW.
 */
void mvpp2_rx_pkts_coal_set(struct mvpp2_port *port,
				   struct mvpp2_rx_queue *rxq, u32 pkts)
{
	u32 val;

	val = (pkts & MVPP2_OCCUPIED_THRESH_MASK);
	mvpp2_write(&(port->priv->hw), MVPP2_RXQ_NUM_REG, rxq->id);
	mvpp2_write(&(port->priv->hw), MVPP2_RXQ_THRESH_REG, val);

	rxq->pkts_coal = pkts;
}

/* Set the time delay in usec before Rx interrupt */
void mvpp2_rx_time_coal_set(struct mvpp2_port *port,
				   struct mvpp2_rx_queue *rxq, u32 usec)
{
	u32 val;

	val = (port->priv->hw.tclk / USEC_PER_SEC) * usec;
	mvpp2_write(&(port->priv->hw), MVPP2_ISR_RX_THRESHOLD_REG(rxq->id), val);
}

/* Set threshold for TX_DONE pkts coalescing */
void mvpp2_tx_done_pkts_coal_set(void *arg)
{
	struct mvpp2_port *port = arg;
	int queue;
	u32 val;

	for (queue = 0; queue < port->num_tx_queues; queue++) {
		struct mvpp2_tx_queue *txq = port->txqs[queue];

		val = (txq->pkts_coal << MVPP2_TRANSMITTED_THRESH_OFFSET) &
		       MVPP2_TRANSMITTED_THRESH_MASK;
		mvpp2_write(&(port->priv->hw), MVPP2_TXQ_NUM_REG, txq->id);
		mvpp2_write(&(port->priv->hw), MVPP2_TXQ_THRESH_REG, val);
	}
}

/* Set the time delay in usec before Rx interrupt */
void mvpp2_tx_done_time_coal_set(struct mvpp2_port *port, u32 usec)
{
	u32 val;

	val = (port->priv->hw.tclk / USEC_PER_SEC) * usec;
	mvpp2_write(&(port->priv->hw), MVPP22_ISR_TX_THRESHOLD_REG(port->id), val);
}


/* Change maximum receive size of the port */
void mvpp2_gmac_max_rx_size_set(struct mvpp2_port *port)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_CTRL_0_REG);
	val &= ~MVPP2_GMAC_MAX_RX_SIZE_MASK;
	val |= (((port->pkt_size - MVPP2_MH_SIZE) / 2) <<
		    MVPP2_GMAC_MAX_RX_SIZE_OFFS);
	writel(val, port->base + MVPP2_GMAC_CTRL_0_REG);
}


/* Set max sizes for Tx queues */
void mvpp2_txp_max_tx_size_set(struct mvpp2_port *port)
{
	u32	val, size, mtu;
	int	txq, tx_port_num;
	struct mvpp2_hw *hw = &(port->priv->hw);


	mtu = port->pkt_size * 8;
	if (mtu > MVPP2_TXP_MTU_MAX)
		mtu = MVPP2_TXP_MTU_MAX;

	/* WA for wrong Token bucket update: Set MTU value = 3*real MTU value */
	mtu = 3 * mtu;

	/* Indirect access to registers */
	tx_port_num = mvpp2_egress_port(port);
	mvpp2_write(hw, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);

	/* Set MTU */
	val = mvpp2_read(hw, MVPP2_TXP_SCHED_MTU_REG);
	val &= ~MVPP2_TXP_MTU_MAX;
	val |= mtu;
	mvpp2_write(hw, MVPP2_TXP_SCHED_MTU_REG, val);

	/* TXP token size and all TXQs token size must be larger that MTU */
	val = mvpp2_read(hw, MVPP2_TXP_SCHED_TOKEN_SIZE_REG);
	size = val & MVPP2_TXP_TOKEN_SIZE_MAX;
	if (size < mtu) {
		size = mtu;
		val &= ~MVPP2_TXP_TOKEN_SIZE_MAX;
		val |= size;
		mvpp2_write(hw, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, val);
	}

	for (txq = 0; txq < port->num_tx_queues; txq++) {
		val = mvpp2_read(hw,
				 MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq));
		size = val & MVPP2_TXQ_TOKEN_SIZE_MAX;

		if (size < mtu) {
			size = mtu;
			val &= ~MVPP2_TXQ_TOKEN_SIZE_MAX;
			val |= size;
			mvpp2_write(hw,
				    MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq),
				    val);
		}
	}
}


/* Set Tx descriptors fields relevant for CSUM calculation */
u32 mvpp2_txq_desc_csum(int l3_offs, int l3_proto,
			       int ip_hdr_len, int l4_proto)
{
	u32 command;

	/* fields: L3_offset, IP_hdrlen, L3_type, G_IPv4_chk,
	 * G_L4_chk, L4_type required only for checksum calculation
	 */
	command = (l3_offs << MVPP2_TXD_L3_OFF_SHIFT);
	command |= (ip_hdr_len << MVPP2_TXD_IP_HLEN_SHIFT);
	command |= MVPP2_TXD_IP_CSUM_DISABLE;

	if (l3_proto == swab16(ETH_P_IP)) {
		command &= ~MVPP2_TXD_IP_CSUM_DISABLE;	/* enable IPv4 csum */
		command &= ~MVPP2_TXD_L3_IP6;		/* enable IPv4 */
	} else {
		command |= MVPP2_TXD_L3_IP6;		/* enable IPv6 */
	}

	if (l4_proto == IPPROTO_TCP) {
		command &= ~MVPP2_TXD_L4_UDP;		/* enable TCP */
		command &= ~MVPP2_TXD_L4_CSUM_FRAG;	/* generate L4 csum */
	} else if (l4_proto == IPPROTO_UDP) {
		command |= MVPP2_TXD_L4_UDP;		/* enable UDP */
		command &= ~MVPP2_TXD_L4_CSUM_FRAG;	/* generate L4 csum */
	} else {
		command |= MVPP2_TXD_L4_CSUM_NOT;
	}

	return command;
}

/* Get number of sent descriptors and decrement counter.
 * The number of sent descriptors is returned.
 * Per-CPU access
 */






/* Tx descriptors helper methods */

/* Get number of Tx descriptors waiting to be transmitted by HW */
int mvpp2_txq_pend_desc_num_get(struct mvpp2_port *port,
				       struct mvpp2_tx_queue *txq)
{
	u32 val;
	struct mvpp2_hw *hw = &(port->priv->hw);

	mvpp2_write(hw, MVPP2_TXQ_NUM_REG, txq->id);
	val = mvpp2_read(hw, MVPP2_TXQ_PENDING_REG);

	return val & MVPP2_TXQ_PENDING_MASK;
}

/* Get pointer to next Tx descriptor to be processed (send) by HW */
struct mvpp2_tx_desc * mvpp2_txq_next_desc_get(struct mvpp2_aggr_tx_queue *aggr_txq)
{
	int tx_desc = aggr_txq->next_desc_to_proc;

	aggr_txq->next_desc_to_proc = MVPP2_QUEUE_NEXT_DESC(aggr_txq, tx_desc);
	return aggr_txq->descs + tx_desc;
}

/* Update HW with number of aggregated Tx descriptors to be sent */
void mvpp2_aggr_txq_pend_desc_add(struct mvpp2_port *port, int pending)
{
	/* aggregated access - relevant TXQ number is written in TX desc */
	mvpp2_write(&(port->priv->hw), MVPP2_AGGR_TXQ_UPDATE_REG, pending);
}


/* Check if there are enough free descriptors in aggregated txq.
 * If not, update the number of occupied descriptors and repeat the check.
 */
int mvpp2_aggr_desc_num_check(struct mvpp2 *priv,
				     struct mvpp2_aggr_tx_queue *aggr_txq, int num)
{
	if ((aggr_txq->count + num) > aggr_txq->size) {
		/* Update number of occupied aggregated Tx descriptors */
		int cpu = smp_processor_id();
		u32 val = mvpp2_read(&priv->hw, MVPP2_AGGR_TXQ_STATUS_REG(cpu));

		aggr_txq->count = val & MVPP2_AGGR_TXQ_PENDING_MASK;
	}

	if ((aggr_txq->count + num) > aggr_txq->size)
		return -ENOMEM;

	return 0;
}

/* Reserved Tx descriptors allocation request */
int mvpp2_txq_alloc_reserved_desc(struct mvpp2 *priv,
					 struct mvpp2_tx_queue *txq, int num)
{
	u32 val;

	val = (txq->id << MVPP2_TXQ_RSVD_REQ_Q_OFFSET) | num;
	mvpp2_write(&priv->hw, MVPP2_TXQ_RSVD_REQ_REG, val);

	val = mvpp2_read(&priv->hw, MVPP2_TXQ_RSVD_RSLT_REG);

	return val & MVPP2_TXQ_RSVD_RSLT_MASK;
}



/* Set rx queue offset */
void mvpp2_rxq_offset_set(struct mvpp2_port *port,
				 int prxq, int offset)
{
	u32 val;
	struct mvpp2_hw *hw = &(port->priv->hw);

	/* Convert offset from bytes to units of 32 bytes */
	offset = offset >> 5;

	val = mvpp2_read(hw, MVPP2_RXQ_CONFIG_REG(prxq));
	val &= ~MVPP2_RXQ_PACKET_OFFSET_MASK;

	/* Offset is in */
	val |= ((offset << MVPP2_RXQ_PACKET_OFFSET_OFFS) &
		MVPP2_RXQ_PACKET_OFFSET_MASK);

	mvpp2_write(hw, MVPP2_RXQ_CONFIG_REG(prxq), val);
}




/* Port configuration routines */

void mvpp2_port_mii_set(struct mvpp2_port *port)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_CTRL_2_REG);

	switch (port->phy_interface) {
	case PHY_INTERFACE_MODE_SGMII:
		val |= MVPP2_GMAC_INBAND_AN_MASK;
		break;
	case PHY_INTERFACE_MODE_RGMII:
		val |= MVPP2_GMAC_PORT_RGMII_MASK;
	default:
		val &= ~MVPP2_GMAC_PCS_ENABLE_MASK;
	}

	writel(val, port->base + MVPP2_GMAC_CTRL_2_REG);
}

void mvpp2_port_fc_adv_enable(struct mvpp2_port *port)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_AUTONEG_CONFIG);
	val |= MVPP2_GMAC_FC_ADV_EN;
	writel(val, port->base + MVPP2_GMAC_AUTONEG_CONFIG);
}

void mvpp2_port_enable(struct mvpp2_port *port)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_CTRL_0_REG);
	val |= MVPP2_GMAC_PORT_EN_MASK;
	val |= MVPP2_GMAC_MIB_CNTR_EN_MASK;
	writel(val, port->base + MVPP2_GMAC_CTRL_0_REG);
}

void mvpp2_port_disable(struct mvpp2_port *port)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_CTRL_0_REG);
	val &= ~(MVPP2_GMAC_PORT_EN_MASK);
	writel(val, port->base + MVPP2_GMAC_CTRL_0_REG);
}

/* Set IEEE 802.3x Flow Control Xon Packet Transmission Mode */
void mvpp2_port_periodic_xon_disable(struct mvpp2_port *port)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_CTRL_1_REG) &
		    ~MVPP2_GMAC_PERIODIC_XON_EN_MASK;
	writel(val, port->base + MVPP2_GMAC_CTRL_1_REG);
}

/* Configure loopback port */
void mvpp2_port_loopback_set(struct mvpp2_port *port)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_CTRL_1_REG);

	if (port->speed == 1000)
		val |= MVPP2_GMAC_GMII_LB_EN_MASK;
	else
		val &= ~MVPP2_GMAC_GMII_LB_EN_MASK;

	if (port->phy_interface == PHY_INTERFACE_MODE_SGMII)
		val |= MVPP2_GMAC_PCS_LB_EN_MASK;
	else
		val &= ~MVPP2_GMAC_PCS_LB_EN_MASK;

	writel(val, port->base + MVPP2_GMAC_CTRL_1_REG);
}

void mvpp2_port_reset(struct mvpp2_port *port)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_CTRL_2_REG) &
		    ~MVPP2_GMAC_PORT_RESET_MASK;
	writel(val, port->base + MVPP2_GMAC_CTRL_2_REG);

	while (readl(port->base + MVPP2_GMAC_CTRL_2_REG) &
	       MVPP2_GMAC_PORT_RESET_MASK)
		continue;
}


/* Refill BM pool */
void mvpp2_pool_refill(struct mvpp2 *priv, u32 pool,
			      dma_addr_t phys_addr, struct sk_buff *cookie)
{

	mvpp2_bm_pool_put(&priv->hw, pool, phys_addr, cookie);
}

/* Set pool buffer size */
void mvpp2_bm_pool_bufsize_set(struct mvpp2_hw *hw,
				      struct mvpp2_bm_pool *bm_pool, int buf_size)
{
	u32 val;

	bm_pool->buf_size = buf_size;

	val = ALIGN(buf_size, 1 << MVPP2_POOL_BUF_SIZE_OFFSET);
	mvpp2_write(hw, MVPP2_POOL_BUF_SIZE_REG(bm_pool->id), val);
}



/* Attach long pool to rxq */
void mvpp21_rxq_long_pool_set(struct mvpp2_hw *hw,
				     int prxq, int long_pool)
{
	u32 val;

	val = mvpp2_read(hw, MVPP2_RXQ_CONFIG_REG(prxq));
	val &= ~MVPP21_RXQ_POOL_LONG_MASK;
	val |= ((long_pool << MVPP21_RXQ_POOL_LONG_OFFS) &
		    MVPP21_RXQ_POOL_LONG_MASK);

	mvpp2_write(hw, MVPP2_RXQ_CONFIG_REG(prxq), val);
}

/* Attach short pool to rxq */
void mvpp21_rxq_short_pool_set(struct mvpp2_hw *hw,
				     int prxq, int short_pool)
{
	u32 val;

	val = mvpp2_read(hw, MVPP2_RXQ_CONFIG_REG(prxq));
	val &= ~MVPP21_RXQ_POOL_SHORT_MASK;
	val |= ((short_pool << MVPP21_RXQ_POOL_SHORT_OFFS) &
		    MVPP21_RXQ_POOL_SHORT_MASK);

	mvpp2_write(hw, MVPP2_RXQ_CONFIG_REG(prxq), val);
}


/* Attach long pool to rxq */
void mvpp22_rxq_long_pool_set(struct mvpp2_hw *hw,
				     int prxq, int long_pool)
{
	u32 val;

	val = mvpp2_read(hw, MVPP2_RXQ_CONFIG_REG(prxq));
	val &= ~MVPP22_RXQ_POOL_LONG_MASK;
	val |= ((long_pool << MVPP22_RXQ_POOL_LONG_OFFS) &
		    MVPP22_RXQ_POOL_LONG_MASK);

	mvpp2_write(hw, MVPP2_RXQ_CONFIG_REG(prxq), val);
}

/* Attach short pool to rxq */
void mvpp22_rxq_short_pool_set(struct mvpp2_hw *hw,
				     int prxq, int short_pool)
{
	u32 val;

	val = mvpp2_read(hw, MVPP2_RXQ_CONFIG_REG(prxq));
	val &= ~MVPP22_RXQ_POOL_SHORT_MASK;
	val |= ((short_pool << MVPP22_RXQ_POOL_SHORT_OFFS) &
		    MVPP22_RXQ_POOL_SHORT_MASK);

	mvpp2_write(hw, MVPP2_RXQ_CONFIG_REG(prxq), val);
}



/* Enable/disable receiving packets */
void mvpp2_ingress_enable(struct mvpp2_port *port)
{
	u32 val;
	int lrxq, queue;
	struct mvpp2_hw *hw = &(port->priv->hw);

	for (lrxq = 0; lrxq < port->num_rx_queues; lrxq++) {
		queue = port->rxqs[lrxq]->id;
		val = mvpp2_read(hw, MVPP2_RXQ_CONFIG_REG(queue));
		val &= ~MVPP2_RXQ_DISABLE_MASK;
		mvpp2_write(hw, MVPP2_RXQ_CONFIG_REG(queue), val);
	}
}

void mvpp2_ingress_disable(struct mvpp2_port *port)
{
	u32 val;
	int lrxq, queue;
	struct mvpp2_hw *hw = &(port->priv->hw);

	for (lrxq = 0; lrxq < port->num_rx_queues; lrxq++) {
		queue = port->rxqs[lrxq]->id;
		val = mvpp2_read(hw, MVPP2_RXQ_CONFIG_REG(queue));
		val |= MVPP2_RXQ_DISABLE_MASK;
		mvpp2_write(hw, MVPP2_RXQ_CONFIG_REG(queue), val);
	}
}

void mvpp2_egress_enable(struct mvpp2_port *port)
{
	u32 qmap;
	int queue;
	int tx_port_num = mvpp2_egress_port(port);
	struct mvpp2_hw *hw = &(port->priv->hw);

	/* Enable all initialized TXs. */
	qmap = 0;
	for (queue = 0; queue < port->num_tx_queues; queue++) {
		struct mvpp2_tx_queue *txq = port->txqs[queue];

		if (txq->descs != NULL)
			qmap |= (1 << queue);
	}

	mvpp2_write(hw, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
	mvpp2_write(hw, MVPP2_TXP_SCHED_Q_CMD_REG, qmap);
}

/* Disable transmit via physical egress queue
 * - HW doesn't take descriptors from DRAM
 */
void mvpp2_egress_disable(struct mvpp2_port *port)
{
	u32 reg_data;
	int delay;
	int tx_port_num = mvpp2_egress_port(port);
	struct mvpp2_hw *hw = &(port->priv->hw);

	/* Issue stop command for active channels only */
	mvpp2_write(hw, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
	reg_data = (mvpp2_read(hw, MVPP2_TXP_SCHED_Q_CMD_REG)) &
		    MVPP2_TXP_SCHED_ENQ_MASK;
	if (reg_data != 0)
		mvpp2_write(hw, MVPP2_TXP_SCHED_Q_CMD_REG,
			    (reg_data << MVPP2_TXP_SCHED_DISQ_OFFSET));

	/* Wait for all Tx activity to terminate. */
	delay = 0;
	do {
		if (delay >= MVPP2_TX_DISABLE_TIMEOUT_MSEC) {
			netdev_warn(port->dev,
				    "Tx stop timed out, status=0x%08x\n",
				    reg_data);
			break;
		}
		mdelay(1);
		delay++;

		/* Check port TX Command register that all
		 * Tx queues are stopped
		 */
		reg_data = mvpp2_read(hw, MVPP2_TXP_SCHED_Q_CMD_REG);
	} while (reg_data & MVPP2_TXP_SCHED_ENQ_MASK);
}



/* Parser default initialization */
int mvpp2_prs_default_init(struct platform_device *pdev,
				  struct mvpp2_hw *hw)
{
	int err, index, i;

	/* Enable tcam table */
	mvpp2_write(hw, MVPP2_PRS_TCAM_CTRL_REG, MVPP2_PRS_TCAM_EN_MASK);

	/* Clear all tcam and sram entries */
	for (index = 0; index < MVPP2_PRS_TCAM_SRAM_SIZE; index++) {
		mvpp2_write(hw, MVPP2_PRS_TCAM_IDX_REG, index);
		for (i = 0; i < MVPP2_PRS_TCAM_WORDS; i++)
			mvpp2_write(hw, MVPP2_PRS_TCAM_DATA_REG(i), 0);

		mvpp2_write(hw, MVPP2_PRS_SRAM_IDX_REG, index);
		for (i = 0; i < MVPP2_PRS_SRAM_WORDS; i++)
			mvpp2_write(hw, MVPP2_PRS_SRAM_DATA_REG(i), 0);
	}

	/* Invalidate all tcam entries */
	for (index = 0; index < MVPP2_PRS_TCAM_SRAM_SIZE; index++)
		mvpp2_prs_hw_inv(hw, index);

	hw->prs_shadow = devm_kcalloc(&pdev->dev, MVPP2_PRS_TCAM_SRAM_SIZE,
					sizeof(struct mvpp2_prs_shadow),
					GFP_KERNEL);
	if (!hw->prs_shadow)
		return -ENOMEM;

	/* Always start from lookup = 0 */
	for (index = 0; index < MVPP2_MAX_PORTS; index++)
		mvpp2_prs_hw_port_init(hw, index, MVPP2_PRS_LU_MH,
				       MVPP2_PRS_PORT_LU_MAX, 0);

	mvpp2_prs_def_flow_init(hw);

	mvpp2_prs_mh_init(hw);

	mvpp2_prs_mac_init(hw);

	mvpp2_prs_dsa_init(hw);

	err = mvpp2_prs_etype_init(hw);
	if (err)
		return err;

	err = mvpp2_prs_vlan_init(pdev, hw);
	if (err)
		return err;

	err = mvpp2_prs_pppoe_init(hw);
	if (err)
		return err;

	err = mvpp2_prs_ip6_init(hw);
	if (err)
		return err;

	err = mvpp2_prs_ip4_init(hw);
	if (err)
		return err;

	return 0;
}





