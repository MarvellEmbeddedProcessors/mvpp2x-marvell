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
#include "mv_pp2x.h"
#include "mv_pp2x_hw.h"
#include "mv_pp2x_debug.h"
#include "mv_pp2x_soc_test.h"

struct mv_pp2x_cls_c3_shadow_hash_entry mvCls3ShadowTbl[MVPP2_CLS_C3_HASH_TBL_SIZE];
int mvCls3ShadowExtTbl[MVPP2_CLS_C3_EXT_TBL_SIZE];
static int SwInitCntSet;

/* Common utilities */
static void mvPp2ClsC3ShadowSet(int hekSize, int index, int ext_index)
{
	mvCls3ShadowTbl[index].size = hekSize;

	if (hekSize > MVPP2_CLS_C3_HEK_BYTES) {
		mvCls3ShadowTbl[index].ext_ptr = ext_index;
		mvCls3ShadowExtTbl[ext_index] = IN_USE;
	} else
		mvCls3ShadowTbl[index].ext_ptr = NOT_IN_USE;
}

/*-----------------------------------------------------------------------------*/
void mvPp2ClsC3ShadowInit(void)
{
	/* clear hash shadow and extension shadow */
	int index;

	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		mvCls3ShadowTbl[index].size = 0;
		mvCls3ShadowTbl[index].ext_ptr = NOT_IN_USE;
	}

	for (index = 0; index < MVPP2_CLS_C3_EXT_TBL_SIZE; index++)
		mvCls3ShadowExtTbl[index] = NOT_IN_USE;
}

/*-----------------------------------------------------------------------------*/
int mvPp2ClsC3ShadowFreeGet(void)
{
	int index;

	/* Go through the all entires from first to last */
	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		if (!mvCls3ShadowTbl[index].size)
			break;
	}
	return index;
}
/*-----------------------------------------------------------------------------*/
int mvPp2ClsC3ShadowExtFreeGet(void)
{
	int index;

	/* Go through the all entires from first to last */
	for (index = 0; index < MVPP2_CLS_C3_EXT_TBL_SIZE; index++) {
		if (mvCls3ShadowExtTbl[index] == NOT_IN_USE)
			break;
	}
	return index;
}

/*-----------------------------------------------------------------------------*/
void mvPp2C3ShadowClear(int index)
{
	int ext_ptr;

	mvCls3ShadowTbl[index].size = 0;
	ext_ptr = mvCls3ShadowTbl[index].ext_ptr;

	if (ext_ptr != NOT_IN_USE)
		mvCls3ShadowExtTbl[ext_ptr] = NOT_IN_USE;

	mvCls3ShadowTbl[index].ext_ptr = NOT_IN_USE;
}

/* return 1 if counters clearing is completed */
static int mvPp2ClsC3HitCntrClearDone(struct mv_pp2x_hw *hw)
{
	unsigned int regVal;

	regVal = mv_pp2x_read(hw, MVPP2_CLS3_STATE_REG);
	regVal &= MVPP2_CLS3_STATE_CLEAR_CTR_DONE_MASK;
	regVal >>= MVPP2_CLS3_STATE_CLEAR_CTR_DONE;
	return regVal;
}

/*-----------------------------------------------------------------------------*/
void mvPp2ClsC3SwClear(struct mv_pp2x_cls_c3_entry *c3)
{
	memset(c3, 0, sizeof(struct mv_pp2x_cls_c3_entry));
}
EXPORT_SYMBOL(mvPp2ClsC3SwClear);

/* APIs for Classification C3 key fields */

int mvPp2ClsC3SwL4infoSet(struct mv_pp2x_cls_c3_entry *c3, int l4info)
{
	c3->key.key_ctrl &= ~KEY_CTRL_L4_MASK;
	c3->key.key_ctrl |= (l4info << KEY_CTRL_L4);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3SwL4infoSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3SwLkpTypeSet(struct mv_pp2x_cls_c3_entry *c3, int lkp_type)
{
	c3->key.key_ctrl &= ~KEY_CTRL_LKP_TYPE_MASK;
	c3->key.key_ctrl |= (lkp_type << KEY_CTRL_LKP_TYPE);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3SwLkpTypeSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3SwPortIDSet(struct mv_pp2x_cls_c3_entry *c3, int type, int portid)
{
	c3->key.key_ctrl &= ~(KEY_CTRL_PRT_ID_MASK | KEY_CTRL_PRT_ID_TYPE_MASK);
	c3->key.key_ctrl |= ((portid << KEY_CTRL_PRT_ID) | (type << KEY_CTRL_PRT_ID_TYPE));

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3SwPortIDSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3SwHekSizeSet(struct mv_pp2x_cls_c3_entry *c3, int hekSize)
{
	c3->key.key_ctrl &= ~KEY_CTRL_HEK_SIZE_MASK;
	c3->key.key_ctrl |= (hekSize << KEY_CTRL_HEK_SIZE);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3SwHekSizeSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3SwHekByteSet(struct mv_pp2x_cls_c3_entry *c3, unsigned int offs, unsigned char byte)
{
	c3->key.hek.bytes[HW_BYTE_OFFS(offs)] = byte;
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3SwHekByteSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3SwHekWordSet(struct mv_pp2x_cls_c3_entry *c3, unsigned int offs, unsigned int word)
{
	c3->key.hek.words[offs] = word;
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3SwHekWordSet);

/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3ColorSet(struct mv_pp2x_cls_c3_entry *c3, int cmd)
{
	c3->sram.regs.actions &= ~MVPP2_CLS2_ACT_COLOR_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_COLOR_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3ColorSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3QueueHighSet(struct mv_pp2x_cls_c3_entry *c3, int cmd, int queue)
{
	/*set command*/
	c3->sram.regs.actions &= ~MVPP2_CLS2_ACT_QH_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_QH_OFF);

	/*set modify High queue value*/
	c3->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK;
	c3->sram.regs.qos_attr |= (queue << MVPP2_CLS2_ACT_QOS_ATTR_QH_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3QueueHighSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3QueueLowSet(struct mv_pp2x_cls_c3_entry *c3, int cmd, int queue)
{
	/*set command*/
	c3->sram.regs.actions &= ~MVPP2_CLS2_ACT_QL_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_QL_OFF);

	/*set modify High queue value*/
	c3->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK;
	c3->sram.regs.qos_attr |= (queue << MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3QueueLowSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3QueueSet(struct mv_pp2x_cls_c3_entry *c3, int cmd, int queue)
{
	int status = MV_OK;
	int qHigh, qLow;

	/* cmd validation in set functions */

	qHigh = (queue & MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK) >> MVPP2_CLS2_ACT_QOS_ATTR_QH_OFF;
	qLow = (queue & MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK) >> MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF;

	status |= mvPp2ClsC3QueueLowSet(c3, cmd, qLow);
	status |= mvPp2ClsC3QueueHighSet(c3, cmd, qHigh);

	return status;
}
EXPORT_SYMBOL(mvPp2ClsC3QueueSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3ForwardSet(struct mv_pp2x_cls_c3_entry *c3, int cmd)
{
	c3->sram.regs.actions &= ~MVPP2_CLS2_ACT_FRWD_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_FRWD_OFF);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3ForwardSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3PolicerSet(struct mv_pp2x_cls_c3_entry *c3, int cmd, int policerId, int bank)
{
	c3->sram.regs.actions &= ~MVPP2_CLS2_ACT_PLCR_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_PLCR_OFF);

	c3->sram.regs.dup_attr &= ~MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_MASK;
	c3->sram.regs.dup_attr |= (policerId << MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_OFF);

	if (bank)
		c3->sram.regs.dup_attr |= MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_MASK;
	else
		c3->sram.regs.dup_attr &= ~MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_MASK;

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3PolicerSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3FlowIdEn(struct mv_pp2x_cls_c3_entry *c3, int flowid_en)
{
	/*set Flow ID enable or disable*/
	if (flowid_en)
		c3->sram.regs.actions |= (1 << MVPP2_CLS2_ACT_FLD_EN_OFF);
	else
		c3->sram.regs.actions &= ~(1 << MVPP2_CLS2_ACT_FLD_EN_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3FlowIdEn);
/*-------------------------------------------------------------------------------*/
int mv_pp2x_cls_c3_rss_set(struct mv_pp2x_cls_c3_entry *c3, int cmd, int rss_en)
{
	if (!c3 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK || rss_en >=
			(1 << MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_BITS))
		return -EINVAL;

	c3->sram.regs.actions &= ~MVPP2_CLS2_ACT_RSS_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_RSS_OFF);

	c3->sram.regs.dup_attr &= ~MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_MASK;
	c3->sram.regs.dup_attr |= (rss_en <<
			MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_OFF);

	return 0;
}
EXPORT_SYMBOL(mv_pp2x_cls_c3_rss_set);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3ModSet(struct mv_pp2x_cls_c3_entry *c3, int data_ptr, int instr_offs, int l4_csum)
{
	c3->sram.regs.hwf_attr &= ~MVPP2_CLS2_ACT_HWF_ATTR_DPTR_MASK;
	c3->sram.regs.hwf_attr &= ~MVPP2_CLS2_ACT_HWF_ATTR_IPTR_MASK;
	c3->sram.regs.hwf_attr &= ~MVPP2_CLS2_ACT_HWF_ATTR_L4CHK_MASK;

	c3->sram.regs.hwf_attr |= (data_ptr << MVPP2_CLS2_ACT_HWF_ATTR_DPTR_OFF);
	c3->sram.regs.hwf_attr |= (instr_offs << MVPP2_CLS2_ACT_HWF_ATTR_IPTR_OFF);
	c3->sram.regs.hwf_attr |= (l4_csum << MVPP2_CLS2_ACT_HWF_ATTR_L4CHK_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3ModSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3MtuSet(struct mv_pp2x_cls_c3_entry *c3, int mtu_inx)
{
	c3->sram.regs.hwf_attr &= ~MVPP2_CLS2_ACT_HWF_ATTR_MTUIDX_MASK;
	c3->sram.regs.hwf_attr |= (mtu_inx << MVPP2_CLS2_ACT_HWF_ATTR_MTUIDX_OFF);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3MtuSet);
/*-------------------------------------------------------------------------------*/

int mvPp2ClsC3DupSet(struct mv_pp2x_cls_c3_entry *c3, int dupid, int count)
{
	/*set flowid and count*/
	c3->sram.regs.dup_attr &= ~(MVPP2_CLS2_ACT_DUP_ATTR_DUPID_MASK | MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_MASK);
	c3->sram.regs.dup_attr |= (dupid << MVPP2_CLS2_ACT_DUP_ATTR_DUPID_OFF);
	c3->sram.regs.dup_attr |= (count << MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3DupSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3SeqSet(struct mv_pp2x_cls_c3_entry *c3, int id, int bits_offs, int bits)
{
	unsigned int low_bits, high_bits = 0;

	if (bits_offs >= DWORD_BITS_LEN)
		high_bits = bits;

	else if (bits_offs + bits > DWORD_BITS_LEN)
		high_bits = (bits_offs + bits) % DWORD_BITS_LEN;

	low_bits = bits - high_bits;

	/*
	*high_bits hold the num of bits that we need to write in seq_h_attr
	*low_bits hold the num of bits that we need to write in seq_l_attr
	*/

	if (low_bits) {
		/* mask and set new value in seq_l_attr*/
		c3->sram.regs.seq_l_attr &= ~(((1 << low_bits) - 1)  << bits_offs);
		c3->sram.regs.seq_l_attr |= (id  << bits_offs);
	}

	if (high_bits) {
		int high_id = id >> low_bits;
		int high_offs = (low_bits == 0) ? (bits_offs % DWORD_BITS_LEN) : 0;

		/* mask and set new value in seq_h_attr*/
		c3->sram.regs.seq_h_attr &= ~(((1 << high_bits) - 1)  << high_offs);
		c3->sram.regs.seq_h_attr |= (high_id << high_offs);
	}

	return MV_OK;

}
EXPORT_SYMBOL(mvPp2ClsC3SeqSet);
/*-------------------------------------------------------------------------------*/
int mv_pp2x_cls_c3_hw_read(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c3_entry *c3, int index)
{
	int i, isExt;
	u32 regVal = 0;
	u32 hashData[MVPP2_CLS3_HASH_DATA_REG_NUM];
	u32 hashExtData[MVPP2_CLS3_HASH_EXT_DATA_REG_NUM];

	mvPp2ClsC3SwClear(c3);

	c3->index = index;
	c3->ext_index = NOT_IN_USE;

	/* write index */
	mv_pp2x_write(hw, MVPP2_CLS3_DB_INDEX_REG, index);

	regVal |= (index << MVPP2_CLS3_HASH_OP_TBL_ADDR);
	mv_pp2x_write(hw, MVPP2_CLS3_HASH_OP_REG, regVal);

	/* read action table */
	c3->sram.regs.actions = mv_pp2x_read(hw, MVPP2_CLS3_ACT_REG);
	c3->sram.regs.qos_attr = mv_pp2x_read(hw, MVPP2_CLS3_ACT_QOS_ATTR_REG);
	c3->sram.regs.hwf_attr = mv_pp2x_read(hw, MVPP2_CLS3_ACT_HWF_ATTR_REG);
	c3->sram.regs.dup_attr = mv_pp2x_read(hw, MVPP2_CLS3_ACT_DUP_ATTR_REG);
	c3->sram.regs.seq_l_attr = mv_pp2x_read(hw, MVPP2_CLS3_ACT_SEQ_L_ATTR_REG);
	c3->sram.regs.seq_h_attr = mv_pp2x_read(hw, MVPP2_CLS3_ACT_SEQ_H_ATTR_REG);

	/* read hash data*/
	for (i = 0; i < MVPP2_CLS3_HASH_DATA_REG_NUM; i++)
		hashData[i] = mv_pp2x_read(hw, MVPP2_CLS3_HASH_DATA_REG(i));

	if (mvCls3ShadowTbl[index].size == 0)
		/* entry not in use */
		return MV_OK;

	c3->key.key_ctrl = 0;

	if (mvCls3ShadowTbl[index].ext_ptr == NOT_IN_USE) {
		isExt = 0;
		/* TODO REMOVE NEXT LINES- ONLY FOR INTERNAL VALIDATION */
		if ((mvCls3ShadowTbl[index].size == 0) ||
		    (mvCls3ShadowTbl[index].ext_ptr != NOT_IN_USE)) {
			pr_err("%s: SW internal error.\n", __func__);
			return MV_ERROR;
		}

		/*read Multihash entry data*/
		c3->key.hek.words[6] = hashData[0]; /* hek 0*/
		c3->key.hek.words[7] = hashData[1]; /* hek 1*/
		c3->key.hek.words[8] = hashData[2]; /* hek 2*/

		/* write key control data to SW */
		c3->key.key_ctrl |= (((hashData[3] & KEY_PRT_ID_MASK(isExt)) >>
					(KEY_PRT_ID(isExt) % DWORD_BITS_LEN)) << KEY_CTRL_PRT_ID);

		c3->key.key_ctrl |= (((hashData[3] & KEY_PRT_ID_TYPE_MASK(isExt)) >>
					(KEY_PRT_ID_TYPE(isExt) % DWORD_BITS_LEN)) << KEY_CTRL_PRT_ID_TYPE);

		c3->key.key_ctrl |= (((hashData[3] & KEY_LKP_TYPE_MASK(isExt)) >>
					(KEY_LKP_TYPE(isExt) % DWORD_BITS_LEN)) << KEY_CTRL_LKP_TYPE);

		c3->key.key_ctrl |= (((hashData[3] & KEY_L4_INFO_MASK(isExt)) >>
					(KEY_L4_INFO(isExt) % DWORD_BITS_LEN)) << KEY_CTRL_L4);

	} else {
		isExt = 1;
		/* TODO REMOVE NEXT LINES- ONLY FOR INTERNAL VALIDATION */
		if ((mvCls3ShadowTbl[index].size == 0) ||
		    (mvCls3ShadowTbl[index].ext_ptr == NOT_IN_USE)) {
			pr_err("%s: SW internal error.\n", __func__);
			return MV_ERROR;
		}
		c3->ext_index = mvCls3ShadowTbl[index].ext_ptr;

		/* write extension index */
		mv_pp2x_write(hw, MVPP2_CLS3_DB_INDEX_REG, mvCls3ShadowTbl[index].ext_ptr);

		/* read hash extesion data*/
		for (i = 0; i < MVPP2_CLS3_HASH_EXT_DATA_REG_NUM; i++)
			hashExtData[i] = mv_pp2x_read(hw, MVPP2_CLS3_HASH_EXT_DATA_REG(i));


		/* heks bytes 35 - 32 */
		c3->key.hek.words[8] = ((hashData[2] & 0x00FFFFFF) << 8) | ((hashData[1] & 0xFF000000) >> 24);

		/* heks bytes 31 - 28 */
		c3->key.hek.words[7] = ((hashData[1] & 0x00FFFFFF) << 8) | ((hashData[0] & 0xFF000000) >> 24);

		/* heks bytes 27 - 24 */
		c3->key.hek.words[6] = ((hashData[0] & 0x00FFFFFF) << 8) | (hashExtData[6] & 0x000000FF);

		c3->key.hek.words[5] = hashExtData[5]; /* heks bytes 23 - 20 */
		c3->key.hek.words[4] = hashExtData[4]; /* heks bytes 19 - 16 */
		c3->key.hek.words[3] = hashExtData[3]; /* heks bytes 15 - 12 */
		c3->key.hek.words[2] = hashExtData[2]; /* heks bytes 11 - 8  */
		c3->key.hek.words[1] = hashExtData[1]; /* heks bytes 7 - 4   */
		c3->key.hek.words[0] = hashExtData[0]; /* heks bytes 3 - 0   */

		/* write key control data to SW*/

		c3->key.key_ctrl |= (((hashData[3] & KEY_PRT_ID_MASK(isExt)) >>
					(KEY_PRT_ID(isExt) % DWORD_BITS_LEN)) << KEY_CTRL_PRT_ID);

		/* PPv2.1 (feature MAS 3.16) LKP_TYPE size and offset changed */

		c3->key.key_ctrl |= (((hashData[3] & KEY_PRT_ID_TYPE_MASK(isExt)) >>
					(KEY_PRT_ID_TYPE(isExt) % DWORD_BITS_LEN)) << KEY_CTRL_PRT_ID_TYPE);

		c3->key.key_ctrl |= ((((hashData[2] & 0xf8000000) >> 27) |
					((hashData[3] & 0x1) << 5)) << KEY_CTRL_LKP_TYPE);


		c3->key.key_ctrl |= (((hashData[2] & KEY_L4_INFO_MASK(isExt)) >>
					(KEY_L4_INFO(isExt) % DWORD_BITS_LEN)) << KEY_CTRL_L4);
	}

	/* update hek size */
	c3->key.key_ctrl |= ((mvCls3ShadowTbl[index].size << KEY_CTRL_HEK_SIZE) & KEY_CTRL_HEK_SIZE_MASK);

	return MV_OK;
}
EXPORT_SYMBOL(mv_pp2x_cls_c3_hw_read);
/*-------------------------------------------------------------------------------*/
static int mvPp2ClsC3SwActDump(struct mv_pp2x_cls_c3_entry *c3)
{
	pr_info("\n");

	/*------------------------------*/
	/*	actions 0x1D40		*/
	/*------------------------------*/

	pr_info("ACT_TBL: COLOR   LOW_Q   HIGH_Q     FWD   POLICER  FID  RSS\n");
	pr_info("CMD:     [%1d]      [%1d]    [%1d]        [%1d]   [%1d]      [%1d]      [%1d]\n",
			((c3->sram.regs.actions & (MVPP2_CLS2_ACT_COLOR_MASK)) >> MVPP2_CLS2_ACT_COLOR_OFF),
			((c3->sram.regs.actions & (MVPP2_CLS2_ACT_QL_MASK)) >> MVPP2_CLS2_ACT_QL_OFF),
			((c3->sram.regs.actions & (MVPP2_CLS2_ACT_QH_MASK)) >> MVPP2_CLS2_ACT_QH_OFF),
			((c3->sram.regs.actions & MVPP2_CLS2_ACT_FRWD_MASK) >> MVPP2_CLS2_ACT_FRWD_OFF),
			((c3->sram.regs.actions & (MVPP2_CLS2_ACT_PLCR_MASK)) >> MVPP2_CLS2_ACT_PLCR_OFF),
			((c3->sram.regs.actions & MVPP2_CLS2_ACT_FLD_EN_MASK) >> MVPP2_CLS2_ACT_FLD_EN_OFF),
			((c3->sram.regs.actions & MVPP2_CLS2_ACT_RSS_MASK) >> MVPP2_CLS2_ACT_RSS_OFF));

	pr_info("VAL:              [%1d]    [0x%x]\n",
		((c3->sram.regs.qos_attr & (MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK)) >> MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF),
		((c3->sram.regs.qos_attr & (MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK)) >> MVPP2_CLS2_ACT_QOS_ATTR_QH_OFF));

	pr_info("\n");
	/*------------------------------*/
	/*	hwf_attr 0x1D48		*/
	/*------------------------------*/

	pr_info("HWF_ATTR: IPTR	DPTR	 CHKSM     MTU_IDX\n");
	pr_info("          0x%1.1x   0x%4.4x   %s   0x%1.1x\n",

		((c3->sram.regs.hwf_attr & MVPP2_CLS2_ACT_HWF_ATTR_IPTR_MASK) >> MVPP2_CLS2_ACT_HWF_ATTR_IPTR_OFF),
		((c3->sram.regs.hwf_attr & MVPP2_CLS2_ACT_HWF_ATTR_DPTR_MASK) >> MVPP2_CLS2_ACT_HWF_ATTR_DPTR_OFF),
		(((c3->sram.regs.hwf_attr &
		MVPP2_CLS2_ACT_HWF_ATTR_L4CHK_MASK) >> MVPP2_CLS2_ACT_HWF_ATTR_L4CHK_OFF) ? "ENABLE" : "DISABLE"),
		((c3->sram.regs.hwf_attr & MVPP2_CLS2_ACT_HWF_ATTR_MTUIDX_MASK) >> MVPP2_CLS2_ACT_HWF_ATTR_MTUIDX_OFF));
	pr_info("\n");
	/*------------------------------*/
	/*	dup_attr 0x1D4C		*/
	/*------------------------------*/
	pr_info("DUP_ATTR:FID	COUNT	POLICER [id    bank]	RSS\n");
	pr_info("         0x%2.2x\t0x%1.1x\t\t[0x%2.2x   0x%1.1x]\t%d\n",
		((c3->sram.regs.dup_attr & MVPP2_CLS2_ACT_DUP_ATTR_DUPID_MASK) >> MVPP2_CLS2_ACT_DUP_ATTR_DUPID_OFF),
		((c3->sram.regs.dup_attr & MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_MASK) >> MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_OFF),
		((c3->sram.regs.dup_attr & MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_MASK) >> MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_OFF),
		((c3->sram.regs.dup_attr & MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_MASK) >> MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_OFF),
		((c3->sram.regs.dup_attr & MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_MASK) >> MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_OFF));
	pr_info("\n");
	pr_info("SEQ_ATTR: HIGH[32:37] LOW[0:31]\n");
	pr_info("          0x%2.2x        0x%8.8x", c3->sram.regs.seq_h_attr, c3->sram.regs.seq_l_attr);


	pr_info("\n\n");

	return MV_OK;
}

/* retun 1 scan procedure completed */
static int mvPp2ClsC3ScanIsComplete(struct mv_pp2x_hw *hw)
{
	unsigned int regVal;

	regVal = mv_pp2x_read(hw, MVPP2_CLS3_STATE_REG);
	regVal &= MVPP2_CLS3_STATE_SC_DONE_MASK;
	regVal >>= MVPP2_CLS3_STATE_SC_DONE;

	return regVal;
}

/* return 1 if that the last CPU access (Query,Add or Delete) was completed */
static int mvPp2ClsC3CpuIsDone(struct mv_pp2x_hw *hw)
{
	unsigned int regVal;

	regVal = mv_pp2x_read(hw, MVPP2_CLS3_STATE_REG);
	regVal &= MVPP2_CLS3_STATE_CPU_DONE_MASK;
	regVal >>= MVPP2_CLS3_STATE_CPU_DONE;
	return regVal;
}

/*-------------------------------------------------------------------------------*/
/*0x0  "ScanCompleted"  scan completed and the scan results are ready in hardware*/
/*0x1  "HitCountersClear"  The engine is clearing the Hit Counters		*/
/*0x2  "ScanWait"  The engine waits for the scan delay timer			*/
/*0x3  "ScanInProgress"  The scan process is in progress			*/
/*------------------------------------------------------------------------------*/
static int mvPp2ClsC3ScanStateGet(struct mv_pp2x_hw *hw, int *state)
{
	unsigned int regVal;

	regVal = mv_pp2x_read(hw, MVPP2_CLS3_STATE_REG);
	regVal &= MVPP2_CLS3_STATE_SC_STATE_MASK;
	regVal >>= MVPP2_CLS3_STATE_SC_STATE;
	*state = regVal;

	return MV_OK;
}

/*-------------------------------------------------------------------------------*/

static int mvPp2ClsC3HwQueryAddRelocate(struct mv_pp2x_hw *hw, int new_idx, int max_depth,
					int cur_depth, struct mv_pp2x_cls_c3_hash_pair *hash_pair_arr)
{
	int status, index_free, idx = 0;
	unsigned char occupied_bmp;
	struct mv_pp2x_cls_c3_entry local_c3;
	int usedIndex[MVPP2_CLS3_HASH_BANKS_NUM] = {0};

	if (cur_depth >= max_depth)
		return MV_ERROR;

	mvPp2ClsC3SwClear(&local_c3);

	if (mv_pp2x_cls_c3_hw_read(hw, &local_c3, new_idx)) {
		pr_err("%s could not get key for index [0x%x]\n", __func__, new_idx);
		return MV_ERROR;
	}

	if (mvPp2ClsC3HwQuery(hw, &local_c3, &occupied_bmp, usedIndex)) {
		pr_err("%s: mvPp2ClsC3HwQuery failed, depth = %d\n", __func__, cur_depth);
		return MV_ERROR;
	}

	/* fill in indices for this key */
	for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++) {
		/* if new index is in the bank index, skip it */
		if (new_idx == usedIndex[idx]) {
			usedIndex[idx] = 0;
			continue;
		}

		/* found a vacant index */
		if (!(occupied_bmp & (1 << idx))) {
			index_free = usedIndex[idx];
			break;
		}
	}

	/* no free index, recurse and relocate another key */
	if (idx == MVPP2_CLS3_HASH_BANKS_NUM) {
		/* recurse over all valid indices */
		for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++) {
			if (usedIndex[idx] == 0)
				continue;

			if (mvPp2ClsC3HwQueryAddRelocate(hw, usedIndex[idx], max_depth,
							cur_depth+1, hash_pair_arr) == MV_OK)
				break;
		}

		/* tried relocate, no valid entries found */
		if (idx == MVPP2_CLS3_HASH_BANKS_NUM)
			return MV_ERROR;

	}

	/* if we reached here, we found a valid free index */
	index_free = usedIndex[idx];

	/* new_idx del is not necessary */

	/*We do not chage extension tabe*/
	status = mvPp2ClsC3HwAdd(hw, &local_c3, index_free, local_c3.ext_index);

	/* update the hash pair */
	if (hash_pair_arr != NULL) {
		hash_pair_arr->old_idx[hash_pair_arr->pair_num] = new_idx;
		hash_pair_arr->new_idx[hash_pair_arr->pair_num] = index_free;
		hash_pair_arr->pair_num++;
	}

	if (status != MV_OK) {
		pr_err("%s:Error - mvPp2ClsC3HwAdd failed, depth = %d\\n", __func__, cur_depth);
		return status;
	}

	pr_info("key relocated  0x%.3x->0x%.3x\n", new_idx, index_free);

	return MV_OK;
}

/*-------------------------------------------------------------------------------*/

int mvPp2ClsC3HwMissRead(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c3_entry *c3, int lkp_type)
{
	unsigned int regVal = 0;

	mvPp2ClsC3SwClear(c3);

	c3->index = lkp_type;
	c3->ext_index = NOT_IN_USE;

	regVal = (lkp_type << MVPP2_CLS3_HASH_OP_TBL_ADDR) | MVPP2_CLS3_MISS_PTR_MASK;
	mv_pp2x_write(hw, MVPP2_CLS3_HASH_OP_REG, regVal);

	/* read action table */
	c3->sram.regs.actions = mv_pp2x_read(hw, MVPP2_CLS3_ACT_REG);
	c3->sram.regs.qos_attr = mv_pp2x_read(hw, MVPP2_CLS3_ACT_QOS_ATTR_REG);
	c3->sram.regs.hwf_attr = mv_pp2x_read(hw, MVPP2_CLS3_ACT_HWF_ATTR_REG);
	c3->sram.regs.dup_attr = mv_pp2x_read(hw, MVPP2_CLS3_ACT_DUP_ATTR_REG);
	c3->sram.regs.seq_l_attr = mv_pp2x_read(hw, MVPP2_CLS3_ACT_SEQ_L_ATTR_REG);
	c3->sram.regs.seq_h_attr = mv_pp2x_read(hw, MVPP2_CLS3_ACT_SEQ_H_ATTR_REG);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HwMissRead);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3SwDump(struct mv_pp2x_cls_c3_entry *c3)
{
	int hekSize;

	pr_info("\n");
	pr_info("INDEX[0x%3.3x] ", c3->index);

	hekSize = ((c3->key.key_ctrl & KEY_CTRL_HEK_SIZE_MASK) >> KEY_CTRL_HEK_SIZE);

	/* print extension index if exist*/
	if (hekSize > MVPP2_CLS_C3_HEK_BYTES)
		/* extension */
		pr_info("EXT_INDEX[0x%2.2x] ", c3->ext_index);
	else
		/* without extension */
		pr_info("EXT_INDEX[ NA ] ");

	pr_info("SIZE[0x%2.2x] ", hekSize);
	pr_info("PRT[ID = 0x%2.2x,TYPE = 0x%1.1x] ",
			((c3->key.key_ctrl & KEY_CTRL_PRT_ID_MASK) >> KEY_CTRL_PRT_ID),
			((c3->key.key_ctrl & KEY_CTRL_PRT_ID_TYPE_MASK) >> KEY_CTRL_PRT_ID_TYPE));

	pr_info("LKP_TYPE[0x%1.1x] ",
			((c3->key.key_ctrl & KEY_CTRL_LKP_TYPE_MASK) >> KEY_CTRL_LKP_TYPE));

	pr_info("L4INFO[0x%1.1x] ",
			((c3->key.key_ctrl & KEY_CTRL_L4_MASK) >> KEY_CTRL_L4));

	pr_info("\n\n");
	pr_info("HEK	");
	if (hekSize > MVPP2_CLS_C3_HEK_BYTES)
		/* extension */
		printk(HEK_EXT_FMT, HEK_EXT_VAL(c3->key.hek.words));
	else
		/* without extension */
		printk(HEK_FMT, HEK_VAL(c3->key.hek.words));
	pr_info("\n");
	return mvPp2ClsC3SwActDump(c3);
}
EXPORT_SYMBOL(mvPp2ClsC3SwDump);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3HwDump(struct mv_pp2x_hw *hw)
{
	int index;
	struct mv_pp2x_cls_c3_entry c3;

	mvPp2ClsC3SwClear(&c3);

	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		if (mvCls3ShadowTbl[index].size > 0) {
			mv_pp2x_cls_c3_hw_read(hw, &c3, index);
			mvPp2ClsC3SwDump(&c3);
			printk("----------------------------------------------------------------------\n");
		}
	}

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HwDump);

/*
*All miss entries are valid,
*the key+heks in miss entries are hot in use and this is the
*reason that we dump onlt action table fields
*/
int mvPp2ClsC3HwMissDump(struct mv_pp2x_hw *hw)
{
	int index;
	struct mv_pp2x_cls_c3_entry c3;

	mvPp2ClsC3SwClear(&c3);

	for (index = 0; index < MVPP2_CLS_C3_MISS_TBL_SIZE; index++) {
		mvPp2ClsC3HwMissRead(hw, &c3, index);
		pr_info("INDEX[0x%3.3X]\n", index);
		mvPp2ClsC3SwActDump(&c3);
		pr_info("----------------------------------------------------------------------\n");
	}

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HwMissDump);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3HwExtDump(struct mv_pp2x_hw *hw)
{
	int index, i;
	unsigned int hashExtData[MVPP2_CLS3_HASH_EXT_DATA_REG_NUM];

	pr_info("INDEX    DATA\n");

	for (index = 0; index <  MVPP2_CLS_C3_EXT_TBL_SIZE; index++) {
		if (mvCls3ShadowExtTbl[index] == IN_USE) {
			/* write extension index */
			mv_pp2x_write(hw, MVPP2_CLS3_DB_INDEX_REG, index);

			/* read hash extesion data*/
			for (i = 0; i < MVPP2_CLS3_HASH_EXT_DATA_REG_NUM; i++)
				hashExtData[i] = mv_pp2x_read(hw, MVPP2_CLS3_HASH_EXT_DATA_REG(i));

			pr_info("[0x%2.2x] %8.8x %8.8x %8.8x %8.8x %8.8x %8.8x %8.8x\n",
					index, hashExtData[6], hashExtData[5], hashExtData[4],
					hashExtData[3], hashExtData[2], hashExtData[1], hashExtData[0]);
		}
	}

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HwExtDump);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3ScanNumOfResGet(struct mv_pp2x_hw *hw, int *resNum)
{
	unsigned int regVal, scState;
	int iter = 0;

	do {
		mvPp2ClsC3ScanStateGet(hw, &scState);
	} while (scState != 0 && ((iter++) < RETRIES_EXCEEDED));/*scan compleated*/

	if (iter >= RETRIES_EXCEEDED) {
		pr_err("%s:Error - retries exceeded.\n", __func__);
		return MV_ERROR;
	}

	regVal = mv_pp2x_read(hw, MVPP2_CLS3_STATE_REG);
	regVal &= MVPP2_CLS3_STATE_NO_OF_SC_RES_MASK;
	regVal >>= MVPP2_CLS3_STATE_NO_OF_SC_RES;
	*resNum = regVal;

	return MV_OK;
}

/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3ScanResDump(struct mv_pp2x_hw *hw)
{
	int addr, cnt, resNum, index;

	mvPp2ClsC3ScanNumOfResGet(hw, &resNum);

	pr_info("INDEX	ADDRESS		COUNTER\n");
	for (index = 0; index < resNum; index++) {
		mvPp2ClsC3ScanResRead(hw, index, &addr, &cnt);
		pr_info("[0x%2.2x]\t[0x%3.3x]\t[0x%6.6x]\n", index, addr, cnt);
	}

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3ScanResDump);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3ScanRegs(struct mv_pp2x_hw *hw)
{
	unsigned int prop, propVal;
	unsigned int treshHold;

	treshHold = mv_pp2x_read(hw, MVPP2_CLS3_SC_TH_REG);
	prop = mv_pp2x_read(hw, MVPP2_CLS3_SC_PROP_REG);
	propVal = mv_pp2x_read(hw, MVPP2_CLS3_SC_PROP_VAL_REG);

	pr_info("%-32s: 0x%x = 0x%08x\n", "MV_PP2_CLS3_SC_PROP_REG", MVPP2_CLS3_SC_PROP_REG, prop);
	pr_info("%-32s: 0x%x = 0x%08x\n", "MV_PP2_CLS3_SC_PROP_VAL_REG", MVPP2_CLS3_SC_PROP_VAL_REG, propVal);
	pr_info("\n");

	pr_info("MODE      = %s\n", ((MVPP2_CLS3_SC_PROP_TH_MODE_MASK & prop) == 0) ? "Below" : "Above");
	pr_info("CLEAR     = %s\n", ((MVPP2_CLS3_SC_PROP_CLEAR_MASK & prop) == 0) ? "NoClear" : "Clear  ");

	/* lookup type */
	((MVPP2_CLS3_SC_PROP_LKP_TYPE_EN_MASK & prop) == 0) ?
		pr_info("LKP_TYPE  = NA\n") :
		pr_info("LKP_TYPE  = 0x%x\n",
			((MVPP2_CLS3_SC_PROP_LKP_TYPE_MASK & prop) >> MVPP2_CLS3_SC_PROP_LKP_TYPE));

	/* start index */
	pr_info("START     = 0x%x\n", (MVPP2_CLS3_SC_PROP_START_ENTRY_MASK & prop) >> MVPP2_CLS3_SC_PROP_START_ENTRY);
	/* threshold */
	pr_info("THRESHOLD = 0x%x\n", (MVPP2_CLS3_SC_TH_MASK & treshHold) >> MVPP2_CLS3_SC_TH);

	/* delay value */
	pr_info("DELAY     = 0x%x\n\n",
			(MVPP2_CLS3_SC_PROP_VAL_DELAY_MASK & propVal) >> MVPP2_CLS3_SC_PROP_VAL_DELAY);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3ScanRegs);
/*-------------------------------------------------------------------------------*/
/* if index or occupied_bmp is NULL dump the data				 */
/* index[] size must be 8							 */
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3HwQuery(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c3_entry *c3, unsigned char *occupied_bmp, int index[])
{
	int idx = 0;
	u32 regVal = 0;

	/* write key control */
	mv_pp2x_write(hw, MVPP2_CLS3_KEY_CTRL_REG, c3->key.key_ctrl);

	/* write hek */
	for (idx = 0; idx < MVPP2_CLS_C3_EXT_HEK_WORDS; idx++)
		mv_pp2x_write(hw, MVPP2_CLS3_KEY_HEK_REG(idx), c3->key.hek.words[idx]);

	/*trigger query operation*/
	mv_pp2x_write(hw, MVPP2_CLS3_QRY_ACT_REG, (1 << MVPP2_CLS3_QRY_ACT));

	idx = 0;
	while (!mvPp2ClsC3CpuIsDone(hw))
		if (++idx >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return MV_ERROR;
		}

	regVal = mv_pp2x_read(hw, MVPP2_CLS3_STATE_REG) & MVPP2_CLS3_STATE_OCCIPIED_MASK;
	regVal = regVal >> MVPP2_CLS3_STATE_OCCIPIED;

	if ((!occupied_bmp) || (!index)) {
		/* print to screen - call from sysfs*/
		for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++)
			pr_info("0x%8.8x	%s\n",
				mv_pp2x_read(hw, MVPP2_CLS3_QRY_RES_HASH_REG(idx)),
				(regVal & (1 << idx)) ? "OCCUPIED" : "FREE");
		return MV_OK;
	}

	*occupied_bmp = regVal;
	for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++)
		index[idx] = mv_pp2x_read(hw, MVPP2_CLS3_QRY_RES_HASH_REG(idx));

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HwQuery);
/*-------------------------------------------------------------------------------*/

int mvPp2ClsC3HitCntrsRead(struct mv_pp2x_hw *hw, int index, u32 *cntr)
{
	u32 counter;

	/*write entry index*/
	mv_pp2x_write(hw, MVPP2_CLS3_DB_INDEX_REG, index);

	/*counter read*/
	counter = mv_pp2x_read(hw, MVPP2_CLS3_HIT_COUNTER_REG) & MVPP2_CLS3_HIT_COUNTER_MASK;

	if (!cntr)
		pr_info("ADDR:0x%3.3x	COUNTER VAL:0x%6.6x\n", index, counter);
	else
		*cntr = counter;
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HitCntrsRead);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3HitCntrsMissRead(struct mv_pp2x_hw *hw, int lkp_type, u32 *cntr)
{
	u32 counter;
	int index;

	/*set miss bit to 1*/
	index = (lkp_type | MVPP2_CLS3_DB_MISS_MASK);

	/*write entry index*/
	mv_pp2x_write(hw, MVPP2_CLS3_DB_INDEX_REG, index);

	/*counter read*/
	counter = mv_pp2x_read(hw, MVPP2_CLS3_HIT_COUNTER_REG) & MVPP2_CLS3_HIT_COUNTER_MASK;

	if (!cntr)
		pr_info("LKPT:0x%3.3x	COUNTER VAL:0x%6.6x\n", lkp_type, counter);
	else
		*cntr = counter;
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HitCntrsMissRead);
/*-------------------------------------------------------------------------------*/

int mvPp2ClsC3HitCntrsReadAll(struct mv_pp2x_hw *hw)
{
	unsigned int counter, index;

	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		mvPp2ClsC3HitCntrsRead(hw, index, &counter);

		/* skip initial counter value */
		if (counter == 0)
			continue;

		pr_info("ADDR:0x%3.3x	COUNTER VAL:0x%6.6x\n", index, counter);
	}

	for (index = 0; index < MVPP2_CLS_C3_MISS_TBL_SIZE; index++) {
		mvPp2ClsC3HitCntrsMissRead(hw, index, &counter);

		/* skip initial counter value */
		if (counter == 0)
			continue;

		pr_info("LKPT:0x%3.3x	COUNTER VAL:0x%6.6x\n", index, counter);
	}
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HitCntrsReadAll);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3HwQueryAdd(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c3_entry *c3, int max_search_depth,
			 struct mv_pp2x_cls_c3_hash_pair *hash_pair_arr)
{
	int usedIndex[MVPP2_CLS3_HASH_BANKS_NUM] = {0};
	unsigned char occupied_bmp;
	int idx, index_free, hekSize, status, ext_index = 0;

	status = mvPp2ClsC3HwQuery(hw, c3, &occupied_bmp, usedIndex);

	if (status != MV_OK) {
		pr_err("%s:Error - mvPp2ClsC3HwQuery failed\n", __func__);
		return status;
	}

	/* Select avaliable entry index */
	for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++) {
		if (!(occupied_bmp & (1 << idx)))
			break;
	}

	/* Avaliable index did not found, try to relocate another key */

	if (idx == MVPP2_CLS3_HASH_BANKS_NUM) {

		for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++) {
			if (mvPp2ClsC3HwQueryAddRelocate(hw, usedIndex[idx], max_search_depth,
							0 /*curren depth*/, hash_pair_arr) == MV_OK)
				break;
		}

		if (idx == MVPP2_CLS3_HASH_BANKS_NUM) {
			/* Avaliable index did not found*/
			pr_err("%s:Error - HASH table is full.\n", __func__);
			return MV_ERROR;
		}
	}

	index_free = usedIndex[idx];

	hekSize = ((c3->key.key_ctrl & KEY_CTRL_HEK_SIZE_MASK) >> KEY_CTRL_HEK_SIZE);

	if (hekSize > MVPP2_CLS_C3_HEK_BYTES) {
		/* Get Free Extension Index */
		ext_index = mvPp2ClsC3ShadowExtFreeGet();

		if (ext_index == MVPP2_CLS_C3_EXT_TBL_SIZE) {
			pr_err("%s:Error - Extension table is full.\n", __func__);
			return MV_ERROR;
		}
	}

	status = mvPp2ClsC3HwAdd(hw, c3, index_free, ext_index);

	if (status != MV_OK) {
		pr_err("%s:Error - mvPp2ClsC3HwAdd failed\n", __func__);
		return status;
	}

	if (hekSize > MVPP2_CLS_C3_HEK_BYTES)
		pr_info("Added C3 entry @ index=0x%.3x ext=0x%.3x\n", index_free, ext_index);
	else
		pr_info("Added C3 entry @ index=0x%.3x\n", index_free);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HwQueryAdd);
/*------------------------------------------------------------------------------*/
/*Add entry to hash table							*/
/*ext_index used only if hek size < 12						*/
/*------------------------------------------------------------------------------*/
int mvPp2ClsC3HwAdd(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c3_entry *c3, int index, int ext_index)
{
	int regStartInd, hekSize, iter = 0;
	unsigned int regVal = 0;

	c3->index = index;

	/* write key control */
	mv_pp2x_write(hw, MVPP2_CLS3_KEY_CTRL_REG, c3->key.key_ctrl);

	hekSize = ((c3->key.key_ctrl & KEY_CTRL_HEK_SIZE_MASK) >> KEY_CTRL_HEK_SIZE);

	if (hekSize > MVPP2_CLS_C3_HEK_BYTES) {
		/* Extension */
		c3->ext_index = ext_index;
		regVal |= (ext_index << MVPP2_CLS3_HASH_OP_EXT_TBL_ADDR);

		/* write 9 hek refisters */
		regStartInd = 0;
	} else
		/* write 3 hek refisters */
		regStartInd = 6;

	for (; regStartInd < MVPP2_CLS_C3_EXT_HEK_WORDS; regStartInd++)
		mv_pp2x_write(hw, MVPP2_CLS3_KEY_HEK_REG(regStartInd), c3->key.hek.words[regStartInd]);


	regVal |= (index << MVPP2_CLS3_HASH_OP_TBL_ADDR);
	regVal &= ~MVPP2_CLS3_MISS_PTR_MASK; /*set miss bit to 0, ppv2.1 mas 3.16*/
	regVal |= (1 << MVPP2_CLS3_HASH_OP_ADD);

	/* set hit counter init value */
	mv_pp2x_write(hw, MVPP2_CLS3_INIT_HIT_CNT_REG, SwInitCntSet << MVPP2_CLS3_INIT_HIT_CNT_OFFS);
	/*trigger ADD operation*/
	mv_pp2x_write(hw, MVPP2_CLS3_HASH_OP_REG, regVal);

	/* wait to cpu access done bit */
	while (!mvPp2ClsC3CpuIsDone(hw))
		if (++iter >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return MV_ERROR;
		}

	/* write action table registers */
	mv_pp2x_write(hw, MVPP2_CLS3_ACT_REG, c3->sram.regs.actions);
	mv_pp2x_write(hw, MVPP2_CLS3_ACT_QOS_ATTR_REG, c3->sram.regs.qos_attr);
	mv_pp2x_write(hw, MVPP2_CLS3_ACT_HWF_ATTR_REG, c3->sram.regs.hwf_attr);
	mv_pp2x_write(hw, MVPP2_CLS3_ACT_DUP_ATTR_REG, c3->sram.regs.dup_attr);
	mv_pp2x_write(hw, MVPP2_CLS3_ACT_SEQ_L_ATTR_REG, c3->sram.regs.seq_l_attr);
	mv_pp2x_write(hw, MVPP2_CLS3_ACT_SEQ_H_ATTR_REG, c3->sram.regs.seq_h_attr);
	/* set entry as valid, extesion pointer in use only if size > 12*/
	mvPp2ClsC3ShadowSet(hekSize, index, ext_index);


	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HwAdd);

/* Add entry to miss hash table */
int mvPp2ClsC3HwMissAdd(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c3_entry *c3, int lkp_type)
{
	unsigned int regVal = 0;

	c3->index = lkp_type;

	regVal |= (lkp_type << MVPP2_CLS3_HASH_OP_TBL_ADDR);
	regVal |= (1 << MVPP2_CLS3_HASH_OP_ADD);
	regVal |= MVPP2_CLS3_MISS_PTR_MASK;/*set miss bit to 1, ppv2.1 mas 3.16*/

	/*index to miss table */
	mv_pp2x_write(hw, MVPP2_CLS3_HASH_OP_REG, regVal);

	/* write action table registers */
	mv_pp2x_write(hw, MVPP2_CLS3_ACT_REG, c3->sram.regs.actions);
	mv_pp2x_write(hw, MVPP2_CLS3_ACT_QOS_ATTR_REG, c3->sram.regs.qos_attr);
	mv_pp2x_write(hw, MVPP2_CLS3_ACT_HWF_ATTR_REG, c3->sram.regs.hwf_attr);
	mv_pp2x_write(hw, MVPP2_CLS3_ACT_DUP_ATTR_REG, c3->sram.regs.dup_attr);
	mv_pp2x_write(hw, MVPP2_CLS3_ACT_SEQ_L_ATTR_REG, c3->sram.regs.seq_l_attr);
	mv_pp2x_write(hw, MVPP2_CLS3_ACT_SEQ_H_ATTR_REG, c3->sram.regs.seq_h_attr);
	/*clear hit counter, clear on read */
	mvPp2ClsC3HitCntrsMissRead(hw, lkp_type, &regVal);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HwMissAdd);
/*-------------------------------------------------------------------------------*/

int mvPp2ClsC3HwDel(struct mv_pp2x_hw *hw, int index)
{
	unsigned int regVal = 0;
	int iter = 0;

	regVal |= (index << MVPP2_CLS3_HASH_OP_TBL_ADDR);
	regVal |= (1 << MVPP2_CLS3_HASH_OP_DEL);
	regVal &= ~MVPP2_CLS3_MISS_PTR_MASK;/*set miss bit to 1, ppv2.1 mas 3.16*/


	/*trigger del operation*/
	mv_pp2x_write(hw, MVPP2_CLS3_HASH_OP_REG, regVal);

	/* wait to cpu access done bit */
	while (!mvPp2ClsC3CpuIsDone(hw))
		if (++iter >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return MV_ERROR;
		}

	/* delete form shadow and extension shadow if exist */
	mvPp2C3ShadowClear(index);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HwDel);
/*-------------------------------------------------------------------------------*/

int mvPp2ClsC3HwDelAll(struct mv_pp2x_hw *hw)
{
	int index, status;

	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		status = mvPp2ClsC3HwDel(hw, index);
		if (status != MV_OK)
			return status;
	}
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HwDelAll);
/*-------------------------------------------------------------------------------*/
void mvPp2ClsC3HwInitCtrSet(int cntVal)
{
	SwInitCntSet = cntVal;
}
EXPORT_SYMBOL(mvPp2ClsC3HwInitCtrSet);
/*-------------------------------------------------------------------------------*/

int mvPp2ClsC3HitCntrsClearAll(struct mv_pp2x_hw *hw)
{
	int iter = 0;

	mv_pp2x_write(hw, MVPP2_CLS3_CLEAR_COUNTERS_REG, MVPP2_CLS3_CLEAR_ALL);
	/* wait to clear het counters done bit */
	while (!mvPp2ClsC3HitCntrClearDone(hw))
		if (++iter >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return MV_ERROR;
		}

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HitCntrsClearAll);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3HitCntrsClear(struct mv_pp2x_hw *hw, int lkpType)
{
	/* clear all counters that entry lookup type corresponding to lkpType */
	int iter = 0;

	mv_pp2x_write(hw, MVPP2_CLS3_CLEAR_COUNTERS_REG, lkpType);

	/* wait to clear het counters done bit */
	while (!mvPp2ClsC3HitCntrClearDone(hw))
		if (++iter >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return MV_ERROR;
		}

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3HitCntrsClear);
/*-------------------------------------------------------------------------------*/
/* APIs for Classification C3 hit counters scan fields operation		 */
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3ScanStart(struct mv_pp2x_hw *hw)
{
	int complete, iter = 0;

	/* trigger scan operation */
	mv_pp2x_write(hw, MVPP2_CLS3_SC_ACT_REG, (1 << MVPP2_CLS3_SC_ACT));

	do {
		complete = mvPp2ClsC3ScanIsComplete(hw);

	} while ((!complete) && ((iter++) < RETRIES_EXCEEDED));/*scan compleated*/

	if (iter >= RETRIES_EXCEEDED)
		return MV_ERROR;

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3ScanStart);
/*-------------------------------------------------------------------------------*/

/*mod = 0 below th . mode = 1 above threshold*/
int mvPp2ClsC3ScanThreshSet(struct mv_pp2x_hw *hw, int mode, int thresh)
{
	u32 regVal;

	regVal = mv_pp2x_read(hw, MVPP2_CLS3_SC_PROP_REG);
	regVal &= ~MVPP2_CLS3_SC_PROP_TH_MODE_MASK;
	regVal |= (mode << MVPP2_CLS3_SC_PROP_TH_MODE);
	mv_pp2x_write(hw, MVPP2_CLS3_SC_PROP_REG, regVal);

	regVal = mv_pp2x_read(hw, MVPP2_CLS3_SC_TH_REG);
	regVal &= ~MVPP2_CLS3_SC_TH_MASK;
	regVal |= (thresh << MVPP2_CLS3_SC_TH);
	mv_pp2x_write(hw, MVPP2_CLS3_SC_TH_REG, regVal);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3ScanThreshSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3ScanLkpTypeSet(struct mv_pp2x_hw *hw, int type)
{
	unsigned int prop;

	prop = mv_pp2x_read(hw, MVPP2_CLS3_SC_PROP_REG);

	if (type == -1)
		/* scan all entries */
		prop &= ~(1 << MVPP2_CLS3_SC_PROP_LKP_TYPE_EN);
	else {
		/* scan according to lookup type */
		prop |= (1 << MVPP2_CLS3_SC_PROP_LKP_TYPE_EN);
		prop &= ~MVPP2_CLS3_SC_PROP_LKP_TYPE_MASK;
		prop |= (type << MVPP2_CLS3_SC_PROP_LKP_TYPE);
	}

	mv_pp2x_write(hw, MVPP2_CLS3_SC_PROP_REG, prop);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3ScanLkpTypeSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3ScanClearBeforeEnSet(struct mv_pp2x_hw *hw, int en)
{
	unsigned int prop;

	prop = mv_pp2x_read(hw, MVPP2_CLS3_SC_PROP_REG);

	prop &= ~MVPP2_CLS3_SC_PROP_CLEAR_MASK;
	prop |= (en << MVPP2_CLS3_SC_PROP_CLEAR);

	mv_pp2x_write(hw, MVPP2_CLS3_SC_PROP_REG, prop);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3ScanClearBeforeEnSet);
/*-------------------------------------------------------------------------------*/

int mvPp2ClsC3ScanStartIndexSet(struct mv_pp2x_hw *hw, int idx)
{
	unsigned int prop;

	prop = mv_pp2x_read(hw, MVPP2_CLS3_SC_PROP_REG);

	prop &= ~MVPP2_CLS3_SC_PROP_START_ENTRY_MASK;
	prop |= (idx << MVPP2_CLS3_SC_PROP_START_ENTRY);

	mv_pp2x_write(hw, MVPP2_CLS3_SC_PROP_REG, prop);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3ScanStartIndexSet);
/*-------------------------------------------------------------------------------*/

int mvPp2ClsC3ScanDelaySet(struct mv_pp2x_hw *hw, int time)
{
	unsigned int propVal;

	propVal = mv_pp2x_read(hw, MVPP2_CLS3_SC_PROP_VAL_REG);
	propVal &= ~MVPP2_CLS3_SC_PROP_VAL_DELAY_MASK;
	propVal |= (time << MVPP2_CLS3_SC_PROP_VAL_DELAY);
	mv_pp2x_write(hw, MVPP2_CLS3_SC_PROP_VAL_REG, propVal);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3ScanDelaySet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC3ScanResRead(struct mv_pp2x_hw *hw, int index, int *addr, int *cnt)
{
	unsigned int regVal, scState, addres, counter;
	int iter = 0;

	do {
		mvPp2ClsC3ScanStateGet(hw, &scState);
	} while (scState != 0 && ((iter++) < RETRIES_EXCEEDED));/*scan compleated*/

	if (iter >= RETRIES_EXCEEDED) {
		pr_err("%s:Error - retries exceeded.\n", __func__);
		return MV_ERROR;
	}

	/*write index*/
	mv_pp2x_write(hw, MVPP2_CLS3_SC_INDEX_REG, index);

	/*read date*/
	regVal = mv_pp2x_read(hw, MVPP2_CLS3_SC_RES_REG);
	addres = (regVal & MVPP2_CLS3_SC_RES_ENTRY_MASK) >> MVPP2_CLS3_SC_RES_ENTRY;
	counter = (regVal & MVPP2_CLS3_SC_RES_CTR_MASK) >> MVPP2_CLS3_SC_RES_CTR;
	/* if one of parameters is null - func call from sysfs*/
	if ((!addr) | (!cnt))
		pr_err("INDEX:0x%2.2x	ADDR:0x%3.3x	COUNTER VAL:0x%6.6x\n", index, addres, counter);
	else {
		*addr = addres;
		*cnt = counter;
	}

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC3ScanResRead);

int mvPp2ClsC3Init(struct mv_pp2x_hw *hw)
{
	int rc;

	mvPp2ClsC3ShadowInit();
	rc = mvPp2ClsC3HitCntrsClearAll(hw);
	return rc;
}
EXPORT_SYMBOL(mvPp2ClsC3Init);

/* C4 code */
int mvPp2ClsC4HwPortToRulesSet(struct mv_pp2x_hw *hw, int port, int set, int rules)
{
	unsigned int regVal;

	regVal = (set << MVPP2_CLS4_PHY_TO_RL_GRP) | (rules << MVPP2_CLS4_PHY_TO_RL_RULE_NUM);
	mv_pp2x_write(hw, MVPP2_CLS4_PHY_TO_RL_REG(port), regVal);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4HwPortToRulesSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4HwUniToRulesSet(struct mv_pp2x_hw *hw, int uni, int set, int rules)
{
	unsigned int regVal;

	regVal = (set << MVPP2_CLS4_PHY_TO_RL_GRP) | (rules << MVPP2_CLS4_PHY_TO_RL_RULE_NUM);
	mv_pp2x_write(hw, MVPP2_CLS4_UNI_TO_RL_REG(uni), regVal);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4HwUniToRulesSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4HwPortToRulesGet(struct mv_pp2x_hw *hw, int port, int *set, int *rules)
{
	unsigned int regVal;

	regVal = mv_pp2x_read(hw, MVPP2_CLS4_PHY_TO_RL_REG(port));

	*rules = (regVal & MVPP2_CLS4_PHY_TO_RL_RULE_NUM_MASK) >> MVPP2_CLS4_PHY_TO_RL_RULE_NUM;
	*set = (regVal & MVPP2_CLS4_PHY_TO_RL_GRP_MASK) >> MVPP2_CLS4_PHY_TO_RL_GRP;

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4HwPortToRulesGet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4HwUniToRulesGet(struct mv_pp2x_hw *hw, int uni, int *set, int *rules)
{
	unsigned int regVal;

	regVal = mv_pp2x_read(hw, MVPP2_CLS4_UNI_TO_RL_REG(uni));

	*rules = (regVal & MVPP2_CLS4_PHY_TO_RL_RULE_NUM_MASK) >> MVPP2_CLS4_PHY_TO_RL_RULE_NUM;
	*set = (regVal & MVPP2_CLS4_PHY_TO_RL_GRP_MASK) >> MVPP2_CLS4_PHY_TO_RL_GRP;

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4HwUniToRulesGet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4HwRead(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c4_entry *C4, int rule, int set)
{
	unsigned int regVal = 0;
	int regInd;

	/* write index reg */
	regVal = (set << MVPP2_CLS4_RL_INDEX_GRP) | (rule << MVPP2_CLS4_RL_INDEX_RULE);
	mv_pp2x_write(hw, MVPP2_CLS4_RL_INDEX_REG, regVal);

	C4->ruleIndex = rule;
	C4->setIndex = set;
	/* read entry rule fields*/
	C4->rules.regs.attr[0] = mv_pp2x_read(hw, MVPP2_CLS4_FATTR1_REG);
	C4->rules.regs.attr[1] = mv_pp2x_read(hw, MVPP2_CLS4_FATTR2_REG);

	for (regInd = 0; regInd < MVPP2_CLS_C4_TBL_DATA_WORDS; regInd++)
		C4->rules.regs.fdataArr[regInd] = mv_pp2x_read(hw, MVPP2_CLS4_FDATA_REG(regInd));

	/* read entry from action table */
	C4->sram.regs.actions = mv_pp2x_read(hw, MVPP2_CLS4_ACT_REG);
	C4->sram.regs.qos_attr = mv_pp2x_read(hw, MVPP2_CLS4_ACT_QOS_ATTR_REG);
	C4->sram.regs.dup_attr = mv_pp2x_read(hw, MVPP2_CLS4_ACT_DUP_ATTR_REG);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4HwRead);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4HwWrite(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c4_entry *C4, int rule, int set)
{
	unsigned int regVal = 0;
	int regInd;

	/* write index reg */
	regVal = (set << MVPP2_CLS4_RL_INDEX_GRP) | (rule << MVPP2_CLS4_RL_INDEX_RULE);
	mv_pp2x_write(hw, MVPP2_CLS4_RL_INDEX_REG, regVal);

	mv_pp2x_write(hw, MVPP2_CLS4_FATTR1_REG, C4->rules.regs.attr[0]);
	mv_pp2x_write(hw, MVPP2_CLS4_FATTR2_REG, C4->rules.regs.attr[1]);

	for (regInd = 0; regInd < MVPP2_CLS_C4_TBL_DATA_WORDS; regInd++)
		mv_pp2x_write(hw, MVPP2_CLS4_FDATA_REG(regInd), C4->rules.regs.fdataArr[regInd]);

	/* read entry from action table */
	mv_pp2x_write(hw, MVPP2_CLS4_ACT_REG, C4->sram.regs.actions);
	mv_pp2x_write(hw, MVPP2_CLS4_ACT_QOS_ATTR_REG, C4->sram.regs.qos_attr);
	mv_pp2x_write(hw, MVPP2_CLS4_ACT_DUP_ATTR_REG, C4->sram.regs.dup_attr);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4HwWrite);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4SwDump(struct mv_pp2x_cls_c4_entry *C4)
{
	int index;

	pr_info("SET: %d	RULE: %d\n", C4->setIndex, C4->ruleIndex);
	pr_info("FIELD  ID  OP	DATA\n");

	/*------------------------------*/
	/*	   fields 0-2		*/
	/*------------------------------*/

	for (index = 0; index <  4; index++) {
		pr_info("%d       %d  %d	",
				index,
				MVPP2_CLS4_FATTR_ID_VAL(index, C4->rules.regs.attr[GET_FIELD_ATRR(index)]),
				MVPP2_CLS4_FATTR_OPCODE_VAL(index, C4->rules.regs.attr[GET_FIELD_ATRR(index)]));

		pr_info(FLD_FMT, FLD_VAL(index, C4->rules.regs.fdataArr));
		pr_info("\n");
	}

	/*------------------------------*/
	/*	   field 4		*/
	/*------------------------------*/

	/* index = 4 after loop */
	pr_info("%d       %d  %d	",
			index,
			MVPP2_CLS4_FATTR_ID_VAL(index, C4->rules.regs.attr[GET_FIELD_ATRR(index)]),
			MVPP2_CLS4_FATTR_OPCODE_VAL(index, C4->rules.regs.attr[GET_FIELD_ATRR(index)]));
	pr_info(FLD4_FMT, FLD4_VAL(C4->rules.regs.fdataArr));
	pr_info("\n");

	/*------------------------------*/
	/*	   field 5		*/
	/*------------------------------*/
	index++;

	pr_info("%d       %d  %d	",
			index,
			MVPP2_CLS4_FATTR_ID_VAL(index, C4->rules.regs.attr[GET_FIELD_ATRR(index)]),
			MVPP2_CLS4_FATTR_OPCODE_VAL(index, C4->rules.regs.attr[GET_FIELD_ATRR(index)]));
	pr_info(FLD5_FMT, FLD5_VAL(C4->rules.regs.fdataArr));
	pr_info("\n");
	pr_info("\n");
	pr_info("VLAN: %d  PPPOE: %d  MACME: %d  L4INFO: %d  L3INFO: %d\n",
			MVPP2_CLS4_VLAN_VAL(C4->rules.regs.fdataArr[6]),
			MVPP2_CLS4_PPPOE_VAL(C4->rules.regs.fdataArr[6]),
			MVPP2_CLS4_MACME_VAL(C4->rules.regs.fdataArr[6]),
			MVPP2_CLS4_L4INFO_VAL(C4->rules.regs.fdataArr[6]),
			MVPP2_CLS4_L3INFO_VAL(C4->rules.regs.fdataArr[6]));
	pr_info("\n");
	/*------------------------------*/
	/*	actions	0x1E80		*/
	/*------------------------------*/
	pr_info("ACT_TBL:COLOR	PRIO	DSCP	GPID	LOW_Q	HIGH_Q	POLICER		FWD\n");
	pr_info("CMD:    [%1d]	[%1d]	[%1d]	[%1d]	[%1d]	[%1d]	[%1d]		[%1d]\n",
			((C4->sram.regs.actions & (MVPP2_CLS2_ACT_COLOR_MASK)) >> MVPP2_CLS2_ACT_COLOR_OFF),
			((C4->sram.regs.actions & (MVPP2_CLS2_ACT_PRI_MASK)) >> MVPP2_CLS2_ACT_PRI_OFF),
			((C4->sram.regs.actions & (MVPP2_CLS2_ACT_DSCP_MASK)) >> MVPP2_CLS2_ACT_DSCP_OFF),
			((C4->sram.regs.actions & (MVPP2_CLS2_ACT_GEM_MASK)) >> MVPP2_CLS2_ACT_GEM_OFF),
			((C4->sram.regs.actions & (MVPP2_CLS2_ACT_QL_MASK)) >> MVPP2_CLS2_ACT_QL_OFF),
			((C4->sram.regs.actions & (MVPP2_CLS2_ACT_QH_MASK)) >> MVPP2_CLS2_ACT_QH_OFF),
			((C4->sram.regs.actions & (MVPP2_CLS2_ACT_PLCR_MASK)) >> MVPP2_CLS2_ACT_PLCR_OFF),
			((C4->sram.regs.actions & MVPP2_CLS2_ACT_FRWD_MASK) >> MVPP2_CLS2_ACT_FRWD_OFF));


	/*------------------------------*/
	/*	qos_attr 0x1E84		*/
	/*------------------------------*/
	pr_info("VAL:		[%1d]	[%1d]	[%1d]	[%1d]	[0x%x]	[id 0x%2.2x bank %1.1x]\n",
		((C4->sram.regs.qos_attr & (MVPP2_CLS2_ACT_QOS_ATTR_PRI_MASK)) >> MVPP2_CLS2_ACT_QOS_ATTR_PRI_OFF),
		((C4->sram.regs.qos_attr & (MVPP2_CLS2_ACT_QOS_ATTR_DSCP_MASK)) >> MVPP2_CLS2_ACT_QOS_ATTR_DSCP_OFF),
		((C4->sram.regs.qos_attr & (MVPP2_CLS2_ACT_QOS_ATTR_GEM_MASK)) >> MVPP2_CLS2_ACT_QOS_ATTR_GEM_OFF),
		((C4->sram.regs.qos_attr & (MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK)) >> MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF),
		((C4->sram.regs.qos_attr & (MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK)) >> MVPP2_CLS2_ACT_QOS_ATTR_QH_OFF),
		((C4->sram.regs.dup_attr & (MVPP2_CLS2_ACT_DUP_ATTR_DUPID_MASK)) >> MVPP2_CLS2_ACT_DUP_ATTR_DUPID_OFF),
		((C4->sram.regs.dup_attr & MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_MASK) >> MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_OFF));


	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4SwDump);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4HwCntDump(struct mv_pp2x_hw *hw, int rule, int set, unsigned int *cnt)
{
	unsigned int regVal;

	/* write index */
	regVal =  MVPP2_CNT_IDX_RULE(rule, set);
	mv_pp2x_write(hw, MVPP2_CNT_IDX_REG, regVal);

	/*read hit counter*/
	regVal = mv_pp2x_read(hw, MVPP2_CLS_C4_TBL_HIT_REG);

	if (cnt)
		*cnt = regVal;
	else
		pr_info("HIT COUNTER: %d\n", regVal);

	return MV_OK;
}
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4HwDumpAll(struct mv_pp2x_hw *hw)
{
	int set, rule;
	struct mv_pp2x_cls_c4_entry C4;

	for (set = 0; set < MVPP2_CLS_C4_GRPS_NUM; set++)
		for (rule = 0; rule <  MVPP2_CLS_C4_GRP_SIZE; rule++) {
			mvPp2ClsC4HwRead(hw, &C4, rule, set);
			mvPp2ClsC4SwDump(&C4);
			mvPp2ClsC4HwCntDump(hw, rule, set, NULL);
			pr_info("--------------------------------------------------------------------\n");
		}
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4HwDumpAll);
/*-------------------------------------------------------------------------------*/
/* mvPp2ClsC4HwHitsDump - dump all non zeroed hit counters and the associated  HWentries */
int mvPp2ClsC4HwHitsDump(struct mv_pp2x_hw *hw)
{
	int set, rule;
	unsigned int cnt;
	struct mv_pp2x_cls_c4_entry C4;

	for (set = 0; set < MVPP2_CLS_C4_GRPS_NUM; set++)
		for (rule = 0; rule <  MVPP2_CLS_C4_GRP_SIZE; rule++) {
			mvPp2ClsC4HwCntDump(hw, rule, set, &cnt);
			if (cnt == 0)
				continue;

			mvPp2ClsC4HwRead(hw, &C4, rule, set);
			mvPp2ClsC4SwDump(&C4);
			pr_info("HITS: %d\n", cnt);
			pr_info("--------------------------------------------------------------------\n");
		}
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4HwHitsDump);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4RegsDump(struct mv_pp2x_hw *hw)
{
	int i = 0;
	char reg_name[100];


	for (i = 0; i < MVPP2_MAX_PORTS; i++) {
		sprintf(reg_name, "MVPP2_CLS4_PHY_TO_RL_%d_REG", i);
		mv_pp2x_print_reg(hw, MVPP2_CLS4_PHY_TO_RL_REG(i), reg_name);
	}

	for (i = 0; i < MVPP2_MAX_PORTS; i++) {
		sprintf(reg_name, "MVPP2_CLS4_UNI_TO_RL_%d_REG", i);
		mv_pp2x_print_reg(hw, MVPP2_CLS4_UNI_TO_RL_REG(i), reg_name);
	}

	mv_pp2x_print_reg(hw, MVPP2_CLS4_FATTR1_REG, "MVPP2_CLS4_FATTR1_REG");
	mv_pp2x_print_reg(hw, MVPP2_CLS4_FATTR2_REG, "MVPP2_CLS4_FATTR2_REG");

	for (i = 0; i < MVPP2_CLS4_FDATA_REGS_NUM; i++) {
		sprintf(reg_name, "MVPP2_CLS4_FDATA_%d_REG", i);
		mv_pp2x_print_reg(hw, MVPP2_CLS4_FDATA_REG(i), reg_name);
	}

	mv_pp2x_print_reg(hw, MVPP2_CLS4_RL_INDEX_REG, "MVPP2_CLS4_RL_INDEX_REG");
	mv_pp2x_print_reg(hw, MVPP2_CLS4_ACT_REG, "M_PP2_CLS4_ACT_REG");
	mv_pp2x_print_reg(hw, MVPP2_CLS4_ACT_QOS_ATTR_REG, "MVPP2_CLS4_ACT_QOS_ATTR_REG");
	mv_pp2x_print_reg(hw, MVPP2_CLS4_ACT_DUP_ATTR_REG, "MVPP2_CLS4_ACT_DUP_ATTR_REG");

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4RegsDump);
/*-------------------------------------------------------------------------------*/
void mvPp2ClsC4SwClear(struct mv_pp2x_cls_c4_entry *C4)
{
	memset(C4, 0, sizeof(struct mv_pp2x_cls_c4_entry));
}
EXPORT_SYMBOL(mvPp2ClsC4SwClear);
/*-------------------------------------------------------------------------------*/
void mvPp2ClsC4HwClearAll(struct mv_pp2x_hw *hw)
{
	int set, rule;
	struct mv_pp2x_cls_c4_entry C4;

	mvPp2ClsC4SwClear(&C4);

	for (set = 0; set < MVPP2_CLS_C4_GRPS_NUM; set++)
		for (rule = 0; rule <  MVPP2_CLS_C4_GRP_SIZE; rule++)
			mvPp2ClsC4HwWrite(hw, &C4, rule, set);
}
EXPORT_SYMBOL(mvPp2ClsC4HwClearAll);

/*
*set two bytes of data in fields
*offs - offset in byte resolution
*/
int mvPp2ClsC4FieldsShortSet(struct mv_pp2x_cls_c4_entry *C4, int field, unsigned int offs, unsigned short data)
{

	if ((offs % 2) != 0) {
		pr_err("mvCls4Hw %s: offset should be even , current func write two bytes of data.\n", __func__);
		return MV_ERROR;
	}

	if (field < 4) {
		/* fields 0,1,2,3 lenght is 2 bytes */
		C4->rules.regs.fdataArr[field/2] &= ~(0xFFFF << (16 * (field % 2)));
		C4->rules.regs.fdataArr[field/2] |= (data << (16 * (field % 2)));
	}

	else if (field == 4) {
		/* field 4 lenght is 16 bytes */
		C4->rules.regs.fdataArr[5 - offs/4] &= ~(0xFFFF << (16 * ((offs / 2) % 2)));
		C4->rules.regs.fdataArr[5 - offs/4] |= (data << (16 * ((offs / 2) % 2)));
	} else {
		/* field 5 lenght is 6 bytes */
		C4->rules.regs.fdataArr[7 - offs/4] &= ~(0xFFFF << (16 * ((offs / 2) % 2)));
		C4->rules.regs.fdataArr[7 - offs/4] |= (data << (16 * ((offs / 2) % 2)));
	}

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4FieldsShortSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4FieldsParamsSet(struct mv_pp2x_cls_c4_entry *C4, int field, unsigned int id, unsigned int op)
{
	/* clear old ID and opcode*/
	C4->rules.regs.attr[GET_FIELD_ATRR(field)] &= ~MVPP2_CLS4_FATTR_ID_MASK(field);
	C4->rules.regs.attr[GET_FIELD_ATRR(field)] &= ~MVPP2_CLS4_FATTR_OPCODE_MASK(field);

	/* write new values */
	C4->rules.regs.attr[GET_FIELD_ATRR(field)] |= (op << MVPP2_CLS4_FATTR_OPCODE(field));
	C4->rules.regs.attr[GET_FIELD_ATRR(field)] |= (id << MVPP2_CLS4_FATTR_ID(field));

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4FieldsParamsSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4SwVlanSet(struct mv_pp2x_cls_c4_entry *C4, int vlan)
{
	C4->rules.regs.fdataArr[6] &= ~MVPP2_CLS4_VLAN_MASK;
	C4->rules.regs.fdataArr[6] |= (vlan << MVPP2_CLS4_FDATA7_VLAN);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4SwVlanSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4SwPppoeSet(struct mv_pp2x_cls_c4_entry *C4, int pppoe)
{
	C4->rules.regs.fdataArr[6] &= ~MVPP2_CLS4_PPPOE_MASK;
	C4->rules.regs.fdataArr[6] |= (pppoe << MVPP2_CLS4_FDATA7_PPPOE);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4SwPppoeSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4SwMacMeSet(struct mv_pp2x_cls_c4_entry *C4, int mac)
{
	C4->rules.regs.fdataArr[6] &= ~MVPP2_CLS4_MACME_MASK;
	C4->rules.regs.fdataArr[6] |= (mac << MVPP2_CLS4_FDATA7_MACME);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4SwMacMeSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4SwL4InfoSet(struct mv_pp2x_cls_c4_entry *C4, int info)
{
	C4->rules.regs.fdataArr[6] &= ~MVPP2_CLS4_L4INFO_MASK;
	C4->rules.regs.fdataArr[6] |= (info << MVPP2_CLS4_FDATA7_L4INFO);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4SwL4InfoSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4SwL3InfoSet(struct mv_pp2x_cls_c4_entry *C4, int info)
{
	C4->rules.regs.fdataArr[6] &= ~MVPP2_CLS4_L3INFO_MASK;
	C4->rules.regs.fdataArr[6] |= (info << MVPP2_CLS4_FDATA7_L3INFO);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4SwL3InfoSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4ColorSet(struct mv_pp2x_cls_c4_entry *C4, int cmd)
{
	C4->sram.regs.actions &= ~MVPP2_CLS2_ACT_COLOR_MASK;
	C4->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_COLOR_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4ColorSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4PrioSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int prio)
{
	/*set command*/
	C4->sram.regs.actions &= ~MVPP2_CLS2_ACT_PRI_MASK;
	C4->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_PRI_OFF);

	/*set modify priority value*/
	C4->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_PRI_MASK;
	C4->sram.regs.qos_attr |= (prio << MVPP2_CLS2_ACT_QOS_ATTR_PRI_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4PrioSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4DscpSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int dscp)
{
	/*set command*/
	C4->sram.regs.actions &= ~MVPP2_CLS2_ACT_DSCP_MASK;
	C4->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_DSCP_OFF);

	/*set modify DSCP value*/
	C4->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_DSCP_MASK;
	C4->sram.regs.qos_attr |= (dscp << MVPP2_CLS2_ACT_QOS_ATTR_DSCP_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4DscpSet);
/*-------------------------------------------------------------------------------*/

int mvPp2ClsC4GpidSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int gid)
{
	/*set command*/
	C4->sram.regs.actions &= ~MVPP2_CLS2_ACT_GEM_MASK;
	C4->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_GEM_OFF);

	/*set modify DSCP value*/
	C4->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_GEM_MASK;
	C4->sram.regs.qos_attr |= (gid << MVPP2_CLS2_ACT_QOS_ATTR_GEM_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4GpidSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4PolicerSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int policerId, int bank)
{
	C4->sram.regs.actions &= ~MVPP2_CLS2_ACT_PLCR_MASK;
	C4->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_PLCR_OFF);

	C4->sram.regs.dup_attr &= ~MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_MASK;
	C4->sram.regs.dup_attr |= (policerId << MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_OFF);

	if (bank)
		C4->sram.regs.dup_attr |= MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_MASK;
	else
		C4->sram.regs.dup_attr &= ~MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_MASK;

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4PolicerSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4QueueHighSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int queue)
{
	/*set command*/
	C4->sram.regs.actions &= ~MVPP2_CLS2_ACT_QH_MASK;
	C4->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_QH_OFF);

	/*set modify High queue value*/
	C4->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK;
	C4->sram.regs.qos_attr |= (queue << MVPP2_CLS2_ACT_QOS_ATTR_QH_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4QueueHighSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4QueueLowSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int queue)
{
	/*set command*/
	C4->sram.regs.actions &= ~MVPP2_CLS2_ACT_QL_MASK;
	C4->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_QL_OFF);

	/*set modify High queue value*/
	C4->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK;
	C4->sram.regs.qos_attr |= (queue << MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4QueueLowSet);
/*-------------------------------------------------------------------------------*/

int mvPp2ClsC4QueueSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int queue)
{
	int status = MV_OK;
	int qHigh, qLow;

	/* cmd validation in set functions */

	qHigh = (queue & MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK) >> MVPP2_CLS2_ACT_QOS_ATTR_QH_OFF;
	qLow = (queue & MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK) >> MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF;

	status |= mvPp2ClsC4QueueLowSet(C4, cmd, qLow);
	status |= mvPp2ClsC4QueueHighSet(C4, cmd, qHigh);

	return status;

}
EXPORT_SYMBOL(mvPp2ClsC4QueueSet);
/*-------------------------------------------------------------------------------*/
int mvPp2ClsC4ForwardSet(struct mv_pp2x_cls_c4_entry *C4, int cmd)
{
	C4->sram.regs.actions &= ~MVPP2_CLS2_ACT_FRWD_MASK;
	C4->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_FRWD_OFF);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2ClsC4ForwardSet);

/* PME */

static char *mvPp2PmeCmdName(enum mv_pp2x_pme_instr cmd)
{
	switch (cmd) {
	case MVPP2_PME_CMD_NONE:		return "NO_MOD";
	case MVPP2_PME_CMD_ADD_2B:		return "ADD_2B";
	case MVPP2_PME_CMD_CFG_VLAN:	return "CFG_VLAN";
	case MVPP2_PME_CMD_ADD_VLAN:	return "ADD_VLAN";
	case MVPP2_PME_CMD_CFG_DSA_1:	return "CFG_DSA_1";
	case MVPP2_PME_CMD_CFG_DSA_2:	return "CFG_DSA_2";
	case MVPP2_PME_CMD_ADD_DSA:	return "ADD_DSA";
	case MVPP2_PME_CMD_DEL_BYTES:	return "DEL_BYTES";
	case MVPP2_PME_CMD_REPLACE_2B: return "REPLACE_2B";
	case MVPP2_PME_CMD_REPLACE_LSB: return "REPLACE_LSB";
	case MVPP2_PME_CMD_REPLACE_MSB: return "REPLACE_MSB";
	case MVPP2_PME_CMD_REPLACE_VLAN: return "REPLACE_VLAN";
	case MVPP2_PME_CMD_DEC_LSB:	return "DEC_LSB";
	case MVPP2_PME_CMD_DEC_MSB:	return "DEC_MSB";
	case MVPP2_PME_CMD_ADD_CALC_LEN: return "ADD_CALC_LEN";
	case MVPP2_PME_CMD_REPLACE_LEN: return "REPLACE_LEN";
	case MVPP2_PME_CMD_IPV4_CSUM:	return "IPV4_CSUM";
	case MVPP2_PME_CMD_L4_CSUM:	return "L4_CSUM";
	case MVPP2_PME_CMD_SKIP:		return "SKIP";
	case MVPP2_PME_CMD_JUMP:		return "JUMP";
	case MVPP2_PME_CMD_JUMP_SKIP:	return "JUMP_SKIP";
	case MVPP2_PME_CMD_JUMP_SUB:	return "JUMP_SUB";
	case MVPP2_PME_CMD_PPPOE:		return "PPPOE";
	case MVPP2_PME_CMD_STORE:		return "STORE";
	case MVPP2_PME_CMD_ADD_IP4_CSUM: return "ADD_L4";
	case MVPP2_PME_CMD_PPPOE_2:	return "PPPOE_2";
	case MVPP2_PME_CMD_REPLACE_MID: return "REPLACE_MID";
	case MVPP2_PME_CMD_ADD_MULT:	return "ADD_MULT";
	case MVPP2_PME_CMD_REPLACE_MULT: return "REPLACE_MULT";
	case MVPP2_PME_CMD_REPLACE_REM_2B: return "REPLACE_REM_2B"; /* For PPv2.1 - A0 only, MAS 3.3 */
	case MVPP2_PME_CMD_ADD_IP6_HDR: return "ADD_IP6_HDR";       /* For PPv2.1 - A0 only, MAS 3.15 */
	case MVPP2_PME_CMD_DROP_PKT:	return "DROP";
	default:
		return "UNKNOWN";
	}
	return NULL;
};


int mvPp2PmeHwWrite(struct mv_pp2x_hw *hw, int idx, struct mv_pp2x_pme_entry *pEntry)
{
	if ((idx < 0) || (idx >= MVPP2_PME_INSTR_SIZE)) {
		pr_err("%s: entry %d is out of range [0..%d]\n", __func__, idx, MVPP2_PME_INSTR_SIZE);
		return MV_ERROR;
	}
	pEntry->index = idx;
	mv_pp2x_write(hw, MVPP2_PME_TBL_IDX_REG, idx);
	mv_pp2x_write(hw, MVPP2_PME_TBL_INSTR_REG, pEntry->word);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeHwWrite);

int mvPp2PmeHwRead(struct mv_pp2x_hw *hw, int idx, struct mv_pp2x_pme_entry *pEntry)
{
	if ((idx < 0) || (idx >= MVPP2_PME_INSTR_SIZE)) {
		pr_err("%s: entry %d is out of range [0..%d]\n", __func__, idx, MVPP2_PME_INSTR_SIZE);
		return MV_ERROR;
	}

	pEntry->index = idx;
	mv_pp2x_write(hw, MVPP2_PME_TBL_IDX_REG, idx);
	pEntry->word = mv_pp2x_read(hw, MVPP2_PME_TBL_INSTR_REG);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeHwRead);

int mvPp2PmeSwDump(struct mv_pp2x_pme_entry *pEntry)
{
	pr_info("%04x %04x: ", MVPP2_PME_CTRL_GET(pEntry), MVPP2_PME_DATA_GET(pEntry));

	pr_info("%s ", mvPp2PmeCmdName(MVPP2_PME_CMD_GET(pEntry)));

	if (pEntry->word & MVPP2_PME_IP4_CSUM_MASK)
		pr_info(", IPv4 csum");

	if (pEntry->word & MVPP2_PME_L4_CSUM_MASK)
		pr_info(", L4 csum");

	if (pEntry->word & MVPP2_PME_LAST_MASK)
		pr_info(", Last");

	pr_info("\n");

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeSwDump);

int mvPp2PmeHwDump(struct mv_pp2x_hw *hw, int mode)
{
	int idx, count = 0;
	struct mv_pp2x_pme_entry entry;
	int status;

	pr_info("PME instraction table: #%d entries\n", MVPP2_PME_INSTR_SIZE);
	for (idx = 0; idx < MVPP2_PME_INSTR_SIZE; idx++) {
		status = mvPp2PmeHwRead(hw, idx, &entry);
		if (status != MV_OK) {
			pr_err("%s failed: idx=%d, status=%d\n", __func__, idx, status);
			return status;
		}
		if (mode == 0) {
			if (!MVPP2_PME_IS_VALID(&entry))
				continue;
		}

		count++;
		pr_info("[%4d]: ", idx);
		mvPp2PmeSwDump(&entry);
	}

	if (!count)
		pr_info("Table is Empty\n");

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeHwDump);

int mvPp2PmeSwClear(struct mv_pp2x_pme_entry *pEntry)
{
	pEntry->word = 0;

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeSwClear);

int mvPp2PmeSwWordSet(struct mv_pp2x_pme_entry *pEntry, u32 word)
{
	pEntry->word = word;

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeSwWordSet);

int mvPp2PmeSwCmdSet(struct mv_pp2x_pme_entry *pEntry, enum mv_pp2x_pme_instr cmd)
{
	pEntry->word &= ~MVPP2_PME_CMD_ALL_MASK;
	pEntry->word |= MVPP2_PME_CMD_MASK(cmd);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeSwCmdSet);

int mvPp2PmeSwCmdTypeSet(struct mv_pp2x_pme_entry *pEntry, int type)
{
	pEntry->word &= ~MVPP2_PME_CMD_TYPE_ALL_MASK;
	pEntry->word |= MVPP2_PME_CMD_TYPE_MASK(type);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeSwCmdTypeSet);

int mvPp2PmeSwCmdLastSet(struct mv_pp2x_pme_entry *pEntry, int last)
{
	if (last)
		pEntry->word |= MVPP2_PME_LAST_MASK;
	else
		pEntry->word &= ~MVPP2_PME_LAST_MASK;

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeSwCmdLastSet);

int mvPp2PmeSwCmdFlagsSet(struct mv_pp2x_pme_entry *pEntry, int last, int ipv4, int l4)
{
	if (last)
		pEntry->word |= MVPP2_PME_LAST_MASK;
	else
		pEntry->word &= ~MVPP2_PME_LAST_MASK;

	if (ipv4)
		pEntry->word |= MVPP2_PME_IP4_CSUM_MASK;
	else
		pEntry->word &= ~MVPP2_PME_IP4_CSUM_MASK;

	if (l4)
		pEntry->word |= MVPP2_PME_L4_CSUM_MASK;
	else
		pEntry->word &= ~MVPP2_PME_L4_CSUM_MASK;

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeSwCmdFlagsSet);

int mvPp2PmeSwCmdDataSet(struct mv_pp2x_pme_entry *pEntry, u16 data)
{
	MVPP2_PME_DATA_SET(pEntry, data);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeSwCmdDataSet);

static inline u32 mvPp2PmeDataTblRegAddr(int tbl)
{
	u32 regAddr;

	switch (tbl) {
	case 0:
		regAddr = MVPP2_PME_TBL_DATA1_REG;
		break;
	case 1:
		regAddr = MVPP2_PME_TBL_DATA2_REG;
		break;
	default:
		regAddr = 0;
		pr_err("%s: tbl %d is out of range [0..1]\n", __func__, tbl);
	}
	return regAddr;
}

static int mvPp2PmeDataTblSize(int tbl)
{
	int max;

	switch (tbl) {
	case 0:
		max = MVPP2_PME_DATA1_SIZE;
		break;
	case 1:
		max = MVPP2_PME_DATA2_SIZE;
		break;
	default:
		max = 0;
		pr_err("%s: tbl %d is out of range [0..1]\n", __func__, tbl);
	}
	return max;
}

/* Functions to access PME data1 and data2 tables */
int mvPp2PmeHwDataTblWrite(struct mv_pp2x_hw *hw, int tbl, int idx, u16 data)
{
	u32  regVal;

	if ((tbl < 0) || (tbl > 1)) {
		pr_err("%s: data table %d is out of range [0..1]\n", __func__, tbl);
		return MV_ERROR;
	}
	if ((idx < 0) || (idx >= mvPp2PmeDataTblSize(tbl))) {
		pr_err("%s: entry index #%d is out of range [0..%d] for data table #%d\n",
					__func__, idx, tbl, mvPp2PmeDataTblSize(tbl));
		return MV_ERROR;
	}

	mv_pp2x_write(hw, MVPP2_PME_TBL_IDX_REG, idx / 2);

	regVal = mv_pp2x_read(hw, mvPp2PmeDataTblRegAddr(tbl));
	regVal &= ~MVPP2_PME_TBL_DATA_MASK(idx % 2);
	regVal |= (data << MVPP2_PME_TBL_DATA_OFFS(idx % 2));

	mv_pp2x_write(hw, MVPP2_PME_TBL_IDX_REG, idx / 2);
	mv_pp2x_write(hw, mvPp2PmeDataTblRegAddr(tbl), regVal);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeHwDataTblWrite);

int mvPp2PmeHwDataTblRead(struct mv_pp2x_hw *hw, int tbl, int idx, u16 *data)
{
	u32  regVal;

	if ((tbl < 0) || (tbl > 1)) {
		pr_err("%s: data table %d is out of range [0..1]\n", __func__, tbl);
		return MV_ERROR;
	}
	if ((idx < 0) || (idx >= mvPp2PmeDataTblSize(tbl))) {
		pr_err("%s: entry index #%d is out of range [0..%d] for data table #%d\n",
					__func__, idx, tbl, mvPp2PmeDataTblSize(tbl));
		return MV_ERROR;
	}

	mv_pp2x_write(hw, MVPP2_PME_TBL_IDX_REG, idx / 2);

	regVal = mv_pp2x_read(hw, mvPp2PmeDataTblRegAddr(tbl));

	if (data)
		*data = (regVal & MVPP2_PME_TBL_DATA_MASK(idx % 2)) >> MVPP2_PME_TBL_DATA_OFFS(idx % 2);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeHwDataTblRead);

int mvPp2PmeHwDataTblDump(struct mv_pp2x_hw *hw, int tbl)
{
	int idx, max, count = 0;
	u16 data;

	if ((tbl < 0) || (tbl > 1)) {
		pr_err("%s: data table %d is out of range [0..1]\n", __func__, tbl);
		return MV_ERROR;
	}
	max = mvPp2PmeDataTblSize(tbl);

	pr_info("PME Data%d table: #%d entries\n", tbl + 1, max);
	for (idx = 0; idx < max; idx++) {
		mvPp2PmeHwDataTblRead(hw, tbl, idx, &data);
		if (data != 0) {
			pr_info("[%4d]: 0x%04x\n", idx, data);
			count++;
		}
	}
	if (count == 0)
		pr_info("Table is Empty\n");

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeHwDataTblDump);

int mvPp2PmeHwDataTblClear(struct mv_pp2x_hw *hw, int tbl)
{
	int max, idx;

	if ((tbl < 0) || (tbl > 1)) {
		pr_err("%s: data table %d is out of range [0..1]\n", __func__, tbl);
		return MV_ERROR;
	}

	max = mvPp2PmeDataTblSize(tbl);
	for (idx = 0; idx < max; idx++)
		mvPp2PmeHwDataTblWrite(hw, tbl, idx, 0);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeHwDataTblClear);

/* Functions to set other PME register fields */
int mvPp2PmeVlanEtherTypeSet(struct mv_pp2x_hw *hw, int idx, u16 ethertype)
{
	u32 regVal = (u32)ethertype;

	if ((idx < 0) || (idx > MVPP2_PME_MAX_VLAN_ETH_TYPES)) {
		pr_err("%s: idx %d is out of range [0..%d]\n", __func__, idx, MVPP2_PME_MAX_VLAN_ETH_TYPES);
		return MV_ERROR;
	}
	mv_pp2x_write(hw, MVPP2_PME_VLAN_ETH_TYPE_REG(idx), regVal);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeVlanEtherTypeSet);

int mvPp2PmeVlanDefaultSet(struct mv_pp2x_hw *hw, u16 ethertype)
{
	mv_pp2x_write(hw, MVPP2_PME_DEF_VLAN_CFG_REG, (u32)ethertype);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeVlanDefaultSet);

int mvPp2PmeDsaDefaultSet(struct mv_pp2x_hw *hw, int idx, u16 ethertype)
{
	u32 regVal = (u32)ethertype;

	if ((idx < 0) || (idx > MVPP2_PME_MAX_DSA_ETH_TYPES)) {
		pr_err("%s: idx %d is out of range [0..%d]\n", __func__, idx, MVPP2_PME_MAX_DSA_ETH_TYPES);
		return MV_ERROR;
	}
	mv_pp2x_write(hw, MVPP2_PME_DEF_DSA_CFG_REG(idx), regVal);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeDsaDefaultSet);

int mvPp2PmeDsaSrcDevSet(struct mv_pp2x_hw *hw, u8 src)
{
	u32 regVal = 0;

	regVal &= ~MVPP2_PME_DSA_SRC_DEV_ALL_MASK;
	regVal |= MVPP2_PME_DSA_SRC_DEV_MASK(src);
	mv_pp2x_write(hw, MVPP2_PME_DEF_DSA_SRC_DEV_REG, regVal);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeDsaSrcDevSet);

int mvPp2PmeTtlZeroSet(struct mv_pp2x_hw *hw, int forward)
{
	u32 regVal = 0;

	regVal |= MVPP2_PME_TTL_ZERO_FRWD_MASK;
	mv_pp2x_write(hw, MVPP2_PME_TTL_ZERO_FRWD_REG, regVal);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeTtlZeroSet);

int mvPp2PmePppoeEtypeSet(struct mv_pp2x_hw *hw, u16 ethertype)
{
	mv_pp2x_write(hw, MVPP2_PME_PPPOE_ETYPE_REG, (u32)ethertype);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmePppoeEtypeSet);

int mvPp2PmePppoeLengthSet(struct mv_pp2x_hw *hw, u16 length)
{
	mv_pp2x_write(hw, MVPP2_PME_PPPOE_LEN_REG, (u32)length);
	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmePppoeLengthSet);

int mvPp2PmePppoeConfig(struct mv_pp2x_hw *hw, u8 version, u8 type, u8 code)
{
	u32 regVal = 0;

	regVal |= MVPP2_PME_PPPOE_VER_MASK(version);
	regVal |= MVPP2_PME_PPPOE_TYPE_MASK(type);
	regVal |= MVPP2_PME_PPPOE_CODE_MASK(code);

	mv_pp2x_write(hw, MVPP2_PME_PPPOE_DATA_REG, regVal);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmePppoeConfig);

int mvPp2PmePppoeProtoSet(struct mv_pp2x_hw *hw, int idx, u16 protocol)
{
	u32 regVal = 0;

	if ((idx < 0) || (idx > 1)) {
		pr_err("%s: idx %d is out of range [0..1]\n", __func__, idx);
		return MV_ERROR;
	}
	regVal = mv_pp2x_read(hw, MVPP2_PME_PPPOE_PROTO_REG);

	regVal &= ~MVPP2_PME_PPPOE_PROTO_ALL_MASK(idx);
	regVal |= MVPP2_PME_PPPOE_PROTO_MASK(idx, protocol);
	mv_pp2x_write(hw, MVPP2_PME_PPPOE_PROTO_REG, regVal);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmePppoeProtoSet);

int mvPp2PmeMaxConfig(struct mv_pp2x_hw *hw, int maxsize, int maxinstr, int errdrop)
{
	u32 regVal = 0;

	regVal |= MVPP2_PME_MAX_INSTR_NUM_MASK(maxinstr);
	regVal |= MVPP2_PME_MAX_HDR_SIZE_MASK(maxsize);
	if (errdrop)
		regVal |= MVPP2_PME_DROP_ON_ERR_MASK;

	mv_pp2x_write(hw, MVPP2_PME_CONFIG_REG, regVal);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeMaxConfig);

void mvPp2PmeHwRegs(struct mv_pp2x_hw *hw)
{
	int    i;
	u32 regVal;

	pr_info("\n[PME registers]\n");

	mv_pp2x_print_reg(hw, MVPP2_PME_TBL_IDX_REG, "MV_PP2_PME_TBL_IDX_REG");
	mv_pp2x_print_reg(hw, MVPP2_PME_TCONT_THRESH_REG, "MV_PP2_PME_TCONT_THRESH_REG");
	mv_pp2x_print_reg(hw, MVPP2_PME_MTU_REG, "MV_PP2_PME_MTU_REG");

	for (i = 0; i < MVPP2_PME_MAX_VLAN_ETH_TYPES; i++)
		mv_pp2x_print_reg2(hw, MVPP2_PME_VLAN_ETH_TYPE_REG(i), "MV_PP2_PME_VLAN_ETH_TYPE_REG", i);

	mv_pp2x_print_reg(hw, MVPP2_PME_DEF_VLAN_CFG_REG, "MV_PP2_PME_DEF_VLAN_CFG_REG");
	for (i = 0; i < MVPP2_PME_MAX_DSA_ETH_TYPES; i++)
		mv_pp2x_print_reg2(hw, MVPP2_PME_DEF_DSA_CFG_REG(i), "MV_PP2_PME_DEF_DSA_CFG_REG", i);

	mv_pp2x_print_reg(hw, MVPP2_PME_DEF_DSA_SRC_DEV_REG, "MV_PP2_PME_DEF_DSA_SRC_DEV_REG");
	mv_pp2x_print_reg(hw, MVPP2_PME_TTL_ZERO_FRWD_REG, "MV_PP2_PME_TTL_ZERO_FRWD_REG");
	mv_pp2x_print_reg(hw, MVPP2_PME_PPPOE_ETYPE_REG, "MV_PP2_PME_PPPOE_ETYPE_REG");
	mv_pp2x_print_reg(hw, MVPP2_PME_PPPOE_DATA_REG, "MV_PP2_PME_PPPOE_DATA_REG");
	mv_pp2x_print_reg(hw, MVPP2_PME_PPPOE_LEN_REG, "MV_PP2_PME_PPPOE_LEN_REG");
	mv_pp2x_print_reg(hw, MVPP2_PME_PPPOE_PROTO_REG, "MV_PP2_PME_PPPOE_PROTO_REG");
	mv_pp2x_print_reg(hw, MVPP2_PME_CONFIG_REG, "MV_PP2_PME_CONFIG_REG");
	mv_pp2x_print_reg(hw, MVPP2_PME_STATUS_1_REG, "MV_PP2_PME_STATUS_1_REG");

	pr_info("\nMV_PP2_PME_STATUS_2_REG[txp] registers that are not zero\n");
	for (i = 0; i < MVPP2_TOTAL_TXP_NUM; i++) {
		regVal = mv_pp2x_read(hw, MVPP2_PME_STATUS_2_REG(i));
		if (regVal != 0)
			pr_info("%-32s[%2d]: 0x%x = 0x%08x\n",
				"MV_PP2_PME_STATUS_2_REG", i, MVPP2_PME_STATUS_2_REG(i), regVal);
	}

	pr_info("\nMV_PP2_PME_STATUS_3_REG[txp] registers that are not zero\n");
	for (i = 0; i < MVPP2_TOTAL_TXP_NUM; i++) {
		regVal = mv_pp2x_read(hw, MVPP2_PME_STATUS_3_REG(i));
		if (regVal != 0)
			pr_info("%-32s[%2d]: 0x%x = 0x%08x\n",
				"MV_PP2_PME_STATUS_3_REG", i, MVPP2_PME_STATUS_3_REG(i), regVal);
	}
}
EXPORT_SYMBOL(mvPp2PmeHwRegs);

int mvPp2PmeHwInv(struct mv_pp2x_hw *hw, int idx)
{
	struct mv_pp2x_pme_entry entry;

	if ((idx < 0) || (idx >= MVPP2_PME_INSTR_SIZE)) {
		pr_err("%s: entry %d is out of range [0..%d]\n", __func__, idx, MVPP2_PME_INSTR_SIZE);
		return MV_ERROR;
	}
	mvPp2PmeSwClear(&entry);

	return mvPp2PmeHwWrite(hw, idx, &entry);
}
EXPORT_SYMBOL(mvPp2PmeHwInv);

int mvPp2PmeHwInvAll(struct mv_pp2x_hw *hw)
{
	int	idx;

	for (idx = 0; idx < MVPP2_PME_INSTR_SIZE; idx++)
		mvPp2PmeHwInv(hw, idx);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2PmeHwInvAll);

/* MC */
int mvPp2McHwWrite(struct mv_pp2x_hw *hw, struct mv_pp2x_mc_entry *mc, int index)
{
	mc->index = index;

	/* write index */
	mv_pp2x_write(hw, MVPP2_MC_INDEX_REG, mc->index);

	/* write data */
	mv_pp2x_write(hw, MVPP2_MC_DATA1_REG, mc->sram.regs.data1);
	mv_pp2x_write(hw, MVPP2_MC_DATA2_REG, mc->sram.regs.data2);
	mv_pp2x_write(hw, MVPP2_MC_DATA3_REG, mc->sram.regs.data3);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2McHwWrite);

/*-------------------------------------------------------------------------------*/
int mvPp2McHwRead(struct mv_pp2x_hw *hw, struct mv_pp2x_mc_entry *mc, int index)
{
	mc->index = index;

	/* write index */
	mv_pp2x_write(hw, MVPP2_MC_INDEX_REG, mc->index);

	/* read data */
	mc->sram.regs.data1 = mv_pp2x_read(hw, MVPP2_MC_DATA1_REG);
	mc->sram.regs.data2 = mv_pp2x_read(hw, MVPP2_MC_DATA2_REG);
	mc->sram.regs.data3 = mv_pp2x_read(hw, MVPP2_MC_DATA3_REG);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2McHwRead);

/*-------------------------------------------------------------------------------*/
int mvPp2McSwDump(struct mv_pp2x_mc_entry *mc)
{
	pr_info("INDEX:0x%2x\t", mc->index);

	pr_info("IPTR:0x%1x\t",
			(mc->sram.regs.data1 >> MVPP2_MC_DATA1_IPTR) & MVPP2_CLS2_ACT_HWF_ATTR_IPTR_MAX);

	pr_info("DPTR:0x%1x\t",
			(mc->sram.regs.data1 >> MVPP2_MC_DATA1_DPTR) & MVPP2_CLS2_ACT_HWF_ATTR_DPTR_MAX);

	if (mc->sram.regs.data2 &  MVPP2_MC_DATA2_GEM_ID_EN)
		pr_info("GPID:0x%3x\t",
			(mc->sram.regs.data2 >> MVPP2_MC_DATA2_GEM_ID) & MVPP2_CLS2_ACT_QOS_ATTR_GEM_MAX);
	else
		pr_info("GPID:INV\t");

	if (mc->sram.regs.data2 &  MVPP2_MC_DATA2_DSCP_EN)
		pr_info("DSCP:0x%1x\t",
			(mc->sram.regs.data2 >> MVPP2_MC_DATA2_DSCP) & MVPP2_CLS2_ACT_QOS_ATTR_DSCP_MAX);
	else
		pr_info("DSCP:INV\t");

	if (mc->sram.regs.data2 &  MVPP2_MC_DATA2_PRI_EN)
		pr_info("PRI:0x%1x \t", (mc->sram.regs.data2 >> MVPP2_MC_DATA2_PRI) & MVPP2_CLS2_ACT_QOS_ATTR_PRI_MAX);
	else
		pr_info("DSCP:INV\t");

	pr_info("QUEUE:0x%2x\t", (mc->sram.regs.data3 >> MVPP2_MC_DATA3_QUEUE) & 0xFF);

	if (mc->sram.regs.data3 & MVPP2_MC_DATA3_HWF_EN)
		pr_info("HW_FWD:ENABLE\t");

	else
		pr_info("HW_FWD:DISABLE\t");

	pr_info("NEXT:0x%2x\t", (mc->sram.regs.data3 >> MVPP2_MC_DATA3_NEXT) & MVPP2_MC_INDEX_MAX);

	pr_info("\n");

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2McSwDump);

/*-------------------------------------------------------------------------------*/
int mvPp2McHwDump(struct mv_pp2x_hw *hw)
{
	int index;
	struct mv_pp2x_mc_entry mc;

	for (index = 0; index < MVPP2_MC_TBL_SIZE; index++) {
		mc.index = index;
		mvPp2McHwRead(hw, &mc, index);
		mvPp2McSwDump(&mc);
		pr_info("-------------------------------------------------------------------------");
		pr_info("-------------------------------------------------------------------------\n");

	}

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2McHwDump);

/*-------------------------------------------------------------------------------*/
void mvPp2McSwClear(struct mv_pp2x_mc_entry *mc)
{
	memset(mc, 0, sizeof(struct mv_pp2x_mc_entry));
}
EXPORT_SYMBOL(mvPp2McSwClear);

/*-------------------------------------------------------------------------------*/
void mvPp2McHwClearAll(struct mv_pp2x_hw *hw)
{
	int index;
	struct mv_pp2x_mc_entry mc;

	mvPp2McSwClear(&mc);

	for (index = 0; index < MVPP2_MC_TBL_SIZE; index++)
		mvPp2McHwWrite(hw, &mc, index);

}
EXPORT_SYMBOL(mvPp2McHwClearAll);

/*-------------------------------------------------------------------------------*/

int mvPp2McSwModSet(struct mv_pp2x_mc_entry *mc, int data_ptr, int instr_offs)
{
	mc->sram.regs.data1 &= ~(MVPP2_CLS2_ACT_HWF_ATTR_DPTR_MAX << MVPP2_MC_DATA1_DPTR);
	mc->sram.regs.data1 |= (data_ptr << MVPP2_MC_DATA1_DPTR);

	mc->sram.regs.data1 &= ~(MVPP2_CLS2_ACT_HWF_ATTR_IPTR_MAX << MVPP2_MC_DATA1_IPTR);
	mc->sram.regs.data1 |= (instr_offs << MVPP2_MC_DATA1_IPTR);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2McSwModSet);

/*-------------------------------------------------------------------------------*/
int mvPp2McSwGpidSet(struct mv_pp2x_mc_entry *mc, int gpid, int enable)
{
	if (enable) {
		mc->sram.regs.data2 &= ~(MVPP2_CLS2_ACT_QOS_ATTR_GEM_MAX << MVPP2_MC_DATA2_GEM_ID);
		mc->sram.regs.data2 |= (gpid << MVPP2_MC_DATA2_GEM_ID);
		mc->sram.regs.data2 |= MVPP2_MC_DATA2_GEM_ID_EN;

	} else
		mc->sram.regs.data2 &= ~MVPP2_MC_DATA2_GEM_ID_EN;

	return MV_OK;

}
EXPORT_SYMBOL(mvPp2McSwGpidSet);

/*-------------------------------------------------------------------------------*/
int mvPp2McSwDscpSet(struct mv_pp2x_mc_entry *mc, int dscp, int enable)
{
	if (enable) {
		mc->sram.regs.data2 &= ~(MVPP2_CLS2_ACT_QOS_ATTR_DSCP_MAX << MVPP2_MC_DATA2_DSCP);
		mc->sram.regs.data2 |= (dscp << MVPP2_MC_DATA2_DSCP);
		mc->sram.regs.data2 |= MVPP2_MC_DATA2_DSCP_EN;

	} else
		mc->sram.regs.data2 &= MVPP2_MC_DATA2_DSCP_EN;

	return MV_OK;

}
EXPORT_SYMBOL(mvPp2McSwDscpSet);

/*-------------------------------------------------------------------------------*/
int mvPp2McSwPrioSet(struct mv_pp2x_mc_entry *mc, int prio, int enable)
{
	if (enable) {
		mc->sram.regs.data2 &= ~(MVPP2_CLS2_ACT_QOS_ATTR_PRI_MAX << MVPP2_MC_DATA2_PRI);
		mc->sram.regs.data2 |= (prio << MVPP2_MC_DATA2_PRI);
		mc->sram.regs.data2 |= MVPP2_MC_DATA2_PRI_EN;

	} else
		mc->sram.regs.data2 &= ~MVPP2_MC_DATA2_PRI_EN;

	return MV_OK;

}
EXPORT_SYMBOL(mvPp2McSwPrioSet);

/*-------------------------------------------------------------------------------*/
int mvPp2McSwQueueSet(struct mv_pp2x_mc_entry *mc, int q)
{
	mc->sram.regs.data3 &= ~(0xFF << MVPP2_MC_DATA3_QUEUE);
	mc->sram.regs.data3 |= (q << MVPP2_MC_DATA3_QUEUE);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2McSwQueueSet);

/*-------------------------------------------------------------------------------*/
int mvPp2McSwForwardEn(struct mv_pp2x_mc_entry *mc, int enable)
{
	if (enable)
		mc->sram.regs.data3 |= MVPP2_MC_DATA3_HWF_EN;
	else
		mc->sram.regs.data3 &= ~MVPP2_MC_DATA3_HWF_EN;

	return MV_OK;

}
EXPORT_SYMBOL(mvPp2McSwForwardEn);

/*-------------------------------------------------------------------------------*/

int mvPp2McSwNext(struct mv_pp2x_mc_entry *mc, int next)
{
	mc->sram.regs.data3 &= ~(MVPP2_MC_INDEX_MAX << MVPP2_MC_DATA3_NEXT);
	mc->sram.regs.data3 |= (next << MVPP2_MC_DATA3_NEXT);

	return MV_OK;

}
EXPORT_SYMBOL(mvPp2McSwNext);

/* RSS */
int mvpp2_rss_tbl_rxq_bind(struct mv_pp2x_hw *hw, struct mv_pp22_rss_entry *rss_entry, int rxq_num, int tbl_ptr)
{
	rss_entry->sel = MVPP22_RSS_ACCESS_POINTER;
	rss_entry->u.pointer.rxq_idx = rxq_num;
	rss_entry->u.pointer.rss_tbl_ptr = tbl_ptr;

	mv_pp22_rss_tbl_entry_set(hw, rss_entry);

	return MV_OK;
}
EXPORT_SYMBOL(mvpp2_rss_tbl_rxq_bind);

int mvpp2_rss_tbl_entry_set(struct mv_pp2x_hw *hw, struct mv_pp22_rss_entry *rss_entry,
			       u8 tbl_id, u8 tbl_line, u8 rxq, u8 width)
{
	rss_entry->sel = MVPP22_RSS_ACCESS_TBL;
	rss_entry->u.entry.tbl_id = tbl_id;
	rss_entry->u.entry.tbl_line = tbl_line;
	rss_entry->u.entry.rxq = rxq;
	rss_entry->u.entry.width = width;

	mv_pp22_rss_tbl_entry_set(hw, rss_entry);

	return MV_OK;
}
EXPORT_SYMBOL(mvpp2_rss_tbl_entry_set);

int mvpp2_rss_hash_sel_set(struct mv_pp2x_hw *hw, enum mv_pp2_rss_hash_select sel)
{
	mv_pp2x_write(hw, MVPP22_RSS_HASH_SEL_REG, sel);

	return MV_OK;
}
EXPORT_SYMBOL(mvpp2_rss_hash_sel_set);

