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
#ifndef _MV_PP2X_SOC_TEST_H_
#define _MV_PP2X_SOC_TEST_H_

/* C3 test Macros */
#define HEK_EXT_FMT				"%8.8x %8.8x %8.8x | %8.8x %8.8x %8.8x %8.8x %8.8x %8.8x"
#define HEK_EXT_VAL(p)				p[8], p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]
#define HEK_FMT					"%8.8x %8.8x %8.8x"
#define HEK_VAL(p)				p[8], p[7], p[6]

/* C4 test Macros */
/* Field 0- 3 */
#define FLD_FMT					"%4.4x"
#define FLD_VAL(field, p)			((p[field/2] >> (16 * (field % 2))) & 0xFFFF)
/* field 4 */
#define FLD4_FMT				"%8.8x %8.8x %8.8x %8.8x"
#define FLD4_VAL(p)				p[2], p[3], p[4], p[5]
/* field 5 */
#define FLD5_FMT				"%4.4x %8.8x"
#define FLD5_VAL(p)				p[6] & 0xFFFF, p[7]
#define GET_FIELD_ATRR(field)			((field) / 3)

/* PME test Macros */
#define MVPP2_PME_IS_VALID(pme)	\
		((((pme)->word & MVPP2_PME_CMD_ALL_MASK) >> MVPP2_PME_CMD_OFFS) != MVPP2_PME_CMD_NONE)

#define MVPP2_PME_CTRL_GET(pme) \
		(u16)(((pme)->word & MVPP2_PME_CTRL_MASK) >> MVPP2_PME_CTRL_OFFS)

#define MVPP2_PME_CMD_GET(pme)           \
		(((pme)->word & MVPP2_PME_CMD_ALL_MASK) >> MVPP2_PME_CMD_OFFS)

#define MVPP2_PME_DATA_GET(pme)           \
		(u16)(((pme)->word & MVPP2_PME_DATA_MASK) >> MVPP2_PME_DATA_OFFS)

#define MVPP2_PME_CMD_SET(pme, cmd)			\
	do {						\
		(pme)->word &= ~MVPP2_PME_CMD_ALL_MASK;	\
		(pme)->word |= MVPP2_PME_CMD_MASK(cmd);	\
	} while (0)

#define MVPP2_PME_DATA_SET(pme, data)				\
	do {							\
		(pme)->word &= ~MVPP2_PME_DATA_MASK;		\
		(pme)->word |= ((data) << MVPP2_PME_DATA_OFFS);	\
	} while (0)

/* C3 Function protype */
int mvPp2ClsC3HwDump(struct mv_pp2x_hw *hw);
int mvPp2ClsC3HwMissDump(struct mv_pp2x_hw *hw);
int mvPp2ClsC3HwExtDump(struct mv_pp2x_hw *hw);
int mvPp2ClsC3SwDump(struct mv_pp2x_cls_c3_entry *c3);
int mvPp2ClsC3ScanResDump(struct mv_pp2x_hw *hw);
int mvPp2ClsC3ScanRegs(struct mv_pp2x_hw *hw);
int mvPp2ClsC3HwQuery(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c3_entry *c3, unsigned char *occupied_bmp, int index[]);
int mvPp2ClsC3HitCntrsMissRead(struct mv_pp2x_hw *hw, int lkp_type, u32 *cntr);
int mvPp2ClsC3HitCntrsReadAll(struct mv_pp2x_hw *hw);
int mv_pp2x_cls_c3_hw_read(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c3_entry *c3, int index);
int mvPp2ClsC3HwQueryAdd(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c3_entry *c3, int max_search_depth,
			 struct mv_pp2x_cls_c3_hash_pair *hash_pair_arr);
int mvPp2ClsC3HwAdd(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c3_entry *c3, int index, int ext_index);
int mvPp2ClsC3HwMissAdd(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c3_entry *c3, int lkp_type);
int mvPp2ClsC3HwDel(struct mv_pp2x_hw *hw, int index);
int mvPp2ClsC3HwDelAll(struct mv_pp2x_hw *hw);
void mvPp2ClsC3SwClear(struct mv_pp2x_cls_c3_entry *c3);
int mvPp2ClsC3SwL4infoSet(struct mv_pp2x_cls_c3_entry *c3, int l4info);
int mvPp2ClsC3SwLkpTypeSet(struct mv_pp2x_cls_c3_entry *c3, int lkp_type);
int mvPp2ClsC3SwPortIDSet(struct mv_pp2x_cls_c3_entry *c3, int type, int portid);
int mvPp2ClsC3SwHekSizeSet(struct mv_pp2x_cls_c3_entry *c3, int hekSize);
int mvPp2ClsC3SwHekByteSet(struct mv_pp2x_cls_c3_entry *c3, unsigned int offs, unsigned char byte);
int mvPp2ClsC3SwHekWordSet(struct mv_pp2x_cls_c3_entry *c3, unsigned int offs, unsigned int word);
int mvPp2ClsC3ColorSet(struct mv_pp2x_cls_c3_entry *c3, int cmd);
int mvPp2ClsC3QueueHighSet(struct mv_pp2x_cls_c3_entry *c3, int cmd, int queue);
int mvPp2ClsC3QueueLowSet(struct mv_pp2x_cls_c3_entry *c3, int cmd, int queue);
int mvPp2ClsC3QueueSet(struct mv_pp2x_cls_c3_entry *c3, int cmd, int queue);
int mvPp2ClsC3ForwardSet(struct mv_pp2x_cls_c3_entry *c3, int cmd);
int mvPp2ClsC3PolicerSet(struct mv_pp2x_cls_c3_entry *c3, int cmd, int policerId, int bank);
int mvPp2ClsC3FlowIdEn(struct mv_pp2x_cls_c3_entry *c3, int flowid_en);
int mv_pp2x_cls_c3_rss_set(struct mv_pp2x_cls_c3_entry *c3, int cmd, int rss_en);
int mvPp2ClsC3ModSet(struct mv_pp2x_cls_c3_entry *c3, int data_ptr, int instr_offs, int l4_csum);
int mvPp2ClsC3MtuSet(struct mv_pp2x_cls_c3_entry *c3, int mtu_inx);
int mvPp2ClsC3DupSet(struct mv_pp2x_cls_c3_entry *c3, int dupid, int count);
int mvPp2ClsC3SeqSet(struct mv_pp2x_cls_c3_entry *c3, int id, int bits_offs, int bits);
int mvPp2ClsC3ScanResRead(struct mv_pp2x_hw *hw, int index, int *addr, int *cnt);
int mvPp2ClsC3HitCntrsRead(struct mv_pp2x_hw *hw, int index, u32 *cntr);
int mvPp2ClsC3HitCntrsClearAll(struct mv_pp2x_hw *hw);
int mvPp2ClsC3HitCntrsClear(struct mv_pp2x_hw *hw, int lkpType);
int mvPp2ClsC3ScanStart(struct mv_pp2x_hw *hw);
int mvPp2ClsC3ScanThreshSet(struct mv_pp2x_hw *hw, int mode, int thresh);
int mvPp2ClsC3ScanClearBeforeEnSet(struct mv_pp2x_hw *hw, int en);
int mvPp2ClsC3ScanStartIndexSet(struct mv_pp2x_hw *hw, int idx);
int mvPp2ClsC3ScanDelaySet(struct mv_pp2x_hw *hw, int time);
int mvPp2ClsC3ScanLkpTypeSet(struct mv_pp2x_hw *hw, int type);
int mvPp2ClsC3Init(struct mv_pp2x_hw *hw);
void mvPp2ClsC3HwInitCtrSet(int cntVal);

/* C4 Function protype */
int mvPp2ClsC4SwDump(struct mv_pp2x_cls_c4_entry *C4);
int mvPp2ClsC4RegsDump(struct mv_pp2x_hw *hw);
int mvPp2ClsC4HwDumpAll(struct mv_pp2x_hw *hw);
int mvPp2ClsC4HwHitsDump(struct mv_pp2x_hw *hw);
int mvPp2ClsC4HwPortToRulesSet(struct mv_pp2x_hw *hw, int port, int set, int rules);
int mvPp2ClsC4HwUniToRulesSet(struct mv_pp2x_hw *hw, int uni, int set, int rules);
int mvPp2ClsC4HwRead(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c4_entry *C4, int rule, int set);
int mvPp2ClsC4HwWrite(struct mv_pp2x_hw *hw, struct mv_pp2x_cls_c4_entry *C4, int rule, int set);
void mvPp2ClsC4SwClear(struct mv_pp2x_cls_c4_entry *C4);
void mvPp2ClsC4HwClearAll(struct mv_pp2x_hw *hw);
int mvPp2ClsC4FieldsShortSet(struct mv_pp2x_cls_c4_entry *C4, int field, unsigned int offs, unsigned short data);
int mvPp2ClsC4FieldsParamsSet(struct mv_pp2x_cls_c4_entry *C4, int field, unsigned int id, unsigned int op);
int mvPp2ClsC4SwVlanSet(struct mv_pp2x_cls_c4_entry *C4, int vlan);
int mvPp2ClsC4SwPppoeSet(struct mv_pp2x_cls_c4_entry *C4, int pppoe);
int mvPp2ClsC4SwMacMeSet(struct mv_pp2x_cls_c4_entry *C4, int mac);
int mvPp2ClsC4SwL4InfoSet(struct mv_pp2x_cls_c4_entry *C4, int info);
int mvPp2ClsC4SwL3InfoSet(struct mv_pp2x_cls_c4_entry *C4, int info);
int mvPp2ClsC4ColorSet(struct mv_pp2x_cls_c4_entry *C4, int cmd);
int mvPp2ClsC4PrioSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int prio);
int mvPp2ClsC4DscpSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int dscp);
int mvPp2ClsC4GpidSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int gid);
int mvPp2ClsC4QueueHighSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int queue);
int mvPp2ClsC4QueueLowSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int queue);
int mvPp2ClsC4ForwardSet(struct mv_pp2x_cls_c4_entry *C4, int cmd);
int mvPp2ClsC4QueueSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int queue);
int mvPp2ClsC4PolicerSet(struct mv_pp2x_cls_c4_entry *C4, int cmd, int policerId, int bank);

/* PME Function protype */
int mvPp2PmeSwDump(struct mv_pp2x_pme_entry *pEntry);
int mvPp2PmeHwDump(struct mv_pp2x_hw *hw, int mode);
int mvPp2PmeSwClear(struct mv_pp2x_pme_entry *pEntry);
int mvPp2PmeSwWordSet(struct mv_pp2x_pme_entry *pEntry, u32 word);
int mvPp2PmeSwCmdSet(struct mv_pp2x_pme_entry *pEntry, enum mv_pp2x_pme_instr cmd);
int mvPp2PmeSwCmdTypeSet(struct mv_pp2x_pme_entry *pEntry, int type);
int mvPp2PmeSwCmdLastSet(struct mv_pp2x_pme_entry *pEntry, int last);
int mvPp2PmeSwCmdFlagsSet(struct mv_pp2x_pme_entry *pEntry, int last, int ipv4, int l4);
int mvPp2PmeSwCmdDataSet(struct mv_pp2x_pme_entry *pEntry, u16 data);
int mvPp2PmeHwDataTblWrite(struct mv_pp2x_hw *hw, int tbl, int idx, u16 data);
int mvPp2PmeHwDataTblRead(struct mv_pp2x_hw *hw, int tbl, int idx, u16 *data);
int mvPp2PmeHwDataTblDump(struct mv_pp2x_hw *hw, int tbl);
int mvPp2PmeHwDataTblClear(struct mv_pp2x_hw *hw, int tbl);
int mvPp2PmeVlanEtherTypeSet(struct mv_pp2x_hw *hw, int idx, u16 ethertype);
int mvPp2PmeVlanDefaultSet(struct mv_pp2x_hw *hw, u16 ethertype);
int mvPp2PmeDsaDefaultSet(struct mv_pp2x_hw *hw, int idx, u16 ethertype);
int mvPp2PmeDsaSrcDevSet(struct mv_pp2x_hw *hw, u8 src);
int mvPp2PmeTtlZeroSet(struct mv_pp2x_hw *hw, int forward);
int mvPp2PmePppoeEtypeSet(struct mv_pp2x_hw *hw, u16 ethertype);
int mvPp2PmePppoeLengthSet(struct mv_pp2x_hw *hw, u16 length);
int mvPp2PmePppoeConfig(struct mv_pp2x_hw *hw, u8 version, u8 type, u8 code);
int mvPp2PmePppoeProtoSet(struct mv_pp2x_hw *hw, int idx, u16 protocol);
int mvPp2PmeMaxConfig(struct mv_pp2x_hw *hw, int maxsize, int maxinstr, int errdrop);
void mvPp2PmeHwRegs(struct mv_pp2x_hw *hw);
int mvPp2PmeHwInv(struct mv_pp2x_hw *hw, int idx);
int mvPp2PmeHwInvAll(struct mv_pp2x_hw *hw);
int mvPp2PmeHwWrite(struct mv_pp2x_hw *hw, int idx, struct mv_pp2x_pme_entry *pEntry);
int mvPp2PmeHwRead(struct mv_pp2x_hw *hw, int idx, struct mv_pp2x_pme_entry *pEntry);

/* MC Function protype */
int mvPp2McHwWrite(struct mv_pp2x_hw *hw, struct mv_pp2x_mc_entry *mc, int index);
int mvPp2McHwRead(struct mv_pp2x_hw *hw, struct mv_pp2x_mc_entry *mc, int index);
int mvPp2McSwDump(struct mv_pp2x_mc_entry *mc);
int mvPp2McHwDump(struct mv_pp2x_hw *hw);
void mvPp2McSwClear(struct mv_pp2x_mc_entry *mc);
void mvPp2McHwClearAll(struct mv_pp2x_hw *hw);
int mvPp2McSwModSet(struct mv_pp2x_mc_entry *mc, int data_ptr, int instr_offs);
int mvPp2McSwGpidSet(struct mv_pp2x_mc_entry *mc, int gpid, int enable);
int mvPp2McSwDscpSet(struct mv_pp2x_mc_entry *mc, int dscp, int enable);
int mvPp2McSwPrioSet(struct mv_pp2x_mc_entry *mc, int prio, int enable);
int mvPp2McSwQueueSet(struct mv_pp2x_mc_entry *mc, int q);
int mvPp2McSwForwardEn(struct mv_pp2x_mc_entry *mc, int enable);
int mvPp2McSwNext(struct mv_pp2x_mc_entry *mc, int next);

/* RSS Function protype */
int mvpp2_rss_tbl_rxq_bind(struct mv_pp2x_hw *hw, struct mv_pp22_rss_entry *rss_entry, int rxq_num, int tbl_ptr);
int mvpp2_rss_tbl_entry_set(struct mv_pp2x_hw *hw, struct mv_pp22_rss_entry *rss_entry,
			       u8 tbl_id, u8 tbl_line, u8 rxq, u8 width);
int mvpp2_rss_hash_sel_set(struct mv_pp2x_hw *hw, enum mv_pp2_rss_hash_select sel);

#endif /* _MV_PP2X_SOC_TEST_H_ */
