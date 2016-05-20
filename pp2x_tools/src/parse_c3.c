/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
licensing terms.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <errno.h>

#include "ezxml.h"
#include "common.h"
#include "DataDictionary.h"
#include "PncGlobals.h"
#include "xml_params.h"
#include "PacketParse.h"
#include "SubfieldParse.h"
#include "ParseUtils.h"

#define	INS_SEQ_INFO_CNT	8

enum xml_entry_data_C3{
	C3_TCAM_INDEX_E,
	C3_NAME_E,
	C3_L4INFO_E,
	C3_LU_TYPE_E,
	C3_PORT_IDTYPE_E,
	C3_PORT_ID_E,
	C3_HEK35_0_E,
	C3_MISS_E,
	C3_MULTIHASH_ENTRY_MODEL_E,
	C3_MULTIHASH_TABLE_ADDRESS_E,
	C3_EXTENSION_TABLE_ADDRESS_E,
	C3_INITIAL_HIT_COUNTER_VALUE_E,
	C3_COLOR_ACTION_E,
	C3_QUEUE_LOW_ACTION_E,
	C3_QUEUE_LOW_VALUE_E,
	C3_QUEUE_HIGH_ACTION_E,
	C3_QUEUE_HIGH_VALUE_E,
	C3_FORWARDING_E,
	C3_POLICER_SELECT_E,
	C3_POLICER_ID_E,
	C3_FLOW_ID_ENABLE_E,
	C3_FLOW_ID_E,
	C3_RSS_ACTION_E,
	C3_RSS_VALUE_E,
	C3_HWFM_DPTR_E,
	C3_HWFM_IPTR_E,
	C3_HWF_L4CHK_ENB_E,
	C3_HW_DUPLICATION_COUNT_E,
	C3_MTU_IDX_E,
	C3_INS_SEQ_INFO1_E,
	C3_INS_SEQ_INFO2_E,
	C3_INS_SEQ_INFO3_E,
	C3_INS_SEQ_INFO4_E,
	C3_INS_SEQ_INFO5_E,
	C3_INS_SEQ_INFO6_E,
	C3_INS_SEQ_INFO7_E,
	C3_INS_SEQ_INFO8_E,
	C3_MAX_DATA
};

/******************************************************************************
 *
 * Function   : get_cls_ins_seq_info_sz
 *
 * Description: return CLS sheet instruction sequence info
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
static int get_cls_ins_seq_info_sz(char 		*xmlFile,
				unsigned int		num_of_info,
				unsigned int		*ins_seq_info_sz)
{
	ezxml_t		xmlHead = NULL;
	ezxml_t		xml_cls, xmlEntry, xmlTmpEntry;
	char 		addtnl_info_str[512], *addtnl_info_ptr;
	PncEntry_S	pnCEntry;
	unsigned int	skip;
	DdEntry		*db_temp = NULL;
	
	memset(ins_seq_info_sz, 0, sizeof(ins_seq_info_sz)*num_of_info);
	
	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
        	return 1;
	}

	/* get Classifier sheet */
	xml_cls = ezxml_get(xmlHead, WORKSHEET_CLS_CONFIG, -1);
	if (xml_cls == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_CLS_CONFIG);
		ezxml_free(xmlHead);
		return 1;
    	}

	xmlEntry = ezxml_child(xml_cls, TABLE_ENTRY);
	if (xmlEntry == NULL){
		ERR_PR("Failed to get %s\n", TABLE_ENTRY);
		ezxml_free(xmlHead);
		return 1;
    	}

	/* search for additional fields column */
	xmlTmpEntry = ezxml_child(xmlEntry, CLS_ADDITIONAL_FILEDS);
	if (NULL == xmlTmpEntry) {
		ERR_PR("Failed to get %s\n", CLS_ADDITIONAL_FILEDS);
		ezxml_free(xmlHead);
		return 1;
	}
	
	strcpy(addtnl_info_str, ezxml_txt(xmlTmpEntry));
	
	skip = 0;
	addtnl_info_ptr = addtnl_info_str;
	while (strlen(addtnl_info_ptr)>0 && dealWithPacket((unsigned char*)addtnl_info_ptr, &pnCEntry, &skip) == true) {
		unsigned int 	field_id;
		char		*field_name;
		char		*subfield_name;
		char		*subfield_value;
		unsigned int 	indx = 0;
		RtSubFldEntry_S	*pRtSubFldEntry;

		while (getRtSubFld(&field_name, &subfield_name, &subfield_value, &field_id, &pRtSubFldEntry, indx)) {
			/* find field name in dictionary */
			db_temp = findMatchingEntry(field_name);
			if (db_temp == NULL) {
				ERR_PR("Can not find %s in ppv2tool dictionary\n", field_name);
				return 1;
			}

			if (!strncmp(field_name, "ins", 3)) { /* insXIdSz */
				if (atoi(db_temp->value) >= num_of_info) {
					ERR_PR("instruction sequence info too big (%d => %d)\n",
						atoi(db_temp->value), num_of_info);
					ezxml_free(xmlHead);
					return 1;
				}
				ins_seq_info_sz[atoi(db_temp->value)] = pRtSubFldEntry->parsedIntValue;
			}
			indx++;
		}
		addtnl_info_ptr = &addtnl_info_ptr[skip];
		DEBUG_PR(DEB_OTHER, "tmp <%s>\n", addtnl_info_ptr);
		skip = 0;
	}
	ezxml_free(xmlHead);

	return 0;
}

static int build_c3_miss_get(xml_entry_data *c3_data, bool *miss_rule)
{
	DdEntry		*miss = NULL;
	
	*miss_rule = false;
	
	if (NULL != c3_data[C3_MISS_E].xmlEntry) {
		miss = findMatchingEntry((char *)ezxml_txt(c3_data[C3_MISS_E].xmlEntry));
		if (miss == NULL) {
			ERR_PR("%s is not valid value\n", MISS);
			return 1;
		}
		*miss_rule = (0 == atoi(miss->value)) ? false : true;
	}

	return 0;
}


static int build_c3_common_sysfs(xml_entry_data *c3_data, bool miss_rule)
{
	char 		sysfs_buf[512];
	unsigned int	tcam_idx;
	unsigned int	l4_info;
	unsigned int	InitialHitCounterValue;
	DdEntry 	*lu_type = NULL;
	DdEntry 	*portid_type = NULL;
	unsigned int	portid;
	
	tcam_idx = atoi(ezxml_txt(c3_data[C3_TCAM_INDEX_E].xmlEntry));
	l4_info = atoi(ezxml_txt(c3_data[C3_L4INFO_E].xmlEntry));
	InitialHitCounterValue = atoi(ezxml_txt(c3_data[C3_INITIAL_HIT_COUNTER_VALUE_E].xmlEntry));

	sprintf(sysfs_buf, "##########################  C3: TCAM:%d [%s]  ##########################\n",
		tcam_idx, ezxml_txt(c3_data[C3_NAME_E].xmlEntry));
	handle_sysfs_command(sysfs_buf, true);

	if (c3_data[C3_LU_TYPE_E].xmlEntry == NULL) {
		ERR_PR("%s is missing for tcam index %d\n", LU_TYPE, tcam_idx);
		return 1;
	}

	lu_type = findMatchingEntry(ezxml_txt(c3_data[C3_LU_TYPE_E].xmlEntry));
	if (lu_type == NULL) {
		ERR_PR("%s is missing for tcam index %d\n", LU_TYPE, tcam_idx);
		return 1;
	}

	/* sw clear */
	sprintf(sysfs_buf, "echo 1                 > %s/sw_clear\n", C3_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	
	/* miss rule, no need to write any register yet */
	if (miss_rule == true)
		return 0;

	if (c3_data[C3_PORT_IDTYPE_E].xmlEntry == NULL) {
		ERR_PR("%s is missing for tcam index %d\n", PORT_IDTYPE, tcam_idx);
		return 1;
	}
	portid_type = findMatchingEntry(ezxml_txt(c3_data[C3_PORT_IDTYPE_E].xmlEntry));
	if (portid_type == NULL) {
		ERR_PR("%s is missing for tcam index %d\n", LU_TYPE, tcam_idx);
		return 1;
	}

	DEBUG_PR(DEB_SYSFS, "tcam %d %s=%s %s=%s\n",tcam_idx,
		LU_TYPE, ezxml_txt(c3_data[C3_LU_TYPE_E].xmlEntry),
		PORT_IDTYPE, ezxml_txt(c3_data[C3_PORT_IDTYPE_E].xmlEntry));

	portid = atoi(ezxml_txt(c3_data[C3_PORT_ID_E].xmlEntry));

	/* set InitialHitCounterValue */
	sprintf(sysfs_buf, "echo 0x%.2x              > %s/sw_init_cnt\n",
		InitialHitCounterValue, C3_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	/* set l4_info */
	sprintf(sysfs_buf, "echo 0x%.2x              > %s/key_sw_l4\n",
		l4_info, C3_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	/* set lookup type */
	sprintf(sysfs_buf, "echo 0x%.2x              > %s/key_sw_lkp_type\n",
		atoi(lu_type->value), C3_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	/* set portId type */
	sprintf(sysfs_buf, "echo 0x%.2x 0x%.2x         > %s/key_sw_port\n",
		portid, atoi(portid_type->value), C3_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	return 0;
}

static int build_c3_bank_sysfs(xml_entry_data *c3_data, bool miss_rule)
{
	char 		sysfs_buf[512];
	unsigned int	MultihashTblAddr;
	unsigned int	ExtensionTblAddr;
	DdEntry 	*MultihashEntryModel = NULL;
	char		*model_name;
	
	if (c3_data[C3_MULTIHASH_ENTRY_MODEL_E].xmlEntry == NULL) {
		ERR_PR("%s is missing\n", MULTIHASH_ENTRY_MODEL);
		return 1;
	}

	MultihashEntryModel = findMatchingEntry(ezxml_txt(c3_data[C3_MULTIHASH_ENTRY_MODEL_E].xmlEntry));
	if (MultihashEntryModel == NULL) {
		ERR_PR("%s is not valid value\n", MULTIHASH_ENTRY_MODEL);
		return 1;
	}
	
	if (true == miss_rule){
		DdEntry	*lu_type = NULL;
		
		if (c3_data[C3_LU_TYPE_E].xmlEntry == NULL) {
			ERR_PR("%s is missing\n", LU_TYPE);
			return 1;
		}

		lu_type = findMatchingEntry(ezxml_txt(c3_data[C3_LU_TYPE_E].xmlEntry));
		if (lu_type == NULL) {
			ERR_PR("%s is missing\n", LU_TYPE);
			return 1;
		}
		sprintf(sysfs_buf, "echo 0x%.2x              > %s/hw_ms_add\n",
			atoi(lu_type->value),
			C3_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* miss action setting, no need to add entry */
		return 0;
	}

	/* supported models: Search, recursSearch, InterHash, HashSel, AutoGen*/	
	model_name = ezxml_txt(c3_data[C3_MULTIHASH_ENTRY_MODEL_E].xmlEntry);
	if (strcmp("recursSearch", model_name) == 0){
		sprintf(sysfs_buf, "echo 3                 > %s/hw_query_add\n", C3_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	} else if (strcmp("HashSel", model_name) == 0){
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(c3_data[C3_MULTIHASH_TABLE_ADDRESS_E].xmlEntry),
			&MultihashTblAddr) == false) {
			ERR_PR("%s is not valid value\n", MULTIHASH_TABLE_ADDRESS);
			return 1;
		}

		if (c3_data[C3_EXTENSION_TABLE_ADDRESS_E].xmlEntry == NULL)
			ExtensionTblAddr = 0;
	 	else if (parseSimpleNumericValue((unsigned char*)ezxml_txt(c3_data[C3_EXTENSION_TABLE_ADDRESS_E].xmlEntry),
			&ExtensionTblAddr) == false) {
			ERR_PR("%s is not valid value\n", EXTENSION_TABLE_ADDRESS);
			return 1;
		}
		
		sprintf(sysfs_buf, "echo 0x%.3x 0x%.3x       > %s/hw_add\n",
			MultihashTblAddr, ExtensionTblAddr, C3_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	} else {
		ERR_PR("unsupported MultiHashModel [%s]\n", model_name);
		return 1;
	}

	DEBUG_PR(DEB_SYSFS, "MultihashTblAddr: [%d], ExtensionTblAddr: [%d], MultihashEntryModel: [%s]\n",
			MultihashTblAddr, ExtensionTblAddr,
			ezxml_txt(c3_data[C3_MULTIHASH_ENTRY_MODEL_E].xmlEntry));
       return 0;
}

/******************************************************************************
 *
 * Function   : build_c3_action_sysfs
 *
 * Description:
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
static int build_c3_action_sysfs(xml_entry_data *c3_data,
				unsigned int	num_of_info,
				unsigned int	*ins_seq_info_sz)
{
	char sysfs_buf[512];
	unsigned int tcam_idx;

	tcam_idx = atoi(ezxml_txt(c3_data[C3_TCAM_INDEX_E].xmlEntry));

	{
		DdEntry *color_action = findMatchingEntry((char *)ezxml_txt(c3_data[C3_COLOR_ACTION_E].xmlEntry));
		if (c3_data[C3_COLOR_ACTION_E].xmlEntry == NULL)
		{
			ERR_PR("%s is missing for tcam index %d\n", COLOR_ACTION, tcam_idx);
			return 1;
		}

		sprintf(sysfs_buf, "echo 0x%.2x              > %s/act_sw_color\n",
			atoi(color_action->value), C3_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *q_low_action = findMatchingEntry((char *)ezxml_txt(c3_data[C3_QUEUE_LOW_ACTION_E].xmlEntry));
		DdEntry *q_high_action = findMatchingEntry((char *)ezxml_txt(c3_data[C3_QUEUE_HIGH_ACTION_E].xmlEntry));
		unsigned int queue_h = atoi(ezxml_txt(c3_data[C3_QUEUE_HIGH_VALUE_E].xmlEntry));
		unsigned int queue_l = atoi(ezxml_txt(c3_data[C3_QUEUE_LOW_VALUE_E].xmlEntry));

		if (c3_data[C3_QUEUE_LOW_ACTION_E].xmlEntry == NULL)
		{
			ERR_PR("%s is missing for tcam index %d\n", QUEUE_LOW_ACTION, tcam_idx);
			return 1;
		}
		if (c3_data[C3_QUEUE_HIGH_ACTION_E].xmlEntry == NULL)
		{
			ERR_PR("%s is missing for tcam index %d\n", QUEUE_HIGH_ACTION, tcam_idx);
			return 1;
		}
		sprintf(sysfs_buf, "echo 0x%.2x 0x%.2x         > %s/act_sw_qh\n",
			atoi(q_high_action->value),queue_h, C3_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
		sprintf(sysfs_buf, "echo 0x%.2x 0x%.2x         > %s/act_sw_ql\n",
			atoi(q_low_action->value),queue_l, C3_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *frw_action = findMatchingEntry((char *)ezxml_txt(c3_data[C3_FORWARDING_E].xmlEntry));
		if (c3_data[C3_FORWARDING_E].xmlEntry == NULL)
		{
			ERR_PR("%s is missing for tcam index %d\n", FORWARDING, tcam_idx);
			return 1;
		}

		sprintf(sysfs_buf, "echo 0x%.2x              > %s/act_sw_fwd\n",
			atoi(frw_action->value), C3_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *policer_action = findMatchingEntry((char *)ezxml_txt(c3_data[C3_POLICER_SELECT_E].xmlEntry));
		unsigned int policer_id = atoi(ezxml_txt(c3_data[C3_POLICER_ID_E].xmlEntry));
		if (c3_data[C3_POLICER_SELECT_E].xmlEntry == NULL)
		{
			ERR_PR("%s is missing for tcam index %d\n", POLICER_SELECT, tcam_idx);
			return 1;
		}
		sprintf(sysfs_buf, "echo 0x%.2x 0x%.2x 0x%.1x     > %s/act_sw_pol\n",
			atoi(policer_action->value),
			policer_id & C3_POLICER_ID_MAX,
			policer_id >> C3_POLICER_ID_BITS,
			C3_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *rss_action = findMatchingEntry((char *)ezxml_txt(c3_data[C3_RSS_ACTION_E].xmlEntry));
		DdEntry *rss_en = findMatchingEntry((char *)ezxml_txt(c3_data[C3_RSS_VALUE_E].xmlEntry));

		if (c3_data[C3_RSS_ACTION_E].xmlEntry == NULL)	{
			ERR_PR("%s is missing for tcam index %d\n", RSS_ENABLE_ACT, tcam_idx);
			return 1;
		}
		if (c3_data[C3_RSS_VALUE_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", RSS_ENABLE_ATTR, tcam_idx);
			return 1;
		}
		sprintf(sysfs_buf, "echo %s %s  > %s/act_sw_rss\n",
       			rss_action->value,
			rss_en->value,
			C3_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *hwf_l4_chk_en = findMatchingEntry((char *)ezxml_txt(c3_data[C3_HWF_L4CHK_ENB_E].xmlEntry));
		unsigned int hwfm_dptr = atoi(ezxml_txt(c3_data[C3_HWFM_DPTR_E].xmlEntry));
		unsigned int hwfm_iptr = atoi(ezxml_txt(c3_data[C3_HWFM_IPTR_E].xmlEntry));

		if (c3_data[C3_HWF_L4CHK_ENB_E].xmlEntry == NULL)
		{
			ERR_PR("%s is missing for tcam index %d\n", HWF_L4CHK_ENB, tcam_idx);
			return 1;
		}
		sprintf(sysfs_buf, "echo 0x%.4x 0x%.2x 0x%.2x  > %s/act_sw_mdf\n",
			hwfm_dptr,
			hwfm_iptr,
			atoi(hwf_l4_chk_en->value),
			C3_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		unsigned int dup_flowid = atoi(ezxml_txt(c3_data[C3_FLOW_ID_E].xmlEntry));
		unsigned int dup_count = atoi(ezxml_txt(c3_data[C3_HW_DUPLICATION_COUNT_E].xmlEntry));

		if (dup_count > 0) {
			sprintf(sysfs_buf, "echo 0x%.2x 0x%.2x         > %s/act_sw_dup\n",
				dup_flowid,
				dup_count,
				C3_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
	}
	
	{
		unsigned int mtu_index = atoi(ezxml_txt(c3_data[C3_MTU_IDX_E].xmlEntry));

		if (c3_data[C3_MTU_IDX_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", MTU_INDEX, tcam_idx);
			return 1;
		}
		sprintf(sysfs_buf, "echo 0x%.2x              > %s/act_sw_mtu\n",
			mtu_index,
			C3_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		int		i;
		unsigned int	ins_seq_info_id;
		unsigned int	offset = 0;
		bool		first_type1 = false;
		unsigned int	ins_seq_info_arr[] = {
			C3_INS_SEQ_INFO1_E,
			C3_INS_SEQ_INFO2_E,
			C3_INS_SEQ_INFO3_E,
			C3_INS_SEQ_INFO4_E,
			C3_INS_SEQ_INFO5_E,
			C3_INS_SEQ_INFO6_E,
			C3_INS_SEQ_INFO7_E,
			C3_INS_SEQ_INFO8_E };

		/* verify if this entry is first_type1 or not
		   info field must be set			*/
		if (NULL != c3_data[ins_seq_info_arr[0]].xmlEntry) {
			/* info1 is used */
			for (i = 1; i < INS_SEQ_INFO_CNT; i++) {
				if (NULL != c3_data[ins_seq_info_arr[i]].xmlEntry) {
					/* found a configured info other then 1,
					   this is a first type1 entry		*/
					first_type1 = true;
					break;
				}
			}
		}
		
		if (first_type1 == true) {
			/* go over all instructions and set ID according to global
			   inst ID size configuration CLS_SEQ_DATA			*/
			for (i = 0; i < INS_SEQ_INFO_CNT; i++) {
				/* no allocation for this sequence info, skip */
				if (0 == ins_seq_info_sz[i])
					continue;

				if (c3_data[ins_seq_info_arr[i]].xmlEntry == NULL)
					ins_seq_info_id = 0;
				else
					ins_seq_info_id = atoi(ezxml_txt(c3_data[ins_seq_info_arr[i]].xmlEntry));
			
				sprintf(sysfs_buf, "echo 0x%.2x 0x%.2x 0x%.2x    > %s/act_sw_sq\n",
					ins_seq_info_id,
					offset,
					ins_seq_info_sz[i],
					C3_SYSFS_PATH);
				handle_sysfs_command(sysfs_buf, false);
				offset += ins_seq_info_sz[i];
			}
		} else {
			ins_seq_info_id = atoi(ezxml_txt(c3_data[ins_seq_info_arr[0]].xmlEntry));
    
			sprintf(sysfs_buf, "echo 0x%.2x 0x%.2x 0x%.2x    > %s/act_sw_sq\n",
				ins_seq_info_id,
				0,
				8,
				C3_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
	}
	
	return 0;
}

/******************************************************************************
 *
 * Function   : build_c3_hek_sysfs
 *
 * Description:
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
static int build_c3_hek_sysfs(xml_entry_data *c3_data)
{
	char		sysfs_buf[512];
	unsigned int	skip = 0;/* must initialized as 0 */
	char		c3_hek_str[512], *c3_hek_ptr;
	unsigned char	c3_hek[36];
	PncEntry_S	pnCEntry;
	unsigned int 	field_id, pre_field_id = 0;
	unsigned char	field_id_num = 0;/* used to record number of field filled in HEK currently */
	unsigned int	c3_hek_bytes_used = 0;/* used to recoed current bytes filled in HEK */
	unsigned int 	i;
	unsigned int    field_id_array[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};

	memset(c3_hek, 0, 36 * sizeof(unsigned char));
	c3_hek_ptr = c3_hek_str;

	/* If header data empty while Num of HEK bytes isn't 0, return ERR */
	if (c3_data[C3_HEK35_0_E].xmlEntry == NULL)
	{
		DEBUG_PR(DEB_XML, "HEK35_0 is missing for tcam index\n");
		return PPV2_RC_OK;
	}
	/* Get C3 HEK str */
	strcpy(c3_hek_str, ezxml_txt(c3_data[C3_HEK35_0_E].xmlEntry));

	if (strcmp(ezxml_txt(c3_data[C3_HEK35_0_E].xmlEntry), EMPTY_START) == 0)
		return PPV2_RC_OK;

	/* Parse field ID and value */
	while ((strlen(c3_hek_ptr) > 0) && (dealWithPacket((unsigned char*)c3_hek_ptr, &pnCEntry, &skip) == true)) {
		char		*field_name;
		char		*subfield_name;
		char		*subfield_value;
		unsigned int 	indx = 0;
		RtSubFldEntry_S	*pRtSubFldEntry;
		unsigned int	field_bytes, left_bits;
		unsigned char   combine = 0;
		unsigned char   comb_offset = 0;

		/* get all parsed sub fields for this field */
		while (getRtSubFld(&field_name, &subfield_name, &subfield_value, &field_id, &pRtSubFldEntry, indx)) {
			/* the field id for this subfield is invalied, skip */
			if (NO_FIELD_ID == field_id) {
				ERR_PR("skip %d - %s %s %s\n", indx, field_name, subfield_name, subfield_value);
				return 1;
			}
			/* Input sequence must obey to field id sequence */
			if (field_id < pre_field_id) {
				ERR_PR("Wrong HEK field sequence for ID %d and ID %d\n", pre_field_id, field_id);
				return PPV2_RC_FAIL;
			}
			/* Parse each field, from HEK35_32 to HEK3_0, and HEK[35] is the first byte filled */
			switch (field_id) {
			case MH_FIELD_ID:
			case MH_UNTAGGED_PRI_FIELD_ID:
			case OUT_VLAN_PRI_FIELD_ID:
			case ETH_TYPE_FIELD_ID:
			case PPPOE_SESID_FIELD_ID:
			case PPPOE_PROTO_FIELD_ID:
			case IP_VER_FIELD_ID:
			case IPV4_DSCP_FIELD_ID:
			case IPV4_LEN_FIELD_ID:
			case IPV4_TTL_FIELD_ID:
			case IPV4_PROTO_FIELD_ID:
			case IPV6_PAYLOAD_LEN_FIELD_ID:
			case IPV6_NH_FIELD_ID:
			case L4_SRC_FIELD_ID:
			case L4_DST_FIELD_ID:
			case TCP_FLAGS_FIELD_ID:
			case PPV2_UDF_OUT_TPID:
			case PPV2_UDF_IN_TPID:
				/* Check field number, the max is 4 */
				if (field_id_num == 4) {
					ERR_PR("4 HEK field had already filled\n");
					return PPV2_RC_FAIL;
				}
				field_id_num++;
				/* Get HEK data and store it in c3_hek, each filed byte boutary */
				if (field_size[field_id] % 8)
					field_bytes = (field_size[field_id] / 8) + 1;
				else
					field_bytes = field_size[field_id] / 8;
				for (i = 0; i < field_bytes; i++) {
					if (field_size[field_id] % 8 == 0)
						c3_hek[35 - c3_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValue & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 1 - i))) & 0xFF);
					else {
						if (i < (field_bytes - 1))
							c3_hek[35 - c3_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValue & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 2 - i) + field_size[field_id] % 8)) & 0xFF);
						else
							c3_hek[35 - c3_hek_bytes_used] = (unsigned char)((pRtSubFldEntry->parsedIntValue << (8 - field_size[field_id] % 8)) & 0xFF);
					}
					c3_hek_bytes_used++;
				}
				break;
			/* Share bits combination */
			case GEM_PORT_ID_FIELD_ID:
			case IN_VLAN_ID_FIELD_ID:
			case OUT_VLAN_ID_FIELD_ID:
				if (pre_field_id == OUT_VLAN_PRI_FIELD_ID) {
					combine = 1;
					comb_offset = 4;
				}
			case IPV4_ECN_FIELD_ID:
				if (pre_field_id == IPV4_DSCP_FIELD_ID) {
					combine = 1;
					comb_offset = 2;
				}
			case IPV6_DSCP_FIELD_ID:
				if (pre_field_id == IP_VER_FIELD_ID) {
					combine = 1;
					comb_offset = 4;
				}
			case IPV6_ECN_FIELD_ID:
				if (pre_field_id == IPV6_DSCP_FIELD_ID) {
					combine = 1;
					comb_offset = 2;
				}
			case IPV6_FLOW_LBL_FIELD_ID:
				if (field_id == IPV6_FLOW_LBL_FIELD_ID && pre_field_id == IPV6_ECN_FIELD_ID) {
					combine = 1;
					if (field_id_array[0] == IP_VER_FIELD_ID && field_id_array[1] == IPV6_DSCP_FIELD_ID)
						comb_offset = 4;
					if (field_id_array[1] != IPV6_DSCP_FIELD_ID)
						comb_offset = 6;
				}
				/* Check field number, the max is 4 */
				if (field_id_num == 4) {
					ERR_PR("4 HEK field had already filled\n");
					return PPV2_RC_FAIL;
				}
				field_id_num++;
				/* Get HEK data and store it in c3_hek, each filed byte boutary */
				if (field_size[field_id] % 8)
					field_bytes = (field_size[field_id] / 8) + 1;
				else
					field_bytes = field_size[field_id] / 8;
				if (combine && (field_size[field_id] < 8) && ((field_size[field_id] + comb_offset) > 8))
					field_bytes++;
				left_bits = field_size[field_id];

				for (i = 0; i < field_bytes; i++) {
					if (combine) {
						c3_hek_bytes_used--;

						c3_hek[35 - c3_hek_bytes_used] |= ((unsigned char)(((pRtSubFldEntry->parsedIntValue >> (field_size[field_id] - comb_offset)) & common_mask_gen(comb_offset)) & 0xFF));
						if (((field_size[field_id] % 8) + comb_offset) > 8 || (field_size[field_id] > 8))
							pRtSubFldEntry->parsedIntValue &= common_mask_gen(field_size[field_id] - comb_offset);
						c3_hek_bytes_used++;
						left_bits = field_size[field_id] - comb_offset;
						combine = 0;
					} else {
						if (left_bits % 8 == 0)
							c3_hek[35 - c3_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValue & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 1 - i))) & 0xFF);
						else {
							if (i < (field_bytes - 1))
								c3_hek[35 - c3_hek_bytes_used] = (unsigned char)((pRtSubFldEntry->parsedIntValue >> ((field_bytes - 1) * 8) & 0xFF));
							else
								c3_hek[35 - c3_hek_bytes_used] = (unsigned char)(pRtSubFldEntry->parsedIntValue & 0xFF);
						}
						c3_hek_bytes_used++;
					}
				}
				break;
			case MAC_DA_FIELD_ID:
			case MAC_SA_FIELD_ID:
			case IPV4_SA_FIELD_ID:
			case IPV4_DA_FIELD_ID:
				/* Check field number, the max is 4 */
				if (field_id_num == 4) {
					ERR_PR("4 HEK field had already filled\n");
					return PPV2_RC_FAIL;
				}
				field_id_num++;

				if (field_id == MAC_DA_FIELD_ID || field_id == MAC_SA_FIELD_ID)
					field_bytes = MACADDR_SIZE;
				else
					field_bytes = IPADDR_SIZE;
				for (i = 0;i < field_bytes; i++) {
					if (field_id == MAC_DA_FIELD_ID || field_id == MAC_SA_FIELD_ID)
						c3_hek[35 - c3_hek_bytes_used] = pRtSubFldEntry->parsedMacAddress[i];
					else
						c3_hek[35 - c3_hek_bytes_used] = pRtSubFldEntry->parsedIpAddress[i];
					c3_hek_bytes_used++;
				}
				break;
			case IPV6_SA_FIELD_ID:
			case IPV6_SA_PREF_FIELD_ID:
			case IPV6_SA_SUFF_FIELD_ID:
			case IPV6_DA_FIELD_ID:
			case IPV6_DA_PREF_FIELD_ID:
			case IPV6_DA_SUFF_FIELD_ID:
				/* Check field number, the max is 4 */
				if (field_id_num == 4) {
					ERR_PR("4 HEK field had already been filled\n");
					return PPV2_RC_FAIL;
				}
				field_id_num++;

				if (field_id == IPV6_SA_FIELD_ID || field_id == IPV6_DA_FIELD_ID)
					field_bytes = IPV6ADDR_SIZE;
				else
					field_bytes = IPV6ADDR_SIZE / 2;
				for (i = 0; i < (field_bytes / 2); i++) {
					/* High byte */
					c3_hek[35 - c3_hek_bytes_used] = (unsigned char) (pRtSubFldEntry->parsedIpv6Address[i] >> 8);
					c3_hek_bytes_used++;
					/* Low byte */
					c3_hek[35 - c3_hek_bytes_used] = (unsigned char) (pRtSubFldEntry->parsedIpv6Address[i] & 0xFF);
					c3_hek_bytes_used++;
				}
				break;
			default:
				ERR_PR("Unsupported field id(%d)\n", field_id);
				return PPV2_RC_FAIL;
			}

			/* record previous id */
			pre_field_id = field_id;
			field_id_array[field_id_num - 1] = field_id;
			/* increase */
			indx++;
		}
		c3_hek_ptr = &c3_hek_str[skip];
		/*skip = 0;*/
	}

	/* set HEK size */
	sprintf(sysfs_buf, "echo 0x%.2x              > %s/key_sw_size\n", c3_hek_bytes_used, C3_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	/* Generate sysfs commands */
	for (i = 0; i < c3_hek_bytes_used; i++) {
		sprintf(sysfs_buf, "echo 0x%.2x 0x%.2x         > %s/key_sw_byte\n",
			(35 - i),
			c3_hek[35 - i],
			C3_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	return PPV2_RC_OK;
}

int parse_xml_c3(char *xmlFile)
{
	ezxml_t     		xmlHead = NULL;
	ezxml_t     		xmlC3, xmlEntry;
	unsigned int		ins_seq_info_sz[INS_SEQ_INFO_CNT];
	xml_entry_data	c3_data[C3_MAX_DATA] = {
		{TCAM_INDEX,		    NULL},
		{NAME,			    NULL},
		{L4INFO,		    NULL},
		{LU_TYPE,		    NULL},
		{PORT_IDTYPE,		    NULL},
		{PORT_ID,		    NULL},
		{HEK35_0,		    NULL},
		{MISS,			    NULL},
		{MULTIHASH_ENTRY_MODEL,     NULL},
		{MULTIHASH_TABLE_ADDRESS,   NULL},
		{EXTENSION_TABLE_ADDRESS,   NULL},
		{INITIAL_HIT_COUNTER_VALUE, NULL},
		{COLOR_ACTION,		    NULL},
		{QUEUE_LOW_ACTION,	    NULL},
		{QUEUE_LOW_VALUE,	    NULL},
		{QUEUE_HIGH_ACTION,	    NULL},
		{QUEUE_HIGH_VALUE,	    NULL},
		{FORWARDING,		    NULL},
		{POLICER_SELECT,	    NULL},
		{POLICER_ID,	            NULL},
		{FLOW_ID_ENABLE,	    NULL},
		{FLOW_ID,	            NULL},
		{RSS_ENABLE_ACT,	    NULL},
		{RSS_ENABLE_ATTR,           NULL},
		{HWFM_DPTR,		    NULL},
		{HWFM_IPTR,		    NULL},
		{HWF_L4CHK_ENB, 	    NULL},
		{HW_DUPLICATION_COUNT,	    NULL},
		{MTU_INDEX,	 	    NULL},
		{INS_SEQ_INFO1, 	    NULL},
		{INS_SEQ_INFO2, 	    NULL},
		{INS_SEQ_INFO3, 	    NULL},
		{INS_SEQ_INFO4, 	    NULL},
		{INS_SEQ_INFO5, 	    NULL},
		{INS_SEQ_INFO6, 	    NULL},
		{INS_SEQ_INFO7, 	    NULL},
		{INS_SEQ_INFO8, 	    NULL}};
	unsigned int	i;
	bool		miss_rule;

	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
		return 1;
	}

	xmlC3 = ezxml_get(xmlHead, WORKSHEET_C3, -1);
	if (xmlC3 == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_C3);
		ezxml_free(xmlHead);
		return 1;
	}

	xmlEntry = ezxml_child(xmlC3, TABLE_ENTRY);
	if (xmlEntry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n", WORKSHEET_C3);
		ezxml_free(xmlHead);
		return 0;
	}

	memset(ins_seq_info_sz, 0 , sizeof(ins_seq_info_sz));
	
	/* get the CLS instruction sequence info size */
	if (get_cls_ins_seq_info_sz(xmlFile, INS_SEQ_INFO_CNT, ins_seq_info_sz)) {
		ezxml_free(xmlHead);
       		return 1;
	}

	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		for (i=0; i < C3_MAX_DATA ;i++) {
			c3_data[i].xmlEntry = ezxml_child(xmlEntry, c3_data[i].name);

			DEBUG_PR(DEB_SYSFS, "%s=%s\n", c3_data[i].name, ezxml_txt(c3_data[i].xmlEntry))
		}

		if (build_c3_miss_get(c3_data, &miss_rule)) {
			ezxml_free(xmlHead);
	       		return 1;
		}
		
		/* set the common sfs cmd */
		if (build_c3_common_sysfs(c3_data, miss_rule)) {
			ezxml_free(xmlHead);
	       		return 1;
		}
		if (build_c3_action_sysfs(c3_data, INS_SEQ_INFO_CNT, ins_seq_info_sz)) {
			ezxml_free(xmlHead);
	       		return 1;
		}

		if (false == miss_rule && build_c3_hek_sysfs(c3_data)) {
			ezxml_free(xmlHead);
	       		return 1;
		}
		if (build_c3_bank_sysfs(c3_data, miss_rule)) {
			ezxml_free(xmlHead);
	       		return 1;
		}
		for (i=0; i < C3_MAX_DATA ;i++)
			c3_data[i].xmlEntry = NULL;
	}

	ezxml_free(xmlHead);

	return 0;
}


