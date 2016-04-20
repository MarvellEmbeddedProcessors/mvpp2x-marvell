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
#include "ParseUtils.h"
#include "DataDictionary.h"
#include "PncGlobals.h"
#include "xml_params.h"
#include "PacketParse.h"
#include "SubfieldParse.h"

enum xml_entry_data_mc{
	MC_MC_TABLE_INDEX_E,
	MC_HWFM_DPTR_E,
	MC_HWFM_IPTR_E,
	MC_GEMPORTID_MOD_EN_E,
	MC_GEMPORTID_VALUE_E,
	MC_PRI_MOD_EN_E,
	MC_PRI_VALUE_E,
	MC_DSCP_MOD_EN_E,
	MC_DSCP_VALUE_E,
	MC_FRWD_TYPE_E,
	MC_QUEUE_NUMBER_E,
	MC_NEXT_MC_TABLE_INDEX_E,
	MC_MAX_DATA
};

int parse_xml_mc(char *xmlFile)
{
	char		sysfs_buf[512];
	ezxml_t     		xmlHead = NULL;
	ezxml_t     		xmlMC, xmlEntry;
	xml_entry_data	mc_data[MC_MAX_DATA] = {
		{MC_TABLE_INDEX,      NULL},
		{HWFM_DPTR,	      NULL},
		{HWFM_IPTR,	      NULL},
		{GEMPORTID_MOD_EN,    NULL},
		{GEMPORTID_VALUE,     NULL},
		{PRI_MOD_EN,	      NULL},
		{PRI_VALUE,	      NULL},
		{DSCP_MOD_EN,	      NULL},
		{DSCP_VALUE,	      NULL},
		{FRWD_TYPE,	      NULL},
		{QUEUE_NUMBER,	      NULL},
		{NEXT_MC_TABLE_INDEX, NULL}};
	unsigned int	i;
	unsigned int    mcTblIdx = 0;
	unsigned int    hwfmDPtr = 0;
	unsigned int    hwfmIPtr = 0;
	unsigned int    gem_portid = 0;
	unsigned int    prio = 0;
	unsigned int    dscp = 0;
	unsigned int    queue = 0;
	unsigned int    next_idx = 0;
	DdEntry 	*gem_portid_en = NULL;
	DdEntry 	*prio_en = NULL;
	DdEntry 	*dscp_en = NULL;
	DdEntry 	*frwd_type = NULL;

	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
		return 1;
	}

	xmlMC = ezxml_get(xmlHead, WORKSHEET_MC_TABLE, -1);
	if (xmlMC == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_MC_TABLE);
		ezxml_free(xmlHead);
		return 1;
	}

	xmlEntry = ezxml_child(xmlMC, TABLE_ENTRY);
	if (xmlEntry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n", WORKSHEET_MC_TABLE);
		ezxml_free(xmlHead);
		return 0;
	}

	sprintf(sysfs_buf, "############  MC_table: ############\n");
	handle_sysfs_command(sysfs_buf, true);

	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		for (i=0; i < MC_MAX_DATA ;i++) {
			mc_data[i].xmlEntry = ezxml_child(xmlEntry, mc_data[i].name);

			DEBUG_PR(DEB_SYSFS, "%s=%s\n", mc_data[i].name, ezxml_txt(mc_data[i].xmlEntry))
		}

		if (NULL == mc_data[MC_MC_TABLE_INDEX_E].xmlEntry) {
	 		ERR_PR("%s is not valid value\n", MC_TABLE_INDEX);
			return 1;
	 	}

		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(mc_data[MC_MC_TABLE_INDEX_E].xmlEntry),
					&mcTblIdx) == false) {
	 		ERR_PR("%s is not valid value\n", MC_TABLE_INDEX);
			return 1;
	 	}
		sprintf(sysfs_buf, "############  MC_Idx: [0x%X]   ############\n", mcTblIdx);
		handle_sysfs_command(sysfs_buf, true);

		gem_portid_en = findMatchingEntry((char *)ezxml_txt(mc_data[MC_GEMPORTID_MOD_EN_E].xmlEntry));
		if (gem_portid_en == NULL) {
			ERR_PR("%s is missing for MC table\n", GEMPORTID_MOD_EN);
			return 1;
		}
		
		prio_en = findMatchingEntry((char *)ezxml_txt(mc_data[MC_PRI_MOD_EN_E].xmlEntry));
		if (prio_en == NULL) {
			ERR_PR("%s is missing for MC table\n", PRI_MOD_EN);
			return 1;
		}
		
		dscp_en = findMatchingEntry((char *)ezxml_txt(mc_data[MC_DSCP_MOD_EN_E].xmlEntry));
		if (dscp_en == NULL) {
			ERR_PR("%s is missing for MC table\n", DSCP_MOD_EN);
			return 1;
		}
		
		frwd_type = findMatchingEntry((char *)ezxml_txt(mc_data[MC_FRWD_TYPE_E].xmlEntry));
		if (frwd_type == NULL) {
			ERR_PR("%s is missing for MC table\n", FRWD_TYPE);
			return 1;
		}

	 	sprintf(sysfs_buf, "echo 1              > %s/sw_clear\n",
	 		MC_SYSFS_PATH);
	 	handle_sysfs_command(sysfs_buf, false);

		if (atoi(dscp_en->value) == 1) {
			dscp = atoi(ezxml_txt(mc_data[MC_DSCP_VALUE_E].xmlEntry));
			
			sprintf(sysfs_buf, "echo %.1d %.1s            > %s/mc_sw_dscp\n",
				dscp,
				dscp_en->value,
				MC_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}

		if (atoi(gem_portid_en->value) == 1) {
			gem_portid = atoi(ezxml_txt(mc_data[MC_GEMPORTID_VALUE_E].xmlEntry));
			
			sprintf(sysfs_buf, "echo 0x%.3x %.1s        > %s/mc_sw_gpid\n",
				gem_portid,
				gem_portid_en->value,
				MC_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
		
		if (atoi(prio_en->value) == 1) {
			prio = atoi(ezxml_txt(mc_data[MC_PRI_VALUE_E].xmlEntry));
			
			sprintf(sysfs_buf, "echo %.1d %.1s            > %s/mc_sw_prio\n",
				prio,
				prio_en->value,
				MC_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
		
		if (mc_data[MC_HWFM_DPTR_E].xmlEntry ||
		    mc_data[MC_HWFM_IPTR_E].xmlEntry) {
		    
			hwfmDPtr = atoi(ezxml_txt(mc_data[MC_HWFM_DPTR_E].xmlEntry));
			hwfmIPtr = atoi(ezxml_txt(mc_data[MC_HWFM_IPTR_E].xmlEntry));
			sprintf(sysfs_buf, "echo 0x%.4x 0x%.4x  > %s/mc_sw_modif\n",
					hwfmDPtr,
					hwfmIPtr,
					MC_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}

		if (mc_data[MC_QUEUE_NUMBER_E].xmlEntry) {
			if (parseSimpleNumericValue((unsigned char*)ezxml_txt(mc_data[MC_QUEUE_NUMBER_E].xmlEntry), &queue) == false) {
				ERR_PR("%s is not valid value\n", MC_QUEUE_NUMBER_E);
				return 1;
			}
			sprintf(sysfs_buf, "echo 0x%.1X            > %s/mc_sw_queue\n",
				queue,
				MC_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
		
		if (mc_data[MC_NEXT_MC_TABLE_INDEX_E].xmlEntry) {
	 		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(mc_data[MC_NEXT_MC_TABLE_INDEX_E].xmlEntry),
				&next_idx) == false) {
				ERR_PR("%s is not valid value\n", NEXT_MC_TABLE_INDEX);
				return 1;
			}
			sprintf(sysfs_buf, "echo 0x%.3x          > %s/mc_sw_next\n",
				next_idx,
				MC_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}

		sprintf(sysfs_buf, "echo %.1s              > %s/mc_sw_hwf\n",
		 	frwd_type->value,
			MC_SYSFS_PATH);
	 	handle_sysfs_command(sysfs_buf, false);

		sprintf(sysfs_buf, "echo 0x%.3x          > %s/hw_write\n",
		 	mcTblIdx,
			MC_SYSFS_PATH);
	 	handle_sysfs_command(sysfs_buf, false);
		
		for (i=0; i < MC_MAX_DATA ;i++) {
			mc_data[i].xmlEntry = NULL;
		}
	}

	ezxml_free(xmlHead);

	return 0;
}


