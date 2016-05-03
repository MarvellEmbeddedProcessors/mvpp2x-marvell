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

enum xml_entry_data_rss{
	ACCESS_MODE_E,
	RSS_TBL_IDX_E,
	RXQ_OWNER_E,
	LINE_INDEX_E,
	RXQ_NEW_E,
	WIDTH_E,
	HASH_SEL_E,
	RSS_MAX_DATA
};

int parse_xml_rss(char *xmlFile)
{
	char		sysfs_buf[512];
	ezxml_t     		xmlHead = NULL;
	ezxml_t     		xmlRSS, xmlEntry;
	xml_entry_data	rss_data[RSS_MAX_DATA] = {
		{RSS_ACCESS_MODE,      NULL},
		{RSS_TABLE_INDEX,      NULL},
		{RXQ_OWNER,	      NULL},
		{RSS_LINE_IDX,	      NULL},
		{RXQ_NEW,             NULL},
		{RSS_WIDTH,           NULL},
		{HASH_SELECT,	      NULL}};
	unsigned int	i;
	unsigned int    accessMode = 0;
	unsigned int    rssTblIdx = 0;
	unsigned int    rssRxqOwner = 0;
	unsigned int    rssTblLine = 0;
	unsigned int    rssRxqNew = 0;
	unsigned int    rssWidth = 0;
	unsigned int    hashSel = 0;

	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
		return 1;
	}

	xmlRSS = ezxml_get(xmlHead, WORKSHEET_RSS_TABLE, -1);
	if (xmlRSS == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_RSS_TABLE);
		ezxml_free(xmlHead);
		return 1;
	}

	xmlEntry = ezxml_child(xmlRSS, TABLE_ENTRY);
	if (xmlEntry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n", WORKSHEET_RSS_TABLE);
		ezxml_free(xmlHead);
		return 0;
	}

	sprintf(sysfs_buf, "############  RSS_table: ############\n");
	handle_sysfs_command(sysfs_buf, true);

	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		for (i=0; i < RSS_MAX_DATA ;i++) {
			rss_data[i].xmlEntry = ezxml_child(xmlEntry, rss_data[i].name);

			DEBUG_PR(DEB_SYSFS, "%s=%s\n", rss_data[i].name, ezxml_txt(rss_data[i].xmlEntry))
		}

		if (rss_data[HASH_SEL_E].xmlEntry) {
			if (parseSimpleNumericValue((unsigned char*)ezxml_txt(rss_data[HASH_SEL_E].xmlEntry),
					&hashSel) == false) {
	 			ERR_PR("%s is not valid value\n", HASH_SELECT);
				return 1;
	 		}
			sprintf(sysfs_buf, "echo %.1d    > %s/rss_hash_sel\n", hashSel, RSS_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}

		if (rss_data[ACCESS_MODE_E].xmlEntry) {
			if (parseSimpleNumericValue((unsigned char*)ezxml_txt(rss_data[ACCESS_MODE_E].xmlEntry),
						&accessMode) == false) {
		 		ERR_PR("%s is not valid value\n", RSS_ACCESS_MODE);
				return 1;
		 	}
			sprintf(sysfs_buf, "############  Access_mode: [%d]   ############\n", accessMode);
			handle_sysfs_command(sysfs_buf, true);

			if (parseSimpleNumericValue((unsigned char*)ezxml_txt(rss_data[RSS_TBL_IDX_E].xmlEntry),
						&rssTblIdx) == false) {
		 		ERR_PR("%s is not valid value\n", RSS_TABLE_INDEX);
				return 1;
		 	}

			if (accessMode == 0) {
				if (parseSimpleNumericValue((unsigned char*)ezxml_txt(rss_data[RXQ_OWNER_E].xmlEntry),
						&rssRxqOwner) == false) {
		 			ERR_PR("%s is not valid value\n", RXQ_OWNER);
					return 1;
		 		}
				sprintf(sysfs_buf, "echo %.1d %.1d    > %s/rss_tbl_rxq_bind\n",
					rssRxqOwner, rssTblIdx, RSS_SYSFS_PATH);
				handle_sysfs_command(sysfs_buf, false);
			} else {
				if (parseSimpleNumericValue((unsigned char*)ezxml_txt(rss_data[LINE_INDEX_E].xmlEntry),
						&rssTblLine) == false) {
		 			ERR_PR("%s is not valid value\n", RSS_LINE_IDX);
					return 1;
		 		}
				if (parseSimpleNumericValue((unsigned char*)ezxml_txt(rss_data[RXQ_NEW_E].xmlEntry),
						&rssRxqNew) == false) {
		 			ERR_PR("%s is not valid value\n", RXQ_NEW);
					return 1;
		 		}
				if (parseSimpleNumericValue((unsigned char*)ezxml_txt(rss_data[WIDTH_E].xmlEntry),
						&rssWidth) == false) {
		 			ERR_PR("%s is not valid value\n", RSS_WIDTH);
					return 1;
		 		}
				sprintf(sysfs_buf, "echo %.1d %.1d %.1d %.1d   > %s/rss_tbl_entry_set\n",
						rssTblIdx, rssTblLine, rssRxqNew, rssWidth, RSS_SYSFS_PATH);
				handle_sysfs_command(sysfs_buf, false);
			}
		}
	}

	ezxml_free(xmlHead);

	return 0;
}


