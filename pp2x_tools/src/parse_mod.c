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
#include "parse_mod_tbl.h"
#include "ParseUtils.h"
#include "xml_params.h"
#include "PacketParse.h"
#include "SubfieldParse.h"


enum xml_entry_mod_cmd{
	CMD_IDX_E,
	CMD_NAME_E,
	CMD_TYPE_E,
	MOD_CMD_E,
	CMD_DATA_E,
	UPD_IPV4_CKSUM_E,
	UPD_L4_CKSUM_E,
	LAST_CMD_E,
	MOD_MAX_CMD_E
};

enum xml_entry_mod_data{
	MODDATA_IDX_E,
	MODDATA_NAME_E,
	MODDATA_DATA_E,
	MODDATA_MAX_E
};

enum xml_entry_mod_cfg{
	MODCFG_MTU_E,
	MODCFG_TPID0_E,
	MODCFG_TPID1_E,
	MODCFG_TPID2_E,
	MODCFG_TPID3_E,
	MODCFG_DEFVLANCFG0_E,
	MODCFG_DEFVLANCFG1_E,
	MODCFG_DEFVLANCFG2_E,
	MODCFG_TTLZERO_E,
	MODCFG_PPOE_ETY_E,
	MODCFG_PPOE_CODE_E,
	MODCFG_PPOE_TYPE_E,
	MODCFG_PPOE_VER_E,
	MODCFG_PPOE_LEN_E,
	MODCFG_PPOE_PROTO0_E,
	MODCFG_PPOE_PROTO1_E,
	MOD_MAX_CFG_E
};


#define MOD_DATA1		0
#define MOD_DATA2		1

/******************************************************************************
 *
 * Function   : build_mod_cmd_sysfs
 *
 * Description: builds the modification command 
 *
 * Parameters : xml_mod - the xml fields
 *
 * Returns    : int (success=0/error=1)
 *
 ******************************************************************************/
static int build_mod_cmd_sysfs(xml_entry_data *xml_mod)
{
	char		sysfs_buf[512];
	unsigned int	idx;
	DdEntry		*mod_cmd;
	
	idx = atoi(ezxml_txt(xml_mod[CMD_IDX_E].xmlEntry));

	{
		sprintf(sysfs_buf, "############  MOD command:%d [%s]  ############\n",
			idx, ezxml_txt(xml_mod[CMD_NAME_E].xmlEntry));
		handle_sysfs_command(sysfs_buf, true);
	}


	/* clear the SW structure */
	sprintf(sysfs_buf, "echo 1          > %s/sw_clear\n", PME_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	if (xml_mod[MOD_CMD_E].xmlEntry == NULL) {
		ERR_PR("%s is missing for index %d\n", MODIFICATION_COMMAND, idx);
		return 1;
	}

	mod_cmd = findMatchingEntry((char *)ezxml_txt(xml_mod[MOD_CMD_E].xmlEntry));
	if (mod_cmd == NULL) {
		ERR_PR("missing %s\n", xml_mod[MOD_CMD_E].name);
		return 1;
	}

	sprintf(sysfs_buf, "echo 0x%.2x       > %s/sw_cmd\n",
		atoi(mod_cmd->value),
		PME_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	
	
	sprintf(sysfs_buf, "echo %.1x          > %s/sw_type\n",
		atoi(ezxml_txt(xml_mod[CMD_TYPE_E].xmlEntry)),
		PME_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	

	/* build data, if the command does not have any data, it will be skipped */
	{
		int		cmd_nr = atoi((char*)mod_cmd->value);
		unsigned short	word;
		
		if (xml_mod[CMD_DATA_E].xmlEntry != NULL) {
			/* get the data word */
			if (parse_mod_data(cmd_nr,
					   (char *)ezxml_txt(xml_mod[CMD_DATA_E].xmlEntry),
					   &word)){
				ERR_PR("missing %s\n", xml_mod[CMD_DATA_E].name);
				return 1;
			}

			sprintf(sysfs_buf, "echo 0x%.4x     > %s/sw_data\n",
				word,
				PME_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
	}

	{
		DdEntry		*upd_ipv4_cksum;
		DdEntry		*upd_l4_cksum;
		DdEntry		*last;
		
		if (xml_mod[UPD_IPV4_CKSUM_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for index %d\n", UPDATE_IPV4_CHCKSUM, idx);
			return 1;
		}


		if (xml_mod[UPD_L4_CKSUM_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for index %d\n", UPDATE_L4CHCKSUM, idx);
			return 1;
		}

		if (xml_mod[LAST_CMD_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for index %d\n", LAST_COMMAND, idx);
			return 1;
		}
		
		upd_ipv4_cksum = findMatchingEntry((char *)ezxml_txt(xml_mod[UPD_IPV4_CKSUM_E].xmlEntry));
		if (upd_ipv4_cksum == NULL) {
			ERR_PR("missing %s\n", xml_mod[UPD_IPV4_CKSUM_E].name);
			return 1;
		}

		upd_l4_cksum = findMatchingEntry((char *)ezxml_txt(xml_mod[UPD_L4_CKSUM_E].xmlEntry));
		if (upd_l4_cksum == NULL) {
			ERR_PR("missing %s\n", xml_mod[UPD_L4_CKSUM_E].name);
			return 1;
		}

		last = findMatchingEntry((char *)ezxml_txt(xml_mod[LAST_CMD_E].xmlEntry));
		if (last == NULL) {
			ERR_PR("missing %s\n", xml_mod[LAST_CMD_E].name);
			return 1;
		}
		
		sprintf(sysfs_buf, "echo %.1s %.1s %.1s      > %s/sw_flags\n",
			last->value,
			upd_ipv4_cksum->value,
			upd_l4_cksum->value,
			PME_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	sprintf(sysfs_buf, "echo %.4d       > %s/hw_i_write\n",
		idx,
		PME_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	return 0;
}


/******************************************************************************
 *
 * Function   : build_mod_data_sysfs
 *
 * Description: build the modification data1/2 commands
 *
 * Parameters : xml_mod - the xml fields
 *		 data_nr - data table number (1/2)
 *
 * Returns    : int (success=0/error=1)
 *
 ******************************************************************************/
static int build_mod_data_sysfs(xml_entry_data *xml_mod, unsigned int data_nr)
{
	char		sysfs_buf[512];
	unsigned int	idx;
	int		data;
	
       idx = atoi(ezxml_txt(xml_mod[MODDATA_IDX_E].xmlEntry));

	if (xml_mod[MODDATA_NAME_E].xmlEntry) {
		sprintf(sysfs_buf, "############  MOD data%d: %d [%s]  ############\n",
			data_nr, idx, ezxml_txt(xml_mod[MODDATA_NAME_E].xmlEntry));
		handle_sysfs_command(sysfs_buf, true);
	}

	if (xml_mod[MODDATA_DATA_E].xmlEntry) {
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod[MODDATA_DATA_E].xmlEntry), &data) == false) {
			ERR_PR("could not convert input %s!!\n", ezxml_txt(xml_mod[MODDATA_DATA_E].xmlEntry));
			return 1;
		}
		
		sprintf(sysfs_buf, "echo %1d %.5d 0x%.4x   > %s/hw_d_write\n",
			data_nr,
			idx,
			data,
			PME_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	return 0;
}

/******************************************************************************
 *
 * Function   : build_mod_cfg_sysfs
 *
 * Description: builds the modification config command
 *
 * Parameters : xml_mod_cfg - the xml fields
 *
 * Returns    : int (success=0/error=1)
 *
 ******************************************************************************/
static int build_mod_cfg_sysfs(xml_entry_data *xml_mod_cfg)
{
	char sysfs_buf[512];
	int value = 0, value1 = 0, value2 = 0;

	sprintf(sysfs_buf, "############  MOD config  ############\n");
	handle_sysfs_command(sysfs_buf, true);

	if (ezxml_txt(xml_mod_cfg[MODCFG_MTU_E].xmlEntry) != EMPTY_STR) {

	}
	if (ezxml_txt(xml_mod_cfg[MODCFG_TPID0_E].xmlEntry) != EMPTY_STR) {
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod_cfg[MODCFG_TPID0_E].xmlEntry), &value)) {
			sprintf(sysfs_buf, "echo 0 0x%x > %s/vlan_etype\n",
				value,
				PME_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			ERR_PR("could not convert input %s!!\n", ezxml_txt(xml_mod_cfg[MODCFG_TPID0_E].xmlEntry));
			return 1;
		}
	}
	if (ezxml_txt(xml_mod_cfg[MODCFG_TPID1_E].xmlEntry) != EMPTY_STR) {
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod_cfg[MODCFG_TPID1_E].xmlEntry), &value)) {
			sprintf(sysfs_buf, "echo 1 0x%x > %s/vlan_etype\n",
				value,
				PME_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			ERR_PR("could not convert input %s!!\n", ezxml_txt(xml_mod_cfg[MODCFG_TPID1_E].xmlEntry));
			return 1;
		}
	}
	if (ezxml_txt(xml_mod_cfg[MODCFG_TPID2_E].xmlEntry) != EMPTY_STR) {
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod_cfg[MODCFG_TPID2_E].xmlEntry), &value)) {
			sprintf(sysfs_buf, "echo 2 0x%x > %s/vlan_etype\n",
				value,
				PME_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			ERR_PR("could not convert input %s!!\n", ezxml_txt(xml_mod_cfg[MODCFG_TPID2_E].xmlEntry));
			return 1;
		}
	}
	if (ezxml_txt(xml_mod_cfg[MODCFG_TPID3_E].xmlEntry) != EMPTY_STR) {
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod_cfg[MODCFG_TPID3_E].xmlEntry), &value)) {
			sprintf(sysfs_buf, "echo 3 0x%x > %s/vlan_etype\n",
				value,
				PME_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			ERR_PR("could not convert input %s!!\n", ezxml_txt(xml_mod_cfg[MODCFG_TPID3_E].xmlEntry));
			return 1;
		}
	}
	if (ezxml_txt(xml_mod_cfg[MODCFG_DEFVLANCFG0_E].xmlEntry) != EMPTY_STR) {

	}
	if (ezxml_txt(xml_mod_cfg[MODCFG_DEFVLANCFG1_E].xmlEntry) != EMPTY_STR) {

	}
	if (ezxml_txt(xml_mod_cfg[MODCFG_DEFVLANCFG2_E].xmlEntry) != EMPTY_STR) {

	}
	if (ezxml_txt(xml_mod_cfg[MODCFG_TTLZERO_E].xmlEntry) != EMPTY_STR) {
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod_cfg[MODCFG_TTLZERO_E].xmlEntry), &value)) {
			sprintf(sysfs_buf, "echo 0x%x > %s/ttl_zero\n",
				((value == 0) ? 0 : 1),
				PME_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			ERR_PR("could not convert input %s!!\n", ezxml_txt(xml_mod_cfg[MODCFG_TTLZERO_E].xmlEntry));
			return 1;
		}
	}
	if (ezxml_txt(xml_mod_cfg[MODCFG_PPOE_ETY_E].xmlEntry) != EMPTY_STR) {
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod_cfg[MODCFG_PPOE_ETY_E].xmlEntry), &value)) {
			sprintf(sysfs_buf, "echo 0x%x > %s/pppoe_etype\n",
				value,
				PME_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			ERR_PR("could not convert input %s!!\n", ezxml_txt(xml_mod_cfg[MODCFG_PPOE_ETY_E].xmlEntry));
			return 1;
		}
	}
	if ((ezxml_txt(xml_mod_cfg[MODCFG_PPOE_CODE_E].xmlEntry) != EMPTY_STR) &&
	    (ezxml_txt(xml_mod_cfg[MODCFG_PPOE_TYPE_E].xmlEntry) != EMPTY_STR) &&
	    (ezxml_txt(xml_mod_cfg[MODCFG_PPOE_VER_E].xmlEntry) != EMPTY_STR)) {
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod_cfg[MODCFG_PPOE_CODE_E].xmlEntry), &value2) &&
		    parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod_cfg[MODCFG_PPOE_TYPE_E].xmlEntry), &value1) &&
		    parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod_cfg[MODCFG_PPOE_VER_E].xmlEntry), &value)) {
			sprintf(sysfs_buf, "echo 0x%x 0x%x 0x%x > %s/pppoe_set\n",
				value,
				value1,
				value2,
				PME_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			ERR_PR("could not parse input of PPPoE code or type or version!!\n");
			return 1;
		}
	}
	if (ezxml_txt(xml_mod_cfg[MODCFG_PPOE_LEN_E].xmlEntry) != EMPTY_STR) {
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod_cfg[MODCFG_PPOE_LEN_E].xmlEntry), &value)) {
			sprintf(sysfs_buf, "echo 0x%x > %s/pppoe_len\n",
				value,
				PME_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			ERR_PR("could not convert input %s!!\n", ezxml_txt(xml_mod_cfg[MODCFG_PPOE_LEN_E].xmlEntry));
			return 1;
		}
	}
	if (ezxml_txt(xml_mod_cfg[MODCFG_PPOE_PROTO0_E].xmlEntry) != EMPTY_STR) {
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod_cfg[MODCFG_PPOE_PROTO0_E].xmlEntry), &value)) {
			sprintf(sysfs_buf, "echo 0 0x%x > %s/pppoe_proto\n",
				value,
				PME_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			ERR_PR("could not convert input %s!!\n", ezxml_txt(xml_mod_cfg[MODCFG_PPOE_PROTO0_E].xmlEntry));
			return 1;
		}
	}
	if (ezxml_txt(xml_mod_cfg[MODCFG_PPOE_PROTO1_E].xmlEntry) != EMPTY_STR) {
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(xml_mod_cfg[MODCFG_PPOE_PROTO1_E].xmlEntry), &value)) {
			sprintf(sysfs_buf, "echo 1 0x%x > %s/pppoe_proto\n",
				value,
				PME_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			ERR_PR("could not convert input %s!!\n", ezxml_txt(xml_mod_cfg[MODCFG_PPOE_PROTO1_E].xmlEntry));
			return 1;
		}
	}

	return 0;
}


/******************************************************************************
 *
 * Function   : parse_xml_mod_cmd
 *              
 * Description: parse the mod_cmd xml sheet and extract data fileds
 *              
 * Parameters : xmlFile - the xml file path
 *
 * Returns    : int (success=0/error=1)
 *              
 ******************************************************************************/
int parse_xml_mod_cmd(char *xmlFile)
{
	ezxml_t		xml_head = NULL,
			xml_mod, 
			xml_entry;
	unsigned int	i;
	xml_entry_data	mod_data[MOD_MAX_CMD_E] = {
		{TPM_TI,		NULL},
		{NAME,			NULL},
		{COMMAND_TYPE,		NULL},
		{MODIFICATION_COMMAND,	NULL},
		{MODIFICATION_DATA,	NULL},
		{UPDATE_IPV4_CHCKSUM,	NULL},
		{UPDATE_L4CHCKSUM,	NULL},
		{LAST_COMMAND,		NULL} };

	xml_head = ezxml_parse_file(xmlFile);
	if (xml_head == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
        	return 1;
	}

	xml_mod = ezxml_get(xml_head, WORKSHEET_MOD_CMD, -1);
	if (xml_mod == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_MOD_CMD);
		ezxml_free(xml_head);
		return 1;
	}

	xml_entry = ezxml_child(xml_mod, TABLE_ENTRY);
	if (xml_entry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n", WORKSHEET_MOD_CMD);
		ezxml_free(xml_head);
		return 0;
    	}

	for (; xml_entry; xml_entry = ezxml_next(xml_entry)) {
		for (i=0; i < MOD_MAX_CMD_E ;i++) {
			mod_data[i].xmlEntry = ezxml_child(xml_entry, mod_data[i].name);

			DEBUG_PR(DEB_XML, "%s=%s\n", mod_data[i].name, ezxml_txt(mod_data[i].xmlEntry))
		}

		/* build the sysfs commands */
		if (build_mod_cmd_sysfs(mod_data)) {
			ezxml_free(xml_head);
       		return 1;
		}
		for (i=0; i < MOD_MAX_CMD_E ;i++)
			mod_data[i].xmlEntry = NULL;
	}

	ezxml_free(xml_head);

	return 0;
}



/******************************************************************************
 *
 * Function   : parse_xml_mod_data
 *              
 * Description: parse the mod_data xml sheet and extract data fileds
 *              
 * Parameters : xmlFile - the xml file path
 *		 data_nr - data table number (1/2)
 *
 * Returns    : int (success=0/error=1)
 *              
 ******************************************************************************/
static int parse_xml_mod_data(char *xmlFile, unsigned int data_nr)
{
	ezxml_t		xml_head = NULL,
			xml_mod, 
			xml_entry;
	unsigned int	i;
	xml_entry_data	mod_data[MODDATA_MAX_E] = {
		{TPM_TI,		NULL},
		{NAME,			NULL},
		{MODIFICATION_DATA,	NULL} };

	xml_head = ezxml_parse_file(xmlFile);
	if (xml_head == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
        	return 1;
	}

	xml_mod = ezxml_get(xml_head,
			(data_nr == MOD_DATA1) ? WORKSHEET_MOD_DATA1 : WORKSHEET_MOD_DATA2,
			-1);
	if (xml_mod == NULL){
		ERR_PR("Failed to get %s\n", (data_nr == 1) ? WORKSHEET_MOD_DATA1 : WORKSHEET_MOD_DATA2);
		ezxml_free(xml_head);
		return 1;
	}

	xml_entry = ezxml_child(xml_mod, TABLE_ENTRY);
	if (xml_entry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n",
			(data_nr == MOD_DATA1) ? WORKSHEET_MOD_DATA1 : WORKSHEET_MOD_DATA2);
		ezxml_free(xml_head);
		return 0;
    	}

	for (; xml_entry; xml_entry = ezxml_next(xml_entry)) {
		for (i=0; i < MODDATA_MAX_E ;i++) {
			mod_data[i].xmlEntry = ezxml_child(xml_entry, mod_data[i].name);

			DEBUG_PR(DEB_XML, "%s=%s\n", mod_data[i].name, ezxml_txt(mod_data[i].xmlEntry))
		}

		/* build the sysfs commands */
		if (build_mod_data_sysfs(mod_data, data_nr)) {
			ezxml_free(xml_head);
       		return 1;
		}
		for (i=0; i < MODDATA_MAX_E ;i++)
			mod_data[i].xmlEntry = NULL;
	}

	ezxml_free(xml_head);

	return 0;
}


/******************************************************************************
 *
 * Function   : parse_xml_mod_data1
 *              
 * Description: parse the mod_data1 xml sheet and extract data fileds
 *              
 * Parameters : xmlFile - the xml file path
 *
 * Returns    : int (success=0/error=1)
 *              
 ******************************************************************************/
int parse_xml_mod_data1(char *xmlFile)
{
	return parse_xml_mod_data(xmlFile, MOD_DATA1);
}


/******************************************************************************
 *
 * Function   : parse_xml_mod_data2
 *              
 * Description: parse the mod_data2 xml sheet and extract data fileds
 *              
 * Parameters : xmlFile - the xml file path
 *
 * Returns    : int (success=0/error=1)
 *              
 ******************************************************************************/
int parse_xml_mod_data2(char *xmlFile)
{
	return parse_xml_mod_data(xmlFile, MOD_DATA2);
}


/******************************************************************************
 *
 * Function   : parse_xml_mod_cfg
 *              
 * Description: parse the mod_cfg xml sheet and extract data fileds
 *              
 * Parameters : xmlFile - the xml file path
 *
 * Returns    : int (success=0/error=1)
 *              
 ******************************************************************************/
int parse_xml_mod_cfg(char *xmlFile)
{
	int i;
	ezxml_t		xml_head = NULL,
			xml_mod, 
			xml_entry;

	xml_entry_data	mod_cfg[MOD_MAX_CFG_E] = {
		{MTU,		NULL},
		{TPID0,		NULL},
		{TPID1,		NULL},
		{TPID2,		NULL},
		{TPID3,		NULL},
		{DEFAULT_VLANCFG0,	NULL},
		{DEFAULT_VLANCFG1,	NULL},
		{DEFAULT_VLANCFG2,	NULL},
		{TTL_ZERO_FORWARD,	NULL},
		{PPPOE_ETHERTYPE,	NULL},
		{PPPOE_CODE,	NULL},
		{PPPOE_TYPE,	NULL},
		{PPPOE_VER,	NULL},
		{PPPOE_LENGTH_CONFIG,	NULL},
		{PPPOE_PROTOCOL0,	NULL},
		{PPPOE_PROTOCOL1,	NULL}};

	xml_head = ezxml_parse_file(xmlFile);
	if (xml_head == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
        	return 1;
	}

	xml_mod = ezxml_get(xml_head, WORKSHEET_MOD_CFG, -1);
	if (xml_mod == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_MOD_CFG);
		ezxml_free(xml_head);
		return 1;
	}

	xml_entry = ezxml_child(xml_mod, TABLE_ENTRY);
	if (xml_entry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n", WORKSHEET_MOD_CFG);
		ezxml_free(xml_head);
		return 0;
    	}

	for (; xml_entry; xml_entry = ezxml_next(xml_entry)) {
		for (i=0; i < MOD_MAX_CFG_E ;i++) {
			mod_cfg[i].xmlEntry = ezxml_child(xml_entry, mod_cfg[i].name);

			DEBUG_PR(DEB_XML, "%s=%s\n", mod_cfg[i].name, ezxml_txt(mod_cfg[i].xmlEntry));
		}

		/* build the sysfs commands */
		if (build_mod_cfg_sysfs(mod_cfg)) {
			ezxml_free(xml_head);
       			return 1;
		}

		for (i=0; i < MOD_MAX_CFG_E ;i++)
			mod_cfg[i].xmlEntry = NULL;
	}

	ezxml_free(xml_head);

	return 0;
}

