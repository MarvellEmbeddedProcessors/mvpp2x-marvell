/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2012, Marvell Semiconductors Inc.                         **/
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PPv2 Tool                                                 **/
/**                                                                          **/
/**  FILE        : parse_cls.c                                               **/
/**                                                                          **/
/**  DESCRIPTION : This file generates shell commands for these work sheet   **/
/**                1. CLS_config                                             **/
/**                2. CLS                                                    **/
/**                3. CLS_flows                                              **/
/**                                                                          **/
/******************************************************************************
 **
 *   MODIFICATION HISTORY:
 *
 *    23-Oct-12   JingHua  - initial version created.
 *
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <errno.h>

#include "ezxml.h"
#include "DataDictionary.h"
#include "xml_params.h"
#include "common.h"
#include "PncGlobals.h"
#include "PacketParse.h"
#include "SubfieldParse.h"
#include "ParseUtils.h"

typedef enum {
	CLS_CFG_ACTIVE_E,
	CLS_CFG_PORT_LOOKUP_WAY_E,
	CLS_CFG_SPID_E,
	CLS_CFG_GEMPORT_ID_E,
	CLS_CFG_ADDITIONAL_FILEDS_E,
	CLS_CONFIG_COL_INDEX_MAX
} cls_config_col_index;

typedef enum {
	CLS_WAY_E,
	CLS_LU_ID_E,
	CLS_INIT_CPU_Q_E,
	CLS_HWF_MOD_INSTRUCT_OFFSET_E,
	CLS_LU_ENABLED_E,
	CLS_FLOW_ID_E,
	CLS_COL_INDEX_MAX
} cls_col_index;

typedef enum {
	CLS_INDEX_E,
	CLS_NAME_E,
	CLS_FLOW_TBL_IDX_E,
	CLS_PORT_TYPE_E,
	CLS_PORT_ID_SEL_E,
	CLS_PORT_ID_E,
	CLS_ENGINE_NO_E,
	CLS_PRIORITY_E,
	CLS_PPPOE_FILTER_E,
	CLS_VLAN_FILTER_E,
	CLS_M2M_FILTER_E,
	CLS_UDF7_FILTER_E,
	CLS_SEQUENCE_CTRL_E,
	CLS_LU_TYPE_E,
	CLS_FIELD1ID_E,
	CLS_FIELD2ID_E,
	CLS_FIELD3ID_E,
	CLS_FIELD4ID_E,
	CLS_LAST_E,
	CLS_FLOW_COL_INDEX_MAX
} cls_flow_col_index;

struct udf_field_t{
	unsigned int udf_flag;
	unsigned int baseoffID;
	unsigned int RelativeOff;
	unsigned int FieldSize;
};

struct eth_port_ctrl_t{
	unsigned int mh;
	unsigned int dis_vid;
	unsigned int dis_uni;
	unsigned int field_cnt;
};

static int build_cls_spid_gemport_add_parse(xml_entry_data *cls_config_data, cls_config_col_index idx)
{
	char 		spid_data_str[512], *spid_data_ptr;
	PncEntry_S	pnCEntry;
	unsigned int	skip;
	DdEntry		*db_temp = NULL;
	unsigned int	physical_port = 0;
	unsigned int	pkt_len_para_num = 0;
	int		pkt_len_tbl_idx, pkt_len_change, pkt_len_dec;
	int		value;

	if (cls_config_data == NULL)
		return 1;

	strcpy(spid_data_str, ezxml_txt(cls_config_data[idx].xmlEntry));
	skip = 0;
	spid_data_ptr = spid_data_str;
	while (strlen(spid_data_ptr)>0 && dealWithPacket((unsigned char*)spid_data_ptr, &pnCEntry, &skip) == true) {
		unsigned int 	field_id;
		char		*field_name;
		char		*subfield_name;
		char		*subfield_value;
		unsigned int 	indx = 0;
		RtSubFldEntry_S	*pRtSubFldEntry;
		char		sysfs_buf[512];
		struct udf_field_t 	udf_field[8];
		struct eth_port_ctrl_t 	eth_port_ctrl;

		memset(udf_field, 0 , sizeof(udf_field));
		memset(&eth_port_ctrl, 0 , sizeof(eth_port_ctrl));
		
		while (getRtSubFld(&field_name, &subfield_name, &subfield_value, &field_id, &pRtSubFldEntry, indx)) {
			/* find field name in dictionary */
			db_temp = findMatchingEntry(field_name);
			if (db_temp == NULL) {
				ERR_PR("Can not find %s in ppv2tool dictionary\n", field_name);
				return 1;
			}
			switch (idx) {
			case CLS_CFG_SPID_E:
				if (strlen(db_temp->name) < 7) {/*SPIDXX*/
					sprintf(sysfs_buf, "echo 0x%x 0x%x > %s/hw_uni_spid\n",
						atoi(db_temp->value),/*UNIx*/
						(pRtSubFldEntry->parsedIntValue & 0x7),
						CLS_SYSFS_PATH);
					handle_sysfs_command(sysfs_buf, false);
				} else {/*PortxSPIDExtract*/
					sprintf(sysfs_buf, "echo 0x%x 0x%x > %s/hw_port_spid\n",
						atoi(db_temp->value),/*physical portx*/
						(pRtSubFldEntry->parsedIntValue & 0x3),
						CLS_SYSFS_PATH);
					handle_sysfs_command(sysfs_buf, false);
				}
				break;
			case CLS_CFG_GEMPORT_ID_E:
				sprintf(sysfs_buf, "echo 0x%x 0x%x > %s/hw_virt_gpid\n",
					atoi(db_temp->value),/*UNIx*/
					(pRtSubFldEntry->parsedIntValue & 0x0FFF),
					CLS_SYSFS_PATH);
				handle_sysfs_command(sysfs_buf, false);
				break;
			case CLS_CFG_ADDITIONAL_FILEDS_E:
				if (field_name[0] == 'U') {/*UDF*/
					value = atoi(db_temp->value);
					
					if (!strcmp(subfield_name, "baseoffID")) {
						udf_field[value].baseoffID = pRtSubFldEntry->parsedIntValue;
						udf_field[value].udf_flag++;
					} else if (!strcmp(subfield_name, "RelativeOff")) {
						udf_field[value].RelativeOff = pRtSubFldEntry->parsedIntValue;
						udf_field[value].udf_flag++;
					} else if (!strcmp(subfield_name, "FieldSize")) {
						udf_field[value].FieldSize = pRtSubFldEntry->parsedIntValue;
						udf_field[value].udf_flag++;
					}
					if (udf_field[value].udf_flag == 3) {
						sprintf(sysfs_buf, "echo 0x%x 0x%x 0x%x 0x%x > %s/hw_udf\n",
							value,/*physical portx*/
							udf_field[value].baseoffID & 0xF,
							udf_field[value].RelativeOff & 0x7FF,
							udf_field[value].FieldSize & 0xFF,
							CLS_SYSFS_PATH);
						handle_sysfs_command(sysfs_buf, false);
						udf_field[value].udf_flag = 0;
					}
				}
				if (field_name[0] == 'M') {/*MTU*/
					if (atoi(db_temp->value) < 16)/*0-15 for PON port*/
						physical_port = 7;
					else
						physical_port = atoi(db_temp->value) - 16;
					sprintf(sysfs_buf, "echo 0x%x 0x%x 0x%x > %s/hw_mtu\n",
						physical_port,
						atoi(db_temp->value),
						(pRtSubFldEntry->parsedIntValue & 0xFFFF),
						CLS_SYSFS_PATH);
					handle_sysfs_command(sysfs_buf, false);
				}
				if (field_name[0] == 'O') {/*OPQN*/
					sprintf(sysfs_buf, "echo 0x%x 0x%x > %s/hw_over_rxq\n",
						atoi(db_temp->value),
						(pRtSubFldEntry->parsedIntValue & 0xFF),
						CLS_SYSFS_PATH);
					handle_sysfs_command(sysfs_buf, false);
				}
				/* Parse packet length info */
				if (!strcmp(field_name, "PLCTI")) {/*PacketLengthChangeTableIndex*/
					pkt_len_tbl_idx = (int)((pRtSubFldEntry->parsedIntValue) & 0xFF);
					pkt_len_para_num++;
				}
				if (!strcmp(field_name, "PLC")) {/*PacketLengthChange*/
					pkt_len_change = (int)((pRtSubFldEntry->parsedIntValue) & 0x7F);
					pkt_len_para_num++;
				}
				if (!strcmp(field_name, "PLDEC")) {/*PacketLengthDecrease, 0-increase, 1-decrease */
					pkt_len_dec = (int)((pRtSubFldEntry->parsedIntValue) & 0x01);
					pkt_len_para_num++;
				}
				if (pkt_len_para_num == 3) {
					if (pkt_len_dec)
						pkt_len_change = 0 - pkt_len_change;
					sprintf(sysfs_buf, "echo %d %d > %s/len_change_hw_set\n",
						pkt_len_tbl_idx,
						pkt_len_change,
						CLS_SYSFS_PATH);
					handle_sysfs_command(sysfs_buf, false);
					pkt_len_para_num = 0;
				}
		    
				if (!strncmp(field_name, "ins", 3)) { /* insXIdSz */
					sprintf(sysfs_buf, "echo 0x%x 0x%x > %s/hw_sq_size\n",
						atoi(db_temp->value),
						pRtSubFldEntry->parsedIntValue,
						CLS_SYSFS_PATH);
					handle_sysfs_command(sysfs_buf, false);
				}
				
				if (!strncmp(field_name, "eth", 3)) {/* ethPortCtrX */
				    if (!strcmp(subfield_name, "MH")) {
					    eth_port_ctrl.mh = pRtSubFldEntry->parsedIntValue;
					    eth_port_ctrl.field_cnt++;
				    } else if (!strcmp(subfield_name, "disVid")) {
					    eth_port_ctrl.dis_vid = (pRtSubFldEntry->parsedIntValue == 0) ? 1 : 0;
					    eth_port_ctrl.field_cnt++;
				    } else if (!strcmp(subfield_name, "disUni")) {
					    eth_port_ctrl.dis_uni = (pRtSubFldEntry->parsedIntValue == 0) ? 1 : 0;
					    eth_port_ctrl.field_cnt++;
				    }
				    if (eth_port_ctrl.field_cnt == 3) {
					    sprintf(sysfs_buf, "echo 0x%x 0x%x 0x%x 0x%x > %s/hw_mh\n",
						    atoi(db_temp->value),/*physical portx*/
						    eth_port_ctrl.dis_vid,
						    eth_port_ctrl.dis_uni,
						    eth_port_ctrl.mh,
						    CLS_SYSFS_PATH);
					    handle_sysfs_command(sysfs_buf, false);
					    memset(&eth_port_ctrl, 0 , sizeof(eth_port_ctrl));
				    }
				}
				break;
			default:
				return 1;/* Invalid idx */
			}
			indx++;
		}
		spid_data_ptr = &spid_data_ptr[skip];
		DEBUG_PR(DEB_OTHER, "tmp <%s>\n", spid_data_ptr);
		skip = 0;
	}
	return 0;
}


int parse_xml_cls_config(ezxml_t xmlHead)
{
	char		sysfs_buf[512];
	DdEntry 	*cls_active = NULL;
	unsigned int	port_lookup_way;
	unsigned int	port_id;
	ezxml_t     		xmlCLS, xmlEntry;

	xml_entry_data	cls_config_data[CLS_CONFIG_COL_INDEX_MAX] = {
		{ACTIVE,                NULL},
		{PORT_LOOKUP_WAY,       NULL},
		{SPID,                  NULL},
		{GEMPORTID,             NULL},
		{CLS_ADDITIONAL_FILEDS, NULL},
		};
	unsigned int	i;

	sprintf(sysfs_buf, "##########################  CLS config: ##########################\n");
	handle_sysfs_command(sysfs_buf, true);

	xmlCLS = ezxml_get(xmlHead, WORKSHEET_CLS_CONFIG, -1);
	if (xmlCLS == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_CLS_CONFIG);
		ezxml_free(xmlHead);
		return 1;
	}

	xmlEntry = ezxml_child(xmlCLS, TABLE_ENTRY);
	if (xmlEntry == NULL){
		ERR_PR("Failed to get %s\n", TABLE_ENTRY);
		ezxml_free(xmlHead);
		return 1;
	}
	/* only one entry */
	for (i=0; i < CLS_CONFIG_COL_INDEX_MAX;i++) {
		cls_config_data[i].xmlEntry = ezxml_child(xmlEntry, cls_config_data[i].name);
#if 0
		if (cls_data[i].xmlEntry == NULL)
			printf("%s is empty\n", cls_data[i].name);
		else
			printf("%s=%s\n", cls_data[i].name, ezxml_txt(cls_data[i].xmlEntry));
#endif
	}

	/*
	 * build cls active command
	 */
	cls_active = findMatchingEntry(ezxml_txt(cls_config_data[CLS_CFG_ACTIVE_E].xmlEntry));
	if (cls_active == NULL) {
		ERR_PR("%s is missing for cls_config\n", ACTIVE);
		return 1;
	}

	sprintf(sysfs_buf, "echo %d > %s/hw_enable\n", atoi(cls_active->value), CLS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	/*
	 * build lookup way command
	 */
	port_lookup_way = strtol(ezxml_txt(cls_config_data[CLS_CFG_PORT_LOOKUP_WAY_E].xmlEntry), NULL, 16);
	for (port_id = 0; port_id < 8; port_id++) {
		sprintf(sysfs_buf, "echo %d %d > %s/hw_port_way\n",
			port_id, (0x1 & (port_lookup_way >> port_id)), CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	/*
	 * build spid command
	 */
	//build_cls_cfg_spid(cls_config_data);
	build_cls_spid_gemport_add_parse(cls_config_data, CLS_CFG_SPID_E);

	/*
	 * build gemportid command
	 */

	build_cls_spid_gemport_add_parse(cls_config_data, CLS_CFG_GEMPORT_ID_E);

	/*
	 * build ClsAdditionalField command
	 */
	build_cls_spid_gemport_add_parse(cls_config_data, CLS_CFG_ADDITIONAL_FILEDS_E);

	return 0;
}

/* Calculate n times square of data */
static unsigned int n_time_square(unsigned int n, unsigned int data)
{
	unsigned int i;
	unsigned int temp;
	if (n == 0)
		return 1;
	temp = data;
	for (i = 1; i < n; i++)
		temp = temp * data;
	return temp;
}

/* Translate HEX string or DEC string to integer */
static unsigned int H_atoi(char *str)
{
	int number=0;
	int k, start = 0;

	/* check Hex */
	if ((strlen(str) > 2) && (str[0] == '0') && (str[1] == 'x' || str[1] == 'X')) {
		start = 2;
		for(k=start;k<strlen(str);k++){
			if(str[k]>='a'&&str[k]<='f')
				number += (str[k]-'a'+10) * n_time_square(strlen(str) - 1 - k,16);
			else if(str[k]>='A'&&str[k]<='F')
				number += (str[k]-'A'+10) * n_time_square(strlen(str) - 1 - k,16);
			else if(str[k]>='0'&&str[k]<='9')
				number += (str[k]-'0') * n_time_square(strlen(str) - 1 - k,16);
		}
	} else {/* Dec */
		number = atoi(str);
	}

	return number;
}

int parse_xml_cls(ezxml_t xmlHead)
{
	char		sysfs_buf[512];
	DdEntry 	*lu_enable = NULL;
	DdEntry 	*way = NULL;
	unsigned int	lookup_id;
	unsigned int	cpuQ;
	unsigned int	mod_offset;
	unsigned int	flow_id;
	ezxml_t     	xmlCLS, xmlEntry;
	unsigned int	i;

	xml_entry_data	cls_data[CLS_COL_INDEX_MAX] = {
		{WAY,                         NULL},
		{LU_ID,                       NULL},
		{INIT_CPU_Q,                  NULL},
		{HWFM_INSTRUCT_OFFSET,        NULL},
		{LU_ENABLED,		      NULL},
		{FLOW_IDX,		      NULL},
		};

	sprintf(sysfs_buf, "##########################  CLS: ##########################\n");
	handle_sysfs_command(sysfs_buf, true);

	xmlCLS = ezxml_get(xmlHead, WORKSHEET_CLS, -1);
	if (xmlCLS == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_CLS);
		ezxml_free(xmlHead);
		return 1;
	}

	xmlEntry = ezxml_child(xmlCLS, TABLE_ENTRY);
	if (xmlEntry == NULL){
		ERR_PR("Failed to get %s\n", TABLE_ENTRY);
		ezxml_free(xmlHead);
		return 1;
	}

	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		for (i=0; i < CLS_COL_INDEX_MAX; i++) {
			cls_data[i].xmlEntry = ezxml_child(xmlEntry, cls_data[i].name);
#if 0
			if (cls_data[i].xmlEntry == NULL)
				printf("%s is empty\n", cls_data[i].name);
			else
				printf("%s=%s\n", cls_data[i].name, ezxml_txt(cls_data[i].xmlEntry));
#endif
		}

		/*
		 * build cls command
		 */
		way = findMatchingEntry(ezxml_txt(cls_data[CLS_WAY_E].xmlEntry));
		if (way == NULL) {
			ERR_PR("%s is missing for cls\n", WAY);
			return 1;
		}

		lu_enable = findMatchingEntry(ezxml_txt(cls_data[CLS_LU_ENABLED_E].xmlEntry));
		if (lu_enable == NULL) {
			ERR_PR("%s is missing for cls\n", LU_ENABLED);
			return 1;
		}

		lookup_id = H_atoi(ezxml_txt(cls_data[CLS_LU_ID_E].xmlEntry));
		cpuQ = atoi(ezxml_txt(cls_data[CLS_INIT_CPU_Q_E].xmlEntry));
		flow_id = atoi(ezxml_txt(cls_data[CLS_FLOW_ID_E].xmlEntry));
		mod_offset = atoi(ezxml_txt(cls_data[CLS_HWF_MOD_INSTRUCT_OFFSET_E].xmlEntry));

		sprintf(sysfs_buf, "##########################  CLS: WAY:[%s], LookUp_ID:[%d]   ##########################\n", way->name, lookup_id);
		handle_sysfs_command(sysfs_buf, true);

		/* clear sw */
		sprintf(sysfs_buf, "echo 1 > %s/lkp_sw_clear\n", CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* cpu Q */
		sprintf(sysfs_buf, "echo %x > %s/lkp_sw_rxq\n", cpuQ, CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* flow_id */
		sprintf(sysfs_buf, "echo %x > %s/lkp_sw_flow\n", flow_id, CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* mod_offset */
		sprintf(sysfs_buf, "echo %x > %s/lkp_sw_mod\n", mod_offset, CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* lookup_en */
		sprintf(sysfs_buf, "echo %x > %s/lkp_sw_en\n", atoi(lu_enable->value), CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* set flow */
		sprintf(sysfs_buf, "echo %x %x > %s/lkp_hw_write\n", lookup_id, atoi(way->value), CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	return 0;
}

int parse_xml_cls_flows(ezxml_t xmlHead)
{
	char		sysfs_buf[512];
	DdEntry 	*port_type = NULL;
	DdEntry 	*engine_no = NULL;
	DdEntry 	*lookup_type = NULL;
	DdEntry 	*last = NULL;
	DdEntry 	*port_id_sel = NULL;
	DdEntry 	*pppoe_filter = NULL;
	DdEntry 	*vlan_filter = NULL;
	DdEntry 	*m2m_filter = NULL;
	DdEntry 	*udf7_filter = NULL;
	DdEntry 	*seq_ctrl = NULL;
	unsigned int	flow_id;
	unsigned int	port_id;
	unsigned int	Pri;
	unsigned int	field_id;
	DdEntry 	*field[4] = {NULL};
	ezxml_t     	xmlCLS, xmlEntry;
	unsigned int	i;
	unsigned int	field_num;

	xml_entry_data	cls_data[CLS_FLOW_COL_INDEX_MAX] = {
		{INDEX, 	NULL},
		{NAME,		NULL},
		{FLOW_TBL_IDX,	NULL},
		{PORT_IDTYPE,	NULL},
		{PORT_ID_SEL,	NULL},
		{PORT_ID,	NULL},
		{ENGINE_NO,	NULL},
		{PRIORITY,	NULL},
		{PPPOE_FILTER,	NULL},
		{VLAN_FILTER,	NULL},
		{M2M_FILTER,	NULL},
		{UDF7_FILTER,	NULL},
		{SEQUENCE_CTRL,	NULL},
		{LU_TYPE,	NULL},
		{FIELD1ID,	NULL},
		{FIELD2ID,	NULL},
		{FIELD3ID,	NULL},
		{FIELD4ID,	NULL},
		{LAST,		NULL},
		};

	sprintf(sysfs_buf, "##########################  CLS_FLOWS: ##########################\n");
	handle_sysfs_command(sysfs_buf, true);

	xmlCLS = ezxml_get(xmlHead, WORKSHEET_CLS_FLOWS, -1);
	if (xmlCLS == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_CLS_FLOWS);
		ezxml_free(xmlHead);
		return 1;
	}

	xmlEntry = ezxml_child(xmlCLS, TABLE_ENTRY);
	if (xmlEntry == NULL){
		ERR_PR("Failed to get %s\n", TABLE_ENTRY);
		ezxml_free(xmlHead);
		return 1;
	}

	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		for (i=0; i < CLS_FLOW_COL_INDEX_MAX; i++) {
			cls_data[i].xmlEntry = ezxml_child(xmlEntry, cls_data[i].name);
#if 0
			if (cls_data[i].xmlEntry == NULL)
				printf("%s is empty\n", cls_data[i].name);
			else
				printf("%s=%s\n", cls_data[i].name, ezxml_txt(cls_data[i].xmlEntry));
#endif
		}

		flow_id = atoi(ezxml_txt(cls_data[CLS_FLOW_TBL_IDX_E].xmlEntry));

		sprintf(sysfs_buf, "##########################	CLS flows: flow_id:[%d]   ##########################\n", flow_id);
		handle_sysfs_command(sysfs_buf, true);

		/*
		 * build cls_flows command
		 */
		port_type = findMatchingEntry(ezxml_txt(cls_data[CLS_PORT_TYPE_E].xmlEntry));
		if (port_type == NULL) {
			ERR_PR("%s is missing for cls\n", PORT_IDTYPE);
			return 1;
		}

		engine_no = findMatchingEntry(ezxml_txt(cls_data[CLS_ENGINE_NO_E].xmlEntry));
		if (engine_no == NULL) {
			ERR_PR("%s is missing for cls\n", ENGINE_NO);
			return 1;
		}

		lookup_type = findMatchingEntry(ezxml_txt(cls_data[CLS_LU_TYPE_E].xmlEntry));
		if (lookup_type == NULL) {
			ERR_PR("%s is missing for cls\n", LU_TYPE);
			return 1;
		}

		last = findMatchingEntry(ezxml_txt(cls_data[CLS_LAST_E].xmlEntry));
		if (last == NULL) {
			ERR_PR("%s is missing for cls\n", LAST);
			return 1;
		}

		port_id_sel = findMatchingEntry(ezxml_txt(cls_data[CLS_PORT_ID_SEL_E].xmlEntry));
		if (port_id_sel == NULL) {
			ERR_PR("%s is missing for cls\n", PORT_ID_SEL);
			return 1;
		}
		
		pppoe_filter = findMatchingEntry(ezxml_txt(cls_data[CLS_PPPOE_FILTER_E].xmlEntry));
		if (pppoe_filter == NULL) {
			ERR_PR("%s is missing for cls\n", PPPOE_FILTER);
			return 1;
		}
		
		vlan_filter = findMatchingEntry(ezxml_txt(cls_data[CLS_VLAN_FILTER_E].xmlEntry));
		if (vlan_filter == NULL) {
			ERR_PR("%s is missing for cls\n", VLAN_FILTER);
			return 1;
		} 
		
		m2m_filter = findMatchingEntry(ezxml_txt(cls_data[CLS_M2M_FILTER_E].xmlEntry));
		if (m2m_filter == NULL) {
			ERR_PR("%s is missing for cls\n", M2M_FILTER);
			return 1;
		}
		
		udf7_filter = findMatchingEntry(ezxml_txt(cls_data[CLS_UDF7_FILTER_E].xmlEntry));
		if (udf7_filter == NULL) {
			ERR_PR("%s is missing for cls\n", UDF7_FILTER);
			return 1;
		}
		
		seq_ctrl = findMatchingEntry(ezxml_txt(cls_data[CLS_SEQUENCE_CTRL_E].xmlEntry));
		if (seq_ctrl == NULL) {
			ERR_PR("%s is missing for cls\n", SEQUENCE_CTRL);
			return 1;
		}
		
		port_id = atoi(ezxml_txt(cls_data[CLS_PORT_ID_E].xmlEntry));
		Pri = atoi(ezxml_txt(cls_data[CLS_PRIORITY_E].xmlEntry));

		/* clear sw */
		sprintf(sysfs_buf, "echo 1 > %s/flow_sw_clear\n", CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* port ID select */
		sprintf(sysfs_buf, "echo 0x%x > %s/flow_sw_portid\n", atoi(port_id_sel->value), CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* PPPeO filter */
		sprintf(sysfs_buf, "echo 0x%x > %s/flow_sw_pppoe\n", atoi(pppoe_filter->value), CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
		
		/* VLAN filter */
		sprintf(sysfs_buf, "echo 0x%x > %s/flow_sw_vlan\n", atoi(vlan_filter->value), CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
		
		/* M2M filter */
		sprintf(sysfs_buf, "echo 0x%x > %s/flow_sw_macme\n", atoi(m2m_filter->value), CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
		
		/* UDF7 filter */
		sprintf(sysfs_buf, "echo 0x%x > %s/flow_sw_udf7\n", atoi(udf7_filter->value), CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
		
		/* Sequence number */
		sprintf(sysfs_buf, "echo 0x%x > %s/flow_sw_sq\n", atoi(seq_ctrl->value), CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
		
		/* port */
		sprintf(sysfs_buf, "echo %x %x > %s/flow_sw_port\n", atoi(port_type->value), port_id, CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* engine_no */
		sprintf(sysfs_buf, "echo %x %x > %s/flow_sw_engine\n",
				atoi(engine_no->value), atoi(last->value), CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* lookup_type, pri */
		sprintf(sysfs_buf, "echo %x %x > %s/flow_sw_extra\n", atoi(lookup_type->value), Pri, CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* flow_sw_hek */
		for (field_id = CLS_FIELD1ID_E, field_num = 0; field_id <= CLS_FIELD4ID_E; field_id++, field_num++) {
			if (cls_data[field_id].xmlEntry == NULL) {
				//DEBUG_PR("number of fields in flow: [%d]\n", field_num);
				break;
			}
			field[field_num] = findMatchingEntry(ezxml_txt(cls_data[field_id].xmlEntry));
			if (field[field_num] == NULL) {
				ERR_PR("field%d is missing for cls_flow\n", field_num + 1);
				return 1;
			}
		}

		if (field_num == 0) {
			ERR_PR("no field in flow, error!\n");
			return 1;
		}
		/* number of field */
		sprintf(sysfs_buf, "echo %x > %s/flow_sw_num_of_heks\n", field_num, CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		for (i = 0; i < field_num; i++) {
			/* field_id */
			sprintf(sysfs_buf, "echo %x %x > %s/flow_sw_hek\n", i, atoi(field[i]->value), CLS_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}

		/* set flow */
		sprintf(sysfs_buf, "echo %x > %s/flow_hw_write\n", flow_id, CLS_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
		
		for (i=0; i < CLS_FLOW_COL_INDEX_MAX; i++)
			cls_data[i].xmlEntry = NULL;
	}

	return 0;
}

int parse_xml_cls_module(char *xmlFile)
{
	ezxml_t     		xmlHead = NULL;

	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
		return 1;
	}

	if (0 != parse_xml_cls_config(xmlHead)){
		ERR_PR("Failed to parse cls_config worksheet\n");
		ezxml_free(xmlHead);
		return 1;
	}

	if (0 != parse_xml_cls(xmlHead)){
		ERR_PR("Failed to parse cls worksheet\n");
		ezxml_free(xmlHead);
		return 1;
	}

	if (0 != parse_xml_cls_flows(xmlHead)){
		ERR_PR("Failed to parse cls_flows worksheet\n");
		ezxml_free(xmlHead);
		return 1;
	}

	return 0;
}
