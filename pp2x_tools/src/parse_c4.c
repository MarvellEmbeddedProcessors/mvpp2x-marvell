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
#include "ParseUtils.h"
#include "PncGlobals.h"
#include "xml_params.h"
#include "PacketParse.h"
#include "SubfieldParse.h"

enum xml_entry_data_c4{
	RULESSET_NO_E,
	RULE_NO_E,
	FIELD0ID_E,
	FIELD0OP_CODE_E,
	FIELD0COMPARE_DATA_E,
	FIELD1ID_E,
	FIELD1OP_CODE_E,
	FIELD1COMPARE_DATA_E,
	FIELD2ID_E,
	FIELD2OP_CODE_E,
	FIELD2COMPARE_DATA_E,
	FIELD3ID_E,
	FIELD3OP_CODE_E,
	FIELD3COMPARE_DATA_E,
	FIELD4ID_E,
	FIELD4OP_CODE_E,
	FIELD4COMPARE_DATA_E,
	FIELD5ID_E,
	FIELD5OP_CODE_E,
	FIELD5COMPARE_DATA_E,
	COLOR_ACTION_E,
	PRI_ACTION_E,
	PRI_VALUE_E,
	DSCP_ACTION_E,
	DSCP_VALUE_E,
	GEMPORTID_ACTION_E,
	GEMPORTID_E,
	QUEUE_LOW_ACTION_E,
	QUEUE_LOW_VALUE_E,
	QUEUE_HIGH_ACTION_E,
	QUEUE_HIGH_VALUE_E,
	FORWARDING_E,
	POLICER_SELECT_E,
	POLICER_ID_E,
	L3INFO_E,
	L4INFO_E,
	MAC2ME_E,
	PPPOE_E,
	VLAN_E,
	C4_MAX_DATA_E
};

enum xml_entry_data_c4_ruleset{
	RS_PORT_NO_E,
	RS_PORT_IDTYPE_E,
	RS_RULESET_NO_E,
	RS_RULES_IN_RULESET_E,
	RS_C4_MAX
};

extern unsigned int field_size[PPV2_FIELD_COUNT];


/******************************************************************************
 *
 * Function   : build_c4_ipv4_sysfs
 *              
 * Description: creates the tcam sysfs command for IPv4
 *              
 * Parameters :  
 *
 * Returns    : int
 *              
 ******************************************************************************/
static void build_c4_ipv4_sysfs(unsigned int	field_nr,
			 	unsigned char	*ip_addr,
				unsigned int	field_src_size,
				unsigned int	field_dst_size)
{
	unsigned int	i;
	char 		sysfs_buf[512];
	unsigned int	word = field_dst_size / 2 - 1;
	
	for (i = 0; i < field_src_size; i+=2, word--){
		sprintf(sysfs_buf, "echo %.1x %.1x 0x%.4x  > %s/rule_two_b\n",
			field_nr,
			word*2,
			(ip_addr[i] << 8) | ip_addr[i+1],
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	if (DEB_LEVEL == DEB_SYSFS){
		printf("IPv4 address ");
	 	for (i = 0; i < field_src_size; i++)
			printf("%d.", ip_addr[i]);
		printf("\n");
	}
}


/******************************************************************************
 *
 * Function   : build_c4_ipv6_sysfs
 *              
 * Description: creates the tcam sysfs command for IPv4
 *              
 * Parameters :  
 *
 * Returns    : int
 *              
 ******************************************************************************/
static void build_c4_ipv6_sysfs(unsigned int		field_nr,
			 	unsigned short int	*ip_addr,
				unsigned int		field_src_size,
				unsigned int		field_dst_size)
{
	unsigned int	i;
	char 		sysfs_buf[512];
	unsigned int	word = field_dst_size / 2 - 1;
	
	for (i = 0; i < field_src_size/2; i++, word--){
		sprintf(sysfs_buf, "echo %.1x %.1x 0x%.4x  > %s/rule_two_b\n",
			field_nr,
			word*2,
			ip_addr[i],
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	if (DEB_LEVEL == DEB_SYSFS){
		printf("IPv6 address ");
	 	for (i = 0; i < field_src_size/sizeof(unsigned short int); i++)
			printf("0x%X.", ip_addr[i]);
		printf("\n");
	}
}


/******************************************************************************
 *
 * Function   : build_c4_mac_sysfs
 *              
 * Description: creates the tcam sysfs command for MAC address
 *              
 * Parameters :  
 *
 * Returns    : int
 *              
 ******************************************************************************/
static void build_c4_mac_sysfs(unsigned int	field_nr,
			       	unsigned char	*mac_addr,
			       	unsigned int	field_src_size,
				unsigned int	field_dst_size)
{
	unsigned int	i;
	char 		sysfs_buf[512];
	unsigned int	word = field_dst_size / 2 - 1;
	
	for (i = 0; i < field_src_size; i+=2, word--){
		sprintf(sysfs_buf, "echo %.1x %.1x 0x%.4x  > %s/rule_two_b\n",
			field_nr,
			word*2,
			(mac_addr[i] << 8) | mac_addr[i+1],
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	if (DEB_LEVEL == DEB_SYSFS){
		printf("MAC address ");
	 	for (i = 0; i < field_src_size; i++)
			printf("%d-", mac_addr[i]);
		printf("\n");
	}
}


/******************************************************************************
 *
 * Function   : build_c4_2_byte_sysfs
 *              
 * Description: creates the tcam sysfs command for a simple field
 *              
 * Parameters :  
 *
 * Returns    : int
 *              
 ******************************************************************************/
static void build_c4_2_byte_sysfs(unsigned int		field_nr,
				unsigned int		value)
{
	char 		sysfs_buf[512];
	
	sprintf(sysfs_buf, "echo %.1x %.1x 0x%.4x  > %s/rule_two_b\n",
		field_nr,
		0,
		value,
		C4_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	DEBUG_PR(DEB_SYSFS, "2 byte data: %d\n", value);
}


/******************************************************************************
 *
 * Function   : align_2_byte_field
 *              
 * Description: alignes the field value to 2 byte output
 *              
 * Parameters :  
 *
 * Returns    : int
 *              
 ******************************************************************************/
static unsigned int align_2_byte_field(unsigned int		field_nr,
					unsigned int		*value)
{

	unsigned int	field_bit_size;
	unsigned int	field_l_shift = 0;
	
	/* prepare 2 byte fields */
	field_bit_size = field_size[field_nr];
	
	switch (field_nr) {
		case OUT_VLAN_ID_FIELD_ID:
			break;
		case MH_FIELD_ID:
		case OUT_VLAN_PRI_FIELD_ID:
		case GEM_PORT_ID_FIELD_ID:
		case MH_UNTAGGED_PRI_FIELD_ID:
		case IN_VLAN_ID_FIELD_ID:
		case ETH_TYPE_FIELD_ID:
		case PPPOE_SESID_FIELD_ID:
		case PPPOE_PROTO_FIELD_ID:
		case IP_VER_FIELD_ID:
		case IPV4_DSCP_FIELD_ID:
		case IPV4_ECN_FIELD_ID:
	 	case IPV4_LEN_FIELD_ID:
		case IPV4_TTL_FIELD_ID:
		case IPV4_PROTO_FIELD_ID:
		//case IPV6_PROTO_FIELD_ID: same as IPv4
	 	case IPV6_DSCP_FIELD_ID:
		case IPV6_ECN_FIELD_ID:
		case IPV6_FLOW_LBL_FIELD_ID:
		case IPV6_PAYLOAD_LEN_FIELD_ID:
		case IPV6_NH_FIELD_ID:
		//case IPV6_HL_FIELD_ID: same as TTL
		case L4_SRC_FIELD_ID:
		case L4_DST_FIELD_ID:
		case TCP_FLAGS_FIELD_ID:
			field_l_shift = 16 - field_bit_size;
		 	break;

		default:
		 	ERR_PR("FieldID (%d) too large, field size=%d\n", field_nr, field_bit_size);
	 		return 1;
	}
	*value = (*value << field_l_shift);
	
	return 0;
}


/******************************************************************************
 *
 * Function   : build_c4_field_data_0_3_sysfs
 *              
 * Description: creates the C4 field 0..3 data sysfs command
 *              
 * Parameters :  
 *
 * Returns    : int
 *              
 ******************************************************************************/
static int build_c4_field_data_0_3_sysfs(xml_entry_data 	*c4_data,
					  unsigned int		rule_nr,
				 	  unsigned int		ruleset_nr)
{
	char 		sysfs_buf[512];
	unsigned int	i;
	unsigned int	field_nr;
	unsigned int	comp_data;
	DdEntry		*op_code;
	DdEntry		*field_id;
	
	/* handle field ids 0..4, with compare data word size */
	for (i = FIELD0ID_E, field_nr = 0; i < FIELD4ID_E; i += 3, field_nr++) {
		if (c4_data[i+1].xmlEntry == NULL ||
		    c4_data[i].xmlEntry == NULL)
		{
			/* field was not set, make it true by default */
			sprintf(sysfs_buf, "echo 0x%x 0 1     > %s/rule_params\n",
				field_nr,
				C4_SYSFS_PATH);
		 	handle_sysfs_command(sysfs_buf, false);
			continue;
		}
		
		if (!parseSimpleNumericValue(ezxml_txt(c4_data[i+2].xmlEntry), &comp_data)){
	 		ERR_PR("compare data :[%s] is not parseable\n", c4_data[i+2].name);
		 	return 1;
		}
			
		if (c4_data[i+1].xmlEntry == NULL) {
			ERR_PR("%s missing for op code\n", c4_data[i+2].name);
		 	return 1;
		}
	 	op_code = findMatchingEntry((char *)ezxml_txt(c4_data[i+1].xmlEntry));
		if (op_code == NULL) {
			ERR_PR("missing %s\n", c4_data[i+1].name);
			return 1;
		}
		field_id = findMatchingEntry((char *)ezxml_txt(c4_data[i].xmlEntry));
		if (field_id == NULL) {
			ERR_PR("missing %s\n", c4_data[i].name);
			return 1;
		}
		
		DEBUG_PR(DEB_SYSFS, "c4 rule field_id = %s(%s) comp_data = %d op_code = %s\n",
			ezxml_txt(c4_data[i].xmlEntry), field_id->value, comp_data, op_code->value);

		sprintf(sysfs_buf, "echo 0x%.1x 0x%.2x %.1s  > %s/rule_params\n",
			field_nr,
			atoi(field_id->value),
			op_code->value,
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		if (align_2_byte_field(atoi(field_id->value), &comp_data)) {
			ERR_PR("c4 rule field_id = %s(%s) does not fit in 2 Bytes\n",
				ezxml_txt(c4_data[i].xmlEntry), field_id->value);
			return 1;
		}

	 	sprintf(sysfs_buf, "echo %.1x %.1x 0x%.4x  > %s/rule_two_b\n",
			field_nr,
			0,
			comp_data,
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}
	return 0;
}


/******************************************************************************
 *
 * Function   : build_c4_field_data_4_5_sysfs
 *              
 * Description: creates the C4 field 4,5 data sysfs command
 *              
 * Parameters :  
 *
 * Returns    : int
 *              
 ******************************************************************************/
static int build_c4_field_data_4_5_sysfs(xml_entry_data	*c4_data,
					  unsigned int		rule_nr,
				 	  unsigned int		ruleset_nr)
{
	unsigned int 	field_id;
	unsigned int 	field_dst_size[6] = { 2, 2, 2, 2, 16, 6 };
	unsigned int	field_nr = (FIELD4ID_E - FIELD0ID_E) / 3;
	char 		sysfs_buf[512];
	char		data_str[512];
	char		*data_ptr = data_str;
	PncEntry_S	pnCEntry = { 0 };
	unsigned int	i;
	char		*field_name;
	char		*subfield_name;
	char		*subfield_value;
	RtSubFldEntry_S	*pRtSubFldEntry;
	DdEntry		*op_code;
	DdEntry		*field_id_entry;

	memset(&pnCEntry, 0, sizeof(pnCEntry));

	for (i = FIELD4ID_E; i < COLOR_ACTION_E; i += 3, field_nr++) {
		if (c4_data[i+1].xmlEntry == NULL ||
		    c4_data[i].xmlEntry == NULL)
		{
			/* field was not set, make it true by default */
			sprintf(sysfs_buf, "echo 0x%x 0 1     > %s/rule_params\n",
				field_nr,
				C4_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
			continue;
		}

		field_id = atoi(ezxml_txt(c4_data[i].xmlEntry));
	 	op_code = findMatchingEntry((char *)ezxml_txt(c4_data[i+1].xmlEntry));
		if (op_code == NULL) {
			ERR_PR("missing %s\n", c4_data[i+1].name);
			return 1;
		}
		
		field_id_entry = findMatchingEntry((char *)ezxml_txt(c4_data[i].xmlEntry));
		if (field_id_entry == NULL) {
			ERR_PR("missing %s\n", c4_data[i].name);
			return 1;
		}		

		DEBUG_PR(DEB_SYSFS, "c4 rule field_id = %s(%s) comp_data = %s op_code = %s\n",
			ezxml_txt(c4_data[i].xmlEntry), field_id_entry->name, 
			ezxml_txt(c4_data[i+2].xmlEntry), op_code->value);

		sprintf(sysfs_buf, "echo 0x%.1x 0x%.2x %.1s  > %s/rule_params\n",
			field_nr,
			atoi(field_id_entry->value),
			op_code->value,
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* get the TCAM data portion from the XML */
		strcpy(data_str, ezxml_txt(c4_data[i+2].xmlEntry));

		/* parse the TCAM string and fill in the fields */
		if (strlen(data_ptr)>0 && dealWithPacket((unsigned char*)data_ptr, &pnCEntry, NULL) == false)
			continue;
		
		/* get all parsed sub fields for this field */
		if (!getRtSubFld(&field_name, &subfield_name, &subfield_value, &field_id, &pRtSubFldEntry, 0)){
			DEBUG_PR(DEB_OTHER, "getRtSubFld could not find a field\n");
			continue;
	 	}

	 	/* the field id for this subfield is invalid */
		if (NO_FIELD_ID == field_id) {
			DEBUG_PR(DEB_OTHER, "invalid field id - %s %s %s\n", field_name, subfield_name, subfield_value);
			continue;
	 	}

		/* got a valid field id, search LU type field id for matching one */
		DEBUG_PR(DEB_SYSFS, "found field_id %d - %s %s\n",
			 field_id, field_name, subfield_name);

       	switch (field_id) {
			case OUT_VLAN_PRI_FIELD_ID:
			case OUT_VLAN_ID_FIELD_ID:
			case GEM_PORT_ID_FIELD_ID:
			case MH_UNTAGGED_PRI_FIELD_ID:
			case IN_VLAN_ID_FIELD_ID:
			case ETH_TYPE_FIELD_ID:
			case PPPOE_SESID_FIELD_ID:
			case PPPOE_PROTO_FIELD_ID:
			case IP_VER_FIELD_ID:
			case IPV4_DSCP_FIELD_ID:
			case IPV4_ECN_FIELD_ID:
		 	case IPV4_LEN_FIELD_ID:
			case IPV4_TTL_FIELD_ID:
			case IPV4_PROTO_FIELD_ID:
			//case IPV6_PROTO_FIELD_ID: same as IPv4
		 	case IPV6_DSCP_FIELD_ID:
			case IPV6_ECN_FIELD_ID:
			case IPV6_FLOW_LBL_FIELD_ID:
			case IPV6_PAYLOAD_LEN_FIELD_ID:
			case IPV6_NH_FIELD_ID:
			//case IPV6_HL_FIELD_ID: same as TTL
			case L4_SRC_FIELD_ID:
			case L4_DST_FIELD_ID:
			case TCP_FLAGS_FIELD_ID:
				if (align_2_byte_field(field_id, &(pRtSubFldEntry->parsedIntValue))) {
					ERR_PR("c4 rule field_id = %d(%s) does not fit in 2 Bytes\n",
						field_id, field_name);
					return 1;
				}
				build_c4_2_byte_sysfs(field_nr, pRtSubFldEntry->parsedIntValue);
				break;
				
			case IPV4_SA_FIELD_ID:
			case IPV4_DA_FIELD_ID:
				build_c4_ipv4_sysfs(field_nr,
						    pRtSubFldEntry->parsedIpAddress,
						    IPADDR_SIZE,
						    field_dst_size[field_nr]);
				break;

			case MAC_DA_FIELD_ID:
			case MAC_SA_FIELD_ID:
				build_c4_mac_sysfs(field_nr,
						   pRtSubFldEntry->parsedMacAddress,
						   MACADDR_SIZE,
						   field_dst_size[field_nr]);
			 	break;

		    	case IPV6_SA_FIELD_ID:
			case IPV6_DA_FIELD_ID:
				if (FIELD4ID_E != i) {
					ERR_PR("%s too large for %s\n",
						field_id_entry->name, c4_data[i+2].name);
		 			return 1;
				}

				build_c4_ipv6_sysfs(field_nr,
						   pRtSubFldEntry->parsedIpv6Address,
						   IPV6ADDR_SIZE,
						   field_dst_size[field_nr]);
				break;
				
		    	case IPV6_SA_PREF_FIELD_ID:
			case IPV6_SA_SUFF_FIELD_ID:
			case IPV6_DA_PREF_FIELD_ID:
			case IPV6_DA_SUFF_FIELD_ID:
				if (FIELD4ID_E != i) {
					ERR_PR("%s too large for %s\n",
						field_id_entry->name, c4_data[i+2].name);
		 			return 1;
				}
				build_c4_ipv6_sysfs(field_nr,
						   pRtSubFldEntry->parsedIpv6Address,
						   IPV6ADDR_SIZE/2,
						   field_dst_size[field_nr]);
				break;
				
			default:
				DEBUG_PR(DEB_OTHER, "skipping %s %s %s, no valid field id\n",
					field_name, subfield_name, subfield_value);
		}
	}

	return 0;
}


/******************************************************************************
 *
 * Function   : build_c4_other_fields_sysfs
 *              
 * Description: creates the C4 aditional fields sysfs command
 *              
 * Parameters :  
 *
 * Returns    : int
 *              
 ******************************************************************************/
static int build_c4_other_fields_sysfs(xml_entry_data		*c4_data,
					  unsigned int		rule_nr,
				 	  unsigned int		ruleset_nr)
{
	char 		sysfs_buf[512];
	
	{
		DdEntry		*color_action;
		
		if (c4_data[COLOR_ACTION_E].xmlEntry == NULL) {
			ERR_PR("%s is missing in ruleset:%d rule:%d \n", COLOR_ACTION, ruleset_nr, rule_nr);
			return 1;
		}
		
		color_action = findMatchingEntry((char *)ezxml_txt(c4_data[COLOR_ACTION_E].xmlEntry));
		if (color_action == NULL) {
			ERR_PR("missing %s\n", c4_data[COLOR_ACTION_E].name);
			return 1;
		}
		DEBUG_PR(DEB_SYSFS, "c4 rule field %s=%s\n", COLOR_ACTION, color_action->value);

		sprintf(sysfs_buf, "echo %s           > %s/act_sw_color\n",
			color_action->value,
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry		*pri_action;
		unsigned int	modified_pri = atoi(ezxml_txt(c4_data[PRI_VALUE_E].xmlEntry));

	 	if (c4_data[PRI_ACTION_E].xmlEntry == NULL) {
			ERR_PR("%s is missing in ruleset:%d rule:%d \n", PRI_ACTION, ruleset_nr, rule_nr);
			return 1;
		}
		pri_action = findMatchingEntry((char *)ezxml_txt(c4_data[PRI_ACTION_E].xmlEntry));
		if (pri_action == NULL) {
			ERR_PR("missing %s\n", c4_data[PRI_ACTION_E].name);
			return 1;
		}
		
		sprintf(sysfs_buf, "echo %s %x         > %s/act_sw_prio\n",
			pri_action->value,
			modified_pri,
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry 	*dscp_action;
		unsigned int 	modified_dscp = atoi(ezxml_txt(c4_data[DSCP_VALUE_E].xmlEntry));

	 	if (c4_data[DSCP_ACTION_E].xmlEntry == NULL) {
			ERR_PR("%s is missing in ruleset:%d rule:%d \n", DSCP_ACTION, ruleset_nr, rule_nr);
			return 1;
		}
		dscp_action = findMatchingEntry((char *)ezxml_txt(c4_data[DSCP_ACTION_E].xmlEntry));
		if (dscp_action == NULL) {
			ERR_PR("missing %s\n", c4_data[DSCP_ACTION_E].name);
			return 1;
		}
		
		sprintf(sysfs_buf, "echo %s 0x%.2x      > %s/act_sw_dscp\n",
			dscp_action->value,
			modified_dscp,
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry		*gem_portid_action;
		unsigned int	modified_gem_portid = atoi(ezxml_txt(c4_data[GEMPORTID_E].xmlEntry));

		if (c4_data[GEMPORTID_ACTION_E].xmlEntry == NULL) {
			ERR_PR("%s is missing in ruleset:%d rule:%d \n", GEMPORTID_ACTION, ruleset_nr, rule_nr);
			return 1;
		}
		gem_portid_action = findMatchingEntry((char *)ezxml_txt(c4_data[GEMPORTID_ACTION_E].xmlEntry));
		if (gem_portid_action == NULL) {
			ERR_PR("missing %s\n", c4_data[GEMPORTID_ACTION_E].name);
			return 1;
		}
		
		sprintf(sysfs_buf, "echo %s 0x%.3x     > %s/act_sw_gpid\n",
       		gem_portid_action->value,
			modified_gem_portid,
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry 	*q_low_action;
		DdEntry 	*q_high_action;
		unsigned int 	queue_h = atoi(ezxml_txt(c4_data[QUEUE_HIGH_VALUE_E].xmlEntry));
		unsigned int 	queue_l = atoi(ezxml_txt(c4_data[QUEUE_LOW_VALUE_E].xmlEntry));

		if (c4_data[QUEUE_LOW_ACTION_E].xmlEntry == NULL) {
			ERR_PR("%s is missing in ruleset:%d rule:%d \n", QUEUE_LOW_ACTION, ruleset_nr, rule_nr);
			return 1;
		}
		if (c4_data[QUEUE_HIGH_ACTION_E].xmlEntry == NULL) {
			ERR_PR("%s is missing in ruleset:%d rule:%d \n", QUEUE_HIGH_ACTION, ruleset_nr, rule_nr);
			return 1;
		}
		q_low_action = findMatchingEntry((char *)ezxml_txt(c4_data[QUEUE_LOW_ACTION_E].xmlEntry));
		if (q_low_action == NULL) {
			ERR_PR("missing %s\n", c4_data[QUEUE_LOW_ACTION_E].name);
			return 1;
		}
		q_high_action = findMatchingEntry((char *)ezxml_txt(c4_data[QUEUE_HIGH_ACTION_E].xmlEntry));
		if (q_high_action == NULL) {
			ERR_PR("missing %s\n", c4_data[QUEUE_HIGH_ACTION_E].name);
			return 1;
		}
		
		sprintf(sysfs_buf, "echo %s %.1x         > %s/act_sw_ql\n",
       		q_low_action->value,
			queue_l,
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
		
		sprintf(sysfs_buf, "echo %s %.2x        > %s/act_sw_qh\n",
			q_high_action->value,
			queue_h,
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *frw_action = findMatchingEntry((char *)ezxml_txt(c4_data[FORWARDING_E].xmlEntry));
		if (c4_data[FORWARDING_E].xmlEntry == NULL) {
			ERR_PR("%s is missing in ruleset:%d rule:%d \n", FORWARDING, ruleset_nr, rule_nr);
			return 1;
		}

		sprintf(sysfs_buf, "echo %s           > %s/act_sw_fwd\n",
			frw_action->value,
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}
	
	{
		DdEntry		*policer_action;
		unsigned int	policer_id = atoi(ezxml_txt(c4_data[POLICER_ID_E].xmlEntry));

		if (c4_data[POLICER_SELECT_E].xmlEntry == NULL)	{
			ERR_PR("%s is missing in ruleset:%d rule:%d \n", POLICER_SELECT, ruleset_nr, rule_nr);
			return 1;
		}
		policer_action = findMatchingEntry((char *)ezxml_txt(c4_data[POLICER_SELECT_E].xmlEntry));
		if (policer_action == NULL) {
			ERR_PR("missing %s\n", c4_data[POLICER_SELECT_E].name);
			return 1;
		}
		sprintf(sysfs_buf, "echo %s 0x%.2x 0x%.1x  > %s/act_sw_pol\n",
       		policer_action->value,
			policer_id & C4_POLICER_ID_MAX,
			policer_id >> C4_POLICER_ID_BITS,
			C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DEBUG_PR(DEB_SYSFS, "c4 rule field %s=%d\n", L3INFO, atoi(ezxml_txt(c4_data[L3INFO_E].xmlEntry)));

		if (c4_data[L3INFO_E].xmlEntry == NULL) {
			sprintf(sysfs_buf, "echo 1           > %s/rule_sw_l3\n", C4_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			sprintf(sysfs_buf, "echo 0x%.1x         > %s/rule_sw_l3\n",
				atoi(ezxml_txt(c4_data[L3INFO_E].xmlEntry)),
				C4_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
	}

	{
		DEBUG_PR(DEB_SYSFS, "c4 rule field %s=%d\n", L4INFO, atoi(ezxml_txt(c4_data[L4INFO_E].xmlEntry)));

		if (c4_data[L4INFO_E].xmlEntry == NULL) {
			sprintf(sysfs_buf, "echo 1           > %s/rule_sw_l4\n", C4_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			sprintf(sysfs_buf, "echo %x           > %s/rule_sw_l4\n",
				atoi(ezxml_txt(c4_data[L4INFO_E].xmlEntry)),
				C4_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
	}

	{
		DdEntry		*mtm;
		
		if (c4_data[MAC2ME_E].xmlEntry == NULL) {
			sprintf(sysfs_buf, "echo 1           > %s/rule_sw_mac\n", C4_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			mtm = findMatchingEntry((char *)ezxml_txt(c4_data[MAC2ME_E].xmlEntry));
			if (mtm == NULL) {
				ERR_PR("missing %s\n", c4_data[MAC2ME_E].name);
				return 1;
			}
			
			DEBUG_PR(DEB_SYSFS, "c4 rule field %s=%s\n", MAC2ME, mtm->value);
	 
			sprintf(sysfs_buf, "echo %s           > %s/rule_sw_mac\n",
				mtm->value,
				C4_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
	}

	{
		DdEntry *pppoe;
		
		if (c4_data[PPPOE_E].xmlEntry == NULL) {
			sprintf(sysfs_buf, "echo 1           > %s/rule_sw_pppoe\n", C4_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			
			pppoe = findMatchingEntry((char *)ezxml_txt(c4_data[PPPOE_E].xmlEntry));
			if (pppoe == NULL) {
				ERR_PR("missing %s\n", c4_data[PPPOE_E].name);
				return 1;
			}
			DEBUG_PR(DEB_SYSFS, "c4 rule field %s=%s\n", PPPOE, pppoe->value);
	 
			sprintf(sysfs_buf, "echo %s           > %s/rule_sw_pppoe\n",
				pppoe->value,
				C4_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
	}

	{
		DdEntry *vlan;
		if (c4_data[VLAN_E].xmlEntry == NULL) {
			sprintf(sysfs_buf, "echo 1           > %s/rule_sw_vlan\n", C4_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		} else {
			
			vlan = findMatchingEntry((char *)ezxml_txt(c4_data[VLAN_E].xmlEntry));
			if (vlan == NULL) {
				ERR_PR("missing %s\n", c4_data[VLAN_E].name);
				return 1;
			}
			DEBUG_PR(DEB_SYSFS, "c4 rule field %s=%s\n", VLAN, vlan->value);
	 
			sprintf(sysfs_buf, "echo 0x%.1x         > %s/rule_sw_vlan\n",
				atoi(vlan->value),
				C4_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
	}

	return 0;
}


/******************************************************************************
 *
 * Function   : build_c4_ruleset_sysfs
 *              
 * Description: creates C4 ruleset table
 *              
 * Parameters :  
 *
 * Returns    : int
 *              
 ******************************************************************************/
static int build_c4_ruleset_sysfs(xml_entry_data *c4_ruleset, char *xmlFile)
{
	char 		sysfs_buf[512];
	unsigned int	ruleset_nr;
	unsigned int	rules_nr;
	unsigned int	port_no;
	DdEntry 	*portid_type = NULL;

	if (c4_ruleset[RS_PORT_IDTYPE_E].xmlEntry == NULL) {
		ERR_PR("missing %s\n", PORT_IDTYPE);
		return 1;
	}

	portid_type = findMatchingEntry((char *)ezxml_txt(c4_ruleset[RS_PORT_IDTYPE_E].xmlEntry));
	if (portid_type == NULL) {
		ERR_PR("missing %s\n", PORT_IDTYPE);
		return 1;
	}
	
	port_no = atoi(ezxml_txt(c4_ruleset[RS_PORT_NO_E].xmlEntry));
	ruleset_nr = atoi(ezxml_txt(c4_ruleset[RS_RULESET_NO_E].xmlEntry));
	rules_nr = atoi(ezxml_txt(c4_ruleset[RS_RULES_IN_RULESET_E].xmlEntry));
	
	sprintf(sysfs_buf, "############  C4 ruleset table: port:%d ############\n",
		port_no);
	handle_sysfs_command(sysfs_buf, true);

	DEBUG_PR(DEB_SYSFS, "c4_ruleset port %d port_type %s ruleset_nr %d rules_nr %d\n",port_no,
		portid_type->value, ruleset_nr, rules_nr);

	sprintf(sysfs_buf, "echo %x %x %x  > %s/hw_%s_rules\n",
		port_no,
		ruleset_nr,
		rules_nr,
		C4_SYSFS_PATH,
		(0 == atoi((char*)portid_type->value)) ? "port" : "uni");
	handle_sysfs_command(sysfs_buf, false);
	
	return 0;
}


/******************************************************************************
 *
 * Function   : parse_xml_c4_ruleset
 *              
 * Description: parses the C4 ruleset excel sheet and builds sysfs commands
 *              
 * Parameters :  
 *
 * Returns    : int
 *              
 ******************************************************************************/
int parse_xml_c4_ruleset(char *xmlFile)
{
	ezxml_t     		xmlHead = NULL;
	ezxml_t     		xmlC4, xmlEntry;
	unsigned int		i;
	xml_entry_data		c4_ruleset[RS_C4_MAX] = {
		{PORT_NO, NULL},
	 	{PORT_IDTYPE, NULL},
		{RULESET_NO, NULL},
		{RULES_IN_RULESET, NULL} };

	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
        	return 1;
	}

	xmlC4 = ezxml_get(xmlHead, WORKSHEET_C4_RULESET, -1);
	if (xmlC4 == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_C4);
		ezxml_free(xmlHead);
       	return 1;
    	}

	xmlEntry = ezxml_child(xmlC4, TABLE_ENTRY);
	if (xmlEntry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n", WORKSHEET_C4);
		ezxml_free(xmlHead);
		return 0;
    	}
	
	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		for (i=0; i < RS_C4_MAX ;i++) {
			c4_ruleset[i].xmlEntry = ezxml_child(xmlEntry, c4_ruleset[i].name);

			DEBUG_PR(DEB_XML, "%s=%s\n", c4_ruleset[i].name, ezxml_txt(c4_ruleset[i].xmlEntry));
		}
		/* set the C4 action */
		if (build_c4_ruleset_sysfs(c4_ruleset, xmlFile)) {
			ezxml_free(xmlHead);
       		return 1;
		}
		for (i=0; i < RS_C4_MAX ;i++)
			c4_ruleset[i].xmlEntry = NULL;
	}

	ezxml_free(xmlHead);

	return 0;
}


/******************************************************************************
 *
 * Function   : parse_xml_c4
 *              
 * Description: parses the C4 excel sheet and builds sysfs commands
 *              
 * Parameters :  
 *
 * Returns    : int
 *              
 ******************************************************************************/
int parse_xml_c4(char *xmlFile)
{
	ezxml_t		xmlHead = NULL;
	ezxml_t		xmlC4, xmlEntry;
	unsigned int	i,
			rule_nr,
			ruleset_nr;
	char 		sysfs_buf[256];
	xml_entry_data	c4_data[C4_MAX_DATA_E] = {
		{RULESSET_NO, NULL}, {RULE_NO, NULL},
		{FIELD0ID, NULL}, {FIELD0OP_CODE, NULL}, {FIELD0COMPARE_DATA, NULL}, 
		{FIELD1ID, NULL}, {FIELD1OP_CODE, NULL}, {FIELD1COMPARE_DATA, NULL}, 
		{FIELD2ID, NULL}, {FIELD2OP_CODE, NULL}, {FIELD2COMPARE_DATA, NULL}, 
		{FIELD3ID, NULL}, {FIELD3OP_CODE, NULL}, {FIELD3COMPARE_DATA, NULL}, 
		{FIELD4ID, NULL}, {FIELD4OP_CODE, NULL}, {FIELD4COMPARE_DATA, NULL}, 
		{FIELD5ID, NULL}, {FIELD5OP_CODE, NULL}, {FIELD5COMPARE_DATA, NULL},
		{COLOR_ACTION, NULL},
		{PRI_ACTION, NULL},		{PRI_VALUE, NULL},
		{DSCP_ACTION, NULL},		{DSCP_VALUE, NULL},
		{GEMPORTID_ACTION, NULL},	{GEMPORTID, NULL},
		{QUEUE_LOW_ACTION,  NULL},	{QUEUE_LOW_VALUE, NULL},
		{QUEUE_HIGH_ACTION, NULL},	{QUEUE_HIGH_VALUE, NULL},
		{FORWARDING, NULL},		{POLICER_SELECT, NULL},		{POLICER_ID, NULL},
		{L3INFO, NULL}, {L4INFO, NULL}, {MAC2ME, NULL}, {PPPOE, NULL}, {VLAN, NULL} };
	
	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
        	return 1;
	}

	xmlC4 = ezxml_get(xmlHead, WORKSHEET_C4, -1);
	if (xmlC4 == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_C4);
		ezxml_free(xmlHead);
       	return 1;
    	}

	xmlEntry = ezxml_child(xmlC4, TABLE_ENTRY);
	if (xmlEntry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n", WORKSHEET_C4);
		ezxml_free(xmlHead);
		return 0;
    	}
	
	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		for (i=0; i < C4_MAX_DATA_E ;i++) {
			c4_data[i].xmlEntry = ezxml_child(xmlEntry, c4_data[i].name);

			DEBUG_PR(DEB_XML, "%s=%s\n", c4_data[i].name, 
				(c4_data[i].xmlEntry) ? ezxml_txt(c4_data[i].xmlEntry) : "<empty>");
		}

		ruleset_nr = atoi(ezxml_txt(c4_data[RULESSET_NO_E].xmlEntry));
		rule_nr = atoi(ezxml_txt(c4_data[RULE_NO_E].xmlEntry));

		sprintf(sysfs_buf, "############  C4 ruleset:%d rule:%d   ############\n",
			ruleset_nr, rule_nr);
		handle_sysfs_command(sysfs_buf, true);

		sprintf(sysfs_buf, "echo 1           > %s/sw_clear\n", C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		/* now parse the XML data and create the sysfs */
		if (build_c4_field_data_0_3_sysfs(c4_data, ruleset_nr, rule_nr)) {
			ezxml_free(xmlHead);
       		return 1;
		}
		if (build_c4_field_data_4_5_sysfs(c4_data, ruleset_nr, rule_nr)) {
			ezxml_free(xmlHead);
       		return 1;
		}
		if (build_c4_other_fields_sysfs(c4_data, ruleset_nr, rule_nr)) {
			ezxml_free(xmlHead);
       		return 1;
		}
		
		sprintf(sysfs_buf, "echo %d %d         > %s/hw_write\n",
			ruleset_nr, rule_nr, C4_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		for (i=0; i < C4_MAX_DATA_E ;i++)
			c4_data[i].xmlEntry = NULL;
	}

	ezxml_free(xmlHead);

	return 0;
}


