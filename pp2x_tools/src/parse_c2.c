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


enum xml_entry_data_C2{
	TCAM_INDEX_E,
	NAME_E,
	LU_TYPE_E,
	PORT_IDTYPE_E,
	PORT_ID_E,
	TCAM_DATA_E,
	QOS_TABLE_ID_E,
	DSCP_TABLE_SEL_E,
	PRI_DSCP_FROM_QOS_E,
	GEM_PORT_ID_FROM_QOS_E,
	QNO_LOW_FROM_QOS_E,
	QNO_HIGH_FROM_QOS_E,
	COLOR_FROM_QOS_E,
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
	RSS_ACTION_E,
	RSS_VALUE_E,
	HWFM_DPTR_E,
	HWFM_IPTR_E,
	HWF_L4_CHK_EN_E,
	MTU_INDEX_E,
	HW_DUPLICATION_FLOWID_E,
	HW_DUPLICATION_COUNT_E,
	ENTRY_ID_E,
	MISS_E,
	C2_MAX_DATA
};

enum xml_entry_data_C2_qos_tbl{
	QOS_TBL_INDEX_E,
	QOS_TBL_TABLE_NO_E,
	QOS_TBL_DSCP_E,
	QOS_TBL_PRI_E,
	QOS_TBL_COLOR_E,
	QOS_TBL_GEM_PORT_ID_E,
	QOS_TBL_QUEUE_NUMBER_E,
	QOS_TBL_MAX_E
};


#define QOS_TBL_DSCP		1
#define QOS_TBL_PRI		0

#if 0
/******************************************************************************
 *
 * Function   : get_prs_lutype_fields
 *
 * Description: return PRS sheet fields for specified LU ID
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
static int get_prs_lutype_fields(char 		*xmlFile,
				  unsigned int	lutype,
				  unsigned int	*num_of_fields,
				  unsigned int	*fields)
{
	ezxml_t		xmlHead = NULL;
	ezxml_t		xml_cls_flows, xmlEntry, xmlTmpEntry;
	DdEntry 	*cls_lutype;
	DdEntry 	*cls_field;
	unsigned int	i, input_field_len;
	char		*field_name[]={ FIELD1ID, FIELD2ID, FIELD3ID, FIELD4ID };

	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
        	return 1;
	}

	/* get Classifier sheet */
	xml_cls_flows = ezxml_get(xmlHead, WORKSHEET_CLS_FLOWS, -1);
	if (xml_cls_flows == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_CLS_FLOWS);
		ezxml_free(xmlHead);
       	return 1;
    	}

	xmlEntry = ezxml_child(xml_cls_flows, TABLE_ENTRY);
	if (xmlEntry == NULL){
		ERR_PR("Failed to get %s\n", TABLE_ENTRY);
		ezxml_free(xmlHead);
       	return 1;
    	}
	input_field_len = *num_of_fields;
	*num_of_fields = 0;

	/* search for same LU type as in C2 */
	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		xmlTmpEntry = ezxml_child(xmlEntry, LU_TYPE);
		if (NULL == xmlTmpEntry) {
			ERR_PR("Failed to get %s\n", LU_TYPE);
			ezxml_free(xmlHead);
			return 1;
		}

		cls_lutype = findMatchingEntry(ezxml_txt(xmlTmpEntry));
		if (NULL == cls_lutype) {
			ERR_PR("Failed to get dictionary entry %s for %s\n", ezxml_txt(xmlTmpEntry), LU_TYPE);
			ezxml_free(xmlHead);
			return 1;
		}

		/* found the lutype searching for */
		if (atoi((char*)cls_lutype->value) == lutype)
			break;
	}

	/* now need to get the fields from the xml */
	for (i = 0; i < input_field_len; i++) {
		xmlTmpEntry = ezxml_child(xmlEntry, field_name[i]);
		if (NULL == xmlTmpEntry) {
			/* no more fields */
			break;
		}

		cls_field = findMatchingEntry(ezxml_txt(xmlTmpEntry));
		if (NULL == cls_field) {
			ERR_PR("Failed to get %s for %s\n", ezxml_txt(xmlTmpEntry), field_name[i]);
			ezxml_free(xmlHead);
			return 1;
		}

		/* save the field in array */
		fields[i] = atoi((char*)cls_field->value);
		(*num_of_fields)++;

		DEBUG_PR(DEB_XML, "Found CLS field %s=%s lutype=%d\n", field_name[i], ezxml_txt(xmlTmpEntry), lutype);
	}

	ezxml_free(xmlHead);

	return 0;
}

/******************************************************************************
 *
 * Function   : build_c2_ipv4_sysfs
 *
 * Description: creates the tcam sysfs command for IPv4
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
static int build_c2_ipv4_sysfs(int		byte_cnt,
			 	unsigned char	*ip_addr,
    				unsigned char	*ip_addr_mask,
				unsigned int	field_size)
{
	char 		sysfs_buf[512];
	unsigned int	i;

	for (i = 0; i < field_size; i++, byte_cnt--){
		if (byte_cnt < 0) {
			ERR_PR("Negative byte count, filed id mismatch!\n");
			return 1;
		}
		sprintf(sysfs_buf, "echo %d 0x%.2x 0x%.2x > %s/act_sw_byte\n",
			byte_cnt,
			ip_addr[i],
			ip_addr_mask[i],
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	if (DEB_LEVEL == DEB_SYSFS){
		printf("IP address ");
	 	for (i = 0; i < field_size; i++)
			printf("%d.", ip_addr[i]);

		printf(" MASK: ");
		for (i = 0; i < field_size; i++)
			printf("%d.", ip_addr_mask[i]);

		printf("\n");
	}
	return 0;
}


/******************************************************************************
 *
 * Function   : build_c2_mac_sysfs
 *
 * Description: creates the tcam sysfs command for MAC address
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
static int build_c2_mac_sysfs(int		byte_cnt,
			       	unsigned char	*mac_addr,
    			       	unsigned char	*mac_addr_mask,
			       	unsigned int	field_size)
{
	char 		sysfs_buf[512];
	unsigned int	i;

	for (i = 0; i < field_size; i++, byte_cnt--){
		if (byte_cnt < 0) {
			ERR_PR("Negative byte count, filed id mismatch!\n");
			return 1;
		}
		
		sprintf(sysfs_buf, "echo %d 0x%.2x 0x%.2x > %s/act_sw_byte\n",
			byte_cnt,
			mac_addr[i],
			mac_addr_mask[i],
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	if (DEB_LEVEL == DEB_SYSFS){
		printf("MAC address: ");

	 	for (i = 0; i < field_size - 1; i++)
			printf("%.2X-", mac_addr[i]);
	 	printf("%.2X\n", mac_addr[i]);
		printf("MASK:        ");

		for (i = 0; i < field_size - 1; i++)
			printf("%.2X-", mac_addr_mask[i]);
	     	printf("%.2X", mac_addr_mask[i]);
		printf("\n");
	}
	return 0;
}

/******************************************************************************
 *
 * Function   : build_c2_simple_sysfs
 *
 * Description: creates the tcam sysfs command for a simple field
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
static int build_c2_simple_sysfs(int		byte_cnt,
			          unsigned int	value,
				  unsigned int	mask)
{
	char 		sysfs_buf[512];

	if (byte_cnt < 0) {
		ERR_PR("Negative byte count, filed id mismatch!\n");
		return 1;
    	}
	sprintf(sysfs_buf, "echo %d 0x%.2x 0x%.2x > %s/act_sw_byte\n",
		byte_cnt,
		(value & 0xff00) >> 8,
		(mask & 0xff00) >> 8,
		C2_SYSFS_PATH);
	byte_cnt--;
	handle_sysfs_command(sysfs_buf, false);

	sprintf(sysfs_buf, "echo %d 0x%.2x 0x%.2x > %s/act_sw_byte\n",
		byte_cnt,
		value & 0xff,
		mask & 0xff,
		C2_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	DEBUG_PR(DEB_SYSFS, "Simple data: %d mask:%d\n", value, mask);
	return 0;
}
#endif

#define CLS_FIELD_CNT	4
/******************************************************************************
 *
 * Function   : build_c2_tcam_data_sysfs
 *
 * Description: creates the tcam 0..3 (bytes 0..7) sysfs command
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
static int build_c2_tcam_data_sysfs(xml_entry_data	*c2_data,
				     char		*xmlFile,
				     unsigned int	lu_type)
{
	char		sysfs_buf[512];
	unsigned int	skip = 0;/* must initialized as 0 */
	char		c2_hek_str[512], *c2_hek_ptr;
	unsigned char	c2_hek[8], c2_hek_mask[8];
	PncEntry_S	pnCEntry;
	unsigned int 	field_id, pre_field_id = 0;
	unsigned char	field_id_num = 0;/* used to record number of field filled in HEK currently */
	unsigned int	c2_hek_bytes_used = 0;/* used to recoed current bytes filled in HEK */
	unsigned int 	i;
	unsigned int    field_id_array[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};

	memset(c2_hek, 0, 8 * sizeof(unsigned char));
	memset(c2_hek_mask, 0, 8 * sizeof(unsigned char));
	c2_hek_ptr = c2_hek_str;

	/* If header data empty while Num of HEK bytes isn't 0, skip */
	if (c2_data[TCAM_DATA_E].xmlEntry == NULL) {
		return PPV2_RC_OK;
	}
	/* Get C2 HEK str */
	strcpy(c2_hek_str, ezxml_txt(c2_data[TCAM_DATA_E].xmlEntry));

	/* Parse field ID and value */
	while ((strlen(c2_hek_ptr) > 0) && (dealWithPacket((unsigned char*)c2_hek_ptr, &pnCEntry, &skip) == true)) {
		char		*field_name;
		char		*subfield_name;
		char		*subfield_value;
		unsigned int 	indx = 0;
		RtSubFldEntry_S	*pRtSubFldEntry;
		unsigned int	field_bytes, left_bits;
		unsigned char   combine = 0, combine1 = 0;
		unsigned char   comb_offset = 0;

		/* get all parsed sub fields for this field */
		while (getRtSubFld(&field_name, &subfield_name, &subfield_value, &field_id, &pRtSubFldEntry, indx)) {
			/* the field id for this subfield is invalied, skip */
			if (NO_FIELD_ID == field_id) {
				ERR_PR("skip %d - %s %s %s\n", indx, field_name, subfield_name, subfield_value);
				indx++;
				continue;
			}

			/* Parse each field, from HEK35_32 to HEK3_0, and HEK[35] is the first byte filled */
			switch (field_id) {
			case MH_FIELD_ID:
			//case GEM_PORT_ID_FIELD_ID:
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

				/* Get HEK data and store it in c2_hek, each filed byte boutary */
				if (field_size[field_id] % 8)
					field_bytes = (field_size[field_id] / 8) + 1;
				else
					field_bytes = field_size[field_id] / 8;

				if (c2_hek_bytes_used >= 8 || (field_bytes > (8 - c2_hek_bytes_used))) {
					ERR_PR("HEK bytes (%d) beyond C2 capcity\n", (c2_hek_bytes_used + field_bytes));
					return PPV2_RC_FAIL;
				}

				for (i = 0; i < field_bytes; i++) {
					if (field_size[field_id] % 8 == 0) {
						c2_hek[c2_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValue & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 1 - i))) & 0xFF);
						c2_hek_mask[c2_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValueMask & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 1 - i))) & 0xFF);
					} else {
						if (i < (field_bytes - 1)) {
							c2_hek[c2_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValue & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 2 - i) + field_size[field_id] % 8)) & 0xFF);
							c2_hek_mask[c2_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValueMask & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 2 - i) + field_size[field_id] % 8)) & 0xFF);
						} else {
							c2_hek[c2_hek_bytes_used] = (unsigned char)((pRtSubFldEntry->parsedIntValue << (8 - field_size[field_id] % 8)) & 0xFF);
							c2_hek_mask[c2_hek_bytes_used] = (unsigned char)((pRtSubFldEntry->parsedIntValueMask << (8 - field_size[field_id] % 8)) & 0xFF);
						}
					}
					c2_hek_bytes_used++;
				}
				field_id_num++;
				break;
			/* Share bits combination */
			case GEM_PORT_ID_FIELD_ID:
			case IN_VLAN_ID_FIELD_ID:
			case OUT_VLAN_ID_FIELD_ID:
				if (pre_field_id == OUT_VLAN_PRI_FIELD_ID) {
					combine = 1;
					combine1 = 1;
					comb_offset = 4;
				}
			case IPV4_ECN_FIELD_ID:
				if (pre_field_id == IPV4_DSCP_FIELD_ID) {
					combine = 1;
					combine1 = 1;
					comb_offset = 2;
				}
			case IPV6_DSCP_FIELD_ID:
				if (pre_field_id == IP_VER_FIELD_ID) {
					combine = 1;
					combine1 = 1;
					comb_offset = 4;
				}
			case IPV6_ECN_FIELD_ID:
				if (pre_field_id == IPV6_DSCP_FIELD_ID) {
					combine = 1;
					combine1 = 1;
					comb_offset = 2;
				}
			case IPV6_FLOW_LBL_FIELD_ID:
				if (field_id == IPV6_FLOW_LBL_FIELD_ID && pre_field_id == IPV6_ECN_FIELD_ID) {
					combine = 1;
					combine1 = 1;
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

				/* Get HEK data and store it in c2_hek, each filed byte boutary */
				if (field_size[field_id] % 8)
					field_bytes = (field_size[field_id] / 8) + 1;
				else
					field_bytes = field_size[field_id] / 8;
				if (combine && (field_size[field_id] < 8) && ((field_size[field_id] + comb_offset) > 8))
					field_bytes++;

				if (c2_hek_bytes_used >= 8 || (field_bytes > (8 - c2_hek_bytes_used))) {
					ERR_PR("HEK bytes (%d) beyond C2 capcity\n", (c2_hek_bytes_used + field_bytes));
					return PPV2_RC_FAIL;
				}

				left_bits = field_size[field_id];

				for (i = 0; i < field_bytes; i++) {
					if (combine1) {
						if (combine) {
							c2_hek_bytes_used--;

							c2_hek[c2_hek_bytes_used] |= ((unsigned char)(((pRtSubFldEntry->parsedIntValue >> (field_size[field_id] - comb_offset)) & common_mask_gen(comb_offset)) & 0xFF));
							c2_hek_mask[c2_hek_bytes_used] |= ((unsigned char)(((pRtSubFldEntry->parsedIntValueMask >> (field_size[field_id] - comb_offset)) & common_mask_gen(comb_offset)) & 0xFF));
							if (((field_size[field_id] % 8) + comb_offset) > 8 || (field_size[field_id] > 8)) {
								pRtSubFldEntry->parsedIntValue &= common_mask_gen(field_size[field_id] - comb_offset);
								pRtSubFldEntry->parsedIntValueMask &= common_mask_gen(field_size[field_id] - comb_offset);
							}
							c2_hek_bytes_used++;
							left_bits = field_size[field_id] - comb_offset;
							combine = 0;
						} else {
							if (left_bits % 8 == 0) {
								c2_hek[c2_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValue & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 1 - i))) & 0xFF);
								c2_hek_mask[c2_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValueMask & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 1 - i))) & 0xFF);
							} else {
								if (i < (field_bytes - 1)) {
									c2_hek[c2_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValue & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 2 - i) + left_bits % 8)) & 0xFF);
									c2_hek_mask[c2_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValueMask & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 2 - i) + left_bits % 8)) & 0xFF);
								} else {
									c2_hek[c2_hek_bytes_used] = (unsigned char)((pRtSubFldEntry->parsedIntValue << (8 - left_bits % 8)) & 0xFF);
									c2_hek_mask[c2_hek_bytes_used] = (unsigned char)((pRtSubFldEntry->parsedIntValueMask << (8 - left_bits % 8)) & 0xFF);
								}
							}
							c2_hek_bytes_used++;
						}
						combine1 = 0;
					} else {
						c2_hek[c2_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValue & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 1 - i))) & 0xFF);
						c2_hek_mask[c2_hek_bytes_used] = (unsigned char)(((pRtSubFldEntry->parsedIntValueMask & common_mask_gen(field_size[field_id])) >> (8 * (field_bytes - 1 - i))) & 0xFF);
						c2_hek_bytes_used++;
					}
				}
				field_id_num++;
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

				if (field_id == MAC_DA_FIELD_ID || field_id == MAC_SA_FIELD_ID)
					field_bytes = MACADDR_SIZE;
				else
					field_bytes = IPADDR_SIZE;

				if (c2_hek_bytes_used >= 8 || (field_bytes > (8 - c2_hek_bytes_used))) {
					ERR_PR("HEK bytes (%d) beyond C2 capcity\n", (c2_hek_bytes_used + field_bytes));
					return PPV2_RC_FAIL;
				}

				for (i = 0;i < field_bytes; i++) {
					if (field_id == MAC_DA_FIELD_ID || field_id == MAC_SA_FIELD_ID) {
						c2_hek[c2_hek_bytes_used] = pRtSubFldEntry->parsedMacAddress[i];
						c2_hek_mask[c2_hek_bytes_used] = pRtSubFldEntry->parsedMacAddressMask[i];
					} else {
						c2_hek[c2_hek_bytes_used] = pRtSubFldEntry->parsedIpAddress[i];
						c2_hek_mask[c2_hek_bytes_used] = pRtSubFldEntry->parsedIpAddressMask[i];
					}
					c2_hek_bytes_used++;
				}
				field_id_num++;
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

				if (field_id == IPV6_SA_FIELD_ID || field_id == IPV6_DA_FIELD_ID)
					field_bytes = IPV6ADDR_SIZE;
				else
					field_bytes = IPV6ADDR_SIZE / 2;

				if (c2_hek_bytes_used >= 8 || (field_bytes > (8 - c2_hek_bytes_used))) {
					ERR_PR("HEK bytes (%d) beyond C2 capcity\n", (c2_hek_bytes_used + field_bytes));
					return PPV2_RC_FAIL;
				}

				for (i = 0; i < (field_bytes / 2); i++) {
					/* High byte */
					c2_hek[c2_hek_bytes_used] = (unsigned char) (pRtSubFldEntry->parsedIpv6Address[i] >> 8);
					c2_hek_mask[c2_hek_bytes_used] = (unsigned char) (pRtSubFldEntry->parsedIpv6AddressMask[i] >> 8);
					c2_hek_bytes_used++;
					/* Low byte */
					c2_hek[c2_hek_bytes_used] = (unsigned char) (pRtSubFldEntry->parsedIpv6Address[i] & 0xFF);
					c2_hek_mask[c2_hek_bytes_used] = (unsigned char) (pRtSubFldEntry->parsedIpv6AddressMask[i] & 0xFF);
					c2_hek_bytes_used++;
				}
				field_id_num++;
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
		c2_hek_ptr = &c2_hek_str[skip];
	}

	/* Generate sysfs commands */
	for (i = 0; i < c2_hek_bytes_used; i++) {
		sprintf(sysfs_buf, "echo %d 0x%.2x 0x%.2x > %s/act_sw_byte\n",
			(7 - i),
			c2_hek[i],
			c2_hek_mask[i],
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	return PPV2_RC_OK;
}



/******************************************************************************
 *
 * Function   : build_c2_tcam_sysfs
 *
 * Description: creates the tcam 0..4 sysfs command
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
static int build_c2_tcam_sysfs(xml_entry_data *c2_data, char *xmlFile)
{
	char 		sysfs_buf[512];
	unsigned int	tcam_idx;
	unsigned int	word_data = 0;
	unsigned int	word_mask = 0;
	DdEntry 	*lu_type = NULL;
	DdEntry 	*portid_type = NULL;
	unsigned int	portid;

	tcam_idx = atoi(ezxml_txt(c2_data[TCAM_INDEX_E].xmlEntry));

	DEBUG_PR(DEB_SYSFS, "tcam %d %s=%s %s=%s\n",tcam_idx,
		LU_TYPE, ezxml_txt(c2_data[LU_TYPE_E].xmlEntry),
		PORT_IDTYPE, ezxml_txt(c2_data[PORT_IDTYPE_E].xmlEntry));

	if (c2_data[LU_TYPE_E].xmlEntry != NULL) {
		lu_type = findMatchingEntry(ezxml_txt(c2_data[LU_TYPE_E].xmlEntry));
		if (lu_type == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", LU_TYPE, tcam_idx);
			return 1;
		}
	}

	if (c2_data[PORT_IDTYPE_E].xmlEntry != NULL) {	
		portid_type = findMatchingEntry(ezxml_txt(c2_data[PORT_IDTYPE_E].xmlEntry));
		if (portid_type == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", LU_TYPE, tcam_idx);
			return 1;
		}
	}
	if (c2_data[PORT_ID_E].xmlEntry != NULL)
		portid = atoi(ezxml_txt(c2_data[PORT_ID_E].xmlEntry));

	if (c2_data[LU_TYPE_E].xmlEntry != NULL) {
		word_data |= (atoi((char*)lu_type->value) & 0x3F);
		word_mask |= 0x003F;
	} else
		word_mask &= 0xFFC0;

	if (c2_data[PORT_IDTYPE_E].xmlEntry != NULL) {
		word_data |= (atoi((char*)portid_type->value) & 0x3) << 6;
		word_mask |= 0x00C0;
	} else
		word_mask &= 0xFF3F;

	if (c2_data[PORT_ID_E].xmlEntry != NULL) {
		word_data |= (portid & 0xFF) << 8;
		word_mask |= 0xFF00;
	} else
		word_mask &= 0x00FF;

	/* insert the two bytes of TcamData4/TcamEnable4/ */
	sprintf(sysfs_buf, "echo 8 0x%.2x 0x%.2x > %s/act_sw_byte\n",
		word_data & 0xff,
		word_mask & 0xff,
		C2_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	sprintf(sysfs_buf, "echo 9 0x%.2x 0x%.2x > %s/act_sw_byte\n",
		(word_data & 0xff00) >> 8,
		(word_mask & 0xff00) >> 8,
		C2_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	if (build_c2_tcam_data_sysfs(c2_data, xmlFile, atoi((char*)lu_type->value))) {
		ERR_PR("build_c2_tcam_data_sysfs returned an error\n");
		return 1;
	}

	sprintf(sysfs_buf, "echo 0x%02x        > %s/act_hw_write\n",
		tcam_idx,
		C2_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	return 0;
}


/******************************************************************************
 *
 * Function   : build_c2_action_sysfs
 *
 * Description: creates the QoS tables
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
static int build_c2_action_sysfs(xml_entry_data *c2_data, char *xmlFile)
{
	char sysfs_buf[512];
	unsigned int tcam_idx;

	tcam_idx = atoi(ezxml_txt(c2_data[TCAM_INDEX_E].xmlEntry));

	sprintf(sysfs_buf, "############  C2 config: TCAM:%d [%s]  ############\n",
		tcam_idx, ezxml_txt(c2_data[NAME_E].xmlEntry));
	handle_sysfs_command(sysfs_buf, true);

	sprintf(sysfs_buf, "echo 1           > %s/act_sw_clear\n", C2_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	{
		unsigned int qos_tbl_idx = atoi(ezxml_txt(c2_data[QOS_TABLE_ID_E].xmlEntry));
		DdEntry *dscp_tbl_sel = findMatchingEntry((char *)ezxml_txt(c2_data[DSCP_TABLE_SEL_E].xmlEntry));

		if (c2_data[DSCP_TABLE_SEL_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", DSCP_TABLE_SEL, tcam_idx);
			return 1;
		}

		sprintf(sysfs_buf, "echo %x %s         > %s/act_sw_qos\n",
			qos_tbl_idx,
			dscp_tbl_sel->value,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *color_action = findMatchingEntry((char *)ezxml_txt(c2_data[COLOR_ACTION_E].xmlEntry));
		DdEntry *color_from_qos = findMatchingEntry((char *)ezxml_txt(c2_data[COLOR_FROM_QOS_E].xmlEntry));
		if (c2_data[COLOR_ACTION_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", COLOR_ACTION, tcam_idx);
			return 1;
		}

		sprintf(sysfs_buf, "echo %s %s         > %s/act_sw_color\n",
			color_action->value,
			color_from_qos->value,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *pri_action = findMatchingEntry((char *)ezxml_txt(c2_data[PRI_ACTION_E].xmlEntry));
		unsigned int modified_pri = atoi(ezxml_txt(c2_data[PRI_VALUE_E].xmlEntry));
		DdEntry *pri_from_qos = findMatchingEntry((char *)ezxml_txt(c2_data[PRI_DSCP_FROM_QOS_E].xmlEntry));

		if (c2_data[PRI_DSCP_FROM_QOS_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", PRI_DSCP_FROM_QOS, tcam_idx);
			return 1;
		}

	 	if (c2_data[PRI_ACTION_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", PRI_ACTION, tcam_idx);
			return 1;
		}

		sprintf(sysfs_buf, "echo %s %x %s       > %s/act_sw_prio\n",
			pri_action->value,
			modified_pri,
			pri_from_qos->value,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *dscp_action = findMatchingEntry((char *)ezxml_txt(c2_data[DSCP_ACTION_E].xmlEntry));
		unsigned int modified_dscp = atoi(ezxml_txt(c2_data[DSCP_VALUE_E].xmlEntry));
		DdEntry *dscp_from_qos = findMatchingEntry((char *)ezxml_txt(c2_data[PRI_DSCP_FROM_QOS_E].xmlEntry));

		if (c2_data[PRI_DSCP_FROM_QOS_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", PRI_DSCP_FROM_QOS, tcam_idx);
			return 1;
		}

	 	if (c2_data[DSCP_ACTION_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", DSCP_ACTION, tcam_idx);
			return 1;
		}

		sprintf(sysfs_buf, "echo %s %x %s       > %s/act_sw_dscp\n",
			dscp_action->value,
			modified_dscp,
			dscp_from_qos->value,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *gem_portid_action = findMatchingEntry((char *)ezxml_txt(c2_data[GEMPORTID_ACTION_E].xmlEntry));
		unsigned int modified_gem_portid = atoi(ezxml_txt(c2_data[GEMPORTID_E].xmlEntry));
		DdEntry *gem_from_qos = findMatchingEntry((char *)ezxml_txt(c2_data[GEM_PORT_ID_FROM_QOS_E].xmlEntry));

		if (c2_data[GEMPORTID_ACTION_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", GEMPORTID_ACTION, tcam_idx);
			return 1;
		}
		sprintf(sysfs_buf, "echo %s %x %s       > %s/act_sw_gpid\n",
       		gem_portid_action->value,
			modified_gem_portid,
			gem_from_qos->value,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *q_low_action = findMatchingEntry((char *)ezxml_txt(c2_data[QUEUE_LOW_ACTION_E].xmlEntry));
		DdEntry *q_high_action = findMatchingEntry((char *)ezxml_txt(c2_data[QUEUE_HIGH_ACTION_E].xmlEntry));
		unsigned int queue_h = atoi(ezxml_txt(c2_data[QUEUE_HIGH_VALUE_E].xmlEntry));
		unsigned int queue_l = atoi(ezxml_txt(c2_data[QUEUE_LOW_VALUE_E].xmlEntry));
		DdEntry *queue_l_from_qos = findMatchingEntry((char *)ezxml_txt(c2_data[QNO_LOW_FROM_QOS_E].xmlEntry));
		DdEntry *queue_h_from_qos = findMatchingEntry((char *)ezxml_txt(c2_data[QNO_HIGH_FROM_QOS_E].xmlEntry));

		if (c2_data[QUEUE_LOW_ACTION_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", QUEUE_LOW_ACTION, tcam_idx);
			return 1;
		}
		if (c2_data[QUEUE_HIGH_ACTION_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", QUEUE_HIGH_ACTION, tcam_idx);
			return 1;
		}
		sprintf(sysfs_buf, "echo %s %x %s       > %s/act_sw_ql\n",
       		q_low_action->value,
			queue_l,
			queue_l_from_qos->value,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
		
		sprintf(sysfs_buf, "echo %s %x %s       > %s/act_sw_qh\n",
			q_high_action->value,
			queue_h,
			queue_h_from_qos->value,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *frw_action = findMatchingEntry((char *)ezxml_txt(c2_data[FORWARDING_E].xmlEntry));
		if (c2_data[FORWARDING_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", FORWARDING, tcam_idx);
			return 1;
		}

		sprintf(sysfs_buf, "echo %s           > %s/act_sw_hwf\n",
			frw_action->value,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *policer_action = findMatchingEntry((char *)ezxml_txt(c2_data[POLICER_SELECT_E].xmlEntry));
		unsigned int policer_id = atoi(ezxml_txt(c2_data[POLICER_ID_E].xmlEntry));

		if (c2_data[POLICER_SELECT_E].xmlEntry == NULL)	{
			ERR_PR("%s is missing for tcam index %d\n", POLICER_SELECT, tcam_idx);
			return 1;
		}
		sprintf(sysfs_buf, "echo %s 0x%.2x 0x%.1x  > %s/act_sw_pol\n",
       		policer_action->value,
			policer_id & C2_POLICER_ID_MAX,
			policer_id >> C2_POLICER_ID_BITS,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		DdEntry *rss_action = findMatchingEntry((char *)ezxml_txt(c2_data[RSS_ACTION_E].xmlEntry));
		DdEntry *rss_en = findMatchingEntry((char *)ezxml_txt(c2_data[RSS_VALUE_E].xmlEntry));

		if (c2_data[RSS_ACTION_E].xmlEntry == NULL)	{
			ERR_PR("%s is missing for tcam index %d\n", RSS_ENABLE_ACT, tcam_idx);
			return 1;
		}
		if (c2_data[RSS_VALUE_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", RSS_ENABLE_ATTR, tcam_idx);
			return 1;
		}
		sprintf(sysfs_buf, "echo %s %s  > %s/act_sw_rss\n",
       			rss_action->value,
			rss_en->value,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		unsigned int mtu_index = atoi(ezxml_txt(c2_data[MTU_INDEX_E].xmlEntry));

		if (c2_data[MTU_INDEX_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", MTU_INDEX, tcam_idx);
			return 1;
		}
		sprintf(sysfs_buf, "echo 0x%02x        > %s/act_sw_mtu\n",
			mtu_index,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}
	
	{
		DdEntry *hwf_l4_chk_en = findMatchingEntry((char *)ezxml_txt(c2_data[HWF_L4_CHK_EN_E].xmlEntry));
		unsigned int hwfm_dptr = atoi(ezxml_txt(c2_data[HWFM_DPTR_E].xmlEntry));
		unsigned int hwfm_iptr = atoi(ezxml_txt(c2_data[HWFM_IPTR_E].xmlEntry));

		if (c2_data[HWF_L4_CHK_EN_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", HWF_L4_CHK_EN, tcam_idx);
			return 1;
		}
		sprintf(sysfs_buf, "echo %x %x %s       > %s/act_sw_mdf\n",
			hwfm_dptr,
			hwfm_iptr,
			hwf_l4_chk_en->value,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	{
		unsigned int dup_flowid = atoi(ezxml_txt(c2_data[HW_DUPLICATION_FLOWID_E].xmlEntry));
		unsigned int dup_count = atoi(ezxml_txt(c2_data[HW_DUPLICATION_COUNT_E].xmlEntry));

		if (dup_count > 0) {
			sprintf(sysfs_buf, "echo %x %x         > %s/act_sw_dup\n",
				dup_flowid,
				dup_count,
				C2_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
	}

	{
		    DdEntry *miss = findMatchingEntry((char *)ezxml_txt(c2_data[MISS_E].xmlEntry));
		    unsigned int entry_id = atoi(ezxml_txt(c2_data[ENTRY_ID_E].xmlEntry));

		    if (c2_data[ENTRY_ID_E].xmlEntry == NULL) {
			    ERR_PR("%s is missing for tcam index %d\n", ENTRY_ID, tcam_idx);
			    return 1;
		    }
		    sprintf(sysfs_buf, "echo %s %x         > %s/act_sw_sq\n",
			    miss->value,
			    entry_id,
			    C2_SYSFS_PATH);
		    handle_sysfs_command(sysfs_buf, false);
	}

	return 0;
}


/******************************************************************************
 *
 * Function   : build_c2_qos_sysfs
 *
 * Description: creates the QoS tables
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
static int build_c2_qos_sysfs(xml_entry_data *c2_data, char *xmlFile, bool dscp)
{
	char sysfs_buf[512];
	unsigned int idx;

	idx = atoi(ezxml_txt(c2_data[QOS_TBL_INDEX_E].xmlEntry));

	sprintf(sysfs_buf, "############  C2 %s QoS table: %d  ############\n",
		(dscp) ? "DSCP":"PRI", idx);
	handle_sysfs_command(sysfs_buf, true);

	sprintf(sysfs_buf, "echo 1          > %s/qos_sw_clear\n", C2_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	{
		DdEntry *color = findMatchingEntry((char *)ezxml_txt(c2_data[QOS_TBL_COLOR_E].xmlEntry));
		unsigned int tbl_no = atoi(ezxml_txt(c2_data[QOS_TBL_TABLE_NO_E].xmlEntry));
		unsigned int dscp_value = atoi(ezxml_txt(c2_data[QOS_TBL_DSCP_E].xmlEntry));
		unsigned int pri = atoi(ezxml_txt(c2_data[QOS_TBL_PRI_E].xmlEntry));
		unsigned int q_num = 0;
		unsigned int gem_portid = atoi(ezxml_txt(c2_data[QOS_TBL_GEM_PORT_ID_E].xmlEntry));

		/* Get queue number */
		if (parseSimpleNumericValue((unsigned char*)ezxml_txt(c2_data[QOS_TBL_QUEUE_NUMBER_E].xmlEntry), &q_num) == false) {
			ERR_PR("%s is not valid value\n", ezxml_txt(c2_data[QOS_TBL_QUEUE_NUMBER_E].xmlEntry));
			return 1;
		}

	 	if (c2_data[QOS_TBL_COLOR_E].xmlEntry == NULL) {
			ERR_PR("%s is missing for tcam index %d\n", COLOR, idx);
			return 1;
		}

		sprintf(sysfs_buf, "echo %x          > %s/qos_sw_dscp\n",
			dscp_value,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		sprintf(sysfs_buf, "echo %x          > %s/qos_sw_prio\n",
			pri,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		sprintf(sysfs_buf, "echo %s          > %s/qos_sw_color\n",
			color->value,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		sprintf(sysfs_buf, "echo %x          > %s/qos_sw_gemid\n",
			gem_portid,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		sprintf(sysfs_buf, "echo %x          > %s/qos_sw_queue\n",
			q_num,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);

		sprintf(sysfs_buf, "echo %x %x %x      > %s/qos_hw_write\n",
			tbl_no,
			(dscp) ? 1 : 0,
			idx,
			C2_SYSFS_PATH);
		handle_sysfs_command(sysfs_buf, false);
	}

	return 0;
}


/******************************************************************************
 *
 * Function   : parse_xml_c2
 *
 * Description: parses the C2 excel sheet and builds sysfs commands
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
int parse_xml_c2(char *xmlFile)
{
	ezxml_t     		xmlHead = NULL;
	ezxml_t     		xmlC2, xmlEntry;
	xml_entry_data	c2_data[C2_MAX_DATA] = {
		{TCAM_INDEX, NULL}, 		{NAME, NULL},			{LU_TYPE, NULL},
		{PORT_IDTYPE, NULL},		{PORT_ID, NULL},		{TCAM_DATA, NULL},
		{QOS_TABLE_ID, NULL},	 	{DSCP_TABLE_SEL, NULL},	 	{PRI_DSCP_FROM_QOS, NULL},
		{GEM_PORT_ID_FROM_QOS, NULL},	{QNO_LOW_FROM_QOS, NULL}, 	{QNO_HIGH_FROM_QOS, NULL},
		{COLOR_FROM_QOS, NULL},		{COLOR_ACTION,  NULL},  	{PRI_ACTION, NULL},
		{PRI_VALUE, NULL},		{DSCP_ACTION, NULL},		{DSCP_VALUE, NULL},
		{GEMPORTID_ACTION, NULL},	{GEMPORTID, NULL},		{QUEUE_LOW_ACTION,  NULL},
		{QUEUE_LOW_VALUE, NULL},	{QUEUE_HIGH_ACTION, NULL},	{QUEUE_HIGH_VALUE, NULL},
		{FORWARDING,  NULL},		{POLICER_SELECT, NULL},		{POLICER_ID, NULL},
		{RSS_ENABLE_ACT, NULL},		{RSS_ENABLE_ATTR, NULL},
		{HWFM_DPTR, NULL},		{HWFM_IPTR, NULL},      	{HWF_L4_CHK_EN, NULL},
		{MTU_INDEX, NULL},		{HW_DUPLICATION_FLOWID, NULL},	{HW_DUPLICATION_COUNT, NULL},
		{ENTRY_ID, NULL},		{MISS, NULL} };
	unsigned int	i;

	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
        	return 1;
	}

	xmlC2 = ezxml_get(xmlHead, WORKSHEET_C2, -1);
	if (xmlC2 == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_C2);
		ezxml_free(xmlHead);
       	return 1;
    	}

	xmlEntry = ezxml_child(xmlC2, TABLE_ENTRY);
	if (xmlEntry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n", WORKSHEET_C2);
		ezxml_free(xmlHead);
		return 0;
    	}

	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		for (i=0; i < C2_MAX_DATA ;i++) {
			c2_data[i].xmlEntry = ezxml_child(xmlEntry, c2_data[i].name);

			DEBUG_PR(DEB_XML, "%s=%s\n", c2_data[i].name, ezxml_txt(c2_data[i].xmlEntry))
		}
		/* set the C2 action */
		if (build_c2_action_sysfs(c2_data, xmlFile)) {
			ezxml_free(xmlHead);
			return 1;
		}

		/* set the C2 TCAM */
		if (build_c2_tcam_sysfs(c2_data, xmlFile)) {
			ezxml_free(xmlHead);
			return 1;
		}
		for (i=0; i < C2_MAX_DATA ;i++)
			c2_data[i].xmlEntry = NULL;
	}

	ezxml_free(xmlHead);

	return 0;
}


/******************************************************************************
 *
 * Function   : parse_xml_c2_qos
 *
 * Description: parses the C2 PRI and DSCP excel sheets and builds sysfs commands
 *
 * Parameters :
 *
 * Returns    : int
 *
 ******************************************************************************/
static int parse_xml_c2_qos(char *xmlFile, bool dscp)
{
	ezxml_t     		xmlHead = NULL;
	ezxml_t     		xmlC2, xmlEntry;
	xml_entry_data		c2_qos_data[QOS_TBL_MAX_E] = {
		{INDEX, NULL},
		{TABLE_NO, NULL},
		{DSCP, NULL},
		{PRI, NULL},
		{COLOR, NULL},
		{GEM_PORT_ID, NULL},
		{QUEUE_NUMBER, NULL} };
	unsigned int	i;

	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
        	return 1;
	}

	/* the two QoS sheets have the same fields, only one fields differs */
	xmlC2 = ezxml_get(xmlHead, (dscp) ? WORKSHEET_C2_DSCP : WORKSHEET_C2_PRI , -1);

	if (xmlC2 == NULL){
		ERR_PR("Failed to get %s\n", (dscp) ? WORKSHEET_C2_DSCP : WORKSHEET_C2_PRI);
		ezxml_free(xmlHead);
		return 1;
	}

	xmlEntry = ezxml_child(xmlC2, TABLE_ENTRY);
	if (xmlEntry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n",
			(dscp) ? WORKSHEET_C2_DSCP : WORKSHEET_C2_PRI);
		ezxml_free(xmlHead);
		return 0;
    	}

	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		DEBUG_PR(DEB_XML, "%s ENTRY\n", (dscp) ? "DSCP" : "PRI")

		for (i=0; i < QOS_TBL_MAX_E ;i++) {
			c2_qos_data[i].xmlEntry = ezxml_child(xmlEntry, c2_qos_data[i].name);

			DEBUG_PR(DEB_XML, "%s=%s\n", c2_qos_data[i].name, ezxml_txt(c2_qos_data[i].xmlEntry))
		}

		/* build the sysfs commands */
		if (build_c2_qos_sysfs(c2_qos_data, xmlFile, dscp)) {
			ezxml_free(xmlHead);
       		return 1;
		}
		for (i=0; i < QOS_TBL_MAX_E ;i++)
			c2_qos_data[i].xmlEntry = NULL;
	}

	ezxml_free(xmlHead);

	return 0;
}

int parse_xml_c2_qos_dscp(char *xmlFile)
{
	return parse_xml_c2_qos(xmlFile,QOS_TBL_DSCP);
}

int parse_xml_c2_qos_pri(char *xmlFile)
{
	return parse_xml_c2_qos(xmlFile,QOS_TBL_PRI);
}
