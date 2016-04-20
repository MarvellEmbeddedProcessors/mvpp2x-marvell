/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PPv2 Tool                                                  **/
/**                                                                          **/
/**  FILE        : parse_PRS.c                                               **/
/**                                                                          **/
/**  DESCRIPTION : This file contains parse of the parse init               **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    23-Oct-12   Evan  - initial version created.                              
 *                                                                      
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <errno.h>

#include "common.h"
#include "ezxml.h"
#include "DataDictionary.h"
#include "xml_params.h"
#include "parse_PRS.h"
#include "PncGlobals.h"
#include "ParseUtils.h"

/* Global field size array */
unsigned int field_size[PPV2_FIELD_COUNT] = {
	MH_FIELD_SIZE,
	GEM_PORT_ID_FIELD_SIZE,
	MH_UNTAGGED_PRI_FIELD_SIZE,
	MAC_DA_FIELD_SIZE,
	MAC_SA_FIELD_SIZE,
	OUT_VLAN_PRI_FIELD_SIZE,
	OUT_VLAN_ID_FIELD_SIZE,
	IN_VLAN_ID_FIELD_SIZE,
	ETH_TYPE_FIELD_SIZE,
	PPPOE_SESID_FIELD_SIZE,
	IP_VER_FIELD_SIZE,
	IPV4_DSCP_FIELD_SIZE,
	IPV4_ECN_FIELD_SIZE,
	IPV4_LEN_FIELD_SIZE,
	IPV4_TTL_FIELD_SIZE,/*IPV6_HL_FIELD_SIZE*/
	IPV4_PROTO_FIELD_SIZE,/*IPV6_PROTO_FIELD_SIZE*/
	IPV4_SA_FIELD_SIZE,
	IPV4_DA_FIELD_SIZE,
	/*IPV6_PROTO_FIELD_SIZE,*/
	IPV6_DSCP_FIELD_SIZE,
	IPV6_ECN_FIELD_SIZE,
	IPV6_FLOW_LBL_FIELD_SIZE,
	IPV6_PAYLOAD_LEN_FIELD_SIZE,
	IPV6_NH_FIELD_SIZE,
	/*IPV6_HL_FIELD_SIZE,*/
	IPV6_SA_FIELD_SIZE,
	IPV6_SA_PREF_FIELD_SIZE,
	IPV6_SA_SUFF_FIELD_SIZE,
	IPV6_DA_FIELD_SIZE,
	IPV6_DA_PREF_FIELD_SIZE,
	IPV6_DA_SUFF_FIELD_SIZE,
	L4_SRC_FIELD_SIZE,
	L4_DST_FIELD_SIZE,
	TCP_FLAGS_FIELD_SIZE,
	UDF_OUT_TPID_FIELD_SIZE,
	UDF_IN_TPID_FIELD_SIZE,
	UDF_IPV4_IHL_FIELD_SIZE,
	0,0,0,0,0,0,0,0,0,0,0,0,0,	/* unused field_ids: 35..47 */
	IPV4_ARP_DA_FIELD_SIZE,
	IN_VLAN_PRI_FIELD_SIZE,
	PPPOE_PROTO_FIELD_SIZE
};

/******************************************************************************
 *
 * Function   : build_prs_ai_info
 *
 * Description: parse the AI bits in tcam or sram
 *
 * Parameters :
 * INPUT ai_str - string contains the AI info
 * OUTPUT ai_bits - AI bits data
 *        ai_bits_mask - AI bits mask
 * Returns    : int
 *
 ******************************************************************************/
static int build_prs_ai_info(char *ai_str, unsigned char *ai_bits, unsigned char *ai_bits_mask)
{
	unsigned int length, i, j;
	unsigned char *locstr;
	bool isEmpty = true;
	char temp[PRS_AI_STR_LEN + 1];

	length = strlen((char *)ai_str);
	/* Check empty or not */
	if (length != 0) {
		if ((locstr = gobble(ai_str)) != 0) {
			if (locstr[0] != '*')
				isEmpty = false;
		}
		*ai_bits = 0x0;
		*ai_bits_mask = 0x0;
		memcpy(temp, ai_str, PRS_AI_STR_LEN);
		/* If empty, not care of AI bits */
		if (isEmpty == false) {
			if (length != PRS_AI_STR_LEN) {
				ERR_PR("Invalid AI input, must be: bxxxxxxxx\n");
				return PPV2_RC_FAIL;
			}

			for (i = 0, j  = (PRS_AI_STR_LEN - 1); i < (PRS_AI_STR_LEN - 1), j > 0; i++, j--) {
				if (temp[j] == 'x') {
					*ai_bits_mask &= (~(1 << i));
				} else if (temp[j] == '0') {
					*ai_bits &= (~(1 << i));
					*ai_bits_mask |= (1 << i);
				} else if (temp[j] == '1') {
					*ai_bits |= (1 << i);
					*ai_bits_mask |= (1 << i);
				} else {
					ERR_PR("Invalid AI input: %s\n", temp);
					return PPV2_RC_FAIL;
				}
			}
		}
	} else {
		ERR_PR("Invalid AI input\n");
		return PPV2_RC_FAIL;
	}

	return PPV2_RC_OK;
}

/* claculate header bits already used */
static unsigned int tcam_header_len_cal(unsigned int *field_id_array, unsigned int field_id, PRS_Flag_t *parse_flag)
{
	unsigned int i, j;
	unsigned int header_len = 0;
	unsigned char out_pri_flag = 0;
	unsigned int temp_filed, temp_start;

	if (field_id == MH_FIELD_ID ||
	    field_id == GEM_PORT_ID_FIELD_ID ||
	    field_id == MH_UNTAGGED_PRI_FIELD_ID)
		return 0;

	if (field_id_array[MH_FIELD_ID] == MH_FIELD_ID ||
	    field_id_array[GEM_PORT_ID_FIELD_ID] == GEM_PORT_ID_FIELD_ID ||
	    field_id_array[MH_UNTAGGED_PRI_FIELD_ID] == MH_UNTAGGED_PRI_FIELD_ID) {
		header_len += 16;
		for (i = 3; i < field_id; i++) {/*No more than 8 byte, left do not care */
			header_len += field_size[i];
		}
	} else {
		for (i = 3; i < field_id; i++) {
			if (field_id_array[i] == i)
				break;
		}
		if (i == field_id) {
			if (field_id_array[PPV2_UDF_OUT_TPID] == PPV2_UDF_OUT_TPID)
				i = PPV2_UDF_OUT_TPID;
			else if(field_id_array[PPV2_UDF_IN_TPID] == PPV2_UDF_IN_TPID)
				i = PPV2_UDF_IN_TPID;
			else if (field_id_array[PPV2_UDF_IPV4_IHL] == PPV2_UDF_IPV4_IHL)
				i = PPV2_UDF_IPV4_IHL;
		}
		if (i == field_id) {/*first one*/
			return 0;
		} else {
			if (field_id == PPV2_UDF_OUT_TPID)
				temp_filed = MAC_SA_FIELD_ID + 1;
			else if (field_id == PPV2_UDF_IN_TPID)
				temp_filed = OUT_VLAN_ID_FIELD_ID + 1;
			else if (field_id == PPV2_UDF_IPV4_IHL)
				temp_filed = IP_VER_FIELD_ID + 1;
			else
				temp_filed = field_id;
			if (i == PPV2_UDF_OUT_TPID) {
				temp_start = OUT_VLAN_PRI_FIELD_ID;
				header_len += field_size[i];
				if (temp_start == field_id)
					return header_len;
			} else if (i == PPV2_UDF_IN_TPID) {
				temp_start = IN_VLAN_ID_FIELD_ID;
				header_len += field_size[i];
				if (temp_start == field_id)
					return header_len;
			} else if (i == PPV2_UDF_IPV4_IHL) {
				temp_start = IPV4_DSCP_FIELD_ID;
				header_len += field_size[PPV2_UDF_IPV4_IHL] + field_size[IP_VER_FIELD_ID];
				if (temp_start == field_id)
					return header_len;
			} else {
				if ((i != PPV2_UDF_IN_TPID) && (field_id == IN_VLAN_ID_FIELD_ID))
					header_len += field_size[PPV2_UDF_IN_TPID];

				temp_start = i;
			}
			for (j = temp_start; j < temp_filed; j++) {
				switch(j) {
					case OUT_VLAN_PRI_FIELD_ID:
						if (i != PPV2_UDF_OUT_TPID)
							header_len += (UDF_OUT_TPID_FIELD_SIZE + 16);
						else
							header_len += 16;
						out_pri_flag = 1;
						break;
					case OUT_VLAN_ID_FIELD_ID:
						if (out_pri_flag == 0) {
							header_len += (UDF_OUT_TPID_FIELD_SIZE + 16);
						}
						break;
					case IN_VLAN_ID_FIELD_ID:
						if (i != PPV2_UDF_IN_TPID)
							header_len += (UDF_IN_TPID_FIELD_SIZE + 16);
						else
							header_len += 16;
						break;
					case ETH_TYPE_FIELD_ID:
						if (field_id == PPPOE_SESID_FIELD_ID)
							header_len += (field_size[j] + 4/*PPPoe ver*/ + 4/*type*/ + 8/*code*/ + 16/*id*/ + 16/*len*/);
						else
							header_len += field_size[j];
						break;
					case PPPOE_SESID_FIELD_ID:
						if (parse_flag->pppoe)
							header_len += field_size[j];
						break;
					case IP_VER_FIELD_ID:
						header_len += 8;
						break;
					case IPV4_DSCP_FIELD_ID:
						if (parse_flag->ipv4)
							header_len += 8;
						break;
					case IPV4_ECN_FIELD_ID:
						if (parse_flag->ipv4) {
							if (field_id_array[IPV4_DSCP_FIELD_ID] != IPV4_DSCP_FIELD_ID)
								header_len += 8;
						}
						break;
					case IPV4_LEN_FIELD_ID:
						if (parse_flag->ipv4)
							header_len += field_size[j] + 16 + 8 + 8;
						break;
					case IPV4_SA_FIELD_ID:
					case IPV4_DA_FIELD_ID:
						if (parse_flag->ipv4)
							header_len += field_size[j];
						break;
					case IPV4_TTL_FIELD_ID:
						if (parse_flag->ipv4)
							header_len += field_size[j];
						if (parse_flag->ipv6 && (field_id > IPV6_NH_FIELD_ID || field_id == IPV6_PROTO_FIELD_ID))
							header_len += field_size[j];
						break;
					case IPV4_PROTO_FIELD_ID:
						if (parse_flag->ipv4)
							header_len += field_size[j];
						if (parse_flag->ipv6)
							header_len += 0;
						break;
					case IPV6_DSCP_FIELD_ID:/* Ipv6, Ipver dscp, ecn, flowlabel, can not devided */
						if (parse_flag->ipv6) {
							if (field_id_array[IP_VER_FIELD_ID] != IP_VER_FIELD_ID)
								header_len += 16;
							else
								header_len += 8;
						}
						break;
					case IPV6_ECN_FIELD_ID:
						break;
					case IPV6_FLOW_LBL_FIELD_ID:
						if (parse_flag->ipv6)
							header_len += 16;
						break;
					case IPV6_PAYLOAD_LEN_FIELD_ID:
					case IPV6_NH_FIELD_ID:
					case IPV6_SA_FIELD_ID:
					case IPV6_SA_PREF_FIELD_ID:
					case IPV6_SA_SUFF_FIELD_ID:
					case IPV6_DA_FIELD_ID:
					case IPV6_DA_PREF_FIELD_ID:
					case IPV6_DA_SUFF_FIELD_ID:
						if (parse_flag->ipv6)
							header_len += field_size[j];
						break;
					default:
						header_len += field_size[j];
						break;
				}
			}
		}
	}
	return header_len;
}
/******************************************************************************
 * Function   : build_prs_tcam_header
 *
 * Description: parse TCAM header data from XML according to their field ID
 *
 * Parameters:
 * INPUT prs_data - XML data contains header data info
 * OUTPUT prs_entry - parse entry contain parsed info
 * Returns : int
 * Comments: none
 ******************************************************************************/
static int build_prs_tcam_header(xml_entry_data *prs_data, PRS_Entry_t *prs_entry)
{
	unsigned int	skip;
	char		tcam_data_str[512], *tcam_data_ptr;
	PncEntry_S	pnCEntry;
	unsigned int 	field_id;
	unsigned int	tcam_bits_used = 0;
	unsigned int	field_id_checked[PPV2_FIELD_COUNT], field_id_skiped[PPV2_FIELD_COUNT];
	unsigned int	i;
	unsigned int	bytes_need = 0;
	PRS_Flag_t	parse_flag;

	memset(&pnCEntry, 0, sizeof(pnCEntry));
	tcam_data_ptr = tcam_data_str;
	for (i = 0; i < PPV2_FIELD_COUNT; i++) {
		field_id_checked[i] = NO_FIELD_ID;
		field_id_skiped[i] = NO_FIELD_ID;
	}
	memset(&parse_flag, 0, sizeof(parse_flag));
	/* If header data empty, return */
	if (ezxml_txt(prs_data[TCAM_HEADER].xmlEntry) == EMPTY_STR ||
	    strcmp(ezxml_txt(prs_data[TCAM_HEADER].xmlEntry), EMPTY_START) == 0)
		return PPV2_RC_OK;
	/* Get the TCAM header data if no empty and restore it to prs_entry.tcam and tcam_mask */
	strcpy(tcam_data_str, ezxml_txt(prs_data[TCAM_HEADER].xmlEntry));
	skip = 0;

	while (strlen(tcam_data_ptr)>0 && dealWithPacket((unsigned char*)tcam_data_ptr, &pnCEntry, &skip) == true) {
		char		*field_name;
		char		*subfield_name;
		char		*subfield_value;
		unsigned int 	indx = 0;
		RtSubFldEntry_S	*pRtSubFldEntry;

		/* get all parsed sub fields for this field */
		while (getRtSubFld(&field_name, &subfield_name, &subfield_value, &field_id, &pRtSubFldEntry, indx)) {
			/* the field id for this subfield is invalied, skip */
			if (NO_FIELD_ID == field_id) {
				ERR_PR("skip %d - %s %s %s\n", indx, field_name, subfield_name, subfield_value);
				indx++;
				continue;
			}

			switch(field_id) {
				case MH_FIELD_ID:
				case GEM_PORT_ID_FIELD_ID:
				case MH_UNTAGGED_PRI_FIELD_ID:
					bytes_need = 2;
					for (i = 0; i < bytes_need; i++) {
						prs_entry->tcam.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValue >> ((bytes_need - 1 - i) * 8));
						prs_entry->tcamMask.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValueMask >> ((bytes_need - 1 - i) * 8));
						tcam_bits_used += 8;
					}
					tcam_bits_used += field_size[MH_FIELD_SIZE];
					field_id_checked[field_id] = field_id;
					break;
				case MAC_DA_FIELD_ID:
				case MAC_SA_FIELD_ID:
					tcam_bits_used = tcam_header_len_cal(field_id_checked, field_id, &parse_flag);
					if (tcam_bits_used + field_size[field_id] > 64) {
						ERR_PR("Exceed TCAM HEADER (%d/64)\n", (tcam_bits_used + 48));
						return PPV2_RC_FAIL;
					}

					for (i = 0; i < MACADDR_SIZE; i++) {
						prs_entry->tcam.header_data[tcam_bits_used/8] |= pRtSubFldEntry->parsedMacAddress[i];
						prs_entry->tcamMask.header_data[tcam_bits_used/8] |= pRtSubFldEntry->parsedMacAddressMask[i];
						tcam_bits_used += 8;
					}
					field_id_checked[field_id] = field_id;
					break;
				case IPV4_SA_FIELD_ID:
				case IPV4_DA_FIELD_ID:
					parse_flag.ipv4 = true;
					tcam_bits_used = tcam_header_len_cal(field_id_checked, field_id, &parse_flag);
					if (tcam_bits_used + field_size[field_id] > 64) {
						ERR_PR("Exceed TCAM HEADER (%d/64)\n", (tcam_bits_used + 48));
						return PPV2_RC_FAIL;
					}
					for (i = 0; i < IPADDR_SIZE; i++) {
						prs_entry->tcam.header_data[tcam_bits_used/8] |= pRtSubFldEntry->parsedIpAddress[i];
						prs_entry->tcamMask.header_data[tcam_bits_used/8] |= pRtSubFldEntry->parsedIpAddressMask[i];
						tcam_bits_used += 8;
					}
					field_id_checked[field_id] = field_id;
					break;
				case PPV2_UDF_OUT_TPID:
				case PPV2_UDF_IN_TPID:
				case ETH_TYPE_FIELD_ID:
				case PPPOE_SESID_FIELD_ID:
					if (field_id == PPPOE_SESID_FIELD_ID)
						parse_flag.pppoe = true;
				case IPV4_LEN_FIELD_ID:
					if (field_id == IPV4_LEN_FIELD_ID)
						parse_flag.ipv4 = true;
				case IPV4_TTL_FIELD_ID:
				case IPV4_PROTO_FIELD_ID:
				case IPV6_PAYLOAD_LEN_FIELD_ID:
					if (field_id == IPV6_PAYLOAD_LEN_FIELD_ID)
						parse_flag.ipv6 = true;
				case IPV6_NH_FIELD_ID:
					if (field_id == IPV6_NH_FIELD_ID)
						parse_flag.ipv6 = true;
				case L4_SRC_FIELD_ID:
				case L4_DST_FIELD_ID:
				case TCP_FLAGS_FIELD_ID:
					tcam_bits_used = tcam_header_len_cal(field_id_checked, field_id, &parse_flag);
					if (tcam_bits_used + field_size[field_id] > 64) {
						ERR_PR("Exceed TCAM HEADER (%d/64)\n", (tcam_bits_used + 48));
						return PPV2_RC_FAIL;
					}
					if (field_size[field_id] % 8)
						bytes_need = field_size[field_id] / 8 + 1;
					else
						bytes_need = field_size[field_id] / 8;
					for (i = 0; i < bytes_need; i++) {
						prs_entry->tcam.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValue >> ((bytes_need - 1 - i) * 8));
						prs_entry->tcamMask.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValueMask >> ((bytes_need - 1 - i) * 8));
						tcam_bits_used += 8;
					}
					field_id_checked[field_id] = field_id;
					break;
				case IPV6_DSCP_FIELD_ID:
					parse_flag.ipv6 = true;
					tcam_bits_used = tcam_header_len_cal(field_id_checked, field_id, &parse_flag);
					if (tcam_bits_used + field_size[field_id] > 64) {
						ERR_PR("Exceed TCAM HEADER (%d/64)\n", (tcam_bits_used + 48));
						return PPV2_RC_FAIL;
					}
					if (field_id_checked[IP_VER_FIELD_ID] == IP_VER_FIELD_ID)
						tcam_bits_used -= 8;
					prs_entry->tcam.header_data[tcam_bits_used/8] |= ((pRtSubFldEntry->parsedIntValue >> 2) & 0xF);
					prs_entry->tcamMask.header_data[tcam_bits_used/8] |= ((pRtSubFldEntry->parsedIntValueMask >> 2) & 0xF);
					tcam_bits_used += 8;
					prs_entry->tcam.header_data[tcam_bits_used/8] |= ((pRtSubFldEntry->parsedIntValue << 6) & 0xC0);
					prs_entry->tcamMask.header_data[tcam_bits_used/8] |= ((pRtSubFldEntry->parsedIntValueMask << 6) & 0xC0);
					tcam_bits_used += 8;

					field_id_checked[field_id] = field_id;
					break;
				case IPV6_ECN_FIELD_ID:
					parse_flag.ipv6 = true;
					tcam_bits_used = tcam_header_len_cal(field_id_checked, field_id, &parse_flag);
					if (tcam_bits_used + field_size[field_id] > 64) {
						ERR_PR("Exceed TCAM HEADER (%d/64)\n", (tcam_bits_used + 48));
						return PPV2_RC_FAIL;
					}
					if (field_id_checked[IPV6_DSCP_FIELD_ID] == IPV6_DSCP_FIELD_ID)
						tcam_bits_used -= 8;
					prs_entry->tcam.header_data[tcam_bits_used/8] |= ((pRtSubFldEntry->parsedIntValue << 4) & 0x30);
					prs_entry->tcamMask.header_data[tcam_bits_used/8] |= ((pRtSubFldEntry->parsedIntValueMask << 4) & 0x30);
					tcam_bits_used += 8;
					field_id_checked[field_id] = field_id;
					break;
				case IPV6_FLOW_LBL_FIELD_ID:
					parse_flag.ipv6 = true;
					tcam_bits_used = tcam_header_len_cal(field_id_checked, field_id, &parse_flag);
					if (tcam_bits_used + field_size[field_id] > 64) {
						ERR_PR("Exceed TCAM HEADER (%d/64)\n", (tcam_bits_used + 48));
						return PPV2_RC_FAIL;
					}
					if (field_id_checked[IPV6_DSCP_FIELD_ID] == IPV6_DSCP_FIELD_ID ||
					    field_id_checked[IPV6_ECN_FIELD_ID] == IPV6_ECN_FIELD_ID)
						tcam_bits_used -= 8;
					prs_entry->tcam.header_data[tcam_bits_used/8] |= ((pRtSubFldEntry->parsedIntValue >> 16) & 0x0F);
					prs_entry->tcamMask.header_data[tcam_bits_used/8] |= ((pRtSubFldEntry->parsedIntValueMask >> 16) & 0x0F);
					tcam_bits_used += 8;
					bytes_need = 2;
					for (i = 0; i < bytes_need; i++) {
						prs_entry->tcam.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValue >> ((bytes_need - 1 - i) * 8));
						prs_entry->tcamMask.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValueMask >> ((bytes_need - 1 - i) * 8));
						tcam_bits_used += 8;
					}
					field_id_checked[field_id] = field_id;
					break;
				case IP_VER_FIELD_ID:
					if (pRtSubFldEntry->parsedIntValue == 4)
						parse_flag.ipv4 = true;
					else
						parse_flag.ipv6 = true;
				case PPV2_UDF_IPV4_IHL:
					if (field_id == PPV2_UDF_IPV4_IHL)
						parse_flag.ipv4 = true;
					tcam_bits_used = tcam_header_len_cal(field_id_checked, field_id, &parse_flag);
					if (tcam_bits_used + field_size[field_id] > 64) {
						ERR_PR("Exceed TCAM HEADER (%d/64)\n", (tcam_bits_used + 48));
						return PPV2_RC_FAIL;
					}
					if (field_size[field_id] % 8)
						bytes_need = field_size[field_id] / 8 + 1;
					else
						bytes_need = field_size[field_id] / 8;
					if (field_id == IP_VER_FIELD_ID) {
						pRtSubFldEntry->parsedIntValue <<= 4;
						pRtSubFldEntry->parsedIntValueMask <<= 4;
					}
					if (field_id == PPV2_UDF_IPV4_IHL && field_id_checked[IP_VER_FIELD_ID] == IP_VER_FIELD_ID)
						tcam_bits_used -= 8;
					for (i = 0; i < bytes_need; i++) {
						prs_entry->tcam.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValue >> ((bytes_need - 1 - i) * 8));
						prs_entry->tcamMask.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValueMask >> ((bytes_need - 1 - i) * 8));
						tcam_bits_used += 8;
					}
					field_id_checked[field_id] = field_id;
					break;
				case IPV4_DSCP_FIELD_ID:
				case IPV4_ECN_FIELD_ID:
					parse_flag.ipv4 = true;
					tcam_bits_used = tcam_header_len_cal(field_id_checked, field_id, &parse_flag);
					if (tcam_bits_used + field_size[field_id] > 64) {
						ERR_PR("Exceed TCAM HEADER (%d/64)\n", (tcam_bits_used + 48));
						return PPV2_RC_FAIL;
					}
					if (field_size[field_id] % 8)
						bytes_need = field_size[field_id] / 8 + 1;
					else
						bytes_need = field_size[field_id] / 8;
					if (field_id == IPV4_DSCP_FIELD_ID) {
						pRtSubFldEntry->parsedIntValue <<= 2;
						pRtSubFldEntry->parsedIntValueMask <<= 2;
					}
					if (field_id == IPV4_ECN_FIELD_ID && field_id_checked[IPV4_DSCP_FIELD_ID] == IPV4_DSCP_FIELD_ID)
						tcam_bits_used -= 8;
					for (i = 0; i < bytes_need; i++) {
						prs_entry->tcam.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValue >> ((bytes_need - 1 - i) * 8));
						prs_entry->tcamMask.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValueMask >> ((bytes_need - 1 - i) * 8));
						tcam_bits_used += 8;
					}
					field_id_checked[field_id] = field_id;
					break;
				case OUT_VLAN_PRI_FIELD_ID:
					tcam_bits_used = tcam_header_len_cal(field_id_checked, field_id, &parse_flag);
					if (tcam_bits_used + 16 > 64) {
						ERR_PR("Exceed TCAM HEADER (%d/64)\n", (tcam_bits_used + 48));
						return PPV2_RC_FAIL;
					}
					pRtSubFldEntry->parsedIntValue <<= 13;
					pRtSubFldEntry->parsedIntValueMask <<= 13;
					bytes_need = 2;
					for (i = 0; i < bytes_need; i++) {
						prs_entry->tcam.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValue >> ((bytes_need - 1 - i) * 8));
						prs_entry->tcamMask.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValueMask >> ((bytes_need - 1 - i) * 8));
						tcam_bits_used += 8;
					}
					field_id_checked[field_id] = field_id;
					break;
				case OUT_VLAN_ID_FIELD_ID:
				case IN_VLAN_ID_FIELD_ID:
					tcam_bits_used = tcam_header_len_cal(field_id_checked, field_id, &parse_flag);
					if ((field_id == IN_VLAN_ID_FIELD_ID) || (field_id_checked[field_id - 1] != OUT_VLAN_PRI_FIELD_ID)) {
						if (tcam_bits_used + 16 > 64) {
							ERR_PR("Exceed TCAM HEADER (%d/64)\n", (tcam_bits_used + 48));
							return PPV2_RC_FAIL;
						}
					} else {
						tcam_bits_used -= 16;
					}
					bytes_need = 2;
					for (i = 0; i < bytes_need; i++) {
						prs_entry->tcam.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValue >> ((bytes_need - 1 - i) * 8));
						prs_entry->tcamMask.header_data[tcam_bits_used/8] |= (pRtSubFldEntry->parsedIntValueMask >> ((bytes_need - 1 - i) * 8));
						tcam_bits_used += 8;
					}
					field_id_checked[field_id] = field_id;
					break;
				default:
					ERR_PR("Field %d not supported in TCAM HEADER\n", field_id);
					return PPV2_RC_FAIL;
			}
			indx++;
		}
		tcam_data_ptr = &tcam_data_ptr[skip];
		skip = 0;
	}
	return PPV2_RC_OK;
}

/******************************************************************************
 * Function   : build_prs_result_info
 *
 * Description: parse SRAM result info
 *
 * Parameters :
 * INPUT prs_data - XML data contains result info
 * OUTPUT prs_entry - parse entry contain parsed info
 * Returns : int
 * Comments: none
 ******************************************************************************/
static int build_prs_result_info(xml_entry_data *prs_data, PRS_Entry_t *prs_entry)
{
	unsigned int i;
	DdEntry *sram_ri = NULL;
	unsigned int sram_ri_bit[17] = {SRAM_RI_MAC2ME_BIT,
					SRAM_RI_ETY_DSA_BIT,
					SRAM_RI_VLAN_BIT,
					SRAM_RI_CPU_CODE_BIT,
					SRAM_RI_L2_VERSION_BIT,
					SRAM_RI_L2_CAST_BIT,
					SRAM_RI_PPPOE_BIT,
					SRAM_RI_UDF1_L3_BIT,
					SRAM_RI_L3_CAST_BIT,
					SRAM_RI_IP_FRAG_BIT,
					SRAM_RI_UDF2_IPV6_BIT,
					SRAM_RI_UDF3_BIT,
					SRAM_RI_UDF4_L4_BIT,
					SRAM_RI_UDF5_BIT,
					SRAM_RI_UDF6_BIT,
					SRAM_RI_UDF7_BIT,
					SRAM_RI_DROP_BIT};

	unsigned int sram_ri_bit_mask[17] = {SRAM_RI_MAC2ME_MASK,
					     SRAM_RI_ETY_DSA_MASK,
					     SRAM_RI_VLAN_MASK,
					     SRAM_RI_CPU_CODE_MASK,
					     SRAM_RI_L2_VERSION_MASK,
					     SRAM_RI_L2_CAST_MASK,
					     SRAM_RI_PPPOE_MASK,
					     SRAM_RI_UDF1_L3_MASK,
					     SRAM_RI_L3_CAST_MASK,
					     SRAM_RI_IP_FRAG_MASK,
					     SRAM_RI_UDF2_IPV6_MASK,
					     SRAM_RI_UDF3_MASK,
					     SRAM_RI_UDF4_L4_MASK,
					     SRAM_RI_UDF5_MASK,
					     SRAM_RI_UDF6_MASK,
					     SRAM_RI_UDF7_MASK,
					     SRAM_RI_DROP_MASK};

	char *sram_ri_str[17] = {RI_MAC2ME,
				   RI_ETY_DSA,
				   RI_VLAN,
				   RI_CPU_CODE,
				   RI_L2_VERSION,
				   RI_L2_CAST,
				   RI_PPPOE,
				   RI_UDF1_L3,
				   RI_L3_CAST,
				   RI_IP_FRAG,
				   RI_UDF2_IPV6,
				   RI_UDF3,
				   RI_UDF4_L4,
				   RI_UDF5,
				   RI_UDF6,
				   RI_UDF7,
				   RI_DROP};

	/* Parse each field in result info */
	for (i = 0; i < 17; i++) {
		if (ezxml_txt(prs_data[i + SRAM_RI_MAC2ME].xmlEntry) != EMPTY_STR) {
			sram_ri = findMatchingEntry(ezxml_txt(prs_data[i + SRAM_RI_MAC2ME].xmlEntry));
			if (sram_ri == NULL) {
				ERR_PR("%s is missing.\n", sram_ri_str[i]);
				continue;
			}
			prs_entry->sram.sram_ri.ri |= ((atoi((char*)sram_ri->value) << sram_ri_bit[i]) & sram_ri_bit_mask[i]);
			prs_entry->sram.sram_ri.ri_mask |= sram_ri_bit_mask[i];
		}
	}

	return PPV2_RC_OK;
}

/*******************************************************************************
* build_prs_init_action_sysfs()
*
* DESCRIPTION: Parse the PRS init info from XML, and generate sysfs command to
*              write the configration to HW
*
* INPUTS:   prs_init_data
*
* OUTPUTS:  None
*
* RETURNS:
*
*******************************************************************************/
static int build_prs_init_action_sysfs(xml_entry_data *prs_init_data)
{
	char sysfs_buf[512];
	unsigned int port_num, init_lu_id, init_lu_off, max_loop;

	port_num = atoi(ezxml_txt(prs_init_data[PORT_IDX].xmlEntry));
	init_lu_id = atoi(ezxml_txt(prs_init_data[INIT_LOOKUP_ID].xmlEntry));
	init_lu_off = atoi(ezxml_txt(prs_init_data[INIT_LOOKUP_OFF].xmlEntry));
	max_loop = atoi(ezxml_txt(prs_init_data[MAX_LOOKUP].xmlEntry));

	sprintf(sysfs_buf, "##  PRS init: PORT:%d init_lookup_id:%d init_lookup_off:%d max_loop%d ##\n", port_num,
		init_lu_id, init_lu_off, max_loop);
	handle_sysfs_command(sysfs_buf, true);

	/* Sysfs command */
	sprintf(sysfs_buf, "echo %d %d %d %d > %s/hw_frst_itr\n",
			port_num,
			init_lu_id,
			max_loop,
			init_lu_off,
			PRS_INIT_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	return PPV2_RC_OK;
}

/*******************************************************************************
* build_prs_action_sysfs()
*
* DESCRIPTION: Parse the PRS info from XML, and generate sysfs command to
*              write the configration to HW
*
* INPUTS:   build_prs_action_sysfs
*
* OUTPUTS:  None
*
* RETURNS: int
*
*******************************************************************************/
static int build_prs_action_sysfs(xml_entry_data *prs_data)
{
	char sysfs_buf[512];
	DdEntry *sram_lu_done = NULL, *sram_cls_lu_id_gen = NULL, *offset_sign = NULL;
	unsigned int tcam_idx, i;
	PRS_Entry_t prs_entry;
	char n_lu, u_def;
	int port_value = 0x0;

	/* Clear prs_entry to 0 first */
	tcam_idx = 0;
	memset(&prs_entry, 0, sizeof(prs_entry));

	/* Parse TCAM Index */
	if (ezxml_txt(prs_data[TCAM_ENTRY_INDEX].xmlEntry) != EMPTY_STR)
		tcam_idx = atoi(ezxml_txt(prs_data[TCAM_ENTRY_INDEX].xmlEntry));
	/* Parse TCAM lookup id */
	if (ezxml_txt(prs_data[TCAM_ENTRY_LU_ID].xmlEntry) != EMPTY_STR)
		prs_entry.tcam.lu_id = atoi(ezxml_txt(prs_data[TCAM_ENTRY_LU_ID].xmlEntry));
	/* Parse TCAM Phy PortID */
	if (ezxml_txt(prs_data[TCAM_PHY_PORT_ID].xmlEntry) != EMPTY_STR) {
		//prs_entry.tcam.portId = atoi(ezxml_txt(prs_data[TCAM_PHY_PORT_ID].xmlEntry));
		parseSimpleNumericValue(ezxml_txt(prs_data[TCAM_PHY_PORT_ID].xmlEntry),&port_value);
		prs_entry.tcam.portId = port_value;
	}
	/* Parse TCAM AI Bits */
	if (build_prs_ai_info(ezxml_txt(prs_data[TCAM_ENTRY_AI].xmlEntry),
					&prs_entry.tcam.addInfo,
					&prs_entry.tcamMask.addInfo) != PPV2_RC_OK) {
		ERR_PR("Build AI bits failed\n");
		return PPV2_RC_FAIL;
	}

	/* Parse TCAM header data */
	if (build_prs_tcam_header(prs_data, &prs_entry) != PPV2_RC_OK) {
		ERR_PR("Parse TCAM header data failed\n");
		return PPV2_RC_FAIL;
	}

	/* Parse SRAM RI */
	if (build_prs_result_info(prs_data, &prs_entry) != PPV2_RC_OK) {
		ERR_PR("Parse result info failed\n");
		return PPV2_RC_FAIL;
	}

	/* Parse SRAM Next lookup offset */
	if (ezxml_txt(prs_data[SRAM_NEXT_LU_OFF].xmlEntry) != EMPTY_STR)
		prs_entry.sram.sram_offset.nlu_off = atoi(ezxml_txt(prs_data[SRAM_NEXT_LU_OFF].xmlEntry));
	/* Parse SRAM Next lookup offset sign */
	offset_sign = findMatchingEntry(ezxml_txt(prs_data[SRAM_NEXT_LU_OFF_SIGN].xmlEntry));
	if (offset_sign != NULL)
		prs_entry.sram.sram_offset.nlu_off_sign = atoi((char*)offset_sign->value);
	else
		prs_entry.sram.sram_offset.nlu_off_sign = 0;
	/* Parse SRAM User define Offset */
	if (ezxml_txt(prs_data[SRAM_UDF_OFF].xmlEntry) != EMPTY_STR)
		prs_entry.sram.sram_offset.udef_off = atoi(ezxml_txt(prs_data[SRAM_UDF_OFF].xmlEntry));
	/* Parse SRAM User define Offset sign */
	offset_sign = findMatchingEntry(ezxml_txt(prs_data[SRAM_UDF_OFF_SIGN].xmlEntry));
	if (offset_sign != NULL)
		prs_entry.sram.sram_offset.udef_off_sign = atoi((char*)offset_sign->value);
	else
		prs_entry.sram.sram_offset.udef_off_sign = 0;
	/* Parse SRAM User define type */
	if (ezxml_txt(prs_data[SRAM_UDF_TYPE].xmlEntry) != EMPTY_STR)
		prs_entry.sram.sram_offset.udef_type= atoi(ezxml_txt(prs_data[SRAM_UDF_TYPE].xmlEntry));
	/* Parse SRAM Operation Select */
	if (ezxml_txt(prs_data[SRAM_OP_SEL].xmlEntry) != EMPTY_STR)
		prs_entry.sram.sram_offset.op_sel = atoi(ezxml_txt(prs_data[SRAM_OP_SEL].xmlEntry));

	/* Parse SRAM Lookup ID */
	if (ezxml_txt(prs_data[SRAM_NEXT_LU_ID].xmlEntry) != EMPTY_STR)
		prs_entry.sram.sram_next_lu.lu_id = atoi(ezxml_txt(prs_data[SRAM_NEXT_LU_ID].xmlEntry));

	/* Parse SRAM Lookup Done */
	sram_lu_done = findMatchingEntry(ezxml_txt(prs_data[SRAM_LU_DONE].xmlEntry));
	if (sram_lu_done != NULL)
		prs_entry.sram.sram_next_lu.lu_done = atoi((char*)sram_lu_done->value);
	else
		prs_entry.sram.sram_next_lu.lu_done = 0;

	/* Parse SRAM AI Bits Info or class lookup ID */
	if (prs_entry.sram.sram_next_lu.lu_done) {/* If lookup done, sram ai bits is classifer lookup ID */
		if (ezxml_txt(prs_data[SRAM_NEXT_AI].xmlEntry) != EMPTY_STR)
			prs_entry.sram.sram_next_lu.addInfo = atoi(ezxml_txt(prs_data[SRAM_NEXT_AI].xmlEntry));
		else
			prs_entry.sram.sram_next_lu.addInfo = 0;
		prs_entry.sram.sram_next_lu.ai_update = 0x3F;
		/* Class lookup ID valid check, 0x3C, 0x3D, 0x3E and 0x3F are reserved */
		if (prs_entry.sram.sram_next_lu.addInfo > 0x3B) {
			ERR_PR("Classifier lookup ID invalid (0x%x)\n", prs_entry.sram.sram_next_lu.addInfo);
			return PPV2_RC_FAIL;
		}
		/* Set SRAM RI as TCAM key */
		memset(&prs_entry.tcam.header_data, 0, PRS_HEADER_LEN);
		memset(&prs_entry.tcamMask.header_data, 0, PRS_HEADER_LEN);
		for (i = 0; i < 4; i++) {
			prs_entry.tcam.header_data[i] = (prs_entry.sram.sram_ri.ri >> (i * 8));
			prs_entry.tcamMask.header_data[i] = (prs_entry.sram.sram_ri.ri_mask >> (i * 8));
		}
	} else {
		if (build_prs_ai_info(ezxml_txt(prs_data[SRAM_NEXT_AI].xmlEntry),
						&prs_entry.sram.sram_next_lu.addInfo,
						&prs_entry.sram.sram_next_lu.ai_update) != PPV2_RC_OK) {
			ERR_PR("Build AI bits failed\n");
			return PPV2_RC_FAIL;
		}
	}

	/* Parse SRAM Classfier LookupID Generate */
	sram_cls_lu_id_gen = findMatchingEntry(ezxml_txt(prs_data[SRAM_CLS_LU_ID_GEN].xmlEntry));
	if (sram_cls_lu_id_gen != NULL)
		prs_entry.sram.sram_next_lu.cls_luid_gen = atoi((char*)sram_cls_lu_id_gen->value);
	else
		prs_entry.sram.sram_next_lu.cls_luid_gen = 0;

	/* LU_DONE and CLS_LKP_GEN can not be set at same entry */
	if (prs_entry.sram.sram_next_lu.cls_luid_gen && prs_entry.sram.sram_next_lu.lu_done) {
		ERR_PR("LU_DONE and CLS_LKP_GEN can not be set at same time\n");
		return PPV2_RC_FAIL;
	}

	sprintf(sysfs_buf, "##  TCAM index:%d ##\n", tcam_idx);
	handle_sysfs_command(sysfs_buf, true);

	/* Sysfs command */

	sprintf(sysfs_buf, "echo 0x%x > %s/hw_read\n",
			tcam_idx,
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	/* Clear sw entry, make it clean */
	sprintf(sysfs_buf, "echo 1 > %s/sw_clear\n",
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	/* lookup id */
	sprintf(sysfs_buf, "echo 0x%x > %s/t_lu\n",
			prs_entry.tcam.lu_id,
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	/* Phy port id */
	sprintf(sysfs_buf, "echo 0x%x > %s/t_port_map\n",
			(prs_entry.tcam.portId),
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	/*TCAM AI bits*/
	sprintf(sysfs_buf, "echo 0x%x 0x%x > %s/t_ai\n",
			prs_entry.tcam.addInfo,
			prs_entry.tcamMask.addInfo,
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	/* TCAM Header data */
	for (i = 0; i < 8; i++) {
		if (prs_entry.tcamMask.header_data[i]) {
			sprintf(sysfs_buf, "echo 0x%x 0x%x 0x%x > %s/t_byte\n",
				i,
				prs_entry.tcam.header_data[i],
				prs_entry.tcamMask.header_data[i],
				PRS_SYSFS_PATH);
			handle_sysfs_command(sysfs_buf, false);
		}
	}

	/* SRAM result info */
	sprintf(sysfs_buf, "echo 0x%x 0x%x > %s/s_ri\n",
			prs_entry.sram.sram_ri.ri,
			prs_entry.sram.sram_ri.ri_mask,
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	/* SRAM Offset: Next lookup offset */
	if (prs_entry.sram.sram_offset.nlu_off_sign)
		n_lu = 0 - (char)prs_entry.sram.sram_offset.nlu_off;
	else
		n_lu = (char)prs_entry.sram.sram_offset.nlu_off;
	sprintf(sysfs_buf, "echo %d > %s/s_shift\n",
			n_lu,
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	/* SRAM Offset: User define offset */
	if (prs_entry.sram.sram_offset.udef_off_sign)
		u_def = 0 - (char)prs_entry.sram.sram_offset.udef_off;
	else
		u_def = (char)prs_entry.sram.sram_offset.udef_off;
	sprintf(sysfs_buf, "echo %d %d > %s/s_offs\n",
			prs_entry.sram.sram_offset.udef_type,
			u_def,
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	/*SRAM Offsets: Operation Select*/
	/*This filed is not open to user, it is second para of mvPp2PrsSwSramShiftSet*/

	/* SRAM Next Lookup, AI bits */
	sprintf(sysfs_buf, "echo 0x%x 0x%x > %s/s_ai\n",
			prs_entry.sram.sram_next_lu.addInfo,
			prs_entry.sram.sram_next_lu.ai_update,
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	/* SRAM Next Lookup, lookup id */
	sprintf(sysfs_buf, "echo 0x%x > %s/s_next_lu\n",
			prs_entry.sram.sram_next_lu.lu_id,
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	/* SRAM Next Lookup, lookup done */
	sprintf(sysfs_buf, "echo 0x%x > %s/s_lu_done\n",
			prs_entry.sram.sram_next_lu.lu_done,
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);
	/* SRAM Next Lookup, cls lookup_id generate  */
	sprintf(sysfs_buf, "echo 0x%x > %s/s_fid_gen\n",
			prs_entry.sram.sram_next_lu.cls_luid_gen,
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	/* Write the sw entry to HW */
	sprintf(sysfs_buf, "echo 0x%x > %s/hw_write\n",
			tcam_idx,
			PRS_SYSFS_PATH);
	handle_sysfs_command(sysfs_buf, false);

	return PPV2_RC_OK;
}


/*******************************************************************************
* parse_xml_prs_init()
*
* DESCRIPTION: Parse the PRS init info from XML, and generate sysfs command to
*              write the configration to HW
*
* INPUTS:   xmlFile        - XML file contains the configuration
*
* OUTPUTS:  None
*
* RETURNS:  US_RC_OK, US_RC_FAIL or US_RC_NOT_FOUND
*
*******************************************************************************/
int parse_xml_prs_init(char *xmlFile)
{
	ezxml_t xmlHead = NULL;
	ezxml_t xmlPRS_init, xmlEntry;
	unsigned int i;

	xml_entry_data	prs_init_data[PRS_INIT_MAX] = {
		{PORT_NUM, NULL},
		{INIT_LU_ID, NULL},
		{INIT_LU_OFFSET, NULL},
		{MAX_LOOP, NULL}
	};

	/* Read XML file */
	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
		return PPV2_RC_FAIL;
	}

	/* Get Worksheet PRS_init */
	xmlPRS_init = ezxml_get(xmlHead, WORKSHEET_PRS_INIT, -1);
	if (xmlPRS_init == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_PRS_INIT);
		ezxml_free(xmlHead);
		return PPV2_RC_FAIL;
	}

	/* Find the first entry */
	xmlEntry = ezxml_child(xmlPRS_init, TABLE_ENTRY);
	if (xmlEntry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n", WORKSHEET_PRS_INIT);
		ezxml_free(xmlHead);
		return PPV2_RC_OK;
	}

	/* Scan All Entry */
	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		/* Parse entry member */
		for (i=0; i < PRS_INIT_MAX ;i++) {
			prs_init_data[i].xmlEntry = ezxml_child(xmlEntry, prs_init_data[i].name);
			if (NULL == prs_init_data[i].xmlEntry) {
				ERR_PR("%s is empty\n", prs_init_data[i].name);
				return PPV2_RC_FAIL;
			}
			/*else
				DEBUG_PR("%s=%s\n", prs_init_data[i].name, ezxml_txt(prs_init_data[i].xmlEntry));*/
		}
		if (build_prs_init_action_sysfs(prs_init_data))
			return PPV2_RC_FAIL;
	}

	ezxml_free(xmlHead);

	return PPV2_RC_OK;
}

/*******************************************************************************
* parse_xml_prs()
*
* DESCRIPTION: Parse the TCAM&SRAM info from XML, and generate sysfs command to
*              write the configration to HW
*
* INPUTS:   xmlFile        - XML file contains the configuration
*
* OUTPUTS:  None
*
* RETURNS:  PPV2_RC_OK, PPV2_RC_FAIL or PPV2_RC_NOT_FOUND
*
*******************************************************************************/
int parse_xml_prs(char *xmlFile)
{
	ezxml_t xmlHead = NULL;
	ezxml_t xmlPRS_init, xmlEntry;
	unsigned int i;
	
	xml_entry_data	prs_data[PRS_MAX] = {
		{TCAM_IDX, NULL},
		{TCAM_LOOKUP_ID, NULL},
		{TCAM_PHYSPORTID, NULL},
		{TCAM_AI, NULL},
		{TCAM_PCKT_HEADER, NULL},
		{RI_MAC2ME, NULL},
		{RI_ETY_DSA, NULL},
		{RI_VLAN, NULL},
		{RI_CPU_CODE, NULL},
		{RI_L2_VERSION, NULL},
		{RI_L2_CAST, NULL},
		{RI_PPPOE, NULL},
		{RI_UDF1_L3, NULL},
		{RI_L3_CAST, NULL},
		{RI_IP_FRAG, NULL},
		{RI_UDF2_IPV6, NULL},
		{RI_UDF3, NULL},
		{RI_UDF4_L4, NULL},
		{RI_UDF5, NULL},
		{RI_UDF6, NULL},
		{RI_UDF7, NULL},
		{RI_DROP, NULL},
		{NLU_OFFSET, NULL},
		{NLU_OFFSET_SIGN, NULL},
		{UDF_OFFSET, NULL},
		{UDF_OFFSET_SIGN, NULL},
		{UDF_OFFSET_TYPE, NULL},
		{OP_SEL, NULL},
		{NLU_AI, NULL},
		{NLU_LU_ID, NULL},
		{LU_DONE, NULL},
		{CLS_LU_ID_GEN, NULL}
	};

	/* Read XML file */
	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
		return PPV2_RC_FAIL;
	}

	/* Get Worksheet PRS */
	xmlPRS_init = ezxml_get(xmlHead, WORKSHEET_PRS, -1);
	if (xmlPRS_init == NULL){
		ERR_PR("Failed to get %s\n", WORKSHEET_PRS);
		ezxml_free(xmlHead);
		return PPV2_RC_FAIL;
	}

	/* Find the first entry */
	xmlEntry = ezxml_child(xmlPRS_init, TABLE_ENTRY);
	if (xmlEntry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n", WORKSHEET_PRS);
		ezxml_free(xmlHead);
		return PPV2_RC_OK;
	}

	/* Scan All Entry */
	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		/* Parse entry member */
		for (i=0; i < PRS_MAX ;i++) {
			prs_data[i].xmlEntry = ezxml_child(xmlEntry, prs_data[i].name);
			/* Warning if no filled */
			/*if (NULL == prs_data[i].xmlEntry)
				DEBUG_PR("%s is empty\n", prs_data[i].name);*/
		}
		if (build_prs_action_sysfs(prs_data))
			return PPV2_RC_FAIL;
	}

	ezxml_free(xmlHead);

	return PPV2_RC_OK;
}
