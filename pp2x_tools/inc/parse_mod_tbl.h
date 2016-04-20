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

#ifndef PARSE_MOD_TBL_H
#define PARSE_MOD_TBL_H

#define FROM_VLAN1_STR			"v1"
#define FROM_VLAN2_STR			"v2"
#define ETHTYPE_REG0_STR		"rg0"
#define ETHTYPE_REG1_STR		"rg1"
#define ETHTYPE_REG2_STR		"rg2"
#define ETHTYPE_REG3_STR		"rg3"
#define NEW_VALUE_STR			"new"
#define CFI_GREEN_STR			"cfg_g"
#define CFI_YELLOW_STR			"cfg_y"

#define ETH_TYPE_STR			"tp"
#define VID_STR				"vid"
#define CFI_STR				"cfi"
#define P_BITS_STR			"pb"
#define SRC_TAGGED_STR			"srcTag"
#define SRC_PORT_STR			"srcPort"
#define TAG_CMD_STR			"tagCmd"
#define VLAN_SEL_STR			"vlanSel"
#define SRC_DEV_STR			"srcDev"
#define SRC_ID_STR			"srcId"
#define VIDX_SEL_STR			"vidxSel"
#define USE_VIDX_STR			"useVidx"
#define SRC_ID_SEL_STR			"srcIdSel"
#define SRC_DEV_SEL_STR			"srcDevSel"
#define SRC_PORT_SEL_STR		"srcPortSel"
#define SRC_EGRESS_FILTER_STR		"egrFiter"
#define SKIP_BEFORE_STR			"skipb"
#define SKIP_AFTER_STR			"skipa"
#define MASK_STR			"mask"
#define SESSID_STR			"sessid"
#define LEN_SRC				"lensrc"
#define L3_OFFSET			"l3off"
#define IP_HEADER_LEN			"iphdlen"
#define LEN_OP				"lenop"
#define MOD_VALUE			"modval"
#define JUMP_COMMAND			"cmd"
#define JUMP_INDEX			"ind"


typedef enum {
	MOD_MAX_VAL	= 0x0FFFF,
	FROM_VLAN1	= 0x10000,
	FROM_VLAN2	= 0x10001,
	ETHTYPE_REG0	= 0x10002,
	ETHTYPE_REG1	= 0x10003,
	ETHTYPE_REG2	= 0x10004,
	ETHTYPE_REG3	= 0x10005,
	NEW_VALUE	= 0x10006,
	PB_GREEN	= 0x10007,
	PB_YELLOW	= 0x10008
} CONF_VLAN;

typedef struct
{
    char		*name;
    bool		(*handler)(char 		*cmd_data_str,
				   unsigned short	*word);
} mod_cmd_handle_entry_s;

int parse_mod_data(int			cmd_code,
		    char		*mod_cmd_data_str,
		    unsigned short	*word);

#endif

