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

#include "common.h"
#include "ParseUtils.h"
#include "DataDictionary.h"
#include "PncGlobals.h"
#include "parse_mod_tbl.h"
#include "ParseUtils.h"


/******************************************************************************
 *
 * Function   : get_csv_value
 *              
 * Description: returns the first string field=value in a CSV string
 *              
 * Parameters : in_str - the input string
 *		 name - the field name (output)
 *		 int_val - the integer value (output)
 *
 * Returns    : char* - the in_str pointer w/o the returned field=value
 *              
 ******************************************************************************/
static char *get_csv_value(char *in_str, char *name, int *int_val)
{
	char	*locstr = in_str;
	int	i;
	char	value[10];

	// Cleanup start of string up to '='
	locstr = gobble(locstr);
	if (locstr == NULL) {
		return NULL;
	}

	if (*locstr == CARRIAGE_RETURN_CHAR)
		locstr++;

	if (*locstr == LINE_FEED_CHAR)
		locstr++;
		
	i = 0;
	/* find the field name */
	while (1) {
		if (isalnum(*locstr) != 0) {
			name[i++] = *locstr++;
		} else {
			name[i] = 0;
			break;
		}
	}

	locstr = gobble(locstr);
	if (locstr == NULL) {
		ERR_PR("missing value\n");
		return NULL;
	}
	
	if (*locstr != EQUAL_CHAR) {
		ERR_PR("missing '=' sign for %s, found [%c]\n", name, *locstr);
		return NULL;
	}
	// increment beyond the '='
	locstr++;
	
	locstr = gobble(locstr);
	if (locstr == NULL) {
		ERR_PR("missing value\n");
		return NULL;
	}

	i = 0;
	/* skip the field */
	while (1) {
		if (isalnum(*locstr) != 0) {
			value[i] = *locstr++;
			
			i++;
		} else {
			value[i] = 0;
			break;
		}
	}

	DEBUG_PR(DEB_OTHER, "%s=%s\n", name, value);
	
	if (scanIntOrDdlookup(value, int_val) == false) {
		ERR_PR("failed to find %s\n", value);
		return NULL;
	}

	return (char *)locstr;
}

#if 0
/******************************************************************************
 *
 * Function   : handle_rsrved
 *              
 * Description: handles the reserved modification commands
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		 word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_rsrved(char 		*cmd_data_str,
			   unsigned short	*word)
{
	ERR_PR("got a %s!!\n", cmd_data_str);
	return true;
}
#endif

/******************************************************************************
 *
 * Function   : handle_2b
 *              
 * Description: handles the 2 byte modification commands
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		 word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_2b(char		*cmd_data_str,
		       unsigned short	*word)
{
	int 	value;
	
	if (parseSimpleNumericValue((UINT8*)cmd_data_str, &value) == false) {
		ERR_PR("could not convert input %s!!\n", cmd_data_str);
		return false;
	}
	DEBUG_PR(DEB_OTHER, "%s=%d\n", cmd_data_str, value);
	
	*word = (unsigned short)value;
	return true;
}


/******************************************************************************
 *
 * Function   : handle_repl_lsb
 *              
 * Description: handles the replace lsb byte modification command
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		 word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_repl_lsb(char		*cmd_data_str,
			     unsigned short	*word)
{
	char	*locstr = cmd_data_str;
	int	cmd_val;
	char	cmd_name[20];
	
	*word = 0;

	/* get and build the CSV values */
	while (1) {
		locstr = get_csv_value((char*)locstr, cmd_name, &cmd_val);
		if (NULL == locstr)
			break;

	 	DEBUG_PR(DEB_OTHER, "cmd_val=0x%x locstr[%s] cmd_name[%s]\n",
			cmd_val, locstr, cmd_name);

		if (strcmp(cmd_name, MASK_STR) == 0) {
			*word |= (cmd_val & 0xFF) << 8;
		} else if (strcmp(cmd_name, NEW_VALUE_STR) == 0) {
			*word |= cmd_val & 0xFF;
		} else {
			ERR_PR("non supported replace 2 bytes command %s command val 0x%x\n", cmd_name, cmd_val);
			return false;
		}
	}

	return true;
}


/******************************************************************************
 *
 * Function   : handle_repl_msb
 *              
 * Description: handles the replace msb byte modification command
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		 word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_repl_msb(char		*cmd_data_str,
			     unsigned short	*word)
{
	char	*locstr = cmd_data_str;
	int	cmd_val;
	char	cmd_name[20];
	
	*word = 0;

	/* get and build the CSV values */
	while (1) {
		locstr = get_csv_value((char*)locstr, cmd_name, &cmd_val);
		if (NULL == locstr)
			break;

	 	DEBUG_PR(DEB_OTHER, "cmd_val=0x%x locstr[%s] cmd_name[%s]\n",
			cmd_val, locstr, cmd_name);

		if (strcmp(cmd_name, MASK_STR) == 0) {
			*word |= cmd_val & 0xFF;
		} else if (strcmp(cmd_name, NEW_VALUE_STR) == 0) {
			*word |= (cmd_val & 0xFF) << 8;
		} else {
			ERR_PR("non supported replace 2 bytes command %s command val 0x%x\n", cmd_name, cmd_val);
			return false;
		}
	}

	return true;
}


/******************************************************************************
 *
 * Function   : handle_cfg_vlan
 *              
 * Description: handles the config VLAN modification command
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		 word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_cfg_vlan(char 		*cmd_data_str,
			     unsigned short	*word)
{
	char		*locstr = cmd_data_str;
	int		cmd_val;
	unsigned short	new_val;
	char		cmd_name[20];
	
	*word = 0;

	/* get and build the CSV values */
	while (1) {
		locstr = get_csv_value((char*)locstr, cmd_name, &cmd_val);
		if (NULL == locstr)
			break;

	 	DEBUG_PR(DEB_OTHER, "cmd_val=0x%x locstr[%s] cmd_name[%s]\n",
			cmd_val, locstr, cmd_name);
		new_val = 0;
		
		if (strcmp(cmd_name, ETH_TYPE_STR) == 0) {
			switch (cmd_val) {
				case FROM_VLAN1:
					new_val = 0x4;
					break;
					
				case FROM_VLAN2:
					new_val = 0x5;
					break;
					
				case ETHTYPE_REG0:
				case ETHTYPE_REG1:
				case ETHTYPE_REG2:
				case ETHTYPE_REG3:
					new_val = cmd_val - ETHTYPE_REG0;
			 		break;
				
				default:
					ERR_PR("VLAN conf cmd_name %s does not support 0x%x\n",
						cmd_name, cmd_val);
			 		return false;
			}
		} else if (strcmp(cmd_name, VID_STR) == 0) {
			switch (cmd_val) {
				case FROM_VLAN1:
				case FROM_VLAN2:
					new_val = cmd_val & MOD_MAX_VAL;
					break;
					
				case NEW_VALUE:
					new_val = 0x2;
			 		break;
				
				default:
					ERR_PR("VLAN conf cmd_name %s does not support 0x%x\n",
						cmd_name, cmd_val);
			 		return false;
			}
		 	new_val <<= 4;
		} else if (strcmp(cmd_name, CFI_STR) == 0) {
			switch (cmd_val) {
				case FROM_VLAN1:
				case FROM_VLAN2:
					new_val = cmd_val & MOD_MAX_VAL;
					break;
					
				case NEW_VALUE:
					new_val = 0x2;
			 		break;
				
				default:
					ERR_PR("VLAN conf cmd_name %s does not support 0x%x\n",
						cmd_name, cmd_val);
			 		return false;
			}
			new_val <<= 6;
		} else if (strcmp(cmd_name, P_BITS_STR) == 0) {
			switch (cmd_val) {
				case FROM_VLAN1:
				case FROM_VLAN2:
					new_val = cmd_val & MOD_MAX_VAL;
					break;
					
				case NEW_VALUE:
					new_val = 0x2;
			 		break;
					
				/* TODO: support PB value settings for yellow/green */
				case PB_GREEN:
			 	case PB_YELLOW:
					new_val = 0x3;
					break;

				default:
					ERR_PR("VLAN conf cmd_name %s does not support 0x%x\n",
						cmd_name, cmd_val);
			 		return false;
			}
			new_val <<= 8;
		} else {
			ERR_PR("non supported VLAN command %s command val 0x%x\n", cmd_name, cmd_val);
			ERR_PR("cammand format [tp=,vid=,cfi=,pb=]\n");
			return false;
		}
		*word |= new_val;
	}

	return true;
}


/******************************************************************************
 *
 * Function   : handle_delete
 *              
 * Description: handles the delete modification command
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		 word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_delete(char 		*cmd_data_str,
			   unsigned short	*word)
{
	char	*locstr = cmd_data_str;
	int	cmd_val;
	char	cmd_name[20];
	
	*word = 0;

	/* get and build the CSV values */
	while (1) {
		locstr = get_csv_value((char*)locstr, cmd_name, &cmd_val);
		if (NULL == locstr)
			break;

	 	DEBUG_PR(DEB_OTHER, "cmd_val=0x%x locstr[%s] cmd_name[%s]\n",
			cmd_val, locstr, cmd_name);

		if (strcmp(cmd_name, SKIP_AFTER_STR) == 0) {
			*word |= (cmd_val & 0xF) << 12;
		} else if (strcmp(cmd_name, SKIP_BEFORE_STR) == 0) {
			*word |= (cmd_val & 0xF) << 8;
		} else if (strcmp(cmd_name, "del") == 0) {
			*word |= (cmd_val & 0xFF);
		} else {
			ERR_PR("non supported DELETE command %s command val 0x%x\n", cmd_name, cmd_val);
			ERR_PR("cammand format [skipa=,skipb=,del=]\n");
			return false;
		}
	}

	return true;
}


/******************************************************************************
 *
 * Function   : handle_add_vlan
 *              
 * Description: handles the add VLAN modification command
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		 word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_add_vlan(char 		*cmd_data_str,
			     unsigned short	*word)
{
	char	*locstr = cmd_data_str;
	int	cmd_val;
	char	cmd_name[20];
	
	*word = 0;

	/* get and build the CSV values */
	while (1) {
		locstr = get_csv_value((char*)locstr, cmd_name, &cmd_val);
		if (NULL == locstr)
			break;

	 	DEBUG_PR(DEB_OTHER, "cmd_val=0x%x locstr[%s] cmd_name[%s]\n",
			cmd_val, locstr, cmd_name);

		if (strcmp(cmd_name, VID_STR) == 0) {
			*word |= (cmd_val & 0xFFF);
		} else if (strcmp(cmd_name, CFI_STR) == 0) {
			*word |= (cmd_val & 0x1) << 12;
		} else if (strcmp(cmd_name, P_BITS_STR) == 0) {
			*word |= (cmd_val & 0x7) << 13;
		} else {
			ERR_PR("non supported VLAN command %s command val 0x%x\n", cmd_name, cmd_val);
			ERR_PR("cammand format [vid=,cfi=,pb=]\n");
			return false;
		}
	}

	return true;
}


/******************************************************************************
 *
 * Function   : handle_conf_dsa1
 *              
 * Description: handles the config DSA modification command
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_conf_dsa1(char 		*cmd_data_str,
			     unsigned short	*word)
{
	char	*locstr = cmd_data_str;
	int	cmd_val;
	char	cmd_name[40];
	
	*word = 0;

	/* get and build the CSV values */
	while (1) {
		locstr = get_csv_value((char*)locstr, cmd_name, &cmd_val);
		if (NULL == locstr)
			break;

	 	DEBUG_PR(DEB_OTHER, "cmd_val=0x%x locstr[%s] cmd_name[%s]\n",
			cmd_val, locstr, cmd_name);

		if (strcmp(cmd_name, VID_STR) == 0) {
			*word |= (cmd_val & 0x3);
		} else if (strcmp(cmd_name, CFI_STR) == 0) {
			*word |= (cmd_val & 0x3) << 2;
		} else if (strcmp(cmd_name, P_BITS_STR) == 0) {
			*word |= (cmd_val & 0x3) << 4;
		} else if (strcmp(cmd_name, SRC_TAGGED_STR) == 0) {
			*word |= (cmd_val & 0x1) << 6;
		} else if (strcmp(cmd_name, SRC_PORT_STR) == 0) {
			*word |= (cmd_val & 0x3f) << 7;
		} else if (strcmp(cmd_name, TAG_CMD_STR) == 0) {
			*word |= (cmd_val & 0x3) << 13;
		} else if (strcmp(cmd_name, VLAN_SEL_STR) == 0) {
			*word |= (cmd_val & 0x1) << 15;
		} else {
			ERR_PR("non supported conf_das1 command %s command val 0x%x\n", cmd_name, cmd_val);
			ERR_PR("cammand format [%s=,%s=,%s=,%s=,%s=,%s=,%s=]\n", 
			       VID_STR,
			       CFI_STR,
			       P_BITS_STR,
			       SRC_TAGGED_STR,
			       SRC_PORT_STR,
			       TAG_CMD_STR,
			       VLAN_SEL_STR);
			return false;
		}
	}

	return true;
}

/******************************************************************************
 *
 * Function   : handle_conf_dsa2
 *              
 * Description: handles the config DSA modification command
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_conf_dsa2(char 		*cmd_data_str,
			     unsigned short	*word)
{
	char	*locstr = cmd_data_str;
	int	cmd_val;
	char	cmd_name[40];
	
	*word = 0;

	/* get and build the CSV values */
	while (1) {
		locstr = get_csv_value((char*)locstr, cmd_name, &cmd_val);
		if (NULL == locstr)
			break;

	 	DEBUG_PR(DEB_OTHER, "cmd_val=0x%x locstr[%s] cmd_name[%s]\n",
			cmd_val, locstr, cmd_name);

		if (strcmp(cmd_name, SRC_DEV_STR) == 0) {
			*word |= (cmd_val & 0x1f);
		} else if (strcmp(cmd_name, SRC_ID_STR) == 0) {
			*word |= (cmd_val & 0x1f) << 5;
		} else if (strcmp(cmd_name, VIDX_SEL_STR) == 0) {
			*word |= (cmd_val & 0x1) << 10;
		} else if (strcmp(cmd_name, USE_VIDX_STR) == 0) {
			*word |= (cmd_val & 0x1) << 11;
		} else if (strcmp(cmd_name, SRC_ID_SEL_STR) == 0) {
			*word |= (cmd_val & 0x1) << 12;
		} else if (strcmp(cmd_name, SRC_DEV_SEL_STR) == 0) {
			*word |= (cmd_val & 0x1) << 13;
		} else if (strcmp(cmd_name, SRC_PORT_SEL_STR) == 0) {
			*word |= (cmd_val & 0x1) << 14;
		} else if (strcmp(cmd_name, SRC_EGRESS_FILTER_STR) == 0) {
			*word |= (cmd_val & 0x1) << 15;
		} else {
			ERR_PR("non supported conf_das2 command %s command val 0x%x\n", cmd_name, cmd_val);
			ERR_PR("cammand format [%s=,%s=,%s=,%s=,%s=,%s=,%s=,%s=]\n", 
			       SRC_DEV_STR,
			       SRC_ID_STR,
			       VIDX_SEL_STR,
			       USE_VIDX_STR,
			       SRC_ID_SEL_STR,
			       SRC_DEV_SEL_STR,
			       SRC_PORT_SEL_STR,
			       SRC_EGRESS_FILTER_STR);
			return false;
		}
	}

	return true;
}

/******************************************************************************
 *
 * Function   : handle_decr_ttl
 *              
 * Description: handles the decrement TTL modification command
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		 word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_decr_ttl(char 		*cmd_data_str,
			     unsigned short	*word)
{
	char	*locstr = cmd_data_str;
	int	cmd_val;
	char	cmd_name[20];
	
	*word = 0;

	/* get and build the CSV values */
	while (1) {
		locstr = get_csv_value((char*)locstr, cmd_name, &cmd_val);
		if (NULL == locstr)
			break;

	 	DEBUG_PR(DEB_OTHER, "cmd_val=0x%x locstr[%s] cmd_name[%s]\n",
			cmd_val, locstr, cmd_name);

		if (strcmp(cmd_name, SKIP_BEFORE_STR) == 0) {
			*word |= (cmd_val & 0xFF) << 8;
		} else if (strcmp(cmd_name, SKIP_AFTER_STR) == 0) {
			*word |= (cmd_val & 0xFF);
		} else {
			ERR_PR("non supported command %s command val 0x%x\n", cmd_name, cmd_val);
			ERR_PR("cammand format [skipb=,skipa=]\n");
			return false;
		}
	}

	return true;
}


/******************************************************************************
 *
 * Function   : handle_pppoe
 *              
 * Description: handles the PPPoE modification command
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		 word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_pppoe(char			*cmd_data_str,
			  unsigned short	*word)
{
	char	*locstr = cmd_data_str;
	int	cmd_val;
	char	cmd_name[20];
	
	*word = 0;

	/* get and build the CSV values */
	while (1) {
		locstr = get_csv_value((char*)locstr, cmd_name, &cmd_val);
		if (NULL == locstr)
			break;

	 	DEBUG_PR(DEB_OTHER, "cmd_val=0x%x locstr[%s] cmd_name[%s]\n",
			cmd_val, locstr, cmd_name);

		if (strcmp(cmd_name, SESSID_STR) == 0) {
			*word |= cmd_val & 0xFFFF;
		} else {
			ERR_PR("non supported command %s command val 0x%x\n", cmd_name, cmd_val);
			ERR_PR("cammand format [sessid=]\n");
			return false;
		}
	}

	return true;
}

/******************************************************************************
 *
 * Function   : handle_add_calc_len
 *              
 * Description: handles the decrement calculate length modification command
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		 word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_add_calc_len(char		*cmd_data_str,
			  	unsigned short	*word)
{
	char	*locstr = cmd_data_str;
	int	cmd_val;
	char	cmd_name[20];
	
	*word = 0;

	/* get and build the CSV values */
	while (1) {
		locstr = get_csv_value((char*)locstr, cmd_name, &cmd_val);
		if (NULL == locstr)
			break;

	 	DEBUG_PR(DEB_OTHER, "cmd_val=0x%x locstr[%s] cmd_name[%s]\n",
			cmd_val, locstr, cmd_name);

		if (strcmp(cmd_name, LEN_SRC) == 0) {
			*word |= (cmd_val & 0x3) << 14;
		} else if (strcmp(cmd_name, L3_OFFSET) == 0) {
			*word |= (cmd_val & 0x3) << 12;
		} else if (strcmp(cmd_name, IP_HEADER_LEN) == 0) {
			*word |= (cmd_val & 0x3) << 10;
		} else if (strcmp(cmd_name, LEN_OP) == 0) {
			*word |= (cmd_val & 0x1) << 9;
		} else if (strcmp(cmd_name, MOD_VALUE) == 0) {
			*word |= (cmd_val & 0xffff);
		} else {
			ERR_PR("non supported command %s command val 0x%x\n", cmd_name, cmd_val);
			ERR_PR("cammand format [lensrc=,l3off=,iphdlen=,lenop=,modval=]\n");
			return false;
	 	}
	}

	return true;
}

#if 0
/******************************************************************************
 *
 * Function   : handle_jump
 *              
 * Description: handles the jump modification command
 *              
 * Parameters : cmd_data_str - command string in field=value format
 *		 word - updated 2 byte command (output)
 *
 * Returns    : bool (success=true/error=false)
 *              
 ******************************************************************************/
static bool handle_jump(char 		*cmd_data_str,
		 	unsigned short	*word)
{
	char	*locstr = cmd_data_str;
	int	cmd_val;
	char	cmd_name[20];
	
	*word = 0;

	/* get and build the CSV values */
	while (1) {
		locstr = get_csv_value((char*)locstr, cmd_name, &cmd_val);
		if (NULL == locstr)
			break;

	 	DEBUG_PR(DEB_OTHER, "cmd_val=0x%x locstr[%s] cmd_name[%s]\n",
			cmd_val, locstr, cmd_name);

		if (strcmp(cmd_name, JUMP_COMMAND) == 0) {
			*word |= (cmd_val & 0x3);
		} else if (strcmp(cmd_name, JUMP_INDEX) == 0) {
			*word |= (cmd_val & 0x3FFF) << 2;
		} else {
			ERR_PR("non supported VLAN command %s command val 0x%x\n", cmd_name, cmd_val);
			return false;
		}
	}

	return true;
}
#endif


/* this array defines per command code that handling routine */
mod_cmd_handle_entry_s mod_cmd_handle_arr[] =
{
	{"none",	NULL			},
	{"add",		handle_2b		},
	{"cfg_vlan",	handle_cfg_vlan		},
	{"add_vlan",	handle_add_vlan		},
	{"cfg_dsa1",	handle_conf_dsa1	},
	{"cfg_dsa2",	handle_conf_dsa2	},
	{"add_dsa",	NULL			},
	{"delete",	handle_delete		},
	{"repl_2b",	handle_2b		},
	{"repl_lsb",	handle_repl_lsb		},
	{"repl_msb",	handle_repl_msb		},
	{"repl_vlan",	handle_add_vlan		},
	{"decr_lsb",	handle_decr_ttl		},
	{"decr_msb",	handle_decr_ttl		},
	{"add_calc_len",handle_add_calc_len	},
	{"repl_len",	handle_add_calc_len	},
	{"repl_ipv4_chk",handle_2b		},
	{"repl_l4_chk",	handle_2b		},
	{"skip",	handle_add_calc_len	},
	{"jmp",		handle_2b		},
	{"jmp_skip",	handle_2b		},
	{"jsr",		handle_2b		},
	{"pppoe",	handle_pppoe		},
	{"store",	handle_add_calc_len	},
	{"add_ipv4_chk",handle_2b		},
	{"pppoe2",	handle_2b		},
	{"repl_mid",	handle_2b		},
	{"mltpl_add",	handle_2b		},
	{"mltpl_repl",	handle_2b		},
	{"repl_rmv_2b",	handle_2b		},
	{"add_ipv6_hdr",handle_2b		},
	{"pkt_drop",	NULL			},
};


/******************************************************************************
 *
 * Function   : parse_mod_data
 *              
 * Description: parse the mod_cmd xml sheet and extract data fileds
 *              
 * Parameters : cmd_code - modification command code
 *		 mod_cmd_data_str - command string in field=value format
 *		 word - updated 2 byte command (output)
 *
 * Returns    : int (success=0/error=1)
 *              
 ******************************************************************************/
int parse_mod_data(int			cmd_code,
		    char		*mod_cmd_data_str,
		    unsigned short	*word)
{
	mod_cmd_handle_entry_s	*p_mod_cmd_entry;

	*word = 0;

	if (cmd_code > sizeof(mod_cmd_handle_arr)/sizeof(mod_cmd_handle_arr[0])) {
		ERR_PR("mod command code %d is out of range\n", cmd_code);
		return 1;
	}

	p_mod_cmd_entry = &mod_cmd_handle_arr[cmd_code];

	if (p_mod_cmd_entry->handler == NULL) {
		DEBUG_PR(DEB_OTHER, "NULL handler for command code %d %s\n", cmd_code, mod_cmd_data_str);
		return 0;
	}
	
	if ((p_mod_cmd_entry->handler)(mod_cmd_data_str, word) == false) {
		ERR_PR("handling failed for %s %s\n", p_mod_cmd_entry->name, mod_cmd_data_str);
		return 1;
	}

	return 0;
}

