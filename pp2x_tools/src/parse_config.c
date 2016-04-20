/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PPv2 Tool                                                  **/
/**                                                                          **/
/**  FILE        : parse_invalid.c                                               **/
/**                                                                          **/
/**  DESCRIPTION : This file contains parse of invalid info                  **/
/**                                                                          **/
/******************************************************************************
**                                                                          
*   MODIFICATION HISTORY:                                                   
*                
*    15-Nov-12   Evan  - initial version created.                              
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
#include "PncGlobals.h"


enum prs_config_data{
	TOOL_VERSION,
	PRS_INVALID,
	CLS_INVALID,
	C2_INVALID,
	C3_INVALID,
	C4_INVALID,
	C5_INVALID,
	MOD_INVALID,
	MC_INVALID,
	PPV2_CONFIG_MAX
};

/*******************************************************************************
* build_config_action_sysfs()
*
* DESCRIPTION: Parse the config info from XML, and generate sysfs command to
*		write the configration to HW
*
* INPUTS:   prs_init_data
*
* OUTPUTS:  None
*
* RETURNS:
*
*******************************************************************************/
static int build_config_action_sysfs(xml_entry_data *config_data)
{
 char sysfs_buf[512];
	DdEntry *temp;
	unsigned int data, i;

	if (strcmp(ezxml_txt(config_data[TOOL_VERSION].xmlEntry), TOOL_MAIN_VER_STR)) {
		ERR_PR("Tool version mismatch! Excel: %s, tool: %s\n", 
			ezxml_txt(config_data[TOOL_VERSION].xmlEntry),
			TOOL_MAIN_VER_STR);
		return PPV2_RC_FAIL;
	}

	for (i = PRS_INVALID; i < PPV2_CONFIG_MAX; i++) {
		temp = findMatchingEntry(ezxml_txt(config_data[i].xmlEntry));
		if (temp != NULL)
			data = atoi((char*)temp->value);
		else
			data = 0;
		if (data) {
			switch(i) {
				case PRS_INVALID:
					sprintf(sysfs_buf, "echo 1 > %s/hw_inv_all\n",
						PRS_SYSFS_PATH);
					handle_sysfs_command(sysfs_buf, false);
					break;
				case CLS_INVALID:
					break;
				case C2_INVALID:
					break;
				case C3_INVALID:
					break;
				case C4_INVALID:
					break;
				case C5_INVALID:
					break;
				case MOD_INVALID:
					break;
				case MC_INVALID:
					break;
				default:
					return PPV2_RC_FAIL;
			}
		}
	}

	return PPV2_RC_OK;
}

/*******************************************************************************
* parse_xml_config_info()
*
* DESCRIPTION: Parse the invalid info from XML, and generate sysfs command to
*		do related operation
*
* INPUTS:   xmlFile	    - XML file contains the configuration
*
* OUTPUTS:  None
*
* RETURNS:  US_RC_OK, US_RC_FAIL or US_RC_NOT_FOUND
*
*******************************************************************************/
int parse_xml_config_info(char *xmlFile)
{
	ezxml_t xmlHead = NULL;
	ezxml_t xml_config, xmlEntry;
	unsigned int i;

	xml_entry_data  config_data[PPV2_CONFIG_MAX] = {
		{PPV2_VERSION, NULL},
		{PPV2_PRS_INVALID, NULL},
		{PPV2_CLS_INVALID, NULL},
		{PPV2_C2_INVALID, NULL},
		{PPV2_C3_INVALID, NULL},
		{PPV2_C4_INVALID, NULL},
		{PPV2_C5_INVALID, NULL},
		{PPV2_MOD_INVALID, NULL},
		{PPV2_MC_INVALID, NULL}
	};

	/* Read XML file */
	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
		return PPV2_RC_FAIL;
	}

	/* Get Worksheet config */
	xml_config = ezxml_get(xmlHead, WORKSHEET_CONFIG, -1);
	if (xml_config == NULL){
		ERR_PR("%Failed to get %s\n", WORKSHEET_CONFIG);
		ezxml_free(xmlHead);
		return PPV2_RC_FAIL;
	}
	/* Find the first entry */
	xmlEntry = ezxml_child(xml_config, TABLE_ENTRY);
	if (xmlEntry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n", WORKSHEET_PRS_INIT);
		ezxml_free(xmlHead);
		return PPV2_RC_OK;
	}

	 /* Scan All Entry */
	 for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		/* Parse entry member */
		for (i=0; i < PPV2_CONFIG_MAX ;i++) {
			config_data[i].xmlEntry = ezxml_child(xmlEntry, config_data[i].name);
			if (NULL == config_data[i].xmlEntry) {
				ERR_PR("%s is empty\n", config_data[i].name);
				continue;
			}
			/*else
				DEBUG_PR("%s=%s\n", prs_init_data[i].name, ezxml_txt(prs_init_data[i].xmlEntry));*/
		}

		if (build_config_action_sysfs(config_data))
			return PPV2_RC_FAIL;
	}

	ezxml_free(xmlHead);

	return PPV2_RC_OK;
}

