/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PPv2 Tool                                                 **/
/**                                                                          **/
/**  FILE        : parse_invalid.c                                           **/
/**                                                                          **/
/**  DESCRIPTION : This file contains parse of invalid info                  **/
/**                                                                          **/
/******************************************************************************/
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
#include "ParseUtils.h"

enum prs_console_cmd{
	CNSL_CMD_DESC_E,
	CNSL_CMD_E,
	CNSL_FIRST_E,
	CNSL_CMD_MAX_E
};


/*******************************************************************************
* parse_xml_console()
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
int parse_xml_console(char *xmlFile, bool first)
{
	ezxml_t 	xmlHead = NULL;
	ezxml_t 	xml_config, xmlEntry;
	unsigned int 	i;
	bool 		invoke_sysfs;
	char		sysfs_buf[512];
	char		*xmlDesc;
	char		*xmlCmd;
	char		*xmlFirst;

	/* Read XML file */
	xmlHead = ezxml_parse_file(xmlFile);
	if (xmlHead == NULL){
		ERR_PR("Failed to find %s\n", xmlFile);
		return PPV2_RC_FAIL;
	}

	/* Get Worksheet config */
	xml_config = ezxml_get(xmlHead, WORKSHEET_CONSOLE, -1);
	if (xml_config == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, sheet is empty\n", WORKSHEET_CONSOLE);
		ezxml_free(xmlHead);
		return PPV2_RC_OK;
	}
	/* Find the first entry */
	xmlEntry = ezxml_child(xml_config, TABLE_ENTRY);
	if (xmlEntry == NULL){
		DEBUG_PR(DEB_XML, "Skipping %s worksheet, no entries found\n", WORKSHEET_CONSOLE);
		ezxml_free(xmlHead);
		return PPV2_RC_OK;
	}

	sprintf(sysfs_buf, "##################	CONSOLE commands  ##################\n");
	handle_sysfs_command(sysfs_buf, true);

	 /* Scan All Entry */
	for (; xmlEntry; xmlEntry = ezxml_next(xmlEntry)) {
		xmlDesc = ezxml_attr(xmlEntry, CNSL_DESC);
		xmlCmd = ezxml_attr(xmlEntry, CNSL_CMD);
		if (xmlCmd == NULL) {
			printf("%s: Failed to get %s\n", __func__, CNSL_CMD);
			ezxml_free(xmlHead);
			return false;
		}
		
		xmlFirst = ezxml_attr(xmlEntry, CNSL_FIRST);
		if (xmlFirst == NULL) {
			printf("%s: Failed to get %s\n", __func__, CNSL_FIRST);
			ezxml_free(xmlHead);
			return false;
		}

		invoke_sysfs = false;
		if (strcmp("yes",xmlFirst) == 0){
			if (true == first)
				invoke_sysfs = true;
	 	}else{
			if (false == first)
				invoke_sysfs = true;
		}
		if (invoke_sysfs) {
			if (strcmp(xmlDesc, "") != 0) {
				sprintf(sysfs_buf, "########  cmd desc: %s  ########\n",xmlDesc);
				handle_sysfs_command(sysfs_buf, true);
			}
			
			sprintf(sysfs_buf, "%s\n",xmlCmd);
			handle_sysfs_command(sysfs_buf, false);
		}
	}

	ezxml_free(xmlHead);

	return PPV2_RC_OK;
}

