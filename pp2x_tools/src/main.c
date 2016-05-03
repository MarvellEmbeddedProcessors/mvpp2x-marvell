/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : main.c                                                    **/
/**                                                                          **/
/**  DESCRIPTION : This file contains main - CLI interface                   **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    17-Jun-09   zeev  - initial version created.                              
 *                                                                      
 ******************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>/*vbs*/

#include "common.h"
#include "PncGlobals.h"
#include "DataDictionary.h"
#include "PncDb.h"
#include "PncParse.h"
#include "PncShellCommand.h"
#include "parse_PRS.h"
#include "parse_api.h"

#define VBS_FILENAME		"ppv2_sysfs_cmd.vbs"
#define SHELL_FILENAME		"ppv2_sysfs_cmd.sh"

unsigned int DEB_LEVEL = 0;

static bool keepGoingFlag		= false;
static unsigned int execution_delay	= 6;

static char *defXmlFile    = "./default.xml";

int vbs_fd = 0;
int out_fd = 0;
int quiet = 0;

parse_sheet_s parse_section[MAX_SHEET] = {
	{ "PRS_init",	parse_xml_prs_init,	1},
	{ "PRS",	parse_xml_prs,		1},
	{ "CLS",	parse_xml_cls_module,	1},
	{ "C2",		parse_xml_c2,		1},
	{ "C2_pri",	parse_xml_c2_qos_pri,	1},
	{ "C2_dscp",	parse_xml_c2_qos_dscp,	1},
	{ "C3",		parse_xml_c3,		1},
	{ "C4_ruleset",	parse_xml_c4_ruleset,	1},
	{ "C4",		parse_xml_c4,		1},
	{ "MOD_cfg",	parse_xml_mod_cfg,	1},
	{ "MOD_cmd",	parse_xml_mod_cmd,	1},
	{ "MOD_d1",	parse_xml_mod_data1,	1},
	{ "MOD_d2",	parse_xml_mod_data2,	1},
	{ "MC",		parse_xml_mc,		1},
	{ "RSS",	parse_xml_rss,		1}
};

inline unsigned int get_execution_delay(void)
{
	return execution_delay;
}

inline void set_execution_delay(unsigned int new_execution_delay)
{
	execution_delay = new_execution_delay;
}


/******************************************************************************
 *
 * Function   : get_native_mode
 *              
 * Description: returns the native flag status
 *              
 * Parameters :  
 *
 * Returns    : void 
 *              
 ******************************************************************************/
bool get_native_mode(void)
{
#ifdef ARCH_ARM
	return true;
#else
	return false;
#endif
}


/******************************************************************************
 *
 * Function   : setKeepGoingFlag
 *              
 * Description: The function sets the keepGoing flag
 *              
 * Parameters :  
 *
 * Returns    : void 
 *              
 ******************************************************************************/
void setKeepGoingFlag(bool flag)
{
    keepGoingFlag = flag;
}


/******************************************************************************
 *
 * Function   : getKeepGoingFlag
 *              
 * Description: The function returns the keepGoing flag
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/
bool getKeepGoingFlag()
{
    return keepGoingFlag;
}



/******************************************************************************
 *
 * Function   : setDebugFlag
 *              
 * Description: The function sets the debug flag
 *              
 * Parameters :  
 *
 * Returns    : void 
 *              
 ******************************************************************************/
void setDebugFlag(unsigned int flag)
{
    DEB_LEVEL = flag;
}


/******************************************************************************
 *
 * Function   : getDebugFlag
 *              
 * Description: The function returns the debug flag
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/
unsigned int getDebugFlag()
{
    return DEB_LEVEL;
}

/******************************************************************************
 *
 * Function   : usage
 *              
 * Description: The function prints usage message and causes program exit
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/
void usage(char *name)
{
    printf("Usage: %s [-h] [-d] [-k] [-q] [-s sheet_name] [-e delay-val] [XML file]\n", name);
    printf("    Default XML file: %s\n", defXmlFile);
    printf("    -h: print this message\n");
    printf("    -d: print debug/progress information\n");
    printf("    -k: keep going even if PnC parse error occurs\n");
    printf("    -s: select the excel sheet to parse\n");
    printf("    -e: set execution delay between PnC entries (def. 6 milliseconds)\n");
    printf("    -q: suppress stdout print\n");

    exit(0);
}


/******************************************************************************
 *
 * Function   : set_parse_sheet
 *
 * Description: sets the parse flag for the specified Excel sheet
 *
 * Parameters : sheet_name - the sheets that needs to be parsed
 *
 * Returns    : int
 *
 ******************************************************************************/
static int set_parse_sheet(char *sheet_name)
{
	unsigned int	i;
	static  int	parse_selection_clear = 1;

	if (NULL == sheet_name)
		goto incorrect_param;

	if (parse_selection_clear){
		for (i = SHEET_PRS_INIT; i < MAX_SHEET; i++) 
			parse_section[i].parse = 0;
		parse_selection_clear = 0;
	}
	
	for (i = SHEET_PRS_INIT; i < MAX_SHEET; i++) {
		if (!strcmp(sheet_name, parse_section[i].name)){
			parse_section[i].parse = 1;
			return 0;
		}
	}

incorrect_param:

       ERR_PR("Sheet name %s unknown, supported sheet name:\n", sheet_name);
	
	for (i = SHEET_PRS_INIT; i < MAX_SHEET; i++) {
		printf("%s ", parse_section[i].name);
	}
	printf("\n");
	
	return 1;
}


/******************************************************************************
 *
 * Function   : parse_xml_file
 *
 * Description: parses the XML file according to enabled Excel sheets flags
 *
 * Parameters : xmlFile - input XML file
 *
 * Returns    : int
 *
 ******************************************************************************/
int parse_xml_file(char *xmlFile)
{
	int	rc;
	int	i;

	rc = parse_xml_console(xmlFile, true);
	if (rc != 0)
		return rc;

	rc = parse_xml_config_info(xmlFile);
	if (rc != 0)
		return rc;

	for (i = SHEET_PRS_INIT; i < MAX_SHEET; i++)
		if ((parse_section[i].parse) && (NULL != parse_section[i].parse_routine)) {
			rc = parse_section[i].parse_routine(xmlFile);
			if (rc != 0){
				ERR_PR("parsing %s FAILED\n", parse_section[i].name);
				return rc;
			}
		}

	rc = parse_xml_console(xmlFile, false);
	if (rc != 0)
		return rc;
		
	return rc;
}


 /* generate vbs file */
int ppv2_vbs_header_build()
{
	unlink(VBS_FILENAME);
	vbs_fd = fopen(VBS_FILENAME, "w");
	if (vbs_fd < 0) {
		printf("%s file open failed\n", VBS_FILENAME);
		return vbs_fd;
	}
	/* build vbs header */
	fprintf(vbs_fd, "#$language = \"VBScript\"");
	fprintf(vbs_fd, "\n#$interface = \"1.0\"");
	fprintf(vbs_fd, "\nDim screen");
	fprintf(vbs_fd, "\nSet screen = crt.Screen");
	fprintf(vbs_fd, "\nNewline = chr(13)");
	fprintf(vbs_fd, "\nSub Main");
	fprintf(vbs_fd, "\n\tscreen.Send \"debug shell\" & Newline");
	fprintf(vbs_fd, "\n\tscreen.WaitForString \"/ # \",1");

	return 0;
}

int ppv2_outfile_open()
{
	unlink(SHELL_FILENAME);
	out_fd = fopen(SHELL_FILENAME, "w");
	if (out_fd < 0) {
		printf("%s file open failed\n", SHELL_FILENAME);
		return out_fd;
	}
	fprintf(out_fd, "\#\! \/bin\/sh\n");
	return 0;
}

int ppv2_vbs_end_build()
{
	fprintf(vbs_fd, "\nEnd sub");
	fclose(vbs_fd);

	return 0;
}

int ppv2_outfile_close()
{
	fclose(out_fd);

	return 0;
}


int main(int argc, char *argv[])
{
	int	indx, i, debug_level = 0;
	char	*nxtArg;
	char	*xmlFile = defXmlFile;
	int 	fileCount = 0;
		
	// Startup print
	printf("PPv2 Tool %s_%s\n\n", TOOL_MAIN_VER_STR, TOOL_SUB_VER_STR);

	for (indx = 1; indx < argc; indx++) {
		nxtArg = argv[indx];

		if (nxtArg[0] == '-') {
	 		if (strlen(nxtArg) > 5) {
				usage(argv[0]);
			}
			if (nxtArg[1] == 'h') {
				usage(argv[0]);
			} else if (nxtArg[1] == 'd') {
		 		for (i=0;i<4; i++)
					if (nxtArg[1+i] == 'd')
						debug_level++;
			 	setDebugFlag(debug_level);
			} else if (nxtArg[1] == 'q') {
				quiet = 1;
			} else if (nxtArg[1] == 'k') {
				setKeepGoingFlag(true);
			} else if (nxtArg[1] == 's') {
				indx++;
				if (set_parse_sheet(argv[indx]))
					usage(argv[0]);
			} else if (nxtArg[1] == 'n') {
				//setNoPrintPncRowAnalysisFlag(true);
			} else if (nxtArg[1] == 'e') {
				/* Set execution delay value */
				indx++;
				set_execution_delay(atoi(argv[indx]));

				if (get_execution_delay() == 0) {
					printf("Invalid delay value - %s\n", argv[indx]);  
					usage(argv[0]);
				}
			} else {
				printf("Unsupported option -%c\n", nxtArg[1]);  
				usage(argv[0]);
			}
		} else {
			if (fileCount == 0) {
				xmlFile = argv[indx];
				fileCount++;
			} else {
				usage(argv[0]);
			}
		}
	}

	// Announce files used
	printf("using %s XML file\n", xmlFile);
    
	if (prepareDataDictionary(xmlFile) == true)
    	{        
		printDictionary();
		
		/* generate vbs file */
		if (ppv2_vbs_header_build())
			exit(1);
		
		if (ppv2_outfile_open())
			exit(1);
		
		if (parse_xml_file(xmlFile))
			exit(1);
		
		ppv2_vbs_end_build();
		ppv2_outfile_close();
	} else
		exit(1);

	return 0;
}
