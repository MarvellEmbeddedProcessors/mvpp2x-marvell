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

#ifndef _COMMON_H_
#define _COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TOOL_MAIN_VER_STR	"2.3"
#define TOOL_SUB_VER_STR	"0"

#define DEB_LEVEL		debug_level
#define DEB_XML		        1
#define DEB_SYSFS		2
#define DEB_OTHER		3

extern unsigned int DEB_LEVEL;

#define DEBUG_PR(level, format , ...)							\
{											\
	if ((level == DEB_LEVEL))							\
		printf("DEBUG %s(%d) "format , __func__ , __LINE__ , ##__VA_ARGS__);	\
}

#define ERR_PR(format , ...) \
	printf("ERR %s(%d) "format , __func__ , __LINE__ , ##__VA_ARGS__);

#define PP2_SYSFS_PATH		"/sys/devices/platform/pp2"
#define C2_SYSFS_PATH		"/sys/devices/platform/pp2/cls2"
#define C3_SYSFS_PATH		"/sys/devices/platform/pp2/cls3"
#define C4_SYSFS_PATH		"/sys/devices/platform/pp2/cls4"
#define CLS_SYSFS_PATH		"/sys/devices/platform/pp2/cls"
#define PME_SYSFS_PATH		"/sys/devices/platform/pp2/pme"
#define PRS_SYSFS_PATH		"/sys/devices/platform/pp2/prs/debug"
#define MC_SYSFS_PATH		"/sys/devices/platform/pp2/mc"
#define RSS_SYSFS_PATH		"/sys/devices/platform/pp2/rss"

typedef enum {
	SHEET_PRS_INIT,
	SHEET_PRS,
	SHEET_CLS,
	SHEET_C2,
	SHEET_C2_PRI,
	SHEET_C2_DSCP,
	SHEET_C3,
	SHEET_C4_RULESET,
	SHEET_C4,
	SHEET_MOD_CFG,
	SHEET_MOD_CMD,
	SHEET_MOD_DATA1,
	SHEET_MOD_DATA2,
	SHEET_MC,
	SHEET_RSS,
	MAX_SHEET
} EXCEL_SHEETS;


typedef struct 
{
	char	*name;
	int	(*parse_routine)(char *xmlFile);
	char	parse;
} parse_sheet_s;

#ifdef __cplusplus
}
#endif

#endif //_COMMON_H_
