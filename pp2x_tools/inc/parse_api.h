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

#ifndef _PARSE_API_H_
#define _PARSE_API_H_

#ifdef __cplusplus
extern "C" {
#endif

int parse_xml_config(char *xmlFile);
int parse_xml_prs_init(char *xmlFile);
int parse_xml_prs(char *xmlFile);
int parse_xml_cls_flows(char *xmlFile);
int parse_xml_c2(char *xmlFile);
int parse_xml_c2_qos_dscp(char *xmlFile);
int parse_xml_c2_qos_pri(char *xmlFile);
int parse_xml_cls_module(char *xmlFile);
int parse_xml_c3(char *xmlFile);
int parse_xml_c4(char *xmlFile);
int parse_xml_c4_ruleset(char *xmlFile);
int parse_xml_mod_cmd(char *xmlFile);
int parse_xml_mod_data1(char *xmlFile);
int parse_xml_mod_data2(char *xmlFile);
int parse_xml_mod_cfg(char *xmlFile);
int parse_xml_config_info(char *xmlFile);
int parse_xml_mc(char *xmlFile);
int parse_xml_console(char *xmlFile, bool);

#ifdef __cplusplus
}
#endif

#endif //_PARSE_API_H_

