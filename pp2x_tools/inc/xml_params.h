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

#ifndef __INCxml_paramsh
#define __INCxml_paramsh

/* Empty string */
#define EMPTY_STR ""
#define EMPTY_START "*"
/*
 * all the tags name in XML
 */

#define TABLE_ENTRY			"entry"

/* worksheet names */
#define WORKSHEET_CONFIG		"cfg"
#define WORKSHEET_PRS_INIT		"PRS_init"
#define WORKSHEET_PRS			"PRS"
#define WORKSHEET_CLS_CONFIG		"CLS_cfg"
#define WORKSHEET_CLS			"CLS"
#define WORKSHEET_CLS_FLOWS		"CLS_flows"
#define WORKSHEET_C2			"C2"
#define WORKSHEET_C2_PRI		"C2_pri"
#define WORKSHEET_C2_DSCP		"C2_dscp"
#define WORKSHEET_C3			"C3"
#define WORKSHEET_C4			"C4"
#define WORKSHEET_C4_RULESET		"C4_ruleset"
#define WORKSHEET_MOD_CMD		"mod_cmd"
#define WORKSHEET_MOD_DATA1		"mod_d1"
#define WORKSHEET_MOD_DATA2		"mod_d2"
#define WORKSHEET_MOD_CFG		"mod_cfg"
#define WORKSHEET_MC_TABLE		"MC"
#define WORKSHEET_RSS_TABLE		"RSS"
#define WORKSHEET_CONSOLE		"cnsl"
#define WORKSHEET_DICTIONARY		"dictionary"

/* Config */
#define PPV2_VERSION                  "version"
#define PPV2_PRS_INVALID              "PRS_invalidate"
#define PPV2_CLS_INVALID              "CLS_invalidate"
#define PPV2_C2_INVALID               "C2_invalidate"
#define PPV2_C3_INVALID               "C3_invalidate"
#define PPV2_C4_INVALID               "C4_invalidate"
#define PPV2_C5_INVALID               "C5_invalidate"
#define PPV2_MOD_INVALID              "mod_invalidate"
#define PPV2_MC_INVALID               "MC_invalidate"

/* PRS_init */
#define PORT_NUM                   "port_num"
#define INIT_LU_ID                 "init_lu_id"
#define INIT_LU_OFFSET             "init_lu_offset"
#define MAX_LOOP                   "max_loop"
/* PRS */
#define TCAM_IDX                   "tcam_idx"
#define TCAM_LOOKUP_ID             "tcam_lu_id"
#define TCAM_PHYSPORTID            "tcam_physPortID"
#define TCAM_AI                    "tcam_ai"
#define TCAM_PCKT_HEADER           "tcam_pckt_header"
#define RI_MAC2ME                  "ri_mac2me"
#define RI_ETY_DSA                 "ri_etype_dsa"
#define RI_VLAN                    "ri_vlan"
#define RI_CPU_CODE                "ri_cpucode"
#define RI_L2_VERSION              "ri_l2_ver"
#define RI_L2_CAST                 "ri_l2_cast"
#define RI_PPPOE                   "ri_pppoe"
#define RI_UDF1_L3                 "ri_udf1_l3"
#define RI_L3_CAST                 "ri_l3_cast"
#define RI_IP_FRAG                 "ri_ip_frag"
#define RI_UDF2_IPV6               "ri_udf2_ipv6"
#define RI_UDF3                    "ri_udf3"
#define RI_UDF4_L4                 "ri_udf4_l4"
#define RI_UDF5                    "ri_udf5"
#define RI_UDF6                    "ri_udf6"
#define RI_UDF7                    "ri_udf7"
#define RI_DROP                    "ri_drop"
#define NLU_OFFSET                 "nlu_offset"
#define NLU_OFFSET_SIGN            "nlu_offset_sign"
#define UDF_OFFSET                 "udf_offset"
#define UDF_OFFSET_SIGN            "udf_offset_sign"
#define UDF_OFFSET_TYPE            "udf_offset_type"
#define OP_SEL                     "op_sel"
#define NLU_AI                     "nlu_ai"
#define NLU_LU_ID                  "nlu_lu_id"
#define LU_DONE                    "lu_done"
#define CLS_LU_ID_GEN              "cls_lu_id_gen"
/* CLS_config */
#define ACTIVE                     "active"
#define PORT_LOOKUP_WAY            "PortLookupWay"
#define SPID                       "SPID"
#define CLS_ADDITIONAL_FILEDS      "CLSAdditionalFileds"
/* CLS */
#define WAY                        "way"
#define LU_ID                      "LU_ID"
#define INIT_CPU_Q                 "InitCPUQ"
#define HWFM_INSTRUCT_OFFSET       "HWFMInstrucOffset"
#define LU_ENABLED                 "LUenabled"
#define FLOW_IDX                   "flowIdx"
/* CLS_flows */
#define INDEX                      "index"
#define NAME                       "name"
#define FLOW_TBL_IDX               "flowTblIdx"
#define PORT_IDTYPE                "PortIdType"
#define PORT_ID_SEL		   "PortIdSel"
#define PORT_ID                    "PortID"
#define ENGINE_NO                  "EngineNo"
#define PRIORITY                   "priority"
#define PPPOE_FILTER		   "pppoe"
#define VLAN_FILTER		   "vlan"
#define M2M_FILTER		   "m2m"
#define UDF7_FILTER		   "udf7"
#define SEQUENCE_CTRL		   "seqCtrl"
#define LU_TYPE                    "LU_type"
#define FIELD1ID                   "Field1ID"
#define FIELD2ID                   "Field2ID"
#define FIELD3ID                   "Field3ID"
#define FIELD4ID                   "Field4ID"
#define LAST                       "last"
/* C2 */
#define TCAM_INDEX                 "TCAM_index"
#define NAME                       "name"
#define LU_TYPE                    "LU_type"
#define PORT_IDTYPE                "PortIdType"
#define PORT_ID                    "PortID"
#define TCAM_DATA                  "TCAM_data"
#define QOS_TABLE_ID               "QOSTableID"
#define DSCP_TABLE_SEL             "DSCPTableSel"
#define PRI_DSCP_FROM_QOS          "PRI_DSCPfromQOS"
#define GEM_PORT_ID_FROM_QOS       "GEM_PORT_IDfromQOS"
#define QNO_LOW_FROM_QOS           "QNO_LOWfromQOS"
#define QNO_HIGH_FROM_QOS          "QNO_HIGHfromQOS"
#define COLOR_FROM_QOS             "COLORfromQOS"
#define COLOR_ACTION               "ColorAction"
#define PRI_ACTION                 "PRIAction"
#define PRI_VALUE                  "PRIValue"
#define DSCP_ACTION                "DSCPAction"
#define DSCP_VALUE                 "DSCPValue"
#define GEMPORTID_ACTION           "GemPortIdAction"
#define GEMPORTID		    "GemPortID"
#define QUEUE_LOW_ACTION           "QueueLowAction"
#define QUEUE_LOW_VALUE            "QueueLowValue"
#define QUEUE_HIGH_ACTION          "QueueHighAction"
#define QUEUE_HIGH_VALUE           "QueueHighValue"
#define FORWARDING                 "Forwarding"
#define POLICER_SELECT             "PolicerSelect"
#define POLICER_ID                 "PolicerId"
#define FLOWID_ENABLE              "FlowIDEnable"
#define HWFM_DPTR                  "HWFM_DPtr"
#define HWFM_IPTR                  "HWFM_IPtr"
#define HWF_L4_CHK_EN              "HWF_L4_Checksum_en"
#define MTU_INDEX		   "MTU_index"
#define HW_DUPLICATION_FLOWID      "HWDuplicationFlowId"
#define HW_DUPLICATION_COUNT       "HWDuplicationCount"
#define ENTRY_ID		   "Entry_ID"
#define MISS		   	   "miss"
/* C2_PRI */
#define INDEX                      "index"
#define TABLE_NO                   "table_no"
#define PRI                        "PRI"
#define COLOR                      "color"
#define QUEUE_NUMBER               "queue_number"
/* C2_DSCP */
#define INDEX                      "index"
#define TABLE_NO                   "table_no"
#define DSCP                       "DSCP"
#define COLOR                      "color"
#define GEM_PORT_ID                "GEMPortID"
#define QUEUE_NUMBER               "queue_number"
/* C3 */
#define TCAM_INDEX                 "TCAM_index"
#define NAME                       "name"
#define L4INFO                     "L4Info"
#define LU_TYPE                    "LU_type"
#define PORT_IDTYPE                "PortIdType"
#define PORT_ID                    "PortID"
#define HEK35_0                    "HEK35_0"
#define MULTIHASH_ENTRY_MODEL      "MultihashEntryModel"
#define MULTIHASH_TABLE_ADDRESS    "MultihashTableAddress"
#define EXTENSION_TABLE_ADDRESS    "ExtensionTableAddress"
#define INITIAL_HIT_COUNTER_VALUE  "InitialHitCounterValue"
#define COLOR_ACTION               "ColorAction"
#define QUEUE_LOW_ACTION           "QueueLowAction"
#define QUEUE_LOW_VALUE            "QueueLowValue"
#define QUEUE_HIGH_ACTION          "QueueHighAction"
#define QUEUE_HIGH_VALUE           "QueueHighValue"
#define FORWARDING                 "Forwarding"
#define POLICER_SELECT             "PolicerSelect"
#define POLICER_ID                 "PolicerId"
#define FLOW_ID_ENABLE             "FlowIDEnable"
#define FLOW_ID                    "HWDuplicationFlowId"
#define HWFM                       "HWFM"
#define HWFM_DPTR                  "HWFM_DPtr"
#define HWFM_IPTR                  "HWFM_IPtr"
#define HWF_L4CHK_ENB              "HWFL4ChkEnb"
#define HW_DUPLICATION_COUNT       "HWDuplicationCount"
#define INS_SEQ_INFO1		   "instruction_seq_info_1"
#define INS_SEQ_INFO2		   "instruction_seq_info_2"
#define INS_SEQ_INFO3		   "instruction_seq_info_3"
#define INS_SEQ_INFO4		   "instruction_seq_info_4"
#define INS_SEQ_INFO5		   "instruction_seq_info_5"
#define INS_SEQ_INFO6		   "instruction_seq_info_6"
#define INS_SEQ_INFO7		   "instruction_seq_info_7"
#define INS_SEQ_INFO8		   "instruction_seq_info_8"
/* C4 */
#define PORT_NO                    "port_no"
#define PORT_IDTYPE                "PortIdType"
#define RULESET_NO                 "ruleSetNo"
#define RULES_IN_RULESET           "rulesInRuleSet"
#define RULE_NO                    "ruleNo"
#define RULESSET_NO                "rulesSetNo"
#define FIELD0ID                   "Field0ID"
#define FIELD0OP_CODE              "Field0OpCode"
#define FIELD0COMPARE_DATA         "Field0CompareData"
#define FIELD1OP_CODE              "Field1OpCode"
#define FIELD1COMPARE_DATA         "Field1CompareData"
#define FIELD2OP_CODE              "Field2OpCode"
#define FIELD2COMPARE_DATA         "Field2CompareData"
#define FIELD3OP_CODE              "Field3OpCode"
#define FIELD3COMPARE_DATA         "Field3CompareData"
#define FIELD4OP_CODE              "Field4OpCode"
#define FIELD4COMPARE_DATA         "Field4CompareData"
#define FIELD5ID                   "Field5ID"
#define FIELD5OP_CODE              "Field5OpCode"
#define FIELD5COMPARE_DATA         "Field5CompareData"
#define L3INFO                     "L3Info"
#define L4INFO                     "L4Info"
#define MAC2ME                     "Mac2Me"
#define PPPOE                      "PPPOE"
#define VLAN                       "VLAN"
#define COLOR_ACTION               "ColorAction"
/* mod */
#define TPM_TI                     "TransmitPacketModificationTblIndex"
#define MODIFICATION_COMMAND       "ModificationCommand"
#define MODIFICATION_DATA          "ModificationData"
#define UPDATE_IPV4_CHCKSUM        "UpdateIPV4ChckSum"
#define UPDATE_L4CHCKSUM           "UpdateL4ChckSum"
#define LAST_COMMAND               "LastCommand"
#define COMMAND_TYPE               "CommandType"
#define MTU                        "MTU"
#define TPID0                      "TPID0"
#define TPID1                      "TPID1"
#define TPID2                      "TPID2"
#define TPID3                      "TPID3"
#define DEFAULT_VLANCFG0           "Defaultvlancfg0"
#define DEFAULT_VLANCFG1           "Defaultvlancfg1"
#define DEFAULT_VLANCFG2           "Defaultvlancfg2"
#define TTL_ZERO_FORWARD           "TTLZeroForward"
#define PPPOE_ETHERTYPE            "PPPOEEthertype"
#define PPPOE_CODE                 "PPPOECode"
#define PPPOE_TYPE                 "PPPOEType"
#define PPPOE_VER                  "PPPOEVer"
#define PPPOE_LENGTH_CONFIG        "PPPOELengthConfig"
#define PPPOE_PROTOCOL0            "PPPOEProtocol0"
#define PPPOE_PROTOCOL1            "PPPOEProtocol1"
/* MC_table */
#define MC_TABLE_INDEX             "MCTableIndex"
#define HWFM_DPTR                  "HWFM_DPtr"
#define HWFM_IPTR                  "HWFM_IPtr"
#define GEMPORTID_MOD_EN           "GEMPORTID_Mod_en"
#define GEMPORTID_VALUE            "GEMPORTID_Value"
#define PRI_MOD_EN                 "Pri_Mod_en"
#define PRI_VALUE                  "PRIValue"
#define DSCP_MOD_EN                "DSCP_Mod_en"
#define DSCP_VALUE                 "DSCPValue"
#define FRWD_TYPE                  "FRWD_Type"
#define QUEUE_NUMBER               "queue_number"
#define NEXT_MC_TABLE_INDEX        "NextMCTableIndex"
/* RSS_table */
#define RSS_ACCESS_MODE            "access_mode"
#define RSS_TABLE_INDEX            "rss_tbl_idx"
#define RXQ_OWNER                  "rxq_owner"
#define RSS_LINE_IDX               "line_index"
#define RXQ_NEW                    "rxq_new"
#define RSS_WIDTH                  "width"
#define HASH_SELECT                "hash_sel"
/* dictionary */
#define CNSL_DESC			"description"
#define CNSL_CMD			"command"
#define CNSL_FIRST			"first"

/* dictionary */
#define DICT_NAME			"name"
#define DICT_VALUE			"value"

/* function return macro */
#define PPV2_RC_OK           0
#define PPV2_RC_FAIL         1
#define PPV2_RC_NOT_FOUND    2

/* debug on/off macro */
//#define PPV2_DEBUG_PRINT

#define PRS_INIT_SYSFS_PATH "/sys/devices/platform/pp2/prs"


typedef struct {
	char 		*name;
	ezxml_t		xmlEntry;
} xml_entry_data;


#endif

