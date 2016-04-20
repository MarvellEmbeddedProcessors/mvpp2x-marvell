/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PPv2 Tool                                                  **/
/**                                                                          **/
/**  FILE        : parse_PRS.h                                              **/
/**                                                                          **/
/**  DESCRIPTION : header file                                               **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    23-Oct-12   Evan  - initial version created.                              
 *                                                                      
 ******************************************************************************/
#define PRS_HEADER_LEN 8
#define PRS_AI_STR_LEN 9 /*example: b0101xxxx*/

/* Result info bit and msk */

#define SRAM_RI_MAC2ME_BIT 0
#define SRAM_RI_MAC2ME_MASK (1 << SRAM_RI_MAC2ME_BIT)

#define SRAM_RI_ETY_DSA_BIT 1
#define SRAM_RI_ETY_DSA_MASK (1 << SRAM_RI_ETY_DSA_BIT)

#define SRAM_RI_VLAN_BIT 2
#define SRAM_RI_VLAN_MASK (0x3 << SRAM_RI_VLAN_BIT)

#define SRAM_RI_CPU_CODE_BIT 4
#define SRAM_RI_CPU_CODE_MASK (0x7 << SRAM_RI_CPU_CODE_BIT)

#define SRAM_RI_L2_VERSION_BIT 7
#define SRAM_RI_L2_VERSION_MASK (0x3 << SRAM_RI_L2_VERSION_BIT)

#define SRAM_RI_L2_CAST_BIT 9
#define SRAM_RI_L2_CAST_MASK (0x3 << SRAM_RI_L2_CAST_BIT)

#define SRAM_RI_PPPOE_BIT 11
#define SRAM_RI_PPPOE_MASK (0x1 << SRAM_RI_PPPOE_BIT)

#define SRAM_RI_UDF1_L3_BIT 12
#define SRAM_RI_UDF1_L3_MASK (0x7 << SRAM_RI_UDF1_L3_BIT)

#define SRAM_RI_L3_CAST_BIT 15
#define SRAM_RI_L3_CAST_MASK (0x3 << SRAM_RI_L3_CAST_BIT)

#define SRAM_RI_IP_FRAG_BIT 17
#define SRAM_RI_IP_FRAG_MASK (0x1 << SRAM_RI_IP_FRAG_BIT)

#define SRAM_RI_UDF2_IPV6_BIT 18
#define SRAM_RI_UDF2_IPV6_MASK (0x3 << SRAM_RI_UDF2_IPV6_BIT)

#define SRAM_RI_UDF3_BIT 20
#define SRAM_RI_UDF3_MASK (0x3 << SRAM_RI_UDF3_BIT)

#define SRAM_RI_UDF4_L4_BIT 22
#define SRAM_RI_UDF4_L4_MASK (0x7 << SRAM_RI_UDF4_L4_BIT)

#define SRAM_RI_UDF5_BIT 25
#define SRAM_RI_UDF5_MASK (0x3 << SRAM_RI_UDF5_BIT)

#define SRAM_RI_UDF6_BIT 27
#define SRAM_RI_UDF6_MASK (0x3 << SRAM_RI_UDF6_BIT)

#define SRAM_RI_UDF7_BIT 29
#define SRAM_RI_UDF7_MASK (0x3 << SRAM_RI_UDF7_BIT)

#define SRAM_RI_DROP_BIT 31
#define SRAM_RI_DROP_MASK (0x1 << SRAM_RI_DROP_BIT)


enum prs_init_entry_data{
	PORT_IDX,
	INIT_LOOKUP_ID,
	INIT_LOOKUP_OFF,
	MAX_LOOKUP,
	PRS_INIT_MAX
};

enum prs_entry_data{
	TCAM_ENTRY_INDEX,
	TCAM_ENTRY_LU_ID,
	TCAM_PHY_PORT_ID,
	TCAM_ENTRY_AI,
	TCAM_HEADER,
	SRAM_RI_MAC2ME,
	SRAM_RI_ETY_DSA,
	SRAM_RI_VLAN,
	SRAM_RI_CPU_CODE,
	SRAM_RI_L2_VERSION,
	SRAM_RI_L2_CAST,
	SRAM_RI_PPPOE,
	SRAM_RI_UDF1_L3,
	SRAM_RI_L3_CAST,
	SRAM_RI_IP_FRAG,
	SRAM_RI_UDF2_IPV6,
	SRAM_RI_UDF3,
	SRAM_RI_UDF4_L4,
	SRAM_RI_UDF5,
	SRAM_RI_UDF6,
	SRAM_RI_UDF7,
	SRAM_RI_DROP,
	SRAM_NEXT_LU_OFF,
	SRAM_NEXT_LU_OFF_SIGN,
	SRAM_UDF_OFF,
	SRAM_UDF_OFF_SIGN,
	SRAM_UDF_TYPE,
	SRAM_OP_SEL,
	SRAM_NEXT_AI,
	SRAM_NEXT_LU_ID,
	SRAM_LU_DONE,
	SRAM_CLS_LU_ID_GEN,
	PRS_MAX
};

typedef struct
{
	unsigned char    lu_id;
	unsigned char    portId;
	unsigned char    addInfo;
	unsigned char    header_data[PRS_HEADER_LEN];
} PRS_Tcam_t;

typedef struct
{
	unsigned int ri;/*31:0*/
	unsigned int ri_mask;/*63:32*/
} PRS_Sram_Result_t;

typedef struct
{
	unsigned char nlu_off;/*71:64*/
	unsigned char nlu_off_sign;/*72*/
	unsigned char udef_off;/*80:73*/
	unsigned char udef_off_sign;/*81*/
	unsigned char udef_type;/*84:82*/
	unsigned char op_sel;/*89:85*/
} PRS_Sram_Offset_t;

typedef struct
{
	unsigned char addInfo;/*97:90*/
	unsigned char ai_update;/*106:98*/
	unsigned char lu_id;/*109:106*/
	unsigned char lu_done;/*110*/
	unsigned char cls_luid_gen;/*111*/
} PRS_Sram_next_lu_t;

typedef struct
{
	PRS_Sram_Result_t sram_ri;
	PRS_Sram_Offset_t sram_offset;
	PRS_Sram_next_lu_t sram_next_lu;
} PRS_Sram_t;

typedef struct
{
	PRS_Tcam_t  tcam;
	PRS_Tcam_t  tcamMask;
	PRS_Sram_t  sram;
} PRS_Entry_t;

typedef struct
{
	bool pppoe;
	bool ipv4;
	bool ipv6;
} PRS_Flag_t;


