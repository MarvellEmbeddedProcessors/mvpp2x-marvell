/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : PncGlobal.h                                               **/
/**                                                                          **/
/**  DESCRIPTION : This file contains PnC global definitions                 **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    17-Jun-09   zeev  - initial version created.                              
 *                                                                      
 ******************************************************************************/


#ifndef __INCPncGlobalh
#define __INCPncGlobalh


typedef struct 
{
    int linesProcessed;
    int linesWithoutRowIndex;
    int lastLineWithError;
    int erroredLines;
} PncStats_S;



#define MAX_PNCLINESIZE               500
#define MAX_SUBFIELDSIZE              MAX_PNCLINESIZE
#define MAX_SUBFIELDS                 32
                                   
                                   
#define BM_LU                         15
#define MAX_LU                        0xF
#define MAX_KEYTYPE                   0xF
                                   
#define MAX_LUDONE                    1
#define DEFAULT_LUDONE                0
                                   
#define MAX_NXTLUOFFIND               7
#define DEFAULT_NXTLUOFFIND           0
                                   
#define MAX_QUE                       15
#define DEFAULT_QUE                   0

#define EXTRACT_INDX_SHIFTUPDT(x)     (((x) >> 7) & 0x7)
#define EXTRACT_VAL_SHIFTUPDT(x)      ((x) & 0x7F)
#define MAKE_SHIFTUPDT(x,y)           ((((x) & 0x7) << 7) | ((y) & 0x7F))
#define DEFAULT_SHIFTUPDT_X           7
#define DEFAULT_SHIFTUPDT_y           0
#define MAX_SHIFTUPDT_X               7
#define MAX_SHIFTUPDT_Y               127
                                    
#define MAX_DIS                       1
#define DEFAULT_DIS                   0
                                    
#define MAX_L4                        3
#define DEFAULT_L4                    0
                                    
#define MAX_L3                        3
#define DEFAULT_L3                    0
                                    
#define MAX_FF                        1
#define DEFAULT_FF                    0
                                    
#define MAX_FM                        7
#define DEFAULT_FM                    0
                                    
#define MAX_COL                       1
#define DEFAULT_COL                   0
                                    
#define MAX_TXP                       15
#define DEFAULT_TXP                   0
                                    
#define MAX_MH                        7
#define MAX_MH_EXP                    15
#define DEFAULT_MH                    0
                                    
#define MAX_GEN                       31
#define MAX_GEN_EXP                   15
#define MAX_GEN2_EXP                  4095
#define MAX_GEN2_BITS                 12
#define DEFAULT_GEN                   0
                                    
#define MAX_PROF                      3
#define DEFAULT_PROF                  0
                                    
#define MAX_SRAM_MOD                  4095
#define MAX_SRAM_MOD_H                3
#define MAX_SRAM_MOD_M                15
#define MAX_SRAM_MOD_L                15
#define DEFAULT_SRAM_MOD              0
                                    
#define MAX_SRAM_GEM                  4095
#define DEFAULT_SRAM_GEM              0
                                    
#define MAX_SRAM_TXP2                 15
#define DEFAULT_SRAM_TXP2             0
                                    
                                    
                                    
#define BM_PORTID                     0x1F
#define MAX_PORTID                    0x1F
                                    
#define MAX_BXINDX                    6
#define BM_ADDINFOMASK                0x7F
                                    
#define MAX_1BYTEVALUE                0xFF
#define MAX_2BYTEVALUE                0xFFFF
                                    
#define SPACE                         ' '
#define COMMA                         ','
#define EQUAL_CHAR                    '='
#define QUOTE_CHAR                    '"'
#define CARRIAGE_RETURN_CHAR	       '\r'
#define LINE_FEED_CHAR		       '\n'
#define BYTE_CHAR                     'B'
#define TAB_CHAR                      9
#define LEFTSQUARE_CHAR               '['
#define RIGHTSQUARE_CHAR              ']'
                                    
                                    
                                    
#define MAX_PNCROWS                   512
                                    
#define PACKET_SIZE                   24
                                    
#define MACADDR_SIZE                  6
#define IPADDR_SIZE                   4
#define IPV6ADDR_SIZE                 16
#define IPV6ADDR_TOKEN_NUMS           8
#define IPV6ADDR_SUFF_TOKEN_NUMS      4
 


typedef enum 
{
    RIENUM_DIS, RIENUM_L4,  RIENUM_L3,  RIENUM_FF,   RIENUM_FM,
    RIENUM_COL, RIENUM_TXP, RIENUM_MH,  RIENUM_GEN,  RIENUM_PROF,
    RIENUM_GEN2
} RIENUM;

typedef enum 
{
    FLOWIDENUM_MOD,     FLOWIDENUM_GEM,     FLOWIDENUM_TXP2,  
    FLOWIDENUM_MODMASK, FLOWIDENUM_GEMMASK, 
} FLOWIDENUM;

typedef enum 
{
    FLOWIDENUM_EXP_MOD_H,      FLOWIDENUM_EXP_MOD_M,      FLOWIDENUM_EXP_MOD_L,     
    FLOWIDENUM_EXP_GEM,        FLOWIDENUM_EXP_TXP2,       FLOWIDENUM_EXP_MOD_H_MASK, 
    FLOWIDENUM_EXP_MOD_M_MASK, FLOWIDENUM_EXP_MOD_L_MASK, FLOWIDENUM_EXP_GEMMASK,
    FLOWIDENUM_EXP_TXP2MASK
} FLOWIDENUM_EXP;


typedef unsigned int        UINT32;
typedef unsigned short int  UINT16;
typedef unsigned char       UINT8;


// TCAM
typedef struct
{
    int    lu;
    int    portId;
    int    addInfo;
    UINT8  packet[PACKET_SIZE];
} Tcam_S;

#define C2_POLICER_ID_BITS		5
#define C2_POLICER_ID_MAX		((1 << C2_POLICER_ID_BITS) - 1)

#define C3_POLICER_ID_BITS		5
#define C3_POLICER_ID_MAX		((1 << C3_POLICER_ID_BITS) - 1)
	
#define C4_POLICER_ID_BITS		5
#define C4_POLICER_ID_MAX		((1 << C4_POLICER_ID_BITS) - 1)
	
#define RI_SIZE                   3
#define RIMASK_SIZE               3
#define FLOWID_SIZE               4
#define RI_EXP_SIZE               2
#define RIMASK_EXP_SIZE           1
#define MAKE_ADDINFOUPDT(v,m)     ((((m) & 0x7F) << 7) | ((v) & 0x7F))
#define DEF_ADDINFOUPDATE_MASK    0x7F
#define MAKE_GEMTXP2(txp2,gem)    ((((txp2) & 0x0F) << 12) | ((gem) & 0xFFF))
#define MAKE_FLOWID(txp2,gem,mod) ((((txp2) & 0x0F) << 28) | (((gem) & 0xFFF) << 16) | ((mod) & 0xFFF))

#define FLOWID_EXP_MOD_NIBBLE_START   0
#define FLOWID_EXP_MOD_NIBBLE_END     2
#define FLOWID_EXP_MOD_NIBBLE_NUM     3

#define FLOWID_EXP_GEM_NIBBLE_START   3
#define FLOWID_EXP_GEM_NIBBLE_END     5
#define FLOWID_EXP_GEM_NIBBLE_NUM     3

#define FLOWID_EXP_TXP2_NIBBLE_START  6
#define FLOWID_EXP_TXP2_NIBBLE_END    6
#define FLOWID_EXP_TXP2_NIBBLE_NUM    1

#define RI_BASE_BITS                  48
#define RI_EXTRA_BITS                 18
#define RI_EXTRA_BITS_PAIRS           6 /* Extra RI data includes 12 bits */

typedef struct
{
    int    mod;
    int    gem;
    int    txp2;
    UINT8  flowIdAndMask[FLOWID_SIZE];
    int    flowModMask;
    int    flowGemMask;
}Sram_reg_S;

typedef struct
{
    int    gen2; 
    UINT8  resultInfoExtraAndMask[RI_EXP_SIZE + RIMASK_EXP_SIZE];
    int    mod_l;
    int    mod_m;
    int    mod_h;
    int    gem;
    int    txp2;
    UINT8  flowIdAndMask[FLOWID_SIZE];
    int    flowModLMask;
    int    flowModMMask;
    int    flowModHMask;
    int    flowGemMask;
    int    flowTxp2Mask;
}Sram_exp_S;

// SRAM
typedef struct
{
    int    nextLuId;
    int    addInfoUpdtFull;
    int    addInfoUpdtValue;
    int    addInfoUpdtMask;
    int    keyType;
    int    luDone;
    int    nextLuOffsetInd;
    int    que;
    int    shiftUpdate;
    int    dis;
    int    l4;
    int    l3;
    int    ff;
    int    fm;
    int    col;
    int    txp;
    int    mh;
    int    gen;
    int    prof;
    UINT8  resultInfoAndMask[RI_SIZE + RIMASK_SIZE];

    union
    {
        Sram_reg_S  sram_reg;
        Sram_exp_S  sram_exp;
    }u;

} Sram_S;



typedef struct
{
    bool    inuse;
    int     dbIndx;
    int     ind;
    int     currTcamPktIndx;
    Tcam_S  tcam;
    Tcam_S  tcamMask;
    Sram_S  sram;
} PncEntry_S;



typedef struct
{
    PncEntry_S   *pentryAra;
    int          numEntries;
} PncDb_S;


/******************************************************************************************/
/******************************************************************************************/
/*               Parsing structures, databases                                            */
/******************************************************************************************/
/******************************************************************************************/

/******************************************************************************************/
/*               Runtime parsing structures                                               */
/******************************************************************************************/


typedef struct
{
    bool    inuse;
    int     subfldIndx;
    UINT8   value[MAX_SUBFIELDSIZE];
    UINT32  parsedIntValue;
    UINT32  parsedIntValueMask;

    UINT8   parsedMacAddress[MACADDR_SIZE];
    UINT8   parsedMacAddressMask[MACADDR_SIZE];

    UINT8   parsedIpAddress[IPADDR_SIZE];
    UINT8   parsedIpAddressMask[IPADDR_SIZE];

    UINT16  parsedIpv6Address[IPV6ADDR_TOKEN_NUMS];
    UINT16  parsedIpv6AddressMask[IPV6ADDR_TOKEN_NUMS];
} RtSubFldEntry_S;



/******************************************************************************************/
/*               Fixed information databases                                              */
/******************************************************************************************/


struct subField_tag;

typedef bool  (*subfldValueHandler_F)(struct subField_tag *pSubField, RtSubFldEntry_S *pRtSubFldEntry); 

typedef struct subField_tag
{
    char                 *name;
    int                  maxvalue;
    int                  enumId;
    int                  highbit;
    int                  lowbit;
    int		  field_id;
    subfldValueHandler_F valHandler;
} SubField_S;


typedef struct
{
    SubField_S *pSubFieldAra;
    int        numEntries;
} SubFieldDb_S;


typedef struct packetField_tag
{
    char          *name;
    int           width;
    SubFieldDb_S  *pSubFieldDb;
} PacketField_S;


typedef struct
{
    PacketField_S *pPacketFieldAra;
    int           numEntries;
} PacketFieldDb_S;


typedef struct 
{
    char                 *name;
    int                  enumId;
    int                  highbit;
    int                  lowbit;
} GenBitField_S;


typedef struct
{
    GenBitField_S *pGenBitFieldAra;
    int           numEntries;
} GenBitFieldDb_S;


/******************************************************************************************/
/******************************************************************************************/
/*               Subfield enums                                                           */
/******************************************************************************************/
/******************************************************************************************/
typedef enum
{
    PKTFLD_GH_SUBFLD_PTI,
    PKTFLD_GH_SUBFLD_CRC,
    PKTFLD_GH_SUBFLD_GEM
} PKTFLD_GH;


typedef enum
{
    PKTFLD_MH_SUBFLD_FID,
    PKTFLD_MH_SUBFLD_PRI,
    PKTFLD_MH_SUBFLD_MAN,
    PKTFLD_MH_SUBFLD_SP
} PKTFLD_MH;


typedef enum
{
    PKTFLD_DSA_SUBFLD_TAGCOM,
    PKTFLD_DSA_SUBFLD_SRCT,
    PKTFLD_DSA_SUBFLD_SRCD,
    PKTFLD_DSA_SUBFLD_SRCPRT,
    PKTFLD_DSA_SUBFLD_IST,
    PKTFLD_DSA_SUBFLD_CFI,
    PKTFLD_DSA_SUBFLD_UP,
    PKTFLD_DSA_SUBFLD_VID
} PKTFLD_DSA;


typedef enum
{
    PKTFLD_VT_SUBFLD_TP,
    PKTFLD_VT_SUBFLD_PB,
    PKTFLD_VT_SUBFLD_CFI,
    PKTFLD_VT_SUBFLD_VID
} PKTFLD_VT;


typedef enum
{
    PKTFLD_IPV4_SUBFLD_VER,
    PKTFLD_IPV4_SUBFLD_IHL,
    PKTFLD_IPV4_SUBFLD_DSCP,
    PKTFLD_IPV4_SUBFLD_TOS,
    PKTFLD_IPV4_SUBFLD_LEN,
    PKTFLD_IPV4_SUBFLD_ID,
    PKTFLD_IPV4_SUBFLD_DF,
    PKTFLD_IPV4_SUBFLD_MF,
    PKTFLD_IPV4_SUBFLD_FRGO,
    PKTFLD_IPV4_SUBFLD_TTL,
    PKTFLD_IPV4_SUBFLD_PROTO,
    PKTFLD_IPV4_SUBFLD_IPSRC,
    PKTFLD_IPV4_SUBFLD_IPDST
} PKTFLD_IPV4;

typedef enum
{
    PKTFLD_IPV6_SUBFLD_VER,
    PKTFLD_IPV6_SUBFLD_DSCP,
    PKTFLD_IPV6_SUBFLD_TC,
    PKTFLD_IPV6_SUBFLD_FL,
    PKTFLD_IPV6_SUBFLD_LEN,
    PKTFLD_IPV6_SUBFLD_NH,
    PKTFLD_IPV6_SUBFLD_HL,
    PKTFLD_IPV6_SUBFLD_IPV6SA,
    PKTFLD_IPV6_SUBFLD_IPV6DA
} PKTFLD_IPV6;

typedef enum
{
    PKTFLD_XDSA_SUBFLD_TAGCOM,
    PKTFLD_XDSA_SUBFLD_SRCT,
    PKTFLD_XDSA_SUBFLD_SRCD,
    PKTFLD_XDSA_SUBFLD_SRCPRT1,
    PKTFLD_XDSA_SUBFLD_IST,
    PKTFLD_XDSA_SUBFLD_CFI,
    PKTFLD_XDSA_SUBFLD_UP,
    PKTFLD_XDSA_SUBFLD_VID,
    PKTFLD_XDSA_SUBFLD_SRCPRT2,
    PKTFLD_XDSA_SUBFLD_SRCID,
    PKTFLD_XDSA_SUBFLD_QOSP,
    PKTFLD_XDSA_SUBFLD_UVIDX,
    PKTFLD_XDSA_SUBFLD_VIDX
} PKTFLD_XDSA;


typedef enum
{
    PKTFLD_LLC_SUBFLD_DSAP,
    PKTFLD_LLC_SUBFLD_SSAP,
    PKTFLD_LLC_SUBFLD_CONTROL,
    PKTFLD_LLC_SUBFLD_OUI1,
    PKTFLD_LLC_SUBFLD_OUI2,
    PKTFLD_LLC_SUBFLD_OUI3,
    PKTFLD_LLC_SUBFLD_ETYPE,
} PKTFLD_LLC;


typedef enum
{
    PKTFLD_MACAD_SUBFLD_ADDR
} PKTFLD_MACAD;


typedef enum
{
    PKTFLD_ETY_SUBFLD_ETH
}PKTFLD_ETY;

typedef enum
{
    PKTFLD_PPPOE_SUBFLD_VER,
    PKTFLD_PPPOE_SUBFLD_TYPE,
    PKTFLD_PPPOE_SUBFLD_CODE,
    PKTFLD_PPPOE_SUBFLD_ID,
    PKTFLD_PPPOE_SUBFLD_LEN,
    PKTFLD_PPPOE_SUBFLD_PROTO
}PKTFLD_PPPOE;


typedef enum
{
    PKTFLD_L4_SUBFLD_PORT
}PKTFLD_L4;

typedef enum
{
    PKTFLD_TCP_SUBFLD_SPORT,
    PKTFLD_TCP_SUBFLD_DPORT,
    PKTFLD_TCP_SUBFLD_FLAGS,
    PKTFLD_TCP_SUBFLD_U,
    PKTFLD_TCP_SUBFLD_A,
    PKTFLD_TCP_SUBFLD_P,
    PKTFLD_TCP_SUBFLD_R,
    PKTFLD_TCP_SUBFLD_S,
    PKTFLD_TCP_SUBFLD_F,
} PKTFLD_TCP;



/******************************************************************************************/
/******************************************************************************************/
/*               Field ID enums                                                           */
/******************************************************************************************/
/******************************************************************************************/
#define MH_FIELD_ID			0
#define GEM_PORT_ID_FIELD_ID		1
#define MH_UNTAGGED_PRI_FIELD_ID	2
#define MAC_DA_FIELD_ID			3
#define MAC_SA_FIELD_ID			4
#define OUT_VLAN_PRI_FIELD_ID		5
#define OUT_VLAN_ID_FIELD_ID		6
#define IN_VLAN_ID_FIELD_ID		7
#define ETH_TYPE_FIELD_ID		8
#define PPPOE_SESID_FIELD_ID		9
#define IP_VER_FIELD_ID			10
#define IPV4_DSCP_FIELD_ID		11
#define IPV4_ECN_FIELD_ID		12
#define IPV4_LEN_FIELD_ID		13
#define IPV4_TTL_FIELD_ID		14
#define IPV4_PROTO_FIELD_ID		15
#define IPV4_SA_FIELD_ID		16
#define IPV4_DA_FIELD_ID		17
#define IPV6_PROTO_FIELD_ID		15
#define IPV6_DSCP_FIELD_ID		18
#define IPV6_ECN_FIELD_ID		19
#define IPV6_FLOW_LBL_FIELD_ID		20
#define IPV6_PAYLOAD_LEN_FIELD_ID	21
#define IPV6_NH_FIELD_ID		22
#define IPV6_HL_FIELD_ID		14
#define IPV6_SA_FIELD_ID		23
#define IPV6_SA_PREF_FIELD_ID		24
#define IPV6_SA_SUFF_FIELD_ID		25
#define IPV6_DA_FIELD_ID		26
#define IPV6_DA_PREF_FIELD_ID		27
#define IPV6_DA_SUFF_FIELD_ID		28
#define L4_SRC_FIELD_ID			29
#define L4_DST_FIELD_ID			30
#define TCP_FLAGS_FIELD_ID		31
#define PPV2_UDF_OUT_TPID		32
#define PPV2_UDF_IN_TPID		33
#define PPV2_UDF_IPV4_IHL		34
#define IPV4_ARP_DA_FIELD_ID		48
#define IN_VLAN_PRI_FIELD_ID		49
#define PPPOE_PROTO_FIELD_ID		50
#define PPV2_FIELD_COUNT		(PPPOE_PROTO_FIELD_ID+1)
#define NO_FIELD_ID			0xFFFF

/******************************************************************************************/
/******************************************************************************************/
/*               Field Size(bits) Macros                                                  */
/******************************************************************************************/
/******************************************************************************************/
#define MH_FIELD_SIZE			16
#define GEM_PORT_ID_FIELD_SIZE		12
#define MH_UNTAGGED_PRI_FIELD_SIZE	3
#define MAC_DA_FIELD_SIZE		48
#define MAC_SA_FIELD_SIZE		48
#define OUT_VLAN_PRI_FIELD_SIZE		3
#define OUT_VLAN_ID_FIELD_SIZE		12
#define IN_VLAN_ID_FIELD_SIZE		12
#define ETH_TYPE_FIELD_SIZE		16
#define PPPOE_SESID_FIELD_SIZE		16
#define IP_VER_FIELD_SIZE		4
#define IPV4_DSCP_FIELD_SIZE		6
#define IPV4_ECN_FIELD_SIZE		2
#define IPV4_LEN_FIELD_SIZE		16
#define IPV4_TTL_FIELD_SIZE		8
#define IPV4_PROTO_FIELD_SIZE		8
#define IPV4_SA_FIELD_SIZE		32
#define IPV4_DA_FIELD_SIZE		32
#define IPV6_PROTO_FIELD_SIZE		8
#define IPV6_DSCP_FIELD_SIZE		6
#define IPV6_ECN_FIELD_SIZE		2
#define IPV6_FLOW_LBL_FIELD_SIZE	20
#define IPV6_PAYLOAD_LEN_FIELD_SIZE	16
#define IPV6_NH_FIELD_SIZE		8
#define IPV6_HL_FIELD_SIZE		8
#define IPV6_SA_FIELD_SIZE		128
#define IPV6_SA_PREF_FIELD_SIZE		64
#define IPV6_SA_SUFF_FIELD_SIZE		64
#define IPV6_DA_FIELD_SIZE		128
#define IPV6_DA_PREF_FIELD_SIZE		64
#define IPV6_DA_SUFF_FIELD_SIZE		64
#define L4_SRC_FIELD_SIZE		16
#define L4_DST_FIELD_SIZE		16
#define TCP_FLAGS_FIELD_SIZE		8
#define UDF_OUT_TPID_FIELD_SIZE		16
#define UDF_IN_TPID_FIELD_SIZE		16
#define UDF_IPV4_IHL_FIELD_SIZE		4
#define IPV4_ARP_DA_FIELD_SIZE		32
#define IN_VLAN_PRI_FIELD_SIZE		3	
#define PPPOE_PROTO_FIELD_SIZE		16

/******************************************************************************************/
/******************************************************************************************/
/*               Global functions e.g. debug flags etc                                    */
/******************************************************************************************/
/******************************************************************************************/

extern unsigned int getDebugFlag();
extern bool getKeepGoingFlag();
extern bool getAutoIndexFlag();
extern bool getPncShellCmdFlag();
extern bool getPncAsicFlag();
extern bool getNoInvalidateAllFlag();
extern bool getFPGAFormatFlag();
extern bool get_native_mode(void);
inline unsigned int get_execution_delay(void);
inline void set_execution_delay(unsigned int new_execution_delay);

/* Global array of field size */
extern unsigned int field_size[PPV2_FIELD_COUNT]; /* defined in file parse_PRS.c */

#endif

