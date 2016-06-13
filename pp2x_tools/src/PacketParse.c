/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : PacketParse.c                                             **/
/**                                                                          **/
/**  DESCRIPTION : This file contains parse of the Packet field              **/
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
#include <ctype.h>

#include "PncGlobals.h"
#include "common.h"
#include "DataDictionary.h"
#include "ParseUtils.h"
#include "SubfieldParse.h"
#include "PacketParse.h"

// GH START
SubField_S ghSubFieldAra[] =
{
    {"pti",        7,      PKTFLD_GH_SUBFLD_PTI,   15, 13,	NO_FIELD_ID,		parseIntegerSubFldValue },
    {"crc",        1,      PKTFLD_GH_SUBFLD_CRC,   12, 12,	NO_FIELD_ID,		parseIntegerSubFldValue },
    {"gem",        4095,   PKTFLD_GH_SUBFLD_GEM,   11, 0,	GEM_PORT_ID_FIELD_ID,	parseIntegerSubFldValue },
};

SubFieldDb_S ghSubFieldDb =
{
    ghSubFieldAra,
    sizeof(ghSubFieldAra)/sizeof(ghSubFieldAra[0])
};
// GH END


// MH START
SubField_S mhSubFieldAra[] =
{
    {"fid",        15,     PKTFLD_MH_SUBFLD_FID,   15, 12,	MH_FIELD_ID,		   parseIntegerSubFldValue },
    {"pri",        7,      PKTFLD_MH_SUBFLD_PRI,   11, 9,	MH_UNTAGGED_PRI_FIELD_ID,  parseIntegerSubFldValue  },
    {"man",        1,      PKTFLD_MH_SUBFLD_MAN,   8,  8,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"sp",         15,     PKTFLD_MH_SUBFLD_SP,    3,  0,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
};

SubFieldDb_S mhSubFieldDb =
{
    mhSubFieldAra,
    sizeof(mhSubFieldAra)/sizeof(mhSubFieldAra[0])
};
// MH END


// DSA START
SubField_S dsaSubFieldAra[] =
{
    {"tagcom",     3,      PKTFLD_DSA_SUBFLD_TAGCOM,  31, 30,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"srct",       1,      PKTFLD_DSA_SUBFLD_SRCT,    29, 29,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"srcd",       31,     PKTFLD_DSA_SUBFLD_SRCD,    28, 24,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"srcprt",     31,     PKTFLD_DSA_SUBFLD_SRCPRT,  23, 19,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"ist",        1,      PKTFLD_DSA_SUBFLD_IST,     18, 18,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"cfi",        1,      PKTFLD_DSA_SUBFLD_CFI,     16, 16,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"up",         7,      PKTFLD_DSA_SUBFLD_UP,      15, 13,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"vid",        4095,   PKTFLD_DSA_SUBFLD_VID,     11, 0,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
};

SubFieldDb_S dsaSubFieldDb =
{
    dsaSubFieldAra,
    sizeof(dsaSubFieldAra)/sizeof(dsaSubFieldAra[0])
};
// DSA END


// VT START
SubField_S vtSubFieldAra[] =
{
    {"tpidout",     0xFFFF,  PKTFLD_VT_SUBFLD_TP,	15, 0,	PPV2_UDF_OUT_TPID,	parseIntegerSubFldValue  },
    {"tpidin",	     0xFFFF,  PKTFLD_VT_SUBFLD_TP,      15, 0,	PPV2_UDF_IN_TPID,	parseIntegerSubFldValue  },
    {"vidin",	       4095,  PKTFLD_VT_SUBFLD_VID,     11, 0,	IN_VLAN_ID_FIELD_ID,	parseIntegerSubFldValue  },
    {"pbout",	          7,  PKTFLD_VT_SUBFLD_PB,      15, 13,	OUT_VLAN_PRI_FIELD_ID,	parseIntegerSubFldValue  },
    {"vidout",        4095,  PKTFLD_VT_SUBFLD_VID,     11, 0,	OUT_VLAN_ID_FIELD_ID,	parseIntegerSubFldValue  },
};

SubFieldDb_S vtSubFieldDb =
{
    vtSubFieldAra,
    sizeof(vtSubFieldAra)/sizeof(vtSubFieldAra[0])
};
// VT END


// IPV4 START
SubField_S ipv4SubFieldAra[] =
{
    {"ver",        0xF,    PKTFLD_IPV4_SUBFLD_VER,     159, 156,	IP_VER_FIELD_ID,	parseIntegerSubFldValue   },
    {"ihl",        0xF,    PKTFLD_IPV4_SUBFLD_IHL,     155, 152,	PPV2_UDF_IPV4_IHL,	parseIntegerSubFldValue   },
    {"dscp",       0x3F,   PKTFLD_IPV4_SUBFLD_DSCP,    151, 146,	IPV4_DSCP_FIELD_ID,     parseIntegerSubFldValue   },
    {"tos",        0xFF,   PKTFLD_IPV4_SUBFLD_TOS,     151, 144,	NO_FIELD_ID,		parseIntegerSubFldValue   },
    {"len",        0xFFFF, PKTFLD_IPV4_SUBFLD_LEN,     143, 128,	IPV4_LEN_FIELD_ID,	parseIntegerSubFldValue   },
    {"ecn",	    0xFFFF, PKTFLD_IPV4_SUBFLD_LEN,     145, 144,	IPV4_ECN_FIELD_ID,	parseIntegerSubFldValue   },
    {"id",         0xFFFF, PKTFLD_IPV4_SUBFLD_ID,      127, 112,	NO_FIELD_ID,		parseIntegerSubFldValue   },
    {"df",         1,      PKTFLD_IPV4_SUBFLD_DF,      110, 110,	NO_FIELD_ID,		parseIntegerSubFldValue   },
    {"mf",         1,      PKTFLD_IPV4_SUBFLD_MF,      109, 109,	NO_FIELD_ID,		parseIntegerSubFldValue   },
    {"frgo",       0x1FFF, PKTFLD_IPV4_SUBFLD_FRGO,    108, 96,	NO_FIELD_ID,		parseIntegerSubFldValue   },
    {"ttl",        0xFF,   PKTFLD_IPV4_SUBFLD_TTL,     95,  88,	IPV4_TTL_FIELD_ID,	parseIntegerSubFldValue   },
    {"proto",      0xFF,   PKTFLD_IPV4_SUBFLD_PROTO,   87,  80,	IPV4_PROTO_FIELD_ID,	parseIntegerSubFldValue   },
    {"sa",         0,      PKTFLD_IPV4_SUBFLD_IPSRC,   31,  0,		IPV4_SA_FIELD_ID,	parseIpAddressSubFldValue },
    {"da",         0,      PKTFLD_IPV4_SUBFLD_IPDST,   31,  0,		IPV4_DA_FIELD_ID,	parseIpAddressSubFldValue },
};

SubFieldDb_S ipv4SubFieldDb =
{
    ipv4SubFieldAra,
    sizeof(ipv4SubFieldAra)/sizeof(ipv4SubFieldAra[0])
};
// IPV4 END


// IPV6 START
SubField_S ipv6SubFieldAra[] =
{
    {"ver",        0xF,     PKTFLD_IPV6_SUBFLD_VER,     191, 188,	NO_FIELD_ID,		   parseIntegerSubFldValue   },
    {"dscp",       0x3F,    PKTFLD_IPV6_SUBFLD_DSCP,    187, 182,	IPV6_DSCP_FIELD_ID,	   parseIntegerSubFldValue   },
    {"ecn",	    0x3F,    PKTFLD_IPV6_SUBFLD_DSCP,    187, 182,	IPV6_ECN_FIELD_ID,	   parseIntegerSubFldValue   },
    {"tc",         0xFF,    PKTFLD_IPV6_SUBFLD_TC,      187, 180,	NO_FIELD_ID,		   parseIntegerSubFldValue   },
    {"fl",         0xFFFFF, PKTFLD_IPV6_SUBFLD_FL,      179, 160,	IPV6_FLOW_LBL_FIELD_ID,	   parseIntegerSubFldValue   },
    {"len",        0xFFFF,  PKTFLD_IPV6_SUBFLD_LEN,     159, 144,	IPV6_PAYLOAD_LEN_FIELD_ID, parseIntegerSubFldValue   },
    {"nh",         0xFF,    PKTFLD_IPV6_SUBFLD_NH,      143, 136,	IPV6_NH_FIELD_ID,	   parseIntegerSubFldValue   },
    {"hl",         0xFF,    PKTFLD_IPV6_SUBFLD_HL,      135, 128,	IPV6_HL_FIELD_ID,	   parseIntegerSubFldValue   },
    {"sa",            0,    PKTFLD_IPV6_SUBFLD_IPV6SA,    0,   0,	IPV6_SA_FIELD_ID,	   parseIpV6AddressSubFldValue   	},
    {"sapref",        0,    PKTFLD_IPV6_SUBFLD_IPV6DA,    0,   0, 	IPV6_SA_PREF_FIELD_ID,     parseIpV6AddrSuffPrefSubFldValue   	},
    {"sasuff",        0,    PKTFLD_IPV6_SUBFLD_IPV6SA,    0,   0, 	IPV6_SA_SUFF_FIELD_ID,	   parseIpV6AddrSuffPrefSubFldValue   	},
    {"da",            0,    PKTFLD_IPV6_SUBFLD_IPV6DA,    0,   0, 	IPV6_DA_FIELD_ID,          parseIpV6AddressSubFldValue   	},
    {"dapref",        0,    PKTFLD_IPV6_SUBFLD_IPV6SA,    0,   0, 	IPV6_DA_PREF_FIELD_ID,	   parseIpV6AddrSuffPrefSubFldValue   	},
    {"dasuff",        0,    PKTFLD_IPV6_SUBFLD_IPV6DA,    0,   0, 	IPV6_DA_SUFF_FIELD_ID,     parseIpV6AddrSuffPrefSubFldValue   	},
};

SubFieldDb_S ipv6SubFieldDb =
{
    ipv6SubFieldAra,
    sizeof(ipv6SubFieldAra)/sizeof(ipv6SubFieldAra[0])
};
// IPV6 END

#if 0
// DIPV6 START
SubField_S dipv6SubFieldAra[] =
{
    {"ipv6da",     0,      PKTFLD_IPV6_SUBFLD_IPV6DA,  127,   0,	NO_FIELD_ID,		   parseAndInsertIpv6AddressSubfieldValue   },
};

SubFieldDb_S dipv6SubFieldDb =
{
    dipv6SubFieldAra,
    sizeof(dipv6SubFieldAra)/sizeof(dipv6SubFieldAra[0])
};
// DIPV6 END
#endif

// XDSA START
SubField_S xdsaSubFieldAra[] =
{
    {"tagcom",     3,      PKTFLD_XDSA_SUBFLD_TAGCOM,     63, 62,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"srct",       1,      PKTFLD_XDSA_SUBFLD_SRCT,       61, 61,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"srcd",       31,     PKTFLD_XDSA_SUBFLD_SRCD,       60, 56,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"srcprt1",    31,     PKTFLD_XDSA_SUBFLD_SRCPRT1,    55, 51,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"ist",        1,      PKTFLD_XDSA_SUBFLD_IST,        50, 50,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"cfi",        1,      PKTFLD_XDSA_SUBFLD_CFI,        48, 48,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"up",         7,      PKTFLD_XDSA_SUBFLD_UP,         47, 45,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"vid",        4095,   PKTFLD_XDSA_SUBFLD_VID,        43, 32,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"srcprt2",    3,      PKTFLD_XDSA_SUBFLD_SRCPRT2,    30, 29,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"srcid",      31,     PKTFLD_XDSA_SUBFLD_SRCID,      24, 20,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"qosp",       0x7F,   PKTFLD_XDSA_SUBFLD_QOSP,       19, 13,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"uvidx",      1,      PKTFLD_XDSA_SUBFLD_UVIDX,      12, 12,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"vidx",       4095,   PKTFLD_XDSA_SUBFLD_VIDX,       11, 0, 	NO_FIELD_ID,		   parseIntegerSubFldValue  },
};

SubFieldDb_S xdsaSubFieldDb =
{
    xdsaSubFieldAra,
    sizeof(xdsaSubFieldAra)/sizeof(xdsaSubFieldAra[0])
};
// XDSA END


// MACADDRESS START - Used by DA and SA fields
SubField_S macSaAddrSubFieldAra[] =
{
    {"addr",       0,      PKTFLD_MACAD_SUBFLD_ADDR,       0,  0,	MAC_SA_FIELD_ID,	parseMacAddressSubFldValue},
};

SubFieldDb_S macSaAddrSubFieldDb =
{
    macSaAddrSubFieldAra,
    sizeof(macSaAddrSubFieldAra)/sizeof(macSaAddrSubFieldAra[0])
};

SubField_S macDaAddrSubFieldAra[] =
{
    {"addr",       0,      PKTFLD_MACAD_SUBFLD_ADDR,       0,  0,	MAC_DA_FIELD_ID,	parseMacAddressSubFldValue},
};

SubFieldDb_S macDaAddrSubFieldDb =
{
    macDaAddrSubFieldAra,
    sizeof(macDaAddrSubFieldAra)/sizeof(macDaAddrSubFieldAra[0])
};

// MACADDRESS END



// ETY START
SubField_S etySubFieldAra[] =
{
    {"eth",       0xFFFF,      PKTFLD_ETY_SUBFLD_ETH,     15,   0,	ETH_TYPE_FIELD_ID,	parseIntegerSubFldValue},
};

SubFieldDb_S etySubFieldDb =
{
    etySubFieldAra,
    sizeof(etySubFieldAra)/sizeof(etySubFieldAra[0])
};
// ETY END

// PPPoE START
SubField_S pppoeSubFieldAra[] =
{
    {"sesid",       0xFFFF,      PKTFLD_PPPOE_SUBFLD_ID,     15,   0,	PPPOE_SESID_FIELD_ID,	parseIntegerSubFldValue},
    {"proto",       0xFFFF,      PKTFLD_PPPOE_SUBFLD_PROTO,  15,   0,	PPPOE_PROTO_FIELD_ID,	parseIntegerSubFldValue},
};

SubFieldDb_S pppoeSubFieldDb =
{
    pppoeSubFieldAra,
    sizeof(pppoeSubFieldAra)/sizeof(pppoeSubFieldAra[0])
};
// PPPoE END



// LLC START
SubField_S llcSubFieldAra[] =
{
    {"dsap",     255,      PKTFLD_LLC_SUBFLD_DSAP,       63, 56,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"ssap",     255,      PKTFLD_LLC_SUBFLD_SSAP,       55, 48,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"cntl",     255,      PKTFLD_LLC_SUBFLD_CONTROL,    47, 40,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"oui1",     255,      PKTFLD_LLC_SUBFLD_OUI1,       39, 32,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"oui2",     255,      PKTFLD_LLC_SUBFLD_OUI2,       31, 24, 	NO_FIELD_ID,		  parseIntegerSubFldValue  },
    {"oui3",     255,      PKTFLD_LLC_SUBFLD_OUI3,       23, 16,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
    {"etype",  0xFFFF,     PKTFLD_LLC_SUBFLD_ETYPE,      15,  0,	NO_FIELD_ID,		   parseIntegerSubFldValue  },
};

SubFieldDb_S llcSubFieldDb =
{
    llcSubFieldAra,
    sizeof(llcSubFieldAra)/sizeof(llcSubFieldAra[0])
};
// LLC END

// L4 START
SubField_S l4srcSubFieldAra[] =
{
    {"port",       0xFFFF,      PKTFLD_L4_SUBFLD_PORT,     15,   0,	L4_SRC_FIELD_ID,		  parseIntegerSubFldValue},
};

SubFieldDb_S l4srcSubFieldDb =
{
    l4srcSubFieldAra,
    sizeof(l4srcSubFieldAra)/sizeof(l4srcSubFieldAra[0])
};

SubField_S l4dstSubFieldAra[] =
{
    {"port",       0xFFFF,      PKTFLD_L4_SUBFLD_PORT,     15,   0,	L4_DST_FIELD_ID,		  parseIntegerSubFldValue},
};

SubFieldDb_S l4dstSubFieldDb =
{
    l4dstSubFieldAra,
    sizeof(l4dstSubFieldAra)/sizeof(l4dstSubFieldAra[0])
};
// L4 END


// TCP START
SubField_S tcpSubFieldAra[] =
{
    {"tcpsrc",     0xFFFF, PKTFLD_TCP_SUBFLD_SPORT,    159, 144,	L4_SRC_FIELD_ID,		   parseIntegerSubFldValue   },
    {"tcpdst",     0xFFFF, PKTFLD_TCP_SUBFLD_DPORT,    143, 128,	L4_DST_FIELD_ID,		   parseIntegerSubFldValue   },
    {"flags",      0x3F,   PKTFLD_TCP_SUBFLD_FLAGS,    53,  48, 	TCP_FLAGS_FIELD_ID,	   parseIntegerSubFldValue   },
    {"u",          1,      PKTFLD_TCP_SUBFLD_U,        53,  53,	NO_FIELD_ID,		    parseIntegerSubFldValue   },
    {"a",          1,      PKTFLD_TCP_SUBFLD_A,        52,  52,	NO_FIELD_ID,		    parseIntegerSubFldValue   },
    {"p",          1,      PKTFLD_TCP_SUBFLD_P,        51,  51, 	NO_FIELD_ID,		   parseIntegerSubFldValue   },
    {"r",          1,      PKTFLD_TCP_SUBFLD_R,        50,  50,	NO_FIELD_ID,		    parseIntegerSubFldValue   },
    {"s",          1,      PKTFLD_TCP_SUBFLD_S,        49,  49,	NO_FIELD_ID,		    parseIntegerSubFldValue   },
    {"f",          1,      PKTFLD_TCP_SUBFLD_F,        48,  48,	NO_FIELD_ID,		    parseIntegerSubFldValue   },
};

SubFieldDb_S tcpSubFieldDb =
{
    tcpSubFieldAra,
    sizeof(tcpSubFieldAra)/sizeof(tcpSubFieldAra[0])
};
// TCP END

PacketField_S packetFieldAra[] =
{
	{"GH",	 2,	&ghSubFieldDb		},
	{"MH",	 2,	&mhSubFieldDb		},
	{"DA",	 6,	&macDaAddrSubFieldDb	},
	{"SA",	 6,	&macSaAddrSubFieldDb	},
	{"DSA",	 4,	&dsaSubFieldDb		},
	{"VT",	 4,	&vtSubFieldDb		},
	{"ETY",  2,	&etySubFieldDb		},
	{"PPPOE",  2,	&pppoeSubFieldDb	},
	{"IPV4",20,	&ipv4SubFieldDb		},
	{"XDSA", 8,	&xdsaSubFieldDb		},
	{"LLC",	 8,	&llcSubFieldDb		},
	{"IPV6",16,	&ipv6SubFieldDb		},
	{"L4SRC",2,	&l4srcSubFieldDb	},
	{"L4DST",2,	&l4dstSubFieldDb	},
	{"TCP", 20,	&tcpSubFieldDb		},
};

PacketFieldDb_S packetFieldDb =
{
    packetFieldAra,
    sizeof(packetFieldAra)/sizeof(packetFieldAra[0])
};

/* For Cls*/

// SPID START
SubField_S spidAra[] =
{
    {"spid",       0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
};

SubFieldDb_S spidDb =
{
    spidAra,
    sizeof(spidAra)/sizeof(spidAra[0])
};
// SPID END

// portEctract START

SubField_S portEctractAra[] =
{
    {"PortSPIDExtract",       0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
};

SubFieldDb_S portEctractDb =
{
    portEctractAra,
    sizeof(portEctractAra)/sizeof(portEctractAra[0])
};
// portEctract END

// GemportID START

SubField_S gemportIDAra[] =
{
    {"gemportID",       0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
};

SubFieldDb_S gemportIDDb =
{
    gemportIDAra,
    sizeof(gemportIDAra)/sizeof(gemportIDAra[0])
};
// GemportID END


// UDF field START
SubField_S udfFieldAra[] =
{
    {"baseoffID",       0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
    {"RelativeOff",       0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
    {"FieldSize",       0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
};

SubFieldDb_S udfFieldDb =
{
    udfFieldAra,
    sizeof(udfFieldAra)/sizeof(udfFieldAra[0])
};

// Eth port control field
SubField_S eth_port_ctrl_FieldAra[] =
{
    {"MH",	0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
    {"disVid",	0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
    {"disUni",	0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
};

SubFieldDb_S eth_port_ctrl_FieldDb =
{
    eth_port_ctrl_FieldAra,
    sizeof(eth_port_ctrl_FieldAra)/sizeof(eth_port_ctrl_FieldAra[0])
};


// BaseoffsetID START
#if 0
SubField_S baseoffsetIDAra[] =
{
    {"baseoffsetID",       0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
};

SubFieldDb_S baseoffsetIDDb =
{
    baseoffsetIDAra,
    sizeof(baseoffsetIDAra)/sizeof(baseoffsetIDAra[0])
};
// BaseoffsetID END

// RelativeFieldOffset START

SubField_S RelativeOffsetAra[] =
{
    {"RelativeOffset",       0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
};

SubFieldDb_S RelativeOffsetDb =
{
    RelativeOffsetAra,
    sizeof(RelativeOffsetAra)/sizeof(RelativeOffsetAra[0])
};
// RelativeFieldOffset END

// FieldSize START

SubField_S FieldSizeAra[] =
{
    {"FieldSize",       0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
};

SubFieldDb_S FieldSizeDb =
{
    FieldSizeAra,
    sizeof(FieldSizeAra)/sizeof(FieldSizeAra[0])
};
// FieldSize END
#endif
// Other cls START

SubField_S otherclsAra[] =
{
    {"othercls",       0xFFFF,      0,     15,   0,	32,	parseIntegerSubFldValue},
};

SubFieldDb_S otherclsDb =
{
    otherclsAra,
    sizeof(otherclsAra)/sizeof(otherclsAra[0])
};
//  Other cls END



PacketField_S clsFieldAra[] =
{
	{"SPID0",  2,	&spidDb	},
	{"SPID1",  2,	&spidDb	},
	{"SPID2",  2,	&spidDb	},
	{"SPID3",  2,	&spidDb	},
	{"SPID4",  2,	&spidDb	},
	{"SPID5",  2,	&spidDb	},
	{"SPID6",  2,	&spidDb	},
	{"SPID7",  2,	&spidDb	},
	{"SPID8",  2,	&spidDb	},
	{"SPID9",  2,	&spidDb	},
	{"SPID10",  2,	&spidDb	},
	{"SPID11",  2,	&spidDb	},
	{"SPID12",  2,	&spidDb	},
	{"SPID13",  2,	&spidDb	},
	{"SPID14",  2,	&spidDb	},
	{"SPID15",  2,	&spidDb	},
	{"SPID16",  2,	&spidDb	},
	{"SPID17",  2,	&spidDb	},
	{"SPID18",  2,	&spidDb	},
	{"SPID19",  2,	&spidDb	},
	{"SPID20",  2,	&spidDb	},
	{"SPID21",  2,	&spidDb	},
	{"SPID22",  2,	&spidDb	},
	{"SPID23",  2,	&spidDb	},
	{"SPID24",  2,	&spidDb	},
	{"SPID25",  2,	&spidDb	},
	{"SPID26",  2,	&spidDb	},
	{"SPID27",  2,	&spidDb	},
	{"SPID28",  2,	&spidDb	},
	{"SPID29",  2,	&spidDb	},
	{"SPID30",  2,	&spidDb	},
	{"SPID31",  2,	&spidDb	},
	{"Port0SPIDExtract",  2,	&portEctractDb	},
	{"Port1SPIDExtract",  2,	&portEctractDb	},
	{"Port2SPIDExtract",  2,	&portEctractDb	},
	{"Port3SPIDExtract",  2,	&portEctractDb	},
	{"Port4SPIDExtract",  2,	&portEctractDb	},
	{"Port5SPIDExtract",  2,	&portEctractDb	},
	{"Port6SPIDExtract",  2,	&portEctractDb	},
	{"VirtualPortID0",  2,	&gemportIDDb	},
	{"VirtualPortID1",  2,	&gemportIDDb	},
	{"VirtualPortID2",  2,	&gemportIDDb	},
	{"VirtualPortID3",  2,	&gemportIDDb	},
	{"VirtualPortID4",  2,	&gemportIDDb	},
	{"VirtualPortID5",  2,	&gemportIDDb	},
	{"VirtualPortID6",  2,	&gemportIDDb	},
	{"VirtualPortID7",  2,	&gemportIDDb	},
	{"VirtualPortID8",  2,	&gemportIDDb	},
	{"VirtualPortID9",  2,	&gemportIDDb	},
	{"VirtualPortID10",  2,	&gemportIDDb	},
	{"VirtualPortID11",  2,	&gemportIDDb	},
	{"VirtualPortID12",  2,	&gemportIDDb	},
	{"VirtualPortID13",  2,	&gemportIDDb	},
	{"VirtualPortID14",  2,	&gemportIDDb	},
	{"VirtualPortID14",  2,	&gemportIDDb	},
	{"VirtualPortID16",  2,	&gemportIDDb	},
	{"VirtualPortID17",  2,	&gemportIDDb	},
	{"VirtualPortID18",  2,	&gemportIDDb	},
	{"VirtualPortID19",  2,	&gemportIDDb	},
	{"VirtualPortID20",  2,	&gemportIDDb	},
	{"VirtualPortID21",  2,	&gemportIDDb	},
	{"VirtualPortID22",  2,	&gemportIDDb	},
	{"VirtualPortID23",  2,	&gemportIDDb	},
	{"VirtualPortID24",  2,	&gemportIDDb	},
	{"VirtualPortID25",  2,	&gemportIDDb	},
	{"VirtualPortID26",  2,	&gemportIDDb	},
	{"VirtualPortID27",  2,	&gemportIDDb	},
	{"VirtualPortID28",  2,	&gemportIDDb	},
	{"VirtualPortID29",  2,	&gemportIDDb	},
	{"VirtualPortID30",  2,	&gemportIDDb	},
	{"VirtualPortID31",  2,	&gemportIDDb	},
	{"VirtualPortID32",  2,	&gemportIDDb	},
	{"VirtualPortID33",  2,	&gemportIDDb	},
	{"VirtualPortID34",  2,	&gemportIDDb	},
	{"VirtualPortID35",  2,	&gemportIDDb	},
	{"VirtualPortID36",  2,	&gemportIDDb	},
	{"VirtualPortID37",  2,	&gemportIDDb	},
	{"VirtualPortID38",  2,	&gemportIDDb	},
	{"VirtualPortID39",  2,	&gemportIDDb	},
	{"VirtualPortID40",  2,	&gemportIDDb	},
	{"VirtualPortID41",  2,	&gemportIDDb	},
	{"VirtualPortID42",  2,	&gemportIDDb	},
	{"VirtualPortID43",  2,	&gemportIDDb	},
	{"VirtualPortID44",  2,	&gemportIDDb	},
	{"VirtualPortID45",  2,	&gemportIDDb	},
	{"VirtualPortID46",  2,	&gemportIDDb	},
	{"VirtualPortID47",  2,	&gemportIDDb	},
	{"VirtualPortID48",  2,	&gemportIDDb	},
	{"VirtualPortID49",  2,	&gemportIDDb	},
	{"VirtualPortID50",  2,	&gemportIDDb	},
	{"VirtualPortID51",  2,	&gemportIDDb	},
	{"VirtualPortID52",  2,	&gemportIDDb	},
	{"VirtualPortID53",  2,	&gemportIDDb	},
	{"VirtualPortID54",  2,	&gemportIDDb	},
	{"VirtualPortID55",  2,	&gemportIDDb	},
	{"VirtualPortID56",  2,	&gemportIDDb	},
	{"VirtualPortID57",  2,	&gemportIDDb	},
	{"VirtualPortID58",  2,	&gemportIDDb	},
	{"VirtualPortID59",  2,	&gemportIDDb	},
	{"VirtualPortID60",  2,	&gemportIDDb	},
	{"VirtualPortID61",  2,	&gemportIDDb	},
	{"VirtualPortID62",  2,	&gemportIDDb	},
	{"VirtualPortID63",  2,	&gemportIDDb	},
	{"UDF0field",  4,	&udfFieldDb	},
	{"UDF1field",  4,	&udfFieldDb	},
	{"UDF2field",  4,	&udfFieldDb	},
	{"UDF3field",  4,	&udfFieldDb	},
	{"UDF4field",  4,	&udfFieldDb	},
	{"UDF5field",  4,	&udfFieldDb	},
	{"UDF6field",  4,	&udfFieldDb	},
	{"UDF7field",  4,	&udfFieldDb	},
	{"MTU0",  2,	&otherclsDb},
	{"MTU1",  2,	&otherclsDb},
	{"MTU2",  2,	&otherclsDb},
	{"MTU3",  2,	&otherclsDb},
	{"MTU4",  2,	&otherclsDb},
	{"MTU5",  2,	&otherclsDb},
	{"MTU6",  2,	&otherclsDb},
	{"MTU7",  2,	&otherclsDb},
	{"MTU8",  2,	&otherclsDb},
	{"MTU9",  2,	&otherclsDb},
	{"MTU10",  2,	&otherclsDb},
	{"MTU11",  2,	&otherclsDb},
	{"MTU12",  2,	&otherclsDb},
	{"MTU13",  2,	&otherclsDb},
	{"MTU14",  2,	&otherclsDb},
	{"MTU15",  2,	&otherclsDb},
	{"MTU16",  2,	&otherclsDb},
	{"MTU17",  2,	&otherclsDb},
	{"MTU18",  2,	&otherclsDb},
	{"MTU19",  2,	&otherclsDb},
	{"MTU20",  2,	&otherclsDb},
	{"MTU21",  2,	&otherclsDb},
	{"MTU22",  2,	&otherclsDb},
	{"OPQN0",  2,	&otherclsDb},
	{"OPQN1",  2,	&otherclsDb},
	{"OPQN2",  2,	&otherclsDb},
	{"OPQN3",  2,	&otherclsDb},
	{"OPQN4",  2,	&otherclsDb},
	{"OPQN5",  2,	&otherclsDb},
	{"OPQN6",  2,	&otherclsDb},
	{"OPQN7",  2,	&otherclsDb},
	{"PLCTI",  2,	&otherclsDb},
	{"PLC",    2,	&otherclsDb},
	{"PLDEC",  2,	&otherclsDb},
	{"ins1IdSz",		2,	&otherclsDb},
	{"ins2IdSz",		2,	&otherclsDb},
	{"ins3IdSz",		2,	&otherclsDb},
	{"ins4IdSz",		2,	&otherclsDb},
	{"ins5IdSz",		2,	&otherclsDb},
	{"ins6IdSz",		2,	&otherclsDb},
	{"ins7IdSz",		2,	&otherclsDb},
	{"ins8IdSz",		2,	&otherclsDb},
	{"ethPortCtrl0",	4,	&eth_port_ctrl_FieldDb},
	{"ethPortCtrl1",	4,	&eth_port_ctrl_FieldDb},
	{"ethPortCtrl2",	4,	&eth_port_ctrl_FieldDb},
	{"ethPortCtrl3",	4,	&eth_port_ctrl_FieldDb},
	{"ethPortCtrl4",	4,	&eth_port_ctrl_FieldDb},
	{"ethPortCtrl5",	4,	&eth_port_ctrl_FieldDb},
	{"ethPortCtrl6",	4,	&eth_port_ctrl_FieldDb},
	{"ethPortCtrl7",	4,	&eth_port_ctrl_FieldDb},
};

PacketFieldDb_S clsFieldDb =
{
    clsFieldAra,
    sizeof(clsFieldAra)/sizeof(clsFieldAra[0])
};

/******************************************************************************
 *
 * Function   : getPacketToken
 *              
 * Description: The function returns a packet field token
 *              It skips '"', gobbles leading spaces, and copies to token string
 *              till it reaches a '"'  or ',' at the correct hierarchy level
 *              e.g. for        VT = [0x8100,b10x, x, 25], IPV4=[d....
 *              it builds VT = [0x8100,b10x, x, 25] as token
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

static void getPacketToken(UINT8 *token, UINT8 **str, unsigned int *skipped)
{
    int    indx = 0;
    int    jndx = 0;
    UINT8  *tempstr = *str;
    bool   square = false;

    if (tempstr[indx] == QUOTE_CHAR)
    {
        indx++;
    }

    while (1)
    {
        if (tempstr[indx] == COMMA)
        {
            if (square == false)
            {
                token[jndx] = 0;
		 indx++;
                *str = &tempstr[indx];
                break;
            }
            else
            {
                token[jndx++] = tempstr[indx++];
            }
        }
        else if (tempstr[indx] == LEFTSQUARE_CHAR)
        {
            token[jndx++] = tempstr[indx++];
            square = true;
        }
        else if (tempstr[indx] == RIGHTSQUARE_CHAR)
        {
            token[jndx++] = tempstr[indx++];
            square = false;
        }
        else if (tempstr[indx] == QUOTE_CHAR)
        {
            token[jndx] = 0;
	     indx++;
            *str = &tempstr[indx];
            break;
        }
        else if (tempstr[indx] != 0)
        {
            token[jndx++] = tempstr[indx++];
        }
        else
        {
            token[jndx] = 0;
            *str = &tempstr[indx];
            break;
        }
    }
    DEBUG_PR(DEB_OTHER, "indx %d jndx %d\n", indx, jndx);
    if (NULL != skipped) {
	     (*skipped) += indx;
    }
}



/******************************************************************************
 *
 * Function   : insertZeroesTcamPacket
 *              
 * Description: The function fills zeroes in the TCAM packet field
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool insertZeroesTcamPacket(PncEntry_S  *pPnCEntry, int length)
{
    bool rc = false;

    if ((pPnCEntry->currTcamPktIndx + length) <= PACKET_SIZE)
    {
        memset(&pPnCEntry->tcam.packet[pPnCEntry->currTcamPktIndx],     0, length); 
        memset(&pPnCEntry->tcamMask.packet[pPnCEntry->currTcamPktIndx], 0, length); 
        rc = true;
    }
    else
    {
        ERR_PR("Overrun TCAM packet currTcamPktIndx = %i, numBytesInValue = %i\n", 
               pPnCEntry->currTcamPktIndx, length);
    }
    return rc;
}



/******************************************************************************
 *
 * Function   : lookupPacketField
 *              
 * Description: The function searches for a matching PacketField_S
 *              
 * Parameters :  
 *
 * Returns    : PacketField_S  * or 0
 *              
 ******************************************************************************/

static PacketField_S *lookupPacketField(UINT8 *str)
{
    PacketField_S  *pPacketField;
    int            indx;

    for (indx = 0; indx < packetFieldDb.numEntries; indx++)
    {
        pPacketField = &packetFieldDb.pPacketFieldAra[indx];

        if (strcmp((char *)str, pPacketField->name) == 0)
        {
            return pPacketField;
        }
    }

    return 0;
}

/******************************************************************************
 *
 * Function   : lookupClsField
 *
 * Description: The function searches for a matching PacketField_S
 *
 * Parameters :
 *
 * Returns    : PacketField_S  * or 0
 *
 ******************************************************************************/

static PacketField_S *lookupClsField(UINT8 *str)
{
    PacketField_S  *pPacketField;
    int            indx;

    for (indx = 0; indx < clsFieldDb.numEntries; indx++)
    {
        pPacketField = &clsFieldDb.pPacketFieldAra[indx];

        if (strcmp((char *)str, pPacketField->name) == 0)
        {
            return pPacketField;
        }
    }

    return 0;
}


/******************************************************************************
 *
 * Function   : isFieldANamedSpacer
 *              
 * Description: The function determines if this is anmed spacer e.g. "VT" and not "VT=[..]
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool isFieldANamedSpacer(UINT8 *str)
{
    int   indx;
    int   length = strlen((char *)str);
    bool  rc     = true;

    for (indx = 0; indx < length; indx++)
    {
        if (str[indx] == EQUAL_CHAR) 
        {
            rc = false;
        }
    }

    return rc;
}




/******************************************************************************
 *
 * Function   : dealWithNamedPacketField
 *              
 * Description: The function deals with named packet fields
 *              e.g. "XDSA=[qosp=b110xxx], VT = [0x88A8,x, x, 100], VT = [0x8100,4, x, 200]"
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool dealWithNamedPacketField(UINT8 *str, PncEntry_S  *pPnCEntry, int *tcamPktBytesUsed)
{
    PacketField_S  *pPacketField;
    UINT8          *locstr = str;
    UINT8          pktFldStr[MAX_PNCLINESIZE];
    bool           isNamedSpacer;
    bool           rc = false;

    isNamedSpacer = isFieldANamedSpacer(locstr);

    if ((locstr = (char*)gobble(locstr)) != 0)
    {
        // First extract the packet field
        if (extractFieldString(locstr, pktFldStr, sizeof(pktFldStr)) == false)
        {
            ERR_PR("extractPacketFieldString failed for %s\n", locstr);
            return rc;
        }
    }
    else
    {
        ERR_PR("Unexpected EOS for %s\n", str);
        return rc;
    }

    if ((pPacketField = lookupPacketField(pktFldStr)) != 0)
    {
        // Advance over the packet field name
        locstr = &locstr[strlen((char *)pktFldStr)];

        if (isNamedSpacer == true)
        {
            // Hit a named spacer
            if ((rc = insertZeroesTcamPacket(pPnCEntry, pPacketField->width)) == true)
            {
                *tcamPktBytesUsed = pPacketField->width;
                rc = true;
            }
        }
        else
        {
            DEBUG_PR(DEB_OTHER, "Packet field %s with details %s\n", pPacketField->name, locstr);
//            if ((pPnCEntry->currTcamPktIndx + pPacketField->width) <= PACKET_SIZE)
//            {
                rc = dealWithPktSubfields(locstr, pPnCEntry, pPacketField);
		 if (false == rc) {
		 	ERR_PR("dealWithPktSubfields failed");
		 	return rc;
		 }
//                {
//                    *tcamPktBytesUsed = pPacketField->width;
//                }
//            }
//            else
//            {
//                ERR_PR("Exceeded packet length currTcamPktIndx/maxWidth = %i/%i,field width = %i\n", 
//                       pPnCEntry->currTcamPktIndx, sizeof(pPnCEntry->tcam.packet), pPacketField->width);
//            }
        }
    }
    else
    {
        ERR_PR("findPacketField failed for field %s\n", pktFldStr);
    }

    return rc;
}

/******************************************************************************
 *
 * Function   : dealWithNamedClsField
 *
 * Description: The function deals with named cls fields
 *              e.g. "SPID0=[0],Port0SPIDExtract=[1]"
 *
 * Parameters :
 *
 * Returns    : bool
 *
 ******************************************************************************/
static bool dealWithNamedClsField(UINT8 *str, PncEntry_S  *pPnCEntry, int *tcamPktBytesUsed)
{
    PacketField_S  *pPacketField;
    UINT8          *locstr = str;
    UINT8          pktFldStr[MAX_PNCLINESIZE];
    bool           isNamedSpacer;
    bool           rc = false;

    isNamedSpacer = isFieldANamedSpacer(locstr);

    if ((locstr = gobble((char*)locstr)) != 0)
    {
        // First extract the packet field
        if (extractFieldString(locstr, pktFldStr, sizeof(pktFldStr)) == false)
        {
            ERR_PR("extractPacketFieldString failed for %s\n", locstr);
            return rc;
        }
    }
    else
    {
        ERR_PR("Unexpected EOS for %s\n", str);
        return rc;
    }

    if ((pPacketField = lookupClsField(pktFldStr)) != 0)
    {
        // Advance over the packet field name
        locstr = &locstr[strlen((char *)pktFldStr)];

        if (isNamedSpacer == true)
        {
            // Hit a named spacer
            if ((rc = insertZeroesTcamPacket(pPnCEntry, pPacketField->width)) == true)
            {
                *tcamPktBytesUsed = pPacketField->width;
                rc = true;
            }
        }
        else
        {
            DEBUG_PR(DEB_OTHER, "Packet field %s with details %s\n", pPacketField->name, locstr);
                rc = dealWithPktSubfields(locstr, pPnCEntry, pPacketField);
		 if (false == rc) {
		 	ERR_PR("dealWithPktSubfields failed");
		 	return rc;
		 }
        }
    }
    else
    {
        ERR_PR("findPacketField failed for field %s\n", pktFldStr);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : getExpectedByteWidthOfValue
 *              
 * Description: The function returns the number of expected bytes in the numeric 
 *              string that is being scanned. See Reurns below
 *              
 * Parameters :  
 *
 * Returns    : int  0: < 2 hex chars met
 *                   1: 2 hex chars met
 *                   2: 4 hex chars met
 *                   3: > 4 hex chars met
 *              
 ******************************************************************************/

static int getExpectedByteWidthOfValue(UINT8 *str)
{
    int length = strlen((char *)str);
    int indx;
    int rval;
    int symbcount = 0;

    for (indx = 0; indx < length; indx++)
    {
        if (isxdigit(str[indx]) != 0)
        {
            symbcount++;
        }
        else
        {
            break;
        }
    }

    if (symbcount == 0)
    {
        rval = 0;
    }
    else if (symbcount < 3)
    {
        rval = 1;
    }
    else if (symbcount < 5)
    {
        rval = 2;
    }
    else
    {
        rval = 3;
    }

    return rval;
}



/******************************************************************************
 *
 * Function   : parseBitStringIntoPacketValueAndMask
 *              
 * Description: The function parses a bit string and places in packet and packet mask
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool parseBitStringIntoPacketValueAndMask(UINT8 *locstr, PncEntry_S *pPnCEntry, int *tcamPktBytesUsed)
{
    unsigned int  value = 0;
    unsigned int  valueMask = 0;
    bool          rc = false;
    int           indx;
    int           length = strlen((char *)locstr);
    int           numBytesInValue = 1;

    if (length <= 17)
    {
        // Leave out leading 'b'
        for (indx = 1; indx < length; indx++)
        {
            if (locstr[indx] == '1')
            {
                value     = value << 1;
                value     |= 0x1;
                valueMask = valueMask << 1;
                valueMask |= 0x1;
            }
            else if (locstr[indx] == '0')
            {
                value     = value << 1;
                valueMask = valueMask << 1;
                valueMask |= 0x1;
            }
            else if (locstr[indx] == 'x')
            {
                value     = value << 1;
                valueMask = valueMask << 1;
            }
            else
            {
                ERR_PR("Unexpected char %c in bit descriptor %s\n", locstr[indx], locstr);
                return false;
            }
        }

        // More than 9 chars means 2 byte value
        if (length > 9)
        {
            numBytesInValue = 2;
        }

        if ((pPnCEntry->currTcamPktIndx + numBytesInValue) <= sizeof(pPnCEntry->tcam.packet))
        {
            if (numBytesInValue == 2)
            {
                pPnCEntry->tcam.packet[pPnCEntry->currTcamPktIndx]         = (UINT8)(value >> 8);  
                pPnCEntry->tcam.packet[pPnCEntry->currTcamPktIndx + 1]     = (UINT8)(value & 0xFF);  
                pPnCEntry->tcamMask.packet[pPnCEntry->currTcamPktIndx]     = (UINT8)(valueMask >> 8);    
                pPnCEntry->tcamMask.packet[pPnCEntry->currTcamPktIndx + 1] = (UINT8)(valueMask & 0xFF);  
            }
            else // (numBytesInValue == 1)
            {
                pPnCEntry->tcam.packet[pPnCEntry->currTcamPktIndx]     = (UINT8)(value & 0xFF);  
                pPnCEntry->tcamMask.packet[pPnCEntry->currTcamPktIndx] = (UINT8)(valueMask & 0xFF);
            }
            *tcamPktBytesUsed = numBytesInValue;
            rc = true;
        }
        else
        {
            ERR_PR("TCAM packet area overrun. currTcamPktIndx = %i, numBytesInValue = %i\n", 
                   pPnCEntry->currTcamPktIndx, numBytesInValue);
        }
    }
    else
    {
        ERR_PR("max bit string is 2 bytes (16 bits)wide. String '%s' has %i bits\n", locstr, length-1);
    }

    return rc;
}




/******************************************************************************
 *
 * Function   : parseByteSpacerIntoPacketValueAndMask
 *              
 * Description: The function parses a byte spacer in packet and packet mask
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool parseByteSpacerIntoPacketValueAndMask(UINT8 *str, PncEntry_S *pPnCEntry, int *tcamPktBytesUsed)
{
    UINT8          *locstr = str;
    unsigned int   value = 0;
    bool           rc = false;

    if (sscanf((char *)locstr, "%i", &value) != 0)
    {
        if ((locstr = skipOverNumber(locstr)) != 0)
        {
            if (locstr[0] == BYTE_CHAR)
            {
                if ((rc = insertZeroesTcamPacket(pPnCEntry, value)) == true)
                {
                    *tcamPktBytesUsed = value;
                    rc = true;
                }
            }
            else
            {
                ERR_PR("Expected 'B', received '%c' in spacer %s\n", locstr[0], locstr);
            }
        }
        else
        {
            ERR_PR("Missing B in spacer %s\n", locstr);
        }
    }
    else
    {
        ERR_PR("Failed to parse integer %s\n", locstr);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : parseHexStringIntoPacketValueAndMask
 *              
 * Description: The function parses a hex string into packet and packet mask
 *              The string starts 0x... i.e. the 3rd char is the first hex char
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool parseHexStringIntoPacketValueAndMask(UINT8 *str, PncEntry_S *pPnCEntry, int *tcamPktBytesUsed)
{
    UINT8          *locstr = str;
    unsigned int   value = 0;
    bool           rc = false;

    int numBytesInValue = getExpectedByteWidthOfValue(&locstr[2]);
    UINT32 maxValueAra[] = { MAX_1BYTEVALUE, MAX_2BYTEVALUE };

    if (numBytesInValue == 1 || numBytesInValue == 2)
    {
        if (sscanf((char *)(&locstr[2]), "%x", &value) != 0)
        {
            if (value <= maxValueAra[numBytesInValue-1])
            {
                if ((pPnCEntry->currTcamPktIndx + numBytesInValue) <= PACKET_SIZE)
                {
                    unsigned short unsh16 = (unsigned short)value;

                    if (numBytesInValue == 2)
                    {
                        pPnCEntry->tcam.packet[pPnCEntry->currTcamPktIndx]         = (UINT8)(unsh16 >> 8);  
                        pPnCEntry->tcam.packet[pPnCEntry->currTcamPktIndx + 1]     = (UINT8)(unsh16 & 0xFF);  
                        pPnCEntry->tcamMask.packet[pPnCEntry->currTcamPktIndx]     = 0xFF;  
                        pPnCEntry->tcamMask.packet[pPnCEntry->currTcamPktIndx + 1] = 0xFF;  
                    }
                    else // (numBytesInValue == 1)
                    {
                        pPnCEntry->tcam.packet[pPnCEntry->currTcamPktIndx]     = (UINT8)(unsh16 & 0xFF);  
                        pPnCEntry->tcamMask.packet[pPnCEntry->currTcamPktIndx] = 0xFF;  
                    }

                    *tcamPktBytesUsed = numBytesInValue;
                    rc = true;
                }
                else
                {
                    ERR_PR("Exceeded TCAM packet length. currTcamPktIndx/maxWidth = %i/%i, field width = %i\n", 
                           pPnCEntry->currTcamPktIndx, sizeof(pPnCEntry->tcam.packet), value);
                }
            }
            else
            {
            }
        }
        else
        {
            ERR_PR("sscanf failed to scan %s\n", &locstr[2]);
        }
    }
    else
    {
        ERR_PR("Width of hex literal invalid. numBytesInValue = %i in received token %s\n", numBytesInValue, locstr);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithNumericPacketField
 *              
 * Description: The function deals with a numeric field e.g. xB where x is
 *              an integer, or 0xabcd a 2 byte wide hex value
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool dealWithNumericPacketField(UINT8 *str, PncEntry_S *pPnCEntry, int *tcamPktBytesUsed)
{
    UINT8          *locstr = str;
    bool           rc = false;

    if (locstr[0] == 'b')
    {
        rc = parseBitStringIntoPacketValueAndMask(locstr, pPnCEntry, tcamPktBytesUsed);
    }
    else if (locstr[0] != '0')
    {
        rc = parseByteSpacerIntoPacketValueAndMask(locstr, pPnCEntry, tcamPktBytesUsed);
    }
    else  if (locstr[1] == 'x' || locstr[1] == 'X')
    {
        rc = parseHexStringIntoPacketValueAndMask(locstr, pPnCEntry, tcamPktBytesUsed);
    }
    else
    {
        ERR_PR("Expected '0x...'. Met only '0' in %s\n", locstr);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : isPacketField
 *              
 * Description: The function checks if given token is a packet field
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool isPacketField(UINT8 *str)
{
    UINT8          *locstr = str;
    UINT8          pktFldStr[MAX_PNCLINESIZE];

    // First extract the packet field
    if (extractFieldString(locstr, pktFldStr, sizeof(pktFldStr)) == true)
    {
        if ((lookupPacketField(pktFldStr)) != 0)
        {
            return true;
        }
    }
    return false;
}

/******************************************************************************
 *
 * Function   : isClsField
 *
 * Description: The function checks if given token is a packet field
 *
 * Parameters :
 *
 * Returns    : bool
 *
 ******************************************************************************/

static bool isClsField(UINT8 *str)
{
    UINT8          *locstr = str;
    UINT8          pktFldStr[MAX_PNCLINESIZE];

    memset(pktFldStr, 0, sizeof(pktFldStr));
    // First extract the packet field
    if (extractFieldString(locstr, pktFldStr, sizeof(pktFldStr)) == true)
    {
        if ((lookupClsField(pktFldStr)) != 0)
        {
            return true;
        }
    }
    return false;
}



/******************************************************************************
 *
 * Function   : dealWithDictionaryName
 *
 * Description: The function checks for a dictionary name that is potentially multiple
 *              comma separated numeric values
 *
 * Parameters :
 *
 * Returns    : bool
 *
 ******************************************************************************/

static bool dealWithDictionaryName(UINT8 *str, PncEntry_S  *pPnCEntry, int *tcamPktBytesUsed)
{
    DdEntry *pDdEntry;
    bool    allOk = true;
    bool    rc = false;
    UINT8   *tempValStr;
    int     numValueBytes;
    int     totalValueBytes = 0;
    int     origTcamPktBytesUsed = pPnCEntry->currTcamPktIndx;

    if ((pDdEntry = findMatchingEntry(str)) != 0)
    {
        UINT8  token[MAX_PNCLINESIZE];
        int    indx = 0;

        tempValStr = pDdEntry->value;

        getPacketToken(token, &tempValStr, NULL);

        while (strlen((char *)token) != 0)
        {
            indx++;

            if ((rc = dealWithNumericPacketField(token, pPnCEntry, &numValueBytes)) == false)
            {
                ERR_PR("dealWithNumericPacketField failed for '%s'\n", token);
                allOk = false;
                break;
            }
            else
            {
                pPnCEntry->currTcamPktIndx += numValueBytes;
                totalValueBytes += numValueBytes;
            }

            getPacketToken(token, &tempValStr, NULL);
        }
        if (allOk == true)
        {
            pPnCEntry->currTcamPktIndx = origTcamPktBytesUsed;
            *tcamPktBytesUsed          = totalValueBytes;
            rc = true;
        }
    }
    else
    {
        ERR_PR("findMatchingEntry failed for '%s'\n", str);
    }
    return rc;
}


/******************************************************************************
 *
 * Function   : dealWithOnePacketToken
 *              
 * Description: The function deals with one packet field
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool dealWithOnePacketToken(UINT8 *pktToken, PncEntry_S  *pPnCEntry, int *tcamPktBytesUsed)
{
    UINT8 *locstr = pktToken;
    bool  rc = false;

    if ((locstr = gobble((char*)locstr)) != 0)
    {
        if (isdigit(locstr[0]) != 0  || isBitString(locstr) == true)
        {
            rc = dealWithNumericPacketField(locstr, pPnCEntry, tcamPktBytesUsed);
        }
        else if (isPacketField(locstr) == true)
        {
            rc = dealWithNamedPacketField(locstr, pPnCEntry, tcamPktBytesUsed);
        }
	else if (isClsField(locstr) == true)
	{
	    rc = dealWithNamedClsField(locstr, pPnCEntry, tcamPktBytesUsed);
	}
        else
        {
            rc = dealWithDictionaryName(locstr, pPnCEntry, tcamPktBytesUsed);
        }
    }
    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithPacket
 *              
 * Description: The function deals with Packet
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithPacket(UINT8 *token, PncEntry_S  *pPnCEntry, unsigned int *skipped)
{
    int      tcamPktBytesUsed;
    int      tokLength;
    bool     rc = false;
    UINT8    *tempToken = token;
    bool     isEmpty = true;

    tokLength = strlen((char *)tempToken);

    if (tokLength != 0)
    {
        if ((tempToken = (char*)gobble(tempToken)) != 0)
        {
            if (tempToken[0] != '*')
            {
                isEmpty = false;
            }
        }
    }
    if (isEmpty != false)
    {
        DEBUG_PR(DEB_OTHER, "Packet field is empty!!!\n");
        rc = true;
    }
    else
    {
        UINT8  pktToken[MAX_PNCLINESIZE];

        getPacketToken(pktToken, &tempToken, skipped);

        if (strlen((char *)pktToken) != 0)
        {
            rc = dealWithOnePacketToken(pktToken, pPnCEntry, &tcamPktBytesUsed);
            if (true == rc)
            {
                pPnCEntry->currTcamPktIndx += tcamPktBytesUsed;
            }
        }
    }

    return rc;
}





