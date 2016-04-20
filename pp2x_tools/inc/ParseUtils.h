/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : ParseUtils.h                                              **/
/**                                                                          **/
/**  DESCRIPTION : This file contains parse utility routines                 **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    17-Jun-09   zeev  - initial version created.                              
 *                                                                      
 ******************************************************************************/


#ifndef __INCParseUtilsh
#define __INCParseUtilsh

#include "ezxml.h"
#include "PncGlobals.h"

unsigned int common_mask_gen(int bit_num);

extern bool parseSimpleNumericValue(UINT8 *str, int *value);
extern bool scanIntOrDdlookup(char *str, int *rvalue);
extern bool scanInt(char *str, int *rvalue);
extern bool intorDdLookupTokenHandler(UINT8 *token, int *result, UINT32 maxValue, UINT32 defaultValue);
extern bool bitPairsLookupTokenHandler(UINT8 *token, int *result, UINT32 maxValue, UINT32 defaultValue, UINT32 maxLen);

extern UINT8 *skipOverNumber(UINT8 *str);
extern UINT8 *skipChar(UINT8 *str, UINT8 ch);
extern char *gobble(char *str);
extern UINT8 *gobble_and_skip(UINT8 *str, unsigned int *skipped);

extern bool parseIntegerSubFldValue(SubField_S *pSubField, RtSubFldEntry_S *pRtSubFldEntry);

extern bool parseMacAddressSubFldValue(SubField_S *pSubField, RtSubFldEntry_S *pRtSubFldEntry);

extern bool parseIpAddressSubfieldValue(SubField_S *pSubField, RtSubFldEntry_S *pRtSubFldEntry);

extern bool parseIpAddressSubFldValue(SubField_S *pSubField, RtSubFldEntry_S *pRtSubFldEntry);

extern bool parseIpV6AddressSubFldValue(SubField_S *pSubField, RtSubFldEntry_S *pRtSubFldEntry);

extern bool parseIpV6AddrSuffPrefSubFldValue(SubField_S *pSubField, RtSubFldEntry_S *pRtSubFldEntry);

extern bool insertIntegerIntoBitfield(UINT8 *byteAra, UINT32 value, int hibit, int lobit, int araSize);

extern bool dealWithOneBx(UINT8 *bxstr, int *pbIndx, int *pbvalue);

extern UINT8 *getNextLocalToken(UINT8 *str, char separator);

extern bool extractFieldString(UINT8 *origStr, UINT8 *tgtStr, int tgtLength);

extern bool isStringInDirectValueFormat(UINT8 *str);

extern bool normalizeAddInfoString(UINT8 *str, UINT8 *tgtStr);

extern bool isTokenEmpty(UINT8 *token);

extern bool isBitString(UINT8 *str);

extern int handle_sysfs_command(char *buf, bool info);

#ifdef __WINDOWS__
#define LITTLE_ENDIAN        (1)
   
#if defined(BIG_ENDIAN) && !defined(LITTLE_ENDIAN)

#define htons(A)  (A)
#define htonl(A)  (A)
#define ntohs(A)  (A)
#define ntohl(A)  (A)

#elif defined(LITTLE_ENDIAN) && !defined(BIG_ENDIAN)

#define htons(A)  ((((UINT16)(A) & 0XFF00)     >> 8)  | \
                   (((UINT16)(A) & 0X00FF)     << 8))
#define htonl(A)  ((((UINT32)(A) & 0XFF000000) >> 24) | \
                   (((UINT32)(A) & 0X00FF0000) >> 8)  | \
                   (((UINT32)(A) & 0X0000FF00) << 8)  | \
                   (((UINT32)(A) & 0X000000FF) << 24))
#define ntohs     htons
#define ntohl     htonl

#else

#error "Either BIG_ENDIAN or LITTLE_ENDIAN must be #defined, but not both."

#endif

#endif


#endif
