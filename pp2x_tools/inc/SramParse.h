/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : SramParse.h                                               **/
/**                                                                          **/
/**  DESCRIPTION : This file contains SRAM parse routines                    **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    17-Jun-09   zeev  - initial version created.                              
 *                                                                      
 ******************************************************************************/


#ifndef __INCSramParseh
#define __INCSramParseh

extern bool dealWithNxtLuId    (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithAddInfoUpdt(UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithKeyType    (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithLuDo       (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithQue        (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithNxtLuOffInd(UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithShiftUpdt  (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithGen        (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithProf       (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithCol        (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithFm         (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithFf         (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithDis        (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithTxp        (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithMh         (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithL3         (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithL4         (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithSramTxp2   (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithSramGem    (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithSramMod    (UINT8 *token, PncEntry_S  *pPnCEntry);

extern bool dealWithMhExp      (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithGenExp     (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithGen2Exp    (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithProfExp    (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithSramModL   (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithSramModM   (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithSramModH   (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithSramGemExp (UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithSramTxp2Exp(UINT8 *token, PncEntry_S  *pPnCEntry);


#endif
