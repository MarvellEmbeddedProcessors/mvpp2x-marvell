/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : TcamParse.h                                               **/
/**                                                                          **/
/**  DESCRIPTION : This file contains TCAM parse routines                    **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    17-Jun-09   zeev  - initial version created.                              
 *                                                                      
 ******************************************************************************/


#ifndef __INCTcamParseh
#define __INCTcamParseh

extern bool dealWithAddInfo(UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithPortId(UINT8 *token, PncEntry_S  *pPnCEntry);
extern bool dealWithLu(UINT8 *luStr, PncEntry_S  *pPnCEntry);


#endif

