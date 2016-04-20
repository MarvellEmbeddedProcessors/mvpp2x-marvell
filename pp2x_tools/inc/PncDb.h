/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : PnCDb.h                                                   **/
/**                                                                          **/
/**  DESCRIPTION : This file contains the runtime PnC DB definitions         **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    17-Jun-09   zeev  - initial version created.                              
 *                                                                      
 ******************************************************************************/


#ifndef __INCPnCDbh
#define __INCPnCDbh


PncEntry_S *findUnusedPncEntry(int rowInd);
extern PncEntry_S *findFirstUsedPncEntry();
extern PncEntry_S *findNextUsedPncEntry(PncEntry_S *pPncEntry);

extern void initPncDb();
extern void printPncDb();

#endif
