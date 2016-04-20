/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : PncParse.h                                                **/
/**                                                                          **/
/**  DESCRIPTION : This file contains PnC parse (top level) definitions      **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    17-Jun-09   zeev  - initial version created.                              
 *                                                                      
 ******************************************************************************/


#ifndef __INCPncParseh
#define __INCPncParseh

extern int  parsePnCFile(char *filename);
extern void printPncStats();


typedef struct
{
    char *tokName;
    bool (*tokHandler)(UINT8 *token, PncEntry_S *pPnCEntry);
} TokenHandlingEntry_S;



#endif

