/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : PncShellCommand.h                                         **/
/**                                                                          **/
/**  DESCRIPTION : This file contains definitions for generating shell       **/
/**                commands from PnC database                                **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    20-Jul-09   zeev  - initial version created.                              
 *                                                                      
 ******************************************************************************/


#ifndef __INCPncShellCommandh
#define __INCPncShellCommandh


typedef struct
{
    char  *field;
    bool  (*fieldHandler)(char *buf, char *cmdFormat, PncEntry_S *pPnCEntry);
    char  *cmdFormat;
} ShellCommand_S;




extern void generatePncShellCommands();


#endif



