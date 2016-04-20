/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : PacketParse.h                                             **/
/**                                                                          **/
/**  DESCRIPTION : This file contains parse definition for the Packet field  **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    17-Jun-09   zeev  - initial version created.                              
 *                                                                      
 ******************************************************************************/


#ifndef __INCPacketParseh
#define __INCPacketParseh


extern bool dealWithPacket(UINT8 *token, PncEntry_S  *pPnCEntry, unsigned int *skip);

#endif


