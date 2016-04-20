/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : SubfieldParse.h                                           **/
/**                                                                          **/
/**  DESCRIPTION : This file is for packet subfield parse                    **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    17-Jun-09   zeev  - initial version created.                              
 *                                                                      
 ******************************************************************************/


#ifndef __INCSubfieldParseh
#define __INCSubfieldParseh

typedef struct
{
    RtSubFldEntry_S *pRtSubFldEntryAra;
    int             numEntries;
    PacketField_S   *pPacketField;
} RtSubFldDb_S;



bool dealWithPktSubfields(UINT8 *str, PncEntry_S *pPnCEntry, PacketField_S *pPacketField);
unsigned int getRtSubFld(char			**field_name,
			 char			**subfield_name,
			 char			**subfield_value,
			 unsigned int 		*field_id,
			 RtSubFldEntry_S	**ppRtSubFldEntry,
			 unsigned int 		indx);

#endif


