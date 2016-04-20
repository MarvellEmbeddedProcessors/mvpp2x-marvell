/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : DataDictionary.h                                          **/
/**                                                                          **/
/**  DESCRIPTION : This file contains definitions of the params sheet DB     **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    17-Jun-09   zeev  - initial version created.                              
 *                                                                      
 ******************************************************************************/


#ifndef __INCDataDictionaryh
#define __INCDataDictionaryh

#define MAX_DICTLINESIZE        100

#define FORWARDSLASH_CHAR       '/'
#define MAX_DICTENTRIES         1500


typedef struct
{
    
    bool       inuse;
    char	name[49];
    char	value[49];
} DdEntry;


typedef struct
{
    DdEntry *pentryAra;
    int     dictSize;
} Dictionary;


extern bool prepareDataDictionary(char *filename);
extern DdEntry *findMatchingEntry(char *name);
extern void printDictionary();

#endif

