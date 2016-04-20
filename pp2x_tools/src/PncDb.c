/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : PnCDb.c                                                   **/
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "PncGlobals.h"


PncEntry_S pncEntryAra[MAX_PNCROWS];

PncDb_S pncDb =
{
    pncEntryAra, sizeof(pncEntryAra)/sizeof(pncEntryAra[0])
};



/******************************************************************************
 *
 * Function   : prepareHexDumpString
 *              
 * Description: The function prepares string for printout
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

void prepareHexDumpString(UINT8 *inStr, UINT8 *outStr, int length)
{
    int indx;

    for (indx = 0; indx < length; indx++)
    {
        sprintf((char *)&outStr[indx*2], "%02x", inStr[indx]); 
    }
}





/******************************************************************************
 *
 * Function   : findNextUsedPncEntry
 *              
 * Description: The function finds the next used PnC DB entry
 *              
 * Parameters :  
 *
 * Returns    : PncEntry_S * or 0
 *              
 ******************************************************************************/

PncEntry_S *findNextUsedPncEntry(PncEntry_S *pPncEntry)
{
    PncEntry_S *pNextPncEntry;
    int        dbIndx = 0;

    for (dbIndx = pPncEntry->dbIndx + 1; dbIndx < pncDb.numEntries; dbIndx++)
    {
        pNextPncEntry = &pncDb.pentryAra[dbIndx];

        if (pNextPncEntry->inuse == true)
        {
            return pNextPncEntry;
        }
    }
    return 0;
}



/******************************************************************************
 *
 * Function   : findFirstUsedPncEntry
 *              
 * Description: The function finds the first used PnC DB entry
 *              
 * Parameters :  
 *
 * Returns    : PncEntry_S * or 0
 *              
 ******************************************************************************/

PncEntry_S *findFirstUsedPncEntry()
{
    PncEntry_S *pPncEntry;
    int        dbIndx = 0;

    for (dbIndx = 0; dbIndx < pncDb.numEntries; dbIndx++)
    {
        pPncEntry = &pncDb.pentryAra[dbIndx];

        if (pPncEntry->inuse == true)
        {
            return pPncEntry;
        }
    }
    return 0;
}



/******************************************************************************
 *
 * Function   : findUnusedPncEntry
 *              
 * Description: The function finds an unused PnC DB entry
 *              
 * Parameters :  
 *
 * Returns    : PncEntry_S * or 0
 *              
 ******************************************************************************/

PncEntry_S *findUnusedPncEntry(int rowInd)
{
    PncEntry_S *pPncEntry;
    int        dbIndx = 0;

    for (dbIndx = 0; dbIndx < pncDb.numEntries; dbIndx++)
    {
        pPncEntry = &pncDb.pentryAra[dbIndx];

        if (pPncEntry->inuse == false)
        {
            // flowGemMask = -1 needed for combined Gem/Txp2 logic
            pPncEntry->sram.u.sram_reg.flowGemMask = -1;
            return pPncEntry;
        }
        else if (pncEntryAra->ind == rowInd)
        {
            printf("%s: Row with duplicate Ind = %i found\n", __FUNCTION__, rowInd);
            break;
        }
    }
    return 0;
}



/******************************************************************************
 *
 * Function   : initPncDb
 *              
 * Description: The function clears PnC DB
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

void initPncDb()
{
    PncEntry_S *pPncEntry;
    int        dbIndx = 0;

    for (dbIndx = 0; dbIndx < pncDb.numEntries; dbIndx++)
    {
        pPncEntry = &pncDb.pentryAra[dbIndx];

        pPncEntry->inuse           = false;
        pPncEntry->dbIndx          = dbIndx;
        pPncEntry->currTcamPktIndx = 0;
    }
}




