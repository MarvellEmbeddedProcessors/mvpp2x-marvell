/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : PncParse.c                                                **/
/**                                                                          **/
/**  DESCRIPTION : This file contains PnC parse (top level)                  **/
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
#include <ctype.h>
#include <errno.h>

#include "PncGlobals.h"
#include "PncDb.h"
#include "ParseUtils.h"
#include "PacketParse.h"
#include "SramParse.h"
#include "TcamParse.h"
#include "PncParse.h"

// Parse statistics
static PncStats_S pncStats = { 0, 0, -1};
static int autoRowIndex = 0;

/******************************************************************************
 *
 * Function   : getPncStatsPtr
 *              
 * Description: The function returns pncStats pointer
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

PncStats_S *getPncStatsPtr()
{
    return &pncStats;
}


/******************************************************************************
 *
 * Function   : printPncStats
 *              
 * Description: The function prints pncStats
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

void printPncStats()
{
    PncStats_S *pPncStats = getPncStatsPtr();

    printf("\nPNC statistics\n--------------\n");
    printf("Lines processed          %i\n", pPncStats->linesProcessed);

    if (pPncStats->erroredLines > 0)
    {
        printf("Number of errored lines  %i\n", pPncStats->erroredLines);
        printf("Last line with error     %i\n", pPncStats->lastLineWithError);
    }
}


/******************************************************************************
 *
 * Function   : dealWithRowIndx
 *              
 * Description: The function deals with row index 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool dealWithRowIndx(UINT8 *indxStr, int *value)
{
    int      tokLength;
    int      tempValue;
    bool     rc = false;

    tokLength = strlen((char *)indxStr);

    if (tokLength != 0)
    {
        if (sscanf((char *)indxStr, "%d", &tempValue) != 0)
        {
            if (tempValue < MAX_PNCROWS)
            {
                *value = tempValue;
                rc = true;
            }
            else
            {
                printf("%s: ***ERROR row index = %d out of range\n", __FUNCTION__, tempValue);
            }
        }
    }
    return rc;
}



/******************************************************************************
 *
 * Function   : isRowUsed
 *              
 * Description: The function verifies that Index field is not empty 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool isRowUsed(UINT8 *token)
{
    bool     rc = false;

    if (strlen((char *)token) != 0)
    {
        rc = true;
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : getToken
 *              
 * Description: The function returns a token from the given string 
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

static void getToken(UINT8 *token, UINT8 **line)
{
    int   indx = 0;
    UINT8 *tempLine = *line;

    while (1)
    {
        if (tempLine[indx] == TAB_CHAR || tempLine[indx] == '\n' || tempLine[indx] == '\r')
        {
            token[indx] = 0;
            *line = &tempLine[indx+1];
            break;
        }
        else if (tempLine[indx] != 0)
        {
            token[indx] = tempLine[indx];
            indx++;
        }
        else
        {
            token[indx] = 0;
            *line = &tempLine[indx];
            break;
        }
    }
}


// Note that Row Index is handled separately
TokenHandlingEntry_S tokenHandlingAra[] =
{
    {"LU",                 dealWithLu},
    {"PortId",             dealWithPortId},
    {"AddInfo",            dealWithAddInfo},
    /*{"Packet",             dealWithPacket},*/
    {"NxtLuId",            dealWithNxtLuId},
    {"AddInfo Updt",       dealWithAddInfoUpdt},
    {"Key Type",           dealWithKeyType},
    {"LU Done",            dealWithLuDo},
    {"Nxt LU Offset Ind",  dealWithNxtLuOffInd},
    {"Que",                dealWithQue},
    {"Shift update",       dealWithShiftUpdt},
    {"Dis",                dealWithDis},
    {"L4",                 dealWithL4},
    {"L3",                 dealWithL3},
    {"Ff",                 dealWithFf},
    {"Fm",                 dealWithFm},
    {"Col",                dealWithCol},
    {"Txp",                dealWithTxp},
    {"Mh",                 dealWithMh},
    {"Gen",                dealWithGen},
    {"Prof",               dealWithProf},
    {"Mod",                dealWithSramMod},
    {"Gem",                dealWithSramGem},
    {"Txp2",               dealWithSramTxp2},
};

TokenHandlingEntry_S tokenHandlingExpandedAra[] =
{
    {"LU",                 dealWithLu},
    {"PortId",             dealWithPortId},
    {"AddInfo",            dealWithAddInfo},
    {"Packet",             dealWithPacket},
    {"NxtLuId",            dealWithNxtLuId},
    {"AddInfo Updt",       dealWithAddInfoUpdt},
    {"Key Type",           dealWithKeyType},
    {"LU Done",            dealWithLuDo},
    {"Nxt LU Offset Ind",  dealWithNxtLuOffInd},
    {"Que",                dealWithQue},
    {"Shift update",       dealWithShiftUpdt},
    {"Dis",                dealWithDis},
    {"L4",                 dealWithL4},
    {"L3",                 dealWithL3},
    {"Ff",                 dealWithFf},
    {"Fm",                 dealWithFm},
    {"Col",                dealWithCol},
    {"Txp",                dealWithTxp},
    {"Mh",                 dealWithMhExp},
    {"Gen",                dealWithGenExp},
    {"Prof",               dealWithProfExp},
    {"Gen2",               dealWithGen2Exp},
    {"Mod_H",              dealWithSramModH},
    {"Mod_M",              dealWithSramModM},
    {"Mod_L",              dealWithSramModL},
    {"Gem",                dealWithSramGemExp},
    {"Txp2",               dealWithSramTxp2Exp},
};


/******************************************************************************
 *
 * Function   : parsePncRow
 *              
 * Description: The function parses a PnC row
 *              First step is to skip rows with leading 2 columns empty 
 *              
 * Parameters :  
 *
 * Returns    : 0 = OK, 1 = error, 2 = empty row
 *              
 ******************************************************************************/

static int parsePncRow(UINT8 *origLine)
{
    int                  status = 0;
    UINT8                token[MAX_PNCLINESIZE];
    UINT8                *line = origLine;
    TokenHandlingEntry_S *pTokenHandlingAra;
    int                  indx;
    int                  tokenNum;
    PncStats_S           *pPncStats = getPncStatsPtr();
    PncEntry_S           *pPnCEntry = 0;
    int                  rowIndValue;

    // IMPORTANT
    // CANNOT USE strtok SINCE THE RECURSION OVER THE SEPARATOR DOES NOT REVEAL 
    // EMPTY CELLS IN THE EXCEL GENERATED FILE

    // ALGORITHM:
    // If first character is separator then the  index cell is empty - skip to next row
    // After that copy characters to token until but not incl separator, and null terminate

    getToken(token, &line);
    if (isRowUsed(token) == false)
    {
        pPncStats->linesWithoutRowIndex++;
        return 2;
    }

    if (getDebugFlag() == true)
    {
        printf("%20s = %s\n", "Row Indx (pre-parse)", token);
    }

    if (dealWithRowIndx(token, &rowIndValue) == true)
    {
        if ((pPnCEntry = findUnusedPncEntry(rowIndValue)) != 0)
        {
            pPnCEntry->inuse = true;
            pPnCEntry->ind   = rowIndValue;

            
                tokenNum = sizeof(tokenHandlingAra)/sizeof(tokenHandlingAra[0]);
                pTokenHandlingAra = tokenHandlingAra;

            for (indx = 0; indx < tokenNum; indx++)
            {
                getToken(token, &line);
    
                if (getDebugFlag() == true)
                {
                    printf("%20s = %s\n", pTokenHandlingAra->tokName, token);
                }
    
                if ((pTokenHandlingAra->tokHandler)(token, pPnCEntry) == false)
                {
                    status = 1;
                    printf("%s: ***ERROR %s handling failed for line with row index %i\n", 
                           __FUNCTION__, pTokenHandlingAra->tokName, pPnCEntry->ind);
                    break;
                }

                pTokenHandlingAra++;
            }
        }
        else
        {
            printf("%s: ***ERROR findUnusedPncEntry failed, line %i\n", __FUNCTION__, pPncStats->linesProcessed);
            status = 1;
        }
    }
    else
    {
        printf("%s: ***ERROR dealWithRowIndx failed for row %s\n", __FUNCTION__, token);
        status = 1;
    }

    if (status == 1 && pPnCEntry != 0)
    {
        pPnCEntry->inuse = false;
    }
    return status;
}




/******************************************************************************
 *
 * Function   : parsePnCFile
 *              
 * Description: The function parses the PnC file
 *              The file is in tab separator format
 *              
 * Parameters :  
 *
 * Returns    : 0 = OK, else there was error
 *              
 ******************************************************************************/

int parsePnCFile(char *filename)
{
    FILE        *fptr;
    UINT8       line[MAX_PNCLINESIZE];
    int         status;
    PncStats_S  *pPncStats = getPncStatsPtr();
    int         rc = 0;

    initPncDb();

    if ((fptr = fopen(filename, "r")) == 0)
    {
        printf("parsePnCFile: failed to open PnC file %s, errno = %i\n", filename, errno);  
        return 1;
    }

    while (1)
    {
        if (fgets((char *)line, sizeof(line), fptr) != 0)
        {
            pPncStats->linesProcessed++;

            if (getDebugFlag() == true)
            {
                printf("Start PnC Line %i: %s\n", pPncStats->linesProcessed, line);
            }

            status = parsePncRow(line);

            if (getDebugFlag() == true)
            {
                printf("End   PnC Line %i. status = %i\n", pPncStats->linesProcessed, status);
            }

            if (status == 1)
            {
                printf("%s **ERROR PnC Line %i: '%s'\n", __FUNCTION__, pPncStats->linesProcessed, line);
                pPncStats->erroredLines++;
                pPncStats->lastLineWithError = pPncStats->linesProcessed;

                // Stop or keep going
                if (getKeepGoingFlag() == false)
                {
                    rc = 1;
                    break;
                }
            }
        }
        else
        {
            printf("%s: EOF reached (%i lines read)\n", __FUNCTION__, pPncStats->linesProcessed+1);
            rc = 1;
            break;
        }
    }

    fclose(fptr);
    return rc;
}






