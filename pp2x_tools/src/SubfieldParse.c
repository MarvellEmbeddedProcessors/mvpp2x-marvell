/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
licensing terms.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>

#include "common.h"
#include "PncGlobals.h"
#include "ParseUtils.h"
#include "SubfieldParse.h"

static RtSubFldEntry_S rtSubFldEntryAra[MAX_SUBFIELDS];

static RtSubFldDb_S rtSubFldDb =
{
    rtSubFldEntryAra, sizeof(rtSubFldEntryAra)/sizeof(rtSubFldEntryAra[0])
};



/******************************************************************************
 *
 * Function   : initRtSubFldDb
 *              
 * Description: The function inits the run time subfield database
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

static void initRtSubFldDb(RtSubFldDb_S *pRtSubFldDb)
{
    RtSubFldEntry_S *pRtSubFldEntry;
    int             indx;

    pRtSubFldDb->pPacketField = 0;

    for (indx =0; indx < pRtSubFldDb->numEntries; indx++)
    {
        pRtSubFldEntry = &pRtSubFldDb->pRtSubFldEntryAra[indx];

        pRtSubFldEntry->inuse               = false;
        pRtSubFldEntry->subfldIndx          = 0;
        pRtSubFldEntry->value[0]            = 0;
        pRtSubFldEntry->parsedIntValue     = 0;
        pRtSubFldEntry->parsedIntValueMask = 0;
    }
}


/******************************************************************************
 *
 * Function   : printRtSubFldDb
 *              
 * Description: The function prints the run time subfield database
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

static void printRtSubFldDb(RtSubFldDb_S *pRtSubFldDb)
{
    RtSubFldEntry_S *pRtSubFldEntry;
    int             indx;
    int             howmany = 0;

    printf("\nRun Time Subfield DB for %s\n", pRtSubFldDb->pPacketField->name);
    printf("SubfldIndx  Subfield name     Value\n");
    printf("----------  -------------     -----\n");

    for (indx =0; indx < pRtSubFldDb->numEntries; indx++)
    {
        pRtSubFldEntry = &pRtSubFldDb->pRtSubFldEntryAra[indx];

        if (pRtSubFldEntry->inuse == true)
        {
            howmany++;
            printf("%-10i  %-16s  %s\n", 
                   pRtSubFldEntry->subfldIndx, 
                   pRtSubFldDb->pPacketField->pSubFieldDb->pSubFieldAra[pRtSubFldEntry->subfldIndx].name,
                   pRtSubFldEntry->value);
        }
    }
    if (howmany == 0)
    {
        printf("*** RtSubFldDb is empty\n");
    }
    printf("\n");
}


unsigned int getRtSubFld(char **field_name, char **subfield_name, char **subfield_value, unsigned int *field_id,
			RtSubFldEntry_S	**ppRtSubFldEntry, unsigned int indx)
{
    RtSubFldDb_S	*pRtSubFldDb = &rtSubFldDb;

    if (indx < pRtSubFldDb->numEntries)
    {
        *ppRtSubFldEntry = &pRtSubFldDb->pRtSubFldEntryAra[indx];

        if ((*ppRtSubFldEntry)->inuse == true)
        {
	 	*field_name = pRtSubFldDb->pPacketField->name;
		*subfield_name = pRtSubFldDb->pPacketField->pSubFieldDb->pSubFieldAra[(*ppRtSubFldEntry)->subfldIndx].name;
		*subfield_value = (*ppRtSubFldEntry)->value;
		*field_id = pRtSubFldDb->pPacketField->pSubFieldDb->pSubFieldAra[(*ppRtSubFldEntry)->subfldIndx].field_id;
	 	return 1;
        }
    }
    return 0;
}


/******************************************************************************
 *
 * Function   : getSubfldIndxForUnnamed
 *              
 * Description: The function searches for a matching SubField_S
 *              
 * Parameters :  
 *
 * Returns    : Index of next available SubField_S  or -1
 *              
 ******************************************************************************/

static int getSubfldIndxForUnnamed()
{
    RtSubFldDb_S     *pRtSubFldDb = &rtSubFldDb;
    RtSubFldEntry_S  *pRtSubFldEntry;
    int              indx;
    int              subfldIndx = -1;

    for (indx =0; indx < pRtSubFldDb->numEntries; indx++)
    {
        pRtSubFldEntry = &pRtSubFldDb->pRtSubFldEntryAra[indx];

        if (pRtSubFldEntry->inuse == true)
        {
            if (subfldIndx == -1)
            {
                subfldIndx = pRtSubFldEntry->subfldIndx;
            }
            else
            {
                if (pRtSubFldEntry->subfldIndx > subfldIndx)
                {
                    subfldIndx = pRtSubFldEntry->subfldIndx;
                }
            }
        }
    }

    if (subfldIndx == -1)
    {
        subfldIndx = 0;
    }
    else
    {
        subfldIndx++;
    }

    return subfldIndx;
}



/******************************************************************************
 *
 * Function   : findUnusedRtSubFldEntry
 *              
 * Description: The function searches for an unused RtSubFldEntry_S
 *              
 * Parameters :  
 *
 * Returns    : RtSubFldEntry_S * or 0
 *              
 ******************************************************************************/

static RtSubFldEntry_S *findUnusedRtSubFldEntry(RtSubFldDb_S *pRtSubFldDb)
{
    RtSubFldEntry_S *pRtSubFldEntry;
    int             indx;

    for (indx =0; indx < pRtSubFldDb->numEntries; indx++)
    {
        pRtSubFldEntry = &pRtSubFldDb->pRtSubFldEntryAra[indx];

        if (pRtSubFldEntry->inuse == false)
        {
            return pRtSubFldEntry;
        }
    }
    return 0;
}



/******************************************************************************
 *
 * Function   : lookupSubFieldIndx
 *              
 * Description: The function searches for a matching SubField_S
 *              
 * Parameters :  
 *
 * Returns    : Index of matching SubField_S  or -1
 *              
 ******************************************************************************/

static int lookupSubFieldIndx(UINT8 *str, PacketField_S *pPacketField)
{
    SubField_S  *pSubField;
    int         indx;

    for (indx = 0; indx < pPacketField->pSubFieldDb->numEntries; indx++)
    {
        pSubField = &pPacketField->pSubFieldDb->pSubFieldAra[indx];

        if (strcmp((char *)str, pSubField->name) == 0)
        {
            return indx;
        }
    }

    return -1;
}




/******************************************************************************
 *
 * Function   : isSubFieldAlreadyUsed
 *              
 * Description: The function checks that the subfield has not already been used
 *              e.g. "VT = [0x88A8,vid=101, x, vid=100]"  or "VT = [0x88A8, 3, pb=2"
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool isSubFieldAlreadyUsed(PacketField_S *pPacketField, RtSubFldDb_S *pRtSubFldDb, int subfldIndx)
{
    RtSubFldEntry_S *pRtSubFldEntry;
    int             indx;
    bool            rc = false;

    for (indx =0; indx < pRtSubFldDb->numEntries; indx++)
    {
        pRtSubFldEntry = &pRtSubFldDb->pRtSubFldEntryAra[indx];

        if (pRtSubFldEntry->inuse == true && pRtSubFldEntry->subfldIndx == subfldIndx)
        {
            rc = true;
            break;
        }
    }
    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithNamedPacketField
 *              
 * Description: The function deals with named packet fields
 *              e.g. "XDSA=[qosp=b110xxx], VT = [0x88A8,x, x, 100], VT = [0x8100,4, x, 200]"
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool dealWithNamedSubfield(UINT8 *str, PncEntry_S  *pPnCEntry, PacketField_S *pPacketField, RtSubFldDb_S *pRtSubFldDb, RtSubFldEntry_S *pRtSubFldEntry)
{
    int     subfldIndx;
    UINT8   *locstr = str;
    UINT8   subFldStr[MAX_SUBFIELDSIZE];
    bool    rc = false;

    if ((locstr = gobble(locstr)) != 0)
    {
        // First extract the packet field
        if (extractFieldString(locstr, subFldStr, sizeof(subFldStr)) == false)
        {
            ERR_PR("extractPacketFieldString failed for %s\n", locstr);
            return rc;
        }
    }
    else
    {
        ERR_PR("Unexpected EOS for %s\n", str);
        return rc;
    }

    if ((subfldIndx = lookupSubFieldIndx(subFldStr, pPacketField)) != -1)
    {
        if (isSubFieldAlreadyUsed(pPacketField, pRtSubFldDb, subfldIndx) == false)
        {
            pRtSubFldEntry->subfldIndx = subfldIndx;
    
            // Advance over the packet field name
            locstr = &locstr[strlen((char *)subFldStr)];
    
            if ((locstr = gobble(locstr)) != 0)
            {
                if ((locstr = skipChar(locstr, EQUAL_CHAR)) != 0)
                {
                    if ((locstr = gobble(locstr)) != 0)
                    {
                        strcpy((char *)pRtSubFldEntry->value, (char *)locstr);
                        rc = true;
                    }
                }
            }
        }
        else
        {
            ERR_PR("Subfield %s used more than once\n", subFldStr);
        }
    }
    else
    {
        ERR_PR("lookupSubField failed for subfield %s\n", subFldStr);
    }

    return rc;
}




/******************************************************************************
 *
 * Function   : dealWithNumericPacketField
 *              
 * Description: The function deals with a numeric subfield. 
 *              The main problem here is assigning the subfield.
 *              The rules are:
 *                 assign 0 if no other subfields are defined till this point
 *                 else search the used entries in RtSubFieldDb and assign the 
 *                 highest index so far + 1 (and check that it is in range)
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool dealWithNumericSubField(UINT8 *str, PncEntry_S  *pPnCEntry, PacketField_S *pPacketField, RtSubFldEntry_S *pRtSubFldEntry)
{
    int     subfldIndx;
    UINT8   *locstr = str;
    bool    rc = false;

    if ((subfldIndx = getSubfldIndxForUnnamed()) != -1)
    {
        if (subfldIndx < pPacketField->pSubFieldDb->numEntries)
        {
            pRtSubFldEntry->subfldIndx = subfldIndx;
            strcpy((char *)pRtSubFldEntry->value, (char *)locstr);
            rc = true;

        }
        else
        {
            ERR_PR("subfldIndx failed for field %s. subfldIndx = %i >=  #subFields (%i)\n", 
                   pPacketField->name, subfldIndx, pPacketField->pSubFieldDb->numEntries);
        }
    }
    else
    {
        ERR_PR("getSubfldIndxForUnnamed failed. pPacketField->name = %s\n", pPacketField->name);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : isTheSubFieldNumeric
 *              
 * Description: The function checks if the supplied subfield is numeric or named
 *              Looking at first character alone fails since b101x1 is a bit pattern value
 *              and a first character x may mean don't care
 *              So look for a '=' as the discriminator
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool isTheSubFieldNumeric(UINT8 *str)
{
    bool  rc = true;
    int   indx;
    int   length = strlen((char *)str);

    for (indx = 0; indx < length; indx++)
    {
        if (str[indx] == EQUAL_CHAR)
        {
            rc = false;
            break;
        }
    }
    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithOneSubfield
 *              
 * Description: The function deals with one packet subfield
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool dealWithOneSubfield(UINT8 *subFieldToken, PncEntry_S *pPnCEntry, PacketField_S *pPacketField, RtSubFldDb_S *pRtSubFldDb)
{
    UINT8            *locsubfstr = subFieldToken;
    RtSubFldEntry_S  *pRtSubFldEntry;
    bool             rc = false;

    if ((locsubfstr = gobble(locsubfstr)) != 0)
    {
        if ((pRtSubFldEntry = findUnusedRtSubFldEntry(pRtSubFldDb)) != 0)
        {

            if (isTheSubFieldNumeric(locsubfstr) == true)
            {
                rc = dealWithNumericSubField(locsubfstr, pPnCEntry, pPacketField, pRtSubFldEntry);
            }
            else
            {
                rc = dealWithNamedSubfield(locsubfstr, pPnCEntry, pPacketField, pRtSubFldDb, pRtSubFldEntry);
            }

            // The   inuse = true  is deferred till here so that DB searches for used are easier (or initialize subfldIndx to -1)
            if (rc == true)
            {
                pRtSubFldEntry->inuse = true;
            }
        }
        else
        {
            ERR_PR("findUnusedRtSubFldEntry failed. pPacketField->name = %s, locsubfstr = --%s--\n", pPacketField->name, locsubfstr);
        }
    }
    else
    {
        ERR_PR("Unexpected EOS. pPacketField->name = %s, subFieldToken = --%s--\n", pPacketField->name, subFieldToken);
    }
    return rc;
}



/******************************************************************************
 *
 * Function   : getSubFieldToken
 *              
 * Description: The function returns a packet subfield token
 *              It skips '[', gobbles leading spaces, and copies to token string
 *              till it reaches a '"'  or ','
 *              e.g. for        [0x8100, b10x, x, vid=25]
 *              it builds 0x8100     b10x    x    vid=25 as 4 tokens, one per routine invocation
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

static void getSubFieldToken(UINT8 *token, UINT8 **str)
{
    int   indx = 0;
    int   jndx = 0;
    UINT8 *tempstr = *str;

    // Rid of leading space
    while (1)
    {
        if (tempstr[indx] == SPACE)
        {
            indx++;
        }
        else if (tempstr[indx] == 0)
        {
            token[0] = 0;
            return;
        }
        else
        {
            break;
        }
    }

    while (1)
    {
        if (tempstr[indx] == COMMA)
        {
            token[jndx] = 0;
            *str = &tempstr[indx+1];
            break;
        }
        else if (tempstr[indx] == RIGHTSQUARE_CHAR)
        {
            token[jndx] = 0;
            *str = &tempstr[indx+1];
            break;
        }
        else if (tempstr[indx] != 0)
        {
            token[jndx++] = tempstr[indx++];
        }
        else
        {
            token[jndx] = 0;
            *str = &tempstr[indx];
            break;
        }
    }
}



/******************************************************************************
 *
 * Function   : insertRtSubFldDbIntoPnc
 *              
 * Description: The function parses subfield values and inserts them into Tcam/Tcam Mask.
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool insertRtSubFldDbIntoPnc(PncEntry_S *pPnCEntry, PacketField_S *pPacketField, RtSubFldDb_S *pRtSubFldDb)
{
    bool             rc = false;
    int              indx;
    RtSubFldEntry_S  *pRtSubFldEntry;
    SubField_S       *pSubField;

    for (indx = 0; indx < pRtSubFldDb->numEntries; indx++)
    {
        pRtSubFldEntry = &pRtSubFldDb->pRtSubFldEntryAra[indx];

        if (pRtSubFldEntry->inuse == true)
        {
            pSubField = &pPacketField->pSubFieldDb->pSubFieldAra[pRtSubFldEntry->subfldIndx];

            if (pSubField->valHandler != 0)
            {
                if ((rc = (pSubField->valHandler)(pSubField, pRtSubFldEntry)) == false)
                {
                    break;
                }
            }
            else
            {
                ERR_PR("No valhandler defined for Field:Subfield %s:%s\n", pPacketField->name, pSubField->name);
            }
        }
    }
    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithPktSubfield
 *              
 * Description: The function deals with packet subfields
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithPktSubfields(UINT8 *str, PncEntry_S *pPnCEntry, PacketField_S *pPacketField)
{
    UINT8 *locstr = str;
    bool  rc = false;

    // Init the runtime subfield DB
    initRtSubFldDb(&rtSubFldDb);

    // Copy in the PacketField_S pointer
    rtSubFldDb.pPacketField = pPacketField;

    // Cleanup start of string up to '='
    if ((locstr = gobble(locstr)) != 0)
    {
        if (locstr[0] == EQUAL_CHAR)
        {
            locstr = &locstr[1];
            if ((locstr = gobble(locstr)) != 0)
            {
                UINT8  subFieldToken[MAX_PNCLINESIZE];
                int    indx = 0;

                if (locstr[0] == LEFTSQUARE_CHAR)
                {
                    locstr = &locstr[1];
                }
                else
                {
                	DEBUG_PR(DEB_OTHER, "Packet field %i without [, field = %s\n", indx, locstr);
                }

                // Loop over all the subfields
                getSubFieldToken(subFieldToken, &locstr);

                while (strlen((char *)subFieldToken) != 0)
                {
                    indx++;
                    if ((rc = dealWithOneSubfield(subFieldToken, pPnCEntry, pPacketField, &rtSubFldDb)) == false)
                    {
                        break;
                    }
                    getSubFieldToken(subFieldToken, &locstr);
                }

                if (rc == true)
                {
                    // Print subfields collected
                    if (getDebugFlag() == true)
                    {
                        printRtSubFldDb(&rtSubFldDb);
                    }
    
                    // NOW PROCESS the RUNTIME SUBFIELDS WE COLLECTED
                    rc = insertRtSubFldDbIntoPnc(pPnCEntry, pPacketField, &rtSubFldDb);
                }
            }
        }
    }
    return rc;
}






