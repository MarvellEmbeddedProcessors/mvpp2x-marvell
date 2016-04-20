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
/**  DESCRIPTION : This file contains TCAM cell parse                        **/
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

#include "PncGlobals.h"
#include "DataDictionary.h"
#include "ParseUtils.h"
#include "TcamParse.h"



/******************************************************************************
 *
 * Function   : dealWithLu
 *              
 * Description: The function deals with LU 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithLu(UINT8 *token, PncEntry_S  *pPnCEntry)
{
    UINT8    *locstr = token;
    int      tokLength;
    int      value;
    bool     isEmpty = true;
    bool     rc = false;

    tokLength = strlen((char *)locstr);

    if (tokLength != 0)
    {
        if ((locstr = gobble(locstr)) != 0)
        {
            if (locstr[0] != '*')
            {
                isEmpty = false;

                if (scanIntOrDdlookup(locstr, &value) == true)
                {
                    if (value <= MAX_LU)
                    {
                        pPnCEntry->tcam.lu     = value;
                        pPnCEntry->tcamMask.lu = BM_LU;
                        rc = true; 
                    }
                }
            }
        }
    }

    if (isEmpty == true)
    {
        pPnCEntry->tcam.lu     = 0;
        pPnCEntry->tcamMask.lu = 0;
        rc = true; 
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithPortId
 *              
 * Description: The function deals with PortId
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithPortId(UINT8 *token, PncEntry_S  *pPnCEntry)
{
    int      tokLength;
    UINT8    *locstr = token;
    bool     rc = false;
    int      value;
    bool     isEmpty = true;

    tokLength = strlen((char *)locstr);

    if (tokLength != 0)
    {
        if ((locstr = gobble(locstr)) != 0)
        {
            if (locstr[0] != '*')
            {
                isEmpty = false;
            }
        }
    }

    if (isEmpty == true)
    {
        pPnCEntry->tcam.portId     = 0;
        pPnCEntry->tcamMask.portId = 0;
        rc = true;
    }
    else
    {
        if (scanIntOrDdlookup(locstr, &value) == true)
        {
            if (value <= MAX_PORTID)
            {
                pPnCEntry->tcam.portId     = value;
                pPnCEntry->tcamMask.portId = (~value) & BM_PORTID;
                rc = true;
            }
            else
            {
                printf("%s: ***ERROR portId %i > MAX_PORTID %i\n", __FUNCTION__, value, MAX_PORTID); 
            }
        }
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : dealWithAddInfo
 *              
 * Description: The function deals with AddInfo (Bx=a,By=b)
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithAddInfo(UINT8 *str, PncEntry_S  *pPnCEntry)
{
    int      tokLength;
    int      bIndx;
    int      bvalue;
    bool     rc = false;
    UINT8    *locstr;
    bool     isEmpty = true;

    tokLength = strlen((char *)str);

    if (tokLength != 0)
    {
        if ((locstr = gobble(str)) != 0)
        {
            if (locstr[0] != '*')
            {
                isEmpty = false;
            }
        }
    }

    if (isEmpty == true)
    {
        pPnCEntry->tcam.addInfo     = 0;
        pPnCEntry->tcamMask.addInfo = 0;
        rc = true;
    }
    else
    {
        UINT8  newstr[MAX_PNCLINESIZE] = { 0 };
        int   indx = 0;
        UINT8 *bxstr;

        if ((rc = normalizeAddInfoString(locstr, newstr)) == true)
        {
            bxstr = newstr;

            while (bxstr != 0)
            {
                //printf("indx = %i   bxstr = %s\n", indx, bxstr);
                if ((rc = dealWithOneBx(bxstr, &bIndx, &bvalue)) == false)
                {
                    rc = false;
                    break;
                }
                else
                {
                    pPnCEntry->tcam.addInfo     |= (bvalue << bIndx);
                    pPnCEntry->tcamMask.addInfo |= (1      << bIndx);
                }
                bxstr = getNextLocalToken(bxstr, COMMA);
                indx++;
            }
        }
    }

    return rc;
}






