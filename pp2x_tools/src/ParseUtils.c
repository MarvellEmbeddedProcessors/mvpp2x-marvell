/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : ParseUtils.c                                              **/
/**                                                                          **/
/**  DESCRIPTION : This file contains parse utility routines                 **/
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
#include <fcntl.h>/*vbs*/

#ifdef __LINUX__
#include <time.h>
#include <netinet/in.h>
#endif

#include "common.h"
#include "PncGlobals.h"
#include "DataDictionary.h"
#include "ParseUtils.h"
#include "ezxml.h"

extern int vbs_fd;
extern int out_fd;
extern int quiet;

static UINT32 bitsMaskAra[] = 
{
    1,          3,          7,           0xF,          0x1F,       0x3F,        0x7F,        0xFF, 
    0x1FF,      0x3FF,      0x7FF,       0xFFF,        0x1FFF,     0x3FFF,      0x7FFF,      0xFFFF,
    0x1FFFF,    0x3FFFF,    0x7FFFF,     0xFFFFF,      0x1FFFFF,   0x3FFFFF,    0x7FFFFF,    0xFFFFFF,
    0x1FFFFFF,  0x3FFFFFF,  0x7FFFFFF,   0xFFFFFFF,    0x1FFFFFFF, 0x3FFFFFFF,  0x7FFFFFFF,  0xFFFFFFFF};



/******************************************************************************
 *
 * Function   : trailingSpaceGobble
 *              
 * Description: The function re-terminates a string by skipping over trailing 
 *              SPACE at end of string 
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

static void trailingSpaceGobble(char *str)
{
    int indx;
    int length = strlen(str);

    for (indx = length-1; indx >= 0; indx--)
    {
        if (str[indx] == SPACE)
        {
            str[indx] = 0;
        }
        else
        {
            break;
        }
    }
}



/******************************************************************************
 *
 * Function   : gobble
 *              
 * Description: The function gobbles string till non-space
 *              
 * Parameters :  
 *
 * Returns    : char * or 0
 *              
 ******************************************************************************/

char *gobble(char *str)
{
    int indx = 0;

    while (str[indx] != 0)
    {
        if (str[indx] != SPACE && str[indx] != COMMA)
        {
            return &str[indx];
        }
        indx++;
    }
    return 0;
}


UINT8 *gobble_and_skip(UINT8 *str, unsigned int *skipped)
{
    int indx = 0;

    while (str[indx] != 0)
    {
        if (str[indx] != SPACE && str[indx] != COMMA)
        {
	     //(*skipped) += indx;
            return &str[indx];
        }
        indx++;
    }
    return 0;
}

/******************************************************************************
 *
 * Function   : skipChar
 *              
 * Description: The function skips over supplied character
 *              
 * Parameters :  
 *
 * Returns    : UINT8 * or 0
 *              
 ******************************************************************************/

UINT8 *skipChar(UINT8 *str, UINT8 ch)
{
    int indx = 0;

    while (str[indx] != 0)
    {
        if (str[indx] != ch)
        {
            return &str[indx];
        }
        indx++;
    }
    return 0;
}



/******************************************************************************
 *
 * Function   : skipOverNumber
 *              
 * Description: The function skips over number
 *              
 * Parameters :  
 *
 * Returns    : char * or 0
 *              
 ******************************************************************************/

UINT8 *skipOverNumber(UINT8 *str)
{
    int indx = 0;

    while (str[indx] != 0)
    {
        if (isdigit(str[indx]) == 0)
        {
            return &str[indx];
        }
        indx++;
    }
    return 0;
}



/******************************************************************************
 *
 * Function   : isBitString
 *              
 * Description: The function hecks if string is a bit string i.e byyy
 *              where y is one of 'x', '0' or '1'
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool isBitString(UINT8 *str)
{
    UINT8       *locstr = str;
    int         indx;
    int         length;
    bool        rc = false;

    length  = strlen((char *)locstr);

    if (locstr[0] == 'b')
    {
        if (length > 1)
        {
            // Reverse the error logic
            rc = true;
            for (indx = 1; indx < length; indx++)
            {
                if ( !(locstr[indx] == '1' || locstr[indx] == '0' || locstr[indx] == 'x'))
                {
                    rc = false;
                    break;
                }
            }
        }
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : parseSimpleNumericValue
 *              
 * Description: The function parses for a bit descriptor, 1234, 0xabcd
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool parseSimpleNumericValue(UINT8 *str, int *value)
{
    int         indx;
    UINT8       *locstr = str;
    int         length;
    bool        rc = false;
    int         tempval = 0;

    if  (gobble(locstr) != 0)
    {
        length  = strlen((char *)locstr);

        if (isBitString(locstr) == true)
        {
            // Reverse the error logic
            rc = true;
            for (indx = 1; indx < length; indx++)
            {
                if (locstr[indx] == '1')
                {
                    tempval = tempval << 1;
                    tempval |= 0x1;
                }
                else if (locstr[indx] == '0')
                {
                    tempval = tempval << 1;
                }
                else if (locstr[indx] == 'x')
                {
                    tempval = tempval << 1;
                }
                else
                {
                    ERR_PR("Unexpected char %c in bit descriptor %s\n", locstr[indx], locstr);
                    rc = false;
                    break;
                }
            }
        }
        else if (locstr[0] == '0' &&  (locstr[1] == 'x' || locstr[1] == 'X'))
        {
            locstr = &locstr[2];

            if (sscanf((char *)locstr, "%x", &tempval) != 0)
            {
                rc = true;
            }
            else
            {        
                ERR_PR("sscanf hex failed for %s\n", locstr);
            }
        }
        else if (isdigit(locstr[0]) != 0)
        {
            if (sscanf((char *)locstr, "%i", &tempval) != 0)
            {
                rc = true;
            }
            else
            {        
                ERR_PR("sscanf decimal failed for %s\n", locstr);
            }
        }
        else
        {        
            ERR_PR("Unexpected subfield value (not numeric, not bit descriptor) %s\n", locstr);
        }
    }
    else
    {        
        ERR_PR("Unexpected EOS (gobble) --%s--\n", locstr);
    }

    if (rc == true)
    {
        *value = tempval;
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : scanIntOrDdlookup
 *              
 * Description: The function scans for an integer value or does Dictionary lookup
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool scanIntOrDdlookup(char *str, int *rvalue)
{
    int   value;
    bool  rc = false;
    char *locstr = str;

    // See if this is an integer or string or bit string
    if ((locstr = gobble(locstr)) != 0 )
    {
        trailingSpaceGobble(locstr);

        if (isBitString(locstr) == true || isdigit(locstr[0]) != 0)
        {
            if ((rc = parseSimpleNumericValue(locstr, &value)) == true)
            {
                *rvalue = value;
            }
        }
        else
        {
            DdEntry *pDdEntry = findMatchingEntry(locstr);

            if (pDdEntry != 0)
            {
                if ((rc = parseSimpleNumericValue(pDdEntry->value, &value)) == true)
                {
                    *rvalue = value;
                }
            }
        }
    }
    return rc;
}



/******************************************************************************
 *
 * Function   : scanInt
 *              
 * Description: The function scans for an integer value
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool scanInt(char *str, int *rvalue)
{
    int  value;
    bool rc = false;

    // See if this is an integer or string
    if (isdigit(str[0]))
    {
        if (sscanf((char *)str, "%d", &value) != 0)
        {
            *rvalue = value;
            rc = true; 
        }
    }
    return rc;
}



/******************************************************************************
 *
 * Function   : intorDdLookupTokenHandler
 *              
 * Description: The function deals with token parsing for integer value or
 *              dictionary lookup
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool intorDdLookupTokenHandler(UINT8 *token, int *result, UINT32 maxValue, UINT32 defaultValue)
{
    int      tokLength;
    bool     rc = false;
    int      value;

    tokLength = strlen((char *)token);

    if (tokLength != 0)
    {
        if (scanIntOrDdlookup(token, &value) == true)
        {
            if (value <= maxValue)
            {
                *result = value;
                rc = true;
            }
            else
            {
                ERR_PR("token '%s' value %i > %i (maxValue)\n", token, value, maxValue);
            }
        }
        else
        {
            ERR_PR("Failed for token '%s'\n", token);
        }
    }
    else
    {
        *result = defaultValue;
        rc = true;
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : bitPairsLookupTokenHandler
 *              
 * Description: The function deals with token parsing for bits value which is
 *              updated by pairs
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/
bool bitPairsLookupTokenHandler(UINT8 *token, int *result, UINT32 maxValue, UINT32 defaultValue, UINT32 maxLen)
{
    int      length;
    UINT8   *locstr = token;
    int      value = 0;
    int      indx, tmpindx;
    bool     rc = true;

    length  = strlen((char *)locstr);

    if (length == 0) {
        *result = defaultValue;
        return true;
    }

    if  (gobble(locstr) != 0)
    {
        if (isBitString(locstr) == true)
        {
            if (length > maxLen+1) 
            {
                ERR_PR("Unexpected length %d --%s--\n", length-1, locstr);
                *result = defaultValue;
                return false;
            }

            // Reverse the error logic
            for (indx = 1; indx < length; indx++)
            {
                if (locstr[indx] == '1')
                {
                    value = value << 1;
                    value |= 0x1;
                }
                else if (locstr[indx] == '0')
                {
                    value = value << 1;
                }
                else if (locstr[indx] == 'x')
                {
                    /* Verify whether the pairs bit is also equal to "x" */
                    if (indx%2)
                    {
                        /* odd index - next symbol should be "x" */
                        tmpindx = indx+1;

                        if (tmpindx >= length) 
                        {
                            ERR_PR("Unexpected number of chars %d in token %s\n", length, locstr);
                            rc = false;
                            break;
                        }
                    }
                    else
                    {
                        /* even index - previous symbol should be "x" */
                        tmpindx = indx-1;
                    }

                    if ( locstr[tmpindx] != 'x') 
                    {
                        ERR_PR("Setting a single bit of a pair %s\n", locstr);
                        rc = false;
                        break;
                    }

                    value = value << 1;
                }
                else
                {
                    ERR_PR("Unexpected char %c in bit descriptor %s\n", locstr[indx], locstr);
                    rc = false;
                    break;
                }
            }

            if (rc == true) {
                if (value <= maxValue)
                    *result = value;
                else
                    ERR_PR("in token '%s' value %i > %i (maxValue)\n", token, value, maxValue);
            }
        }
        else
            ERR_PR("Unexpected subfield value (not bit descriptor) %s\n", locstr);
    }
    else
        ERR_PR("Unexpected EOS (gobble) --%s--\n", token);

    return rc;
}


/******************************************************************************
 *
 * Function   : normalizeIpAddressValueString
 *              
 * Description: The function verifies that the value string is parsable
 *              Copies in dictionary value if necessary
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

static void normalizeIpAddressValueString(RtSubFldEntry_S *pRtSubFldEntry)
{
    int         indx;
    UINT8       *locstr = pRtSubFldEntry->value;
    int         length  = strlen((char *)locstr);
    DdEntry     *pDdEntry;

    if  (gobble(locstr) != 0)
    {
        for (indx = 1; indx < length; indx++)
        {
            if (locstr[indx] == '.')
            {
                return;
            }
        }

        // So we probaly need to do a dictionary look
        if ((pDdEntry = findMatchingEntry(locstr)) != 0)
        {
            strcpy((char *)pRtSubFldEntry->value, (char *)pDdEntry->value);
        }
        else
        {
            ERR_PR("findMatchingEntry failed for %s\n", locstr);
        }
    }
}



/******************************************************************************
 *
 * Function   : parseOneIpAddrPartAsNumber
 *              
 * Description: The function parses one component of IP address as number
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool parseOneIpAddrPartAsNumber(UINT8 *token, UINT8 *value, UINT8 *valuemask)
{
    bool rc = true;
    int  ipnum;

    if (sscanf((char *)token, "%i", &ipnum) != 0)
    {
        if (ipnum > 255)
        {
            ERR_PR("Invalid number %i in IP address\n", ipnum);
            rc = false;
        }
        else
        {
            *value     = (UINT8)ipnum;
            *valuemask = 0xFF;
        }
    }
    else
    {
        ERR_PR("Invalid number %i in IP address\n", ipnum);
        rc = false;
    }
    return rc;
}



/******************************************************************************
 *
 * Function   : parseOneByteAsBitString
 *              
 * Description: The function parses one byte as bitstring b10x
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool parseOneByteAsBitString(UINT8 *token, UINT8 *value, UINT8 *valueMask)
{
    bool    rc = true;
    int     indx;
    UINT8   *locstr = token;
    int     length = strlen(locstr);
    UINT32  temp = 0;
    UINT32  tempMask = 0;;

#define MAX_BITSTRINGLENGTHFORUINT8         9    

    if (length <= MAX_BITSTRINGLENGTHFORUINT8)
    {
        for (indx = 1; indx < length; indx++)
        {
            if (locstr[indx] == '1')
            {
                temp     = temp << 1;
                temp     |= 0x1;
                tempMask = tempMask << 1;
                tempMask |= 0x1;
            }
            else if (locstr[indx] == '0')
            {
                temp     = temp << 1;
                tempMask = tempMask << 1;
                tempMask |= 0x1;
            }
            else if (locstr[indx] == 'x')
            {
                temp     = temp << 1;
                tempMask = tempMask << 1;
            }
            else
            {
                ERR_PR("Unexpected char %c in bit descriptor %s\n", locstr[indx], locstr);
                rc = false;
                break;
            }
        }
    }
    else
    {
        ERR_PR("Too many bits (%i) in token %s\n", length-1, locstr);
        rc = false;
    }

    if (rc == true)
    {
        *value     = (UINT8)temp;
        *valueMask = (UINT8)tempMask;
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : parseOneIpAddrPartAsDontCare
 *              
 * Description: The function parses one component of IP address as Don't care =x
 *              We check only the first character for 'x'
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool parseOneIpAddrPartAsDontCare(UINT8 *token, UINT8 *value, UINT8 *valueMask)
{
    bool    rc = true;
    UINT8   *locstr = token;

    if (locstr[0] == 'x')
    {
        *value     = 0;
        *valueMask = 0;

        if (strlen(&locstr[1]) != 0)
        {
            ERR_PR("Unexpected character(s) '%s' after DC character are ignored\n", &locstr[1]);
        }
    }
    else
    {
        ERR_PR("Unexpected DC token %s\n", token);
        rc = false;
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : parseIpAddressSubFldValue
 *              
 * Description: The function parses IP Address subfield value
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool parseIpAddressSubFldValue(SubField_S *pSubField, RtSubFldEntry_S *pRtSubFldEntry)
{
    UINT8       *locstr;
    bool        rc = false;
    int         numsIndx = 0;       
    UINT8       *token;

    normalizeIpAddressValueString(pRtSubFldEntry);

    locstr = pRtSubFldEntry->value;

    if  (gobble(locstr) != 0)
    {
        token = (UINT8 *)strtok((char *)locstr, ".");

        while (token != 0)
        {

            if (numsIndx >= 4)
            {
                ERR_PR("Too many IP address tokens\n", __FUNCTION__);
                rc = false;
                break;
            }
            if  (gobble(locstr) != 0)
            {
                // Now support number, x, and b01x
                if (isdigit(token[0]) != 0)
                {
                    rc = parseOneIpAddrPartAsNumber(token, 
                                                    &pRtSubFldEntry->parsedIpAddress[numsIndx], 
                                                    &pRtSubFldEntry->parsedIpAddressMask[numsIndx]);
                }
                else if (isBitString(token) == true)
                {
                    rc = parseOneByteAsBitString(token, 
                                                       &pRtSubFldEntry->parsedIpAddress[numsIndx], 
                                                       &pRtSubFldEntry->parsedIpAddressMask[numsIndx]);
                }
                else
                {
                    rc = parseOneIpAddrPartAsDontCare(token, 
                                                      &pRtSubFldEntry->parsedIpAddress[numsIndx], 
                                                      &pRtSubFldEntry->parsedIpAddressMask[numsIndx]);
                }
    
                // if the parse step failed - exit
                if (rc == false)
                {
                    break;
                }
    
                numsIndx++;
    
                token = (UINT8 *)strtok(NULL, ".");
            }
            else
            {
                break;
            }
        }
    }

    if (rc == true && numsIndx != 4)
    {
        printf("%s: ***ERROR Too few IP address tokens\n", __FUNCTION__);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : parseOneIpAddrPartAsDontCare
 *              
 * Description: The function parses one component of IP address as Don't care =x
 *              We check only the first character for 'x'
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool parseOneIpv6AddrPartAsDontCare(UINT8 *token, UINT16 *value, UINT16 *valueMask)
{
    bool    rc = true;
    UINT8   *locstr = token;

    if (locstr[0] == 'x')
    {
        *value     = 0;
        *valueMask = 0;

        if (strlen(&locstr[1]) != 0)
        {
            DEBUG_PR(DEB_OTHER, "Unexpected character(s) '%s' after DC character are ignored\n", &locstr[1]);
        }
    }
    else
    {
        ERR_PR("Unexpected DC token %s\n", token);
        rc = false;
    }

    return rc;
}

/******************************************************************************
 *
 * Function   : parseOneIpv6AddrPartAsNumber
 *              
 * Description: The function parses one component of IPv6 address as number
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool parseOneIpv6AddrPartAsNumber(UINT8 *token, UINT16 *value, UINT16 *valuemask)
{
    bool rc = true;
    int  ipnum;

    if (sscanf((char *)token, "%i", &ipnum) != 0)
    {
        if (ipnum > 0xFFFF)
        {
            ERR_PR("Invalid number %x in IPv6 address\n", ipnum);
            rc = false;
        }
        else
        {
            *value     = (UINT16)ipnum;
            *valuemask = 0xFFFF;
        }
    }
    else
    {
        ERR_PR("Invalid number %x in IPv6 address\n", ipnum);
        rc = false;
    }
    return rc;
}

/******************************************************************************
 *
 * Function   : normalizeIpv6AddressValueString
 *              
 * Description: The function verifies that the value string is parsable
 *              Copies in dictionary value if necessary
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

static void normalizeIpv6AddressValueString(RtSubFldEntry_S *pRtSubFldEntry)
{
    int         indx;
    UINT8       *locstr = pRtSubFldEntry->value;
    int         length  = strlen((char *)locstr);
    DdEntry     *pDdEntry;

    if  (gobble(locstr) != 0)
    {
        for (indx = 1; indx < length; indx++)
        {
            if (locstr[indx] == ':')
            {
                return;
            }
        }

        // So we probaly need to do a dictionary look
        if ((pDdEntry = findMatchingEntry(locstr)) != 0)
        {
            strcpy((char *)pRtSubFldEntry->value, (char *)pDdEntry->value);
        }
        else
        {
            ERR_PR("findMatchingEntry failed for %s\n", locstr);
        }
    }
}

/******************************************************************************
 *
 * Function   : parseIpV6AddressSubFldValue
 *              
 * Description: The function parses IPv6 Address subfield value
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/
bool parseIpV6AddressSubFldValue(SubField_S *pSubField, RtSubFldEntry_S *pRtSubFldEntry)
{
    UINT8       *locstr;
    bool        rc = false;
    int         numsIndx = 0;       
    UINT8       *token;

    normalizeIpv6AddressValueString(pRtSubFldEntry);

    locstr = pRtSubFldEntry->value;

    if  (gobble(locstr) != 0)
    {
        token = (UINT8 *)strtok((char *)locstr, ":");

        while (token != 0)
        {
            if (numsIndx >= IPV6ADDR_TOKEN_NUMS)
            {
                ERR_PR("Too many IP address tokens\n", __FUNCTION__);
                rc = false;
                break;
            }

            if  (gobble(locstr) != 0)
            {
                // Now support number and x
                if (isdigit(token[0]) != 0)
                {
                    rc = parseOneIpv6AddrPartAsNumber(token, 
                                                      &pRtSubFldEntry->parsedIpv6Address[numsIndx], 
                                                      &pRtSubFldEntry->parsedIpv6AddressMask[numsIndx]);
                }
                else
                {
                    rc = parseOneIpv6AddrPartAsDontCare(token, 
                                                        &pRtSubFldEntry->parsedIpv6Address[numsIndx], 
                                                        &pRtSubFldEntry->parsedIpv6AddressMask[numsIndx]);
                }
    
                // if the parse step failed - exit
                if (rc == false)
                {
                    break;
                }

                numsIndx++;
                token = (UINT8 *)strtok(NULL, ":");
            }
            else
            { 
                break;
            }
        } /* while */
    }

    if ((rc == true) && (numsIndx != IPV6ADDR_TOKEN_NUMS))
    {
        ERR_PR("Too few IP address tokens\n", __FUNCTION__);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : parseIpV6AddrSuffPrefSubFldValue
 *              
 * Description: The function parses IPv6 Address suffix/prefix subfield value
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/
bool parseIpV6AddrSuffPrefSubFldValue( SubField_S *pSubField, RtSubFldEntry_S *pRtSubFldEntry)
{
    UINT8       *locstr;
    bool        rc = false;
    int         numsIndx = 0;       
    UINT8       *token;

    normalizeIpv6AddressValueString(pRtSubFldEntry);

    locstr = pRtSubFldEntry->value;

    if  (gobble(locstr) != 0)
    {
        token = (UINT8 *)strtok((char *)locstr, ":");

        while (token != 0)
        {
            if (numsIndx >= IPV6ADDR_SUFF_TOKEN_NUMS)
            {
                ERR_PR("Too many IP address suffix/prefix tokens\n");
                rc = false;
                break;
            }

            if  (gobble(locstr) != 0)
            {
                // Now support number and x
                if (isdigit(token[0]) != 0)
                {
                    rc = parseOneIpv6AddrPartAsNumber(token, 
                                                      &pRtSubFldEntry->parsedIpv6Address[numsIndx], 
                                                      &pRtSubFldEntry->parsedIpv6AddressMask[numsIndx]);
                }
                else
                {
                    rc = parseOneIpv6AddrPartAsDontCare(token, 
                                                        &pRtSubFldEntry->parsedIpv6Address[numsIndx], 
                                                        &pRtSubFldEntry->parsedIpv6AddressMask[numsIndx]);
                }
    
                // if the parse step failed - exit
                if (rc == false)
                {
                    break;
                }

                numsIndx++;
                token = (UINT8 *)strtok(NULL, ":");
            }
            else
            { 
                break;
            }
        } /* while */
    }

    if ((rc == true) && (numsIndx != IPV6ADDR_SUFF_TOKEN_NUMS))
    {
        ERR_PR("Too few IP address suffix/prefix tokens\n", __FUNCTION__);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : normalizeMacAddressValueString
 *              
 * Description: The function verifies that the value string is parsable
 *              Copies in dictionary value if necessary
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

static void normalizeMacAddressValueString(RtSubFldEntry_S *pRtSubFldEntry)
{
    int         indx;
    UINT8       *locstr = pRtSubFldEntry->value;
    int         length  = strlen((char *)locstr);
    DdEntry     *pDdEntry;

    if  (gobble(locstr) != 0)
    {
        for (indx = 1; indx < length; indx++)
        {
            if (locstr[indx] == '-')
            {
                return;
            }
        }

        // So we probaly need to do a dictionary look
        if ((pDdEntry = findMatchingEntry(locstr)) != 0)
        {
            strcpy((char *)pRtSubFldEntry->value, (char *)pDdEntry->value);
        }
        else
        {
            ERR_PR("findMatchingEntry failed for %s\n", locstr);
        }
    }
}



/******************************************************************************
 *
 * Function   : convertCharToHexValue
 *              
 * Description: The function converts a char to a hex value
 *              
 * Parameters :  
 *
 * Returns    : int   -1 invalid value, else number
 *              
 ******************************************************************************/

static int convertCharToHexValue(UINT8 ch)
{
    if (isdigit(ch) != 0)
    {
        return ch - '0';
    }
    else if (isxdigit(ch) != 0)
    {
        return tolower(ch) - 'a' + 10;
    }
    return -1;
}



/******************************************************************************
 *
 * Function   : parsMacAddressSubFldValue
 *              
 * Description: The function parses Mac Address subfield value
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool parseMacAddressSubFldValue(SubField_S *pSubField, RtSubFldEntry_S *pRtSubFldEntry)
{
    int         indx;
    UINT8       *locstr;
    int         length;
    int         ch;
    bool        rc = true;
    int         charsParsed = 0;       

    normalizeMacAddressValueString(pRtSubFldEntry);

    locstr = pRtSubFldEntry->value;

    if  (gobble(locstr) != 0)
    {
        length = strlen((char *)pRtSubFldEntry->value);

        DEBUG_PR(DEB_OTHER, "Parsing MacAddress %s. length = %i\n", pRtSubFldEntry->value, length);

        for (indx = 0; indx < length; indx++)
        {
            //printf("%s: value[%i] = 0x%02x\n", indx, pRtSubFldEntry->value[indx]);
            if (indx % 3 == 0)
            {
                if ((ch = convertCharToHexValue(pRtSubFldEntry->value[indx])) >= 0)
                {
                    pRtSubFldEntry->parsedMacAddress[indx/3]     = ch << 4;
                    pRtSubFldEntry->parsedMacAddressMask[indx/3] = 0xF0;
                    charsParsed++;
                }
                else if (pRtSubFldEntry->value[indx] == 'x')
                {
                    pRtSubFldEntry->parsedMacAddress[indx/3]     = 0;
                    pRtSubFldEntry->parsedMacAddressMask[indx/3] = 0;
                    charsParsed++;
                }
                else
                {
                    rc = false;
                    break;
                }
            }
            else if (indx % 3 == 1)
            {
                if ((ch = convertCharToHexValue(pRtSubFldEntry->value[indx])) >= 0)
                {
                    pRtSubFldEntry->parsedMacAddress[indx/3]     |= ch;
                    pRtSubFldEntry->parsedMacAddressMask[indx/3] |= 0xF;
                    charsParsed++;
		     DEBUG_PR(DEB_OTHER, "MacAddress[%i] = 0x%x. MacAddressMask[%i] = 0x%x\n", 
                           indx/3, pRtSubFldEntry->parsedMacAddress[indx/3], indx/3, 
			    pRtSubFldEntry->parsedMacAddressMask[indx/3]);
                }
                else if (pRtSubFldEntry->value[indx] == 'x')
                {
                    // Do nothing - i.e. leave fields as zero
		     DEBUG_PR(DEB_OTHER, "DC MacAddress[%i] = 0x%x. MacAddressMask[%i] = 0x%x\n", 
			indx/3, pRtSubFldEntry->parsedMacAddress[indx/3], indx/3,
			pRtSubFldEntry->parsedMacAddressMask[indx/3]);
                    charsParsed++;
                }
                else
                {
                    rc = false;
                    break;
                }
            }
            else  // (indx % 3 == 0)
            {
                if (pRtSubFldEntry->value[indx] != '-')
                {
                    rc = false;
                    break;
                }
                else
                {
                    charsParsed++;
                }
            }
            //printf("%s: indx = %i: val 0x%02x, mask 0x%02x\n", indx, pRtSubFldEntry->parsedMacAddress[indx/3], pRtSubFldEntry->parsedMacAddressMask[indx/3]);
        }
    }

    if (rc == false)
    {
        ERR_PR("Failed to parse %s\n", pRtSubFldEntry->value);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : normalizeIntegerValueString
 *              
 * Description: The function verifies that the value string is parsable
 *              Copies in dictionary value if necessary
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

static void normalizeIntegerValueString(RtSubFldEntry_S *pRtSubFldEntry)
{
    int         indx;
    UINT8       *locstr = pRtSubFldEntry->value;
    int         length  = strlen((char *)locstr);
    DdEntry     *pDdEntry;

    if  (gobble(locstr) != 0)
    {
        if (isdigit(locstr[0]) != 0)
        {
            return;
        }
        else if (locstr[0] == 'b')
        {
            int nonbitChars = 0;

            for (indx = 1; indx < length; indx++)
            {
                if (!(locstr[indx] == '0' || locstr[indx] == '1' || locstr[indx] == 'x'))
                {
                    nonbitChars++;
                }
            }

            if (nonbitChars == 0)
            {
                return;
            }
        }
        else  if (locstr[0] == 'x')
        {
            int alnumChars = 0;

            for (indx = 1; indx < length; indx++)
            {
                if (isalnum(locstr[indx]) != 0)
                {
                    alnumChars++;
                    break;
                }
            }
            if (alnumChars == 0)
            {
                return;
            }
        }

        // So we probaly need to do a dictionary look
        if ((pDdEntry = findMatchingEntry(locstr)) != 0)
        {
            strcpy((char *)pRtSubFldEntry->value, (char *)pDdEntry->value);
        }
        else
        {
            ERR_PR("findMatchingEntry failed for %s\n", locstr);
        }
    }
}



/******************************************************************************
 *
 * Function   : parseIntegerSubFldValue
 *              
 * Description: The function parses subfield value - either decimal, hexa or
 *              bit descriptor
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool parseIntegerSubFldValue(SubField_S *pSubField, RtSubFldEntry_S *pRtSubFldEntry)
{
    int         indx;
    UINT8       *locstr = pRtSubFldEntry->value;
    int         length;
    bool        rc = false;
        
    if  (gobble(locstr) != 0)
    {
        normalizeIntegerValueString(pRtSubFldEntry);

        locstr = pRtSubFldEntry->value;
        length  = strlen((char *)locstr);

        if (isBitString(locstr) == true)
        {
            // Reverse the error logic
            rc = true;
            for (indx = 1; indx < length; indx++)
            {
                if (locstr[indx] == '1')
                {
                    pRtSubFldEntry->parsedIntValue     = pRtSubFldEntry->parsedIntValue << 1;
                    pRtSubFldEntry->parsedIntValue     |= 0x1;
                    pRtSubFldEntry->parsedIntValueMask = pRtSubFldEntry->parsedIntValueMask << 1;
                    pRtSubFldEntry->parsedIntValueMask |= 0x1;
                }
                else if (locstr[indx] == '0')
                {
                    pRtSubFldEntry->parsedIntValue     = pRtSubFldEntry->parsedIntValue << 1;
                    pRtSubFldEntry->parsedIntValueMask = pRtSubFldEntry->parsedIntValueMask << 1;
                    pRtSubFldEntry->parsedIntValueMask |= 0x1;
                }
                else if (locstr[indx] == 'x')
                {
                    pRtSubFldEntry->parsedIntValue     = pRtSubFldEntry->parsedIntValue << 1;
                    pRtSubFldEntry->parsedIntValueMask = pRtSubFldEntry->parsedIntValueMask << 1;
                }
                else
                {
                    ERR_PR("Unexpected char %c in bit descriptor %s\n", locstr[indx], locstr);
                    rc = false;
                    break;
                }
            }
        }
        else if (locstr[0] == 'x')
        {
            pRtSubFldEntry->parsedIntValue     = 0;
            pRtSubFldEntry->parsedIntValueMask = 0;
            rc = true;
        }
        else if (locstr[0] == '0' &&  (locstr[1] == 'x' || locstr[1] == 'X'))
        {
            locstr = &locstr[2];

            if (sscanf((char *)locstr, "%x", &pRtSubFldEntry->parsedIntValue) != 0)
            {
                pRtSubFldEntry->parsedIntValueMask = bitsMaskAra[pSubField->highbit - pSubField->lowbit];
                rc = true;
            }
            else
            {        
                ERR_PR("sscanf hex value failed for %s\n", locstr);
            }
        }
        else if (isdigit(locstr[0]) != 0)
        {
            if (sscanf((char *)locstr, "%i", &pRtSubFldEntry->parsedIntValue) != 0)
            {
                pRtSubFldEntry->parsedIntValueMask = bitsMaskAra[pSubField->highbit - pSubField->lowbit];
                rc = true;
            }
            else
            {        
                ERR_PR("sscanf decimal failed for %s\n", locstr);
            }
        }
        else
        {        
            ERR_PR("Unexpected subfield value (not numeric, not bit descriptor) %s\n", locstr);
        }
    }
    else
    {        
        ERR_PR("Unexpected EOS (gobble) --%s--\n", pRtSubFldEntry->value);
    }

    if (rc == true)
    {
        if (pRtSubFldEntry->parsedIntValue > pSubField->maxvalue)
        {
            ERR_PR("Subfield %s, parsed value %u  > MAX value %i\n", 
                   __FUNCTION__, pSubField->name, pRtSubFldEntry->parsedIntValue, pSubField->maxvalue);
            rc = false;
        }
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : insertOneIntegerSubFldIntoTcam
 *              
 * Description: The function inserts a single subfield into Tcam/Tcam Mask
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool insertOneIntegerSubFldIntoTcam(PncEntry_S *pPnCEntry, SubField_S *pSubField, RtSubFldEntry_S *pRtSubFldEntry, int fieldWidth)
{
    int         rbyteIndx;
    UINT32      tempVal        = pRtSubFldEntry->parsedIntValue;
    UINT32      tempValMask    = pRtSubFldEntry->parsedIntValueMask;
    int         rightbitIndex  = pSubField->lowbit;
    int         bitsAvailable  = pSubField->highbit - pSubField->lowbit + 1;
    int         bitsInByte;
    UINT8       byteValue;
    UINT8       byteValueMask;
    UINT8       usedBitsMaskAra[] = { 1, 3, 7, 0xF, 0x1F, 0x3F, 0x7F, 0xFF }; 

    while (bitsAvailable > 0)
    {
        DEBUG_PR(DEB_OTHER, "[1] SubFld = %s, bitsAvailable = %i, rightbitIndex = %i, tempVal = 0x%x, tempValMask = 0x%x\n", 
              pSubField->name, bitsAvailable, rightbitIndex, tempVal, tempValMask);

        // No of bits in byte
        bitsInByte = 8 - (rightbitIndex % 8);

        // Mask out unused bits
        byteValue     = tempVal     & usedBitsMaskAra[bitsInByte -1];
        byteValueMask = tempValMask & usedBitsMaskAra[bitsInByte -1];

        // Index in the byte array 
        rbyteIndx = fieldWidth - (rightbitIndex/8) - 1;

        // Move value into position in the byte and write into the array
        byteValue     = byteValue     << (8 - bitsInByte);
        byteValueMask = byteValueMask << (8 - bitsInByte);

        DEBUG_PR(DEB_OTHER, "[2] rbyteIndx = %i, byteValue = 0x%x, byteValueMask = 0x%x\n", 
                rbyteIndx, byteValue, byteValueMask);


        if ((pPnCEntry->currTcamPktIndx + rbyteIndx) <= PACKET_SIZE)
        {
            pPnCEntry->tcam.packet[pPnCEntry->currTcamPktIndx + rbyteIndx]     |= byteValue;
            pPnCEntry->tcamMask.packet[pPnCEntry->currTcamPktIndx + rbyteIndx] |= byteValueMask;
        }
        else
        {
            ERR_PR("Exceeded packet length currTcamPktIndx = %i, numBytesInValue = %i\n", 
                   __FUNCTION__, pPnCEntry->currTcamPktIndx, rbyteIndx);
            return false;
        }

        // Increment section    
        tempVal     = tempVal      >> bitsInByte;
        tempValMask = tempValMask  >> bitsInByte;

        rightbitIndex  += bitsInByte;
        bitsAvailable -= bitsInByte;
    }
    return true;
}


/******************************************************************************
 *
 * Function   : insertIntegerIntoBitfield
 *              
 * Description: The function inserts an integer into supplied bit field
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool insertIntegerIntoBitfield(UINT8 *byteAra, UINT32 value, int hibit, int lobit, int araSize)
{
    int         rbyteIndx;
    UINT32      tempVal        = value;
    int         rightbitIndex  = lobit;
    int         bitsAvailable  = hibit - lobit + 1;
    int         bitsInByte;
    UINT8       byteValue;
    UINT8       usedBitsMaskAra[] = { 1, 3, 7, 0xF, 0x1F, 0x3F, 0x7F, 0xFF }; 

    if ( (hibit < (araSize * 8)) && (hibit >= lobit) )
    {
        while (bitsAvailable > 0)
        {
            DEBUG_PR(DEB_OTHER, "[1] bitsAvailable = %i, rightbitIndex = %i, tempVal = 0x%x\n", 
            		bitsAvailable, rightbitIndex, tempVal);
    
            // No of bits in byte
            bitsInByte = 8 - (rightbitIndex % 8);
    
            // Mask out unused bits
            byteValue     = tempVal     & usedBitsMaskAra[bitsInByte -1];
    
            // Index in the byte array 
            rbyteIndx = araSize - (rightbitIndex/8) - 1;
    
            // Move value into position in the byte and write into the array
            byteValue     = byteValue     << (8 - bitsInByte);
    
            DEBUG_PR(DEB_OTHER, "[2] rbyteIndx = %i, byteValue = 0x%x\n", rbyteIndx, byteValue);
    
            byteAra[rbyteIndx] |= byteValue;
    
            // Increment section    
            tempVal     = tempVal      >> bitsInByte;
    
            rightbitIndex  += bitsInByte;
            bitsAvailable -= bitsInByte;
        }
        return true;
    }
    else
    {
        ERR_PR("Either hibit=%i < lobit=%i, or hibit=%i > araSize*8 (%i)\n", 
               __FUNCTION__, hibit, lobit, hibit, araSize*8);
        return false;
    }
}



/******************************************************************************
 *
 * Function   : dealWithOneBx
 *              
 * Description: The function deals with one Bx=y string
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithOneBx(UINT8 *bxstr, int *pbIndx, int *pbvalue)
{
    UINT8  *locbxstr = bxstr;
    int    bIndx;
    int    bvalue;
    bool   rc = false;

    if ((locbxstr = skipChar(locbxstr, QUOTE_CHAR)) != 0)
    {
        if ((locbxstr = gobble(locbxstr)) != 0)
        {
            if (locbxstr[0] == 'B')
            {
                if (sscanf((char *)&locbxstr[1], "%i", &bIndx) != 0)
                {
                    if (bIndx <= MAX_BXINDX)
                    {
                        if ((locbxstr = skipOverNumber(&locbxstr[1])) != 0)
                        {
                            if (locbxstr[0] == EQUAL_CHAR)
                            {
                                if ((locbxstr = gobble(&locbxstr[1])) != 0)
                                {
                                    if (sscanf((char *)locbxstr, "%i", &bvalue) != 0)
                                    {
                                        if (bvalue == 0 || bvalue == 1)
                                        {
                                            *pbIndx  = bIndx;
                                            *pbvalue = bvalue;

                                            rc = true;
                                        }
                                        else
                                        {
                                            ERR_PR("In Bx=y, y must be 0..1. Found %i\n", bvalue);
                                        }
                                    }
                                    else
                                    {
                                        ERR_PR("Failed to parse y in Bx=y, Source string was %s\n", locbxstr);
                                    }
                                }
                                else
                                {
                                    ERR_PR("Unexpected EOS for Bx=y in %s\n", bxstr);
                                }
                            }
                            else
                            {
                                ERR_PR("Expected '=', got '%c'\n", locbxstr[0]);
                            }
                        }
                        else
                        {
                            ERR_PR("Unexpected EOS for Bx=y in %s\n", bxstr);
                        }
                    }
                    else
                    {
                        ERR_PR("B%i not valid. B%i is the last valid Bx\n",bIndx, MAX_BXINDX);
                    }
                }
                else
                {
                    ERR_PR("integer sscanf failed for %s\n", &locbxstr[1]);
                }
            }
            else
            {
                ERR_PR("Expected B, got '%c'\n", locbxstr[0]);
            }
        }
    }
    return rc;
}



/******************************************************************************
 *
 * Function   : getNextLocalToken
 *              
 * Description: The function gets next token
 *              
 * Parameters :  
 *
 * Returns    : char * or 0
 *              
 ******************************************************************************/

UINT8 *getNextLocalToken(UINT8 *str, char separator)
{
    int   length;
    int   indx;
    bool  found = false;

    str = gobble(str);

    length = strlen((char *)str);

    for (indx = 0; indx < length; indx++)
    {
        if (str[indx] == 0)
        {
            return 0;
        }
        else if (str[indx] == separator)
        {
            found = true;
        }
        else if (found == true)
        {
            return &str[indx];
        }
    }
    return 0;
}



/******************************************************************************
 *
 * Function   : extractPacketFieldString
 *              
 * Description: The function extracts the target field string
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool extractFieldString(UINT8 *origStr, UINT8 *tgtStr, int tgtLength)
{
    UINT8 *str = origStr;
    bool  rc   = false;
    int   indx = 0;

    while (1)
    {
        if (isalnum(str[indx]) != 0)
        {
            tgtStr[indx] = str[indx];
            indx++;
        }
        else
        {
            tgtStr[indx] = 0;
            rc = true;
            break;
        }
        // Verify that we are in range
        if (indx >= tgtLength)
        {
            break;
        }
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : isStringInDirectValueFormat
 *              
 * Description: The function checks if the string is potentially in "direct" 
 *              non lookup format.
 *              Here we check that it includes an '='
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool isStringInDirectValueFormat(UINT8 *str)
{
    int   indx;
    int   length = strlen((char *)str);
    bool  rc = false;

    for (indx = 0; indx < length; indx++)
    {
        if (str[indx] == EQUAL_CHAR)
        {
            rc = true;
            break;
        }
    }
    return rc;
}



/******************************************************************************
 *
 * Function   : normalizeAddInfoString
 *              
 * Description: The function returns a normalized AddInfo string
 *              It deals with the case of concatenated symbols and strings
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool normalizeAddInfoString(UINT8 *str, UINT8 *tgtStr)
{
    DdEntry *pDdEntry;
    UINT8   *tok;
    bool    firstElmt = true;
    bool    rc = true;

    tok = (UINT8 *)strtok((char *)str, ",");

    while (tok != 0)
    {
        tok = gobble(tok);
        if (isStringInDirectValueFormat(tok) == false)
        {
            pDdEntry = findMatchingEntry(tok);
            if (pDdEntry != 0)
            {
                if (firstElmt == false)
                {
                    strcat((char *)tgtStr, ",");
                }
                strcat((char *)tgtStr, (char *)pDdEntry->value);
                firstElmt = false;
            }
            else
            {
                ERR_PR("findMatchingEntry failed for %s\n", tok);
                rc = false;
                break;
            }
        }
        else
        {
            if (firstElmt == false)
            {
                strcat((char *)tgtStr, ",");
            }
            strcat((char *)tgtStr, (char *)tok);
            firstElmt = false;
        }
        tok = (UINT8 *)strtok(NULL, ",");
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : isTokenEmpty
 *              
 * Description: The function checks if token is empty
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool isTokenEmpty(UINT8 *token)
{
    UINT8 *locstr = token;
    bool  rc = true;

    if ((locstr = gobble(locstr)) != 0)
    {
        if (strlen((char *)locstr) != 0)
        {
            rc = false;
        }
    }
    return rc;
}


unsigned int get_dig_number (const char  *arg)
{
    unsigned int    val = 0;

    if ((arg[1] == 'x') || (arg[1] == 'X'))
        sscanf(&arg[2], "%x", &val);
    else
        val = atoi(arg);

    return val;
}

int handle_sysfs_command(char *buf, bool info)
{
	char tmp_str[512];
        
	if (NULL == buf) {
		ERR_PR("received NULL pointer\n")
		return 1;
	}
	/* write to vbs */
	if (info == false) {
		memcpy(tmp_str, buf, strlen(buf));
		tmp_str[strlen(buf)-1] = '\0';
		fprintf(vbs_fd, "\n\tscreen.Send \"");
		fprintf(vbs_fd, "%s \" & Newline", tmp_str);
		fprintf(vbs_fd, "\n\tscreen.WaitForString \"#\",1");
		fprintf(out_fd, buf);
	} else {
		memcpy(tmp_str, buf, strlen(buf));
		tmp_str[strlen(buf)-1] = '\0';
		fprintf(vbs_fd, "\n\' %s", tmp_str);
		fprintf(out_fd, "\n#");
		fprintf(out_fd, buf);
	}

	if (true == get_native_mode()) {
		if (info){
			/* informative data, can not execute it */
			if (0 == quiet)
				printf(buf);
		} else {
#ifdef __LINUX__
			struct timespec tmReq;
	 
			tmReq.tv_sec = (time_t)(get_execution_delay() / 1000);
			tmReq.tv_nsec = (get_execution_delay() % 1000) * 1000 * 1000;
	 
			/* we're not interested in remaining time nor in return value */
			(void)nanosleep(&tmReq, NULL);			

			/* run the command */
			system(buf);
#endif
		}
	}
       else
		if (0 == quiet)
	 		printf(buf);

	return 0;
}


/******************************************************************************
 *
 * Function   : common_mask_gen
 *
 * Description: generate mask according to bits number
 *
 * Parameters :
 * INPUT bit_num - bit num needed
 * OUTPUT None
 * Returns    : mask
 * Comments: for example, bit_num = 2, mask = 0x3(b00000011)
 ******************************************************************************/
unsigned int common_mask_gen(int bit_num)
{
	unsigned int temp = 0x1;
	int i;

	if (bit_num == 0)
		return 0;

	for (i = 1; i < bit_num; i++) {
		temp = temp << 1;
		temp |= 0x1;
	}

	return temp;
}

