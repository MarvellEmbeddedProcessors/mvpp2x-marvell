/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : SramParse.c                                               **/
/**                                                                          **/
/**  DESCRIPTION : This file contains SRAM cell parse                        **/
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
#include "SramParse.h"


static UINT32 bitmaskAra[] =
{
    1, 3, 7, 0xF, 0x1F, 0x3F, 0x7F, 0xFF, 0x1FF, 0x3FF, 0x7FF, 0xFFF, 0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF
};


// SRAM RI START
GenBitField_S riGenBitFieldAra[] =
{
    [RIENUM_DIS]  = {"Dis",      RIENUM_DIS,    0,    0},
    [RIENUM_L4]   = {"L4",       RIENUM_L4,     2,    1},
    [RIENUM_L3]   = {"L3",       RIENUM_L3,     4,    3},
    [RIENUM_FF]   = {"Ff",       RIENUM_FF,     5,    5},
    [RIENUM_FM]   = {"Fm",       RIENUM_FM,     8,    6},
    [RIENUM_COL]  = {"Col",      RIENUM_COL,    9,    9},
    [RIENUM_TXP]  = {"Txp",      RIENUM_TXP,   13,    10},
    [RIENUM_MH]   = {"Mh",       RIENUM_MH,    16,    14},
    [RIENUM_GEN]  = {"Gen",      RIENUM_GEN,   21,    17},
    [RIENUM_PROF] = {"Prof",     RIENUM_PROF,  23,    22}
};

GenBitFieldDb_S riGenBitFieldDb =
{
    riGenBitFieldAra,
    sizeof(riGenBitFieldAra)/sizeof(riGenBitFieldAra[0])
};

GenBitField_S riGenBitFieldExpandedAra[] =
{
    [RIENUM_DIS]      = {"Dis",      RIENUM_DIS,        0,    0},
    [RIENUM_L4]       = {"L4",       RIENUM_L4,         2,    1},
    [RIENUM_L3]       = {"L3",       RIENUM_L3,         4,    3},
    [RIENUM_FF]       = {"Ff",       RIENUM_FF,         5,    5},
    [RIENUM_FM]       = {"Fm",       RIENUM_FM,         8,    6},
    [RIENUM_COL]      = {"Col",      RIENUM_COL,        9,    9},
    [RIENUM_TXP]      = {"Txp",      RIENUM_TXP,       13,    10},
    [RIENUM_MH]       = {"Mh",       RIENUM_MH,        17,    14},
    [RIENUM_GEN]      = {"Gen",      RIENUM_GEN,       21,    18},
    [RIENUM_PROF]     = {"Prof",     RIENUM_PROF,      23,    22},
    [RIENUM_GEN2]     = {"Gen2",     RIENUM_GEN2,      59,    48}
};

GenBitFieldDb_S riGenBitFieldExpandedDb =
{
    riGenBitFieldExpandedAra,
    sizeof(riGenBitFieldExpandedAra)/sizeof(riGenBitFieldExpandedAra[0])
};
// SRAM RI END



// SRAM FL START
GenBitField_S flGenBitFieldAra[] =
{
    [FLOWIDENUM_MOD]     = {"Mod",     FLOWIDENUM_MOD,       11,  0},
    [FLOWIDENUM_GEM]     = {"Gem",     FLOWIDENUM_GEM,       27,  16},
    [FLOWIDENUM_TXP2]    = {"Txp2",    FLOWIDENUM_TXP2,      31,  28},
    [FLOWIDENUM_GEMMASK] = {"Fm",      FLOWIDENUM_GEMMASK,   33,  33}
};

GenBitFieldDb_S flGenBitFieldDb =
{
    flGenBitFieldAra,
    sizeof(flGenBitFieldAra)/sizeof(flGenBitFieldAra[0])
};

GenBitField_S flGenBitFieldExpandedAra[] =
{
    [FLOWIDENUM_EXP_MOD_L]     = {"Mod_L",     FLOWIDENUM_EXP_MOD_L,        3,  0},
    [FLOWIDENUM_EXP_MOD_M]     = {"Mod_M",     FLOWIDENUM_EXP_MOD_M,        7,  4},
    [FLOWIDENUM_EXP_MOD_H]     = {"Mod_H",     FLOWIDENUM_EXP_MOD_H,        9,  8},
    [FLOWIDENUM_EXP_GEM]       = {"Gem",       FLOWIDENUM_EXP_GEM,         23,  12},
    [FLOWIDENUM_EXP_TXP2]      = {"Txp2",      FLOWIDENUM_EXP_TXP2,        27,  24},
    [FLOWIDENUM_EXP_MOD_L_MASK]= {"Lmask",     FLOWIDENUM_EXP_MOD_L_MASK,  32,  32},
    [FLOWIDENUM_EXP_MOD_M_MASK]= {"Mmask",     FLOWIDENUM_EXP_MOD_M_MASK,  33,  33},
    [FLOWIDENUM_EXP_MOD_H_MASK]= {"Hmask",     FLOWIDENUM_EXP_MOD_H_MASK,  34,  34},
    [FLOWIDENUM_EXP_GEMMASK]   = {"Gmask",     FLOWIDENUM_EXP_GEMMASK,     37,  35},
    [FLOWIDENUM_EXP_TXP2MASK]  = {"Tmask",     FLOWIDENUM_EXP_TXP2MASK,    38,  38}
};

GenBitFieldDb_S flGenBitFieldExpandedDb =
{
    flGenBitFieldExpandedAra,
    sizeof(flGenBitFieldExpandedAra)/sizeof(flGenBitFieldExpandedAra[0])
};
// SRAM FL END




/******************************************************************************
 *
 * Function   : findFlGenBitField
 *              
 * Description: The function the matching FlowId GenBitField_S via enumId
 *              
 * Parameters :  
 *
 * Returns    : GenBitField_S * or 0
 *              
 ******************************************************************************/

static GenBitField_S *findFlGenBitField(int enumId)
{
    if (enumId < flGenBitFieldDb.numEntries)
    {
        return &flGenBitFieldDb.pGenBitFieldAra[enumId];
    }

    return 0;
}


/******************************************************************************
 *
 * Function   : findFlGenBitFieldExp
 *              
 * Description: The function the matching FlowId GenBitField_S via enumId
 *              
 * Parameters :  
 *
 * Returns    : GenBitField_S * or 0
 *              
 ******************************************************************************/

static GenBitField_S *findFlGenBitFieldExp(int enumId)
{
    if (enumId < flGenBitFieldExpandedDb.numEntries)
    {
        return &flGenBitFieldExpandedDb.pGenBitFieldAra[enumId];
    }

    return 0;
}



/******************************************************************************
 *
 * Function   : findRiGenBitField
 *              
 * Description: The function finds the matching RI GenBitField_S via enumId
 *              
 * Parameters :  
 *
 * Returns    : GenBitField_S * or 0
 *              
 ******************************************************************************/

static GenBitField_S *findRiGenBitField(int enumId)
{
    if (enumId < riGenBitFieldDb.numEntries)
    {
        return &riGenBitFieldDb.pGenBitFieldAra[enumId];
    }

    return 0;
}

/******************************************************************************
 *
 * Function   : findRiGenBitFieldExp
 *              
 * Description: The function finds the matching RI GenBitField_S via enumId
 *              
 * Parameters :  
 *
 * Returns    : GenBitField_S * or 0
 *              
 ******************************************************************************/

static GenBitField_S *findRiGenBitFieldExp(int enumId)
{
    if (enumId < riGenBitFieldExpandedDb.numEntries)
    {
        return &riGenBitFieldExpandedDb.pGenBitFieldAra[enumId];
    }

    return 0;
}



/******************************************************************************
 *
 * Function   : insertRiBitField
 *              
 * Description: The function inserts bit field of RI byte array 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool insertRiBitField(UINT8 *token, PncEntry_S *pPnCEntry, UINT32 value, int enumId)
{
    UINT32 maskValue;
    bool   rc = false;

    GenBitField_S *pGenBitField;

    if ((pGenBitField = findRiGenBitField(enumId)) != 0)
    {
        if ((rc = insertIntegerIntoBitfield(&pPnCEntry->sram.resultInfoAndMask[0], value, 
                                            pGenBitField->highbit, pGenBitField->lowbit, RI_SIZE)) == true)
        {
            if (value == 0)
            {
                maskValue = bitmaskAra[pGenBitField->highbit - pGenBitField->lowbit];
                rc = insertIntegerIntoBitfield(&pPnCEntry->sram.resultInfoAndMask[RI_SIZE], maskValue, 
                                               pGenBitField->highbit, pGenBitField->lowbit, RIMASK_SIZE);
            }
        }
    }
    else
    {
        printf("%s: findRiGenBitField failed for %s, enumId = %i", __FUNCTION__, token, enumId);
    }
    return rc;
}

/******************************************************************************
 *
 * Function   : insertRiBitFieldExp
 *              
 * Description: The function inserts bit field of RI byte array 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool insertRiBitFieldExp(UINT8 *token, PncEntry_S *pPnCEntry, UINT32 value, int enumId)
{
    UINT32 maskValue;
    bool   rc = false;

    GenBitField_S *pGenBitField;

    if ((pGenBitField = findRiGenBitFieldExp(enumId)) != 0)
    {
        if ((rc = insertIntegerIntoBitfield(&pPnCEntry->sram.resultInfoAndMask[0], value, 
                                            pGenBitField->highbit, pGenBitField->lowbit, RI_SIZE)) == true)
        {
            if (value == 0)
            {
                maskValue = bitmaskAra[pGenBitField->highbit - pGenBitField->lowbit];
                rc = insertIntegerIntoBitfield(&pPnCEntry->sram.resultInfoAndMask[RI_SIZE], maskValue, 
                                               pGenBitField->highbit, pGenBitField->lowbit, RIMASK_SIZE);
            }
        }
    }
    else
    {
        printf("%s: findRiGenBitFieldExp failed for %s, enumId = %i", __FUNCTION__, token, enumId);
    }
    return rc;
}

/******************************************************************************
 *
 * Function   : insertRiBitFieldExtra
 *              
 * Description: The function inserts bit field of extra RI byte array 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool insertRiBitFieldExtra(UINT8 *token, PncEntry_S *pPnCEntry, UINT32 value, int enumId)
{
    UINT32          maskValue;
    bool            rc = false;
    GenBitField_S   *pGenBitField;
    int             hbit, lbit;

    if ((pGenBitField = findRiGenBitFieldExp(enumId)) != 0)
    {
        if ((rc = insertIntegerIntoBitfield(&pPnCEntry->sram.u.sram_exp.resultInfoExtraAndMask[0], 
                                            value, 
                                            pGenBitField->highbit - RI_BASE_BITS, 
                                            pGenBitField->lowbit - RI_BASE_BITS, 
                                            RI_EXP_SIZE)) == true)
        {
            if (value == 0)
            {
                lbit = 0;
                hbit = RI_EXTRA_BITS_PAIRS - 1;
                maskValue = bitmaskAra[hbit - lbit];
                rc = insertIntegerIntoBitfield(&pPnCEntry->sram.u.sram_exp.resultInfoExtraAndMask[RI_EXP_SIZE], 
                                               maskValue, hbit, lbit, RIMASK_EXP_SIZE);
            }
        }
    }
    else
    {
        printf("%s: findRiGenBitFieldExp failed for %s, enumId = %i", __FUNCTION__, token, enumId);
    }
    return rc;
}


/******************************************************************************
 *
 * Function   : insertFlBitField
 *              
 * Description: The function inserts bit field of FL byte array
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool insertFlBitField(UINT8 *token, PncEntry_S *pPnCEntry, UINT32 value, int enumId)
{
    bool   rc = false;

    GenBitField_S *pGenBitField;

    if ((pGenBitField = findFlGenBitField(enumId)) != 0)
    {
        rc = insertIntegerIntoBitfield(&pPnCEntry->sram.u.sram_reg.flowIdAndMask[0], value, 
                                       pGenBitField->highbit, pGenBitField->lowbit, FLOWID_SIZE);
    }
    else
    {
        printf("%s: findFlGenBitField failed for %s, enumId = %i", __FUNCTION__, token, enumId);
    }
    return rc;
}

/******************************************************************************
 *
 * Function   : insertFlBitFieldExp
 *              
 * Description: The function inserts bit field of FL byte array
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool insertFlBitFieldExp(UINT8 *token, PncEntry_S *pPnCEntry, UINT32 value, int enumId)
{
    bool   rc = false;

    GenBitField_S *pGenBitField;

    if ((pGenBitField = findFlGenBitFieldExp(enumId)) != 0)
    {
        rc = insertIntegerIntoBitfield(&pPnCEntry->sram.u.sram_exp.flowIdAndMask[0], value, 
                                       pGenBitField->highbit, pGenBitField->lowbit, FLOWID_SIZE);
    }
    else
    {
        printf("%s: findFlGenBitFieldExp failed for %s, enumId = %i", __FUNCTION__, token, enumId);
    }
    return rc;
}


/******************************************************************************
 *
 * Function   : dealWithLuDo
 *              
 * Description: The function deals with LU Done 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithLuDo(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.luDone, MAX_LUDONE, DEFAULT_LUDONE);

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithKeyType
 *              
 * Description: The function deals with Key Type 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithKeyType(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;
    DdEntry  *pDdEntry;
    int      defVal;

    if ((pDdEntry = findMatchingEntry((unsigned char *)"KEY_TYPE_DEF")) == 0)
    {
        printf("%s: ***ERROR KEY_TYPE_DEF not in dictionary\n", __FUNCTION__);
        return false;
    }
    else if (sscanf((char *)pDdEntry->value, "%i", &defVal) == 0)
    {
        printf("%s: ***ERROR failed to parse KEY_TYPE_DEF vale %s\n", __FUNCTION__, pDdEntry->value);
        return false;
    }

    rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.keyType, MAX_KEYTYPE, defVal);

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithAddInfoUpdt
 *              
 * Description: The function deals with SRAM AddInfoM (Bx=a,By=b)
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithAddInfoUpdt(UINT8 *str, PncEntry_S *pPnCEntry)
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
        pPnCEntry->sram.addInfoUpdtValue = 0;
        pPnCEntry->sram.addInfoUpdtMask  = DEF_ADDINFOUPDATE_MASK;
        pPnCEntry->sram.addInfoUpdtFull  = MAKE_ADDINFOUPDT(pPnCEntry->sram.addInfoUpdtValue,pPnCEntry->sram.addInfoUpdtMask);
        rc = true;
    }
    else
    {
        UINT8 newstr[MAX_PNCLINESIZE] = { 0 };
        int   indx = 0;
        UINT8 *bxstr;

        pPnCEntry->sram.addInfoUpdtMask = DEF_ADDINFOUPDATE_MASK;

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
                    pPnCEntry->sram.addInfoUpdtValue |= (bvalue << bIndx);
                    pPnCEntry->sram.addInfoUpdtMask  &= ~(1      << bIndx);
                    pPnCEntry->sram.addInfoUpdtFull  = MAKE_ADDINFOUPDT(pPnCEntry->sram.addInfoUpdtValue,pPnCEntry->sram.addInfoUpdtMask);
                }
                bxstr = getNextLocalToken(bxstr, COMMA);
                indx++;
            }
        }
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithNxtLuId
 *              
 * Description: The function deals with next LU Id 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithNxtLuId(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.nextLuId, MAX_LU, 0);

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithNxtLuOffInd
 *              
 * Description: The function deals with NxtLuOffInd 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithNxtLuOffInd(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.nextLuOffsetInd, MAX_NXTLUOFFIND, DEFAULT_NXTLUOFFIND);

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithQue
 *              
 * Description: The function deals with Que 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithQue(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.que, MAX_QUE, DEFAULT_QUE);

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithShiftUpdt
 *              
 * Description: The function deals with SRAM ShiftUpdt ([x]=y)
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithShiftUpdt(UINT8 *token, PncEntry_S *pPnCEntry)
{
    int      xval;
    int      yval;
    bool     rc = false;
    UINT8    *locstr = token;

    if (strlen((char *)locstr) == 0)
    {
        pPnCEntry->sram.shiftUpdate = MAKE_SHIFTUPDT(DEFAULT_SHIFTUPDT_X, DEFAULT_SHIFTUPDT_y);
        rc = true;
    }
    else
    {
        if ((locstr = gobble(locstr)) != 0)
        {
            if (isStringInDirectValueFormat(locstr) == false)
            {
                DdEntry *pDdEntry = findMatchingEntry(locstr);
        
                if (pDdEntry != 0)
                {
                    locstr = pDdEntry->value;
                }
                else
                {
                    printf("%s: ***ERROR findMatchingEntry failed for %s\n", __FUNCTION__, locstr);
                }
            }
        
            if (locstr[0] == LEFTSQUARE_CHAR)
            {
                if ((locstr = gobble(&locstr[1])) != 0)
                {
                    if (scanInt(locstr, &xval) == true)
                    {
                        if (xval <= MAX_SHIFTUPDT_X)
                        {
                            if ((locstr = skipOverNumber(locstr)) != 0)
                            {
                                if ((locstr = gobble(locstr)) != 0)
                                {
                                    if (locstr[0] == RIGHTSQUARE_CHAR)
                                    {
                                        if ((locstr = gobble(&locstr[1])) != 0)
                                        {
                                            if (locstr[0] == EQUAL_CHAR)
                                            {
                                                if ((locstr = gobble(&locstr[1])) != 0)
                                                {
                                                    if (scanInt(locstr, &yval) == true)
                                                    {
                                                        if (yval <= MAX_SHIFTUPDT_Y)
                                                        {
                                                            pPnCEntry->sram.shiftUpdate = MAKE_SHIFTUPDT(xval, yval);
                                                            rc = true;
                                                        }
                                                        else
                                                        {
                                                            printf("%s: ***ERROR yval %i > MAX_SHIFTUPDT_Y (%i)\n", __FUNCTION__, yval, MAX_SHIFTUPDT_Y); 
                                                        }
                                                    }
                                                    else
                                                    {
                                                        printf("%s: ***ERROR ShiftUpdt. yval not found in %s\n", __FUNCTION__, token); 
                                                    }
                                                }
                                            }
                                            else
                                            {
                                                printf("%s: ***ERROR '=' not found in %s\n", __FUNCTION__, token); 
                                            }
                                        }
                                    }
                                    else
                                    {
                                        printf("%s: ***ERROR ']' not found in %s\n", __FUNCTION__, token); 
                                    }
                                }
                            }
                            else
                            {
                                printf("%s: ***ERROR skipOverNumber failed\n", __FUNCTION__); 
                            }
                        }
                        else
                        {
                            printf("%s: ***ERROR xval %i > MAX_SHIFTUPDT_X (%i)\n", __FUNCTION__, xval, MAX_SHIFTUPDT_X); 
                        }
                    }
                    else
                    {
                        printf("%s: ***ERROR ShiftUpdt. xval not found in %s\n", __FUNCTION__, token); 
                    }
                }
            }
            else
            {
                printf("%s: ***ERROR '[' not found in %s\n", __FUNCTION__, token); 
            }
        }
    }
    return rc;
}




/******************************************************************************
 *
 * Function   : dealWithDis
 *              
 * Description: The function deals with SRAM Dis 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithDis(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.dis, MAX_DIS, DEFAULT_DIS)) == true)
    {
        rc = insertRiBitField(token, pPnCEntry, pPnCEntry->sram.dis, RIENUM_DIS);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithL4
 *              
 * Description: The function deals with SRAM L4
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithL4(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.l4, MAX_L4, DEFAULT_L4)) == true)
    {
        rc = insertRiBitField(token, pPnCEntry, pPnCEntry->sram.l4, RIENUM_L4);
    }

    return rc;
}




/******************************************************************************
 *
 * Function   : dealWithL3
 *              
 * Description: The function deals with SRAM L3
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithL3(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.l3, MAX_L3, DEFAULT_L3)) == true)
    {
        rc = insertRiBitField(token, pPnCEntry, pPnCEntry->sram.l3, RIENUM_L3);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithFf
 *              
 * Description: The function deals with SRAM Ff 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithFf(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.ff, MAX_FF, DEFAULT_FF)) == true)
    {
        rc = insertRiBitField(token, pPnCEntry, pPnCEntry->sram.ff, RIENUM_FF);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithFm
 *              
 * Description: The function deals with SRAM Fm
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithFm(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.fm, MAX_FM, DEFAULT_FM)) == true)
    {
        rc = insertRiBitField(token, pPnCEntry, pPnCEntry->sram.fm, RIENUM_FM);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithCol
 *              
 * Description: The function deals with SRAM Fm
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithCol(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.col, MAX_COL, DEFAULT_COL)) == true)
    {
        rc = insertRiBitField(token, pPnCEntry, pPnCEntry->sram.col, RIENUM_COL);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithTxp
 *              
 * Description: The function deals with SRAM Txp
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithTxp(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.txp, MAX_TXP, DEFAULT_TXP)) == true)
    {
        rc = insertRiBitField(token, pPnCEntry, pPnCEntry->sram.txp, RIENUM_TXP);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithMh
 *              
 * Description: The function deals with SRAM Mh
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithMh(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.mh, MAX_MH, DEFAULT_MH)) == true)
    {
        rc = insertRiBitField(token, pPnCEntry, pPnCEntry->sram.mh, RIENUM_MH);
    }

    return rc;
}

/******************************************************************************
 *
 * Function   : dealWithMhExp
 *              
 * Description: The function deals with SRAM Mh for phase A0
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithMhExp(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.mh, MAX_MH_EXP, DEFAULT_MH)) == true)
    {
        rc = insertRiBitFieldExp(token, pPnCEntry, pPnCEntry->sram.mh, RIENUM_MH);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithGen
 *              
 * Description: The function deals with SRAM Gen
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithGen(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.gen, MAX_GEN, DEFAULT_GEN)) == true)
    {
        rc = insertRiBitField(token, pPnCEntry, pPnCEntry->sram.gen, RIENUM_GEN);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : dealWithGenExp
 *              
 * Description: The function deals with SRAM Gen for phase A0
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithGenExp(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.gen, MAX_GEN_EXP, DEFAULT_GEN)) == true)
    {
        rc = insertRiBitFieldExp(token, pPnCEntry, pPnCEntry->sram.gen, RIENUM_GEN);
    }

    return rc;
}

/******************************************************************************
 *
 * Function   : dealWithGen2Exp
 *              
 * Description: The function deals with SRAM Gen2 for phase A0
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithGen2Exp(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = bitPairsLookupTokenHandler(token, &pPnCEntry->sram.u.sram_exp.gen2, MAX_GEN2_EXP, DEFAULT_GEN, MAX_GEN2_BITS)) == true)
    {
        rc = insertRiBitFieldExtra(token, pPnCEntry, pPnCEntry->sram.u.sram_exp.gen2, RIENUM_GEN2);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : dealWithProf
 *              
 * Description: The function deals with SRAM Prof
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithProf(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.prof, MAX_PROF, DEFAULT_PROF)) == true)
    {
        rc = insertRiBitField(token, pPnCEntry, pPnCEntry->sram.prof, RIENUM_PROF);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : dealWithProfExp
 *              
 * Description: The function deals with SRAM Prof
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithProfExp(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.prof, MAX_PROF, DEFAULT_PROF)) == true)
    {
        rc = insertRiBitFieldExp(token, pPnCEntry, pPnCEntry->sram.prof, RIENUM_PROF);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : dealWithSramMod
 *              
 * Description: The function deals with SRAM Mod
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithSramMod(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if (isTokenEmpty(token) == true)
    {
        pPnCEntry->sram.u.sram_reg.flowModMask = 1;
    }

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.u.sram_reg.mod, MAX_SRAM_MOD, DEFAULT_SRAM_MOD)) == true)
    {
        rc = insertFlBitField(token, pPnCEntry, pPnCEntry->sram.u.sram_reg.mod, FLOWIDENUM_MOD);
    }

    return rc;
}

/******************************************************************************
 *
 * Function   : dealWithSramModL
 *              
 * Description: The function deals with SRAM Mod
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithSramModL(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if (isTokenEmpty(token) == true)
    {
        pPnCEntry->sram.u.sram_exp.flowModLMask = 1;
    }

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.u.sram_exp.mod_l, MAX_SRAM_MOD_L, DEFAULT_SRAM_MOD)) == true)
    {
        rc = insertFlBitFieldExp(token, pPnCEntry, pPnCEntry->sram.u.sram_exp.mod_l, FLOWIDENUM_EXP_MOD_L);
    }

    return rc;
}

/******************************************************************************
 *
 * Function   : dealWithSramModM
 *              
 * Description: The function deals with SRAM Mod
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithSramModM(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if (isTokenEmpty(token) == true)
    {
        pPnCEntry->sram.u.sram_exp.flowModMMask = 1;
    }

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.u.sram_exp.mod_m, MAX_SRAM_MOD_M, DEFAULT_SRAM_MOD)) == true)
    {
        rc = insertFlBitFieldExp(token, pPnCEntry, pPnCEntry->sram.u.sram_exp.mod_m, FLOWIDENUM_EXP_MOD_M);
    }

    return rc;
}

/******************************************************************************
 *
 * Function   : dealWithSramModH
 *              
 * Description: The function deals with SRAM Mod
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithSramModH(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    pPnCEntry->sram.u.sram_exp.flowModHMask = 0;

    if (isTokenEmpty(token) == true)
    {
        pPnCEntry->sram.u.sram_exp.flowModHMask = 1;
    }

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.u.sram_exp.mod_h, MAX_SRAM_MOD_H, DEFAULT_SRAM_MOD)) == true)
    {
        rc = insertFlBitFieldExp(token, pPnCEntry, pPnCEntry->sram.u.sram_exp.mod_h, FLOWIDENUM_EXP_MOD_H);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : setFlowGemTxp2Mask
 *              
 * Description: The function deals with Gem/Txp2 flow mask logic
 *              If both Gem and Txp2 are empty, then mask is 1, else 0
 *              Note: the initial value of the mask is -1
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static void setFlowGemTxp2Mask(UINT8 *token, PncEntry_S *pPnCEntry)
{
    if (isTokenEmpty(token) == true)
    {
        if (pPnCEntry->sram.u.sram_reg.flowGemMask != 0)
        {
            pPnCEntry->sram.u.sram_reg.flowGemMask = 1;
        }
    }
    else
    {
        pPnCEntry->sram.u.sram_reg.flowGemMask = 0;
    }
}



/******************************************************************************
 *
 * Function   : dealWithSramGem
 *              
 * Description: The function deals with SRAM Gem
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithSramGem(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    setFlowGemTxp2Mask(token, pPnCEntry);

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.u.sram_reg.gem, MAX_SRAM_GEM, DEFAULT_SRAM_GEM)) == true)
    {
        rc = insertFlBitField(token, pPnCEntry, pPnCEntry->sram.u.sram_reg.gem, FLOWIDENUM_GEM);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : dealWithSramGemExp
 *              
 * Description: The function deals with SRAM Gem
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithSramGemExp(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool     rc;

    if (isTokenEmpty(token) == true)
    {
        pPnCEntry->sram.u.sram_exp.flowGemMask = 1;
    }

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.u.sram_exp.gem, MAX_SRAM_GEM, DEFAULT_SRAM_GEM)) == true)
    {
        rc = insertFlBitFieldExp(token, pPnCEntry, pPnCEntry->sram.u.sram_exp.gem, FLOWIDENUM_EXP_GEM);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : dealWithSramTxp2
 *              
 * Description: The function deals with SRAM Txp2
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithSramTxp2(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool rc;

    setFlowGemTxp2Mask(token, pPnCEntry);

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.u.sram_reg.txp2, MAX_SRAM_TXP2, DEFAULT_SRAM_TXP2)) == true)
    {
        rc = insertFlBitField(token, pPnCEntry, pPnCEntry->sram.u.sram_reg.txp2, FLOWIDENUM_TXP2);
    }

    return rc;
}

/******************************************************************************
 *
 * Function   : dealWithSramTxp2Exp
 *              
 * Description: The function deals with SRAM Txp2
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool dealWithSramTxp2Exp(UINT8 *token, PncEntry_S *pPnCEntry)
{
    bool rc;

    if (isTokenEmpty(token) == true)
    {
        pPnCEntry->sram.u.sram_exp.flowTxp2Mask = 1;
    }

    if ((rc = intorDdLookupTokenHandler(token, &pPnCEntry->sram.u.sram_exp.txp2, MAX_SRAM_TXP2, DEFAULT_SRAM_TXP2)) == true)
    {
        rc = insertFlBitFieldExp(token, pPnCEntry, pPnCEntry->sram.u.sram_exp.txp2, FLOWIDENUM_EXP_TXP2);
    }

    return rc;
}


