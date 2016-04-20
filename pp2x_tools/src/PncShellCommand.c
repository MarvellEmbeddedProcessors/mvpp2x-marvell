/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : PncShellCommand.c                                         **/
/**                                                                          **/
/**  DESCRIPTION : This file generates shell commands from PnC database      **/
/**                                                                          **/
/******************************************************************************
 **                                                                          
 *   MODIFICATION HISTORY:                                                   
 *                
 *    20-Jul-09   zeev  - initial version created.                              
 *                                                                      
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>

#ifdef __LINUX__
#include <time.h>
#endif

#include "PncGlobals.h"
#include "PncDb.h"
#include "PncShellCommand.h"

#if 0

static char *cdCmd              = "cd /sys/devices/platform/neta/pnc/";

#ifdef __WINDOWS__
static char *chdirfolder        = ".";
#endif

#ifdef __LINUX__
static char *chdirfolder        = "/sys/devices/platform/neta/pnc/";
#endif

static char *swClearCmd         = "echo 1 > sw_clear";
static char *hwWriteCmd         = "echo 0x%x > hw_write";
static char *hwInvalidateAllCmd = "echo 1 > hw_inv_all";
static char *cmdTerminator      = ";";

static UINT8 bitAra[] = { 1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80 };

/******************************************************************************
 *
 * Function   : delayExecution
 *              
 * Description: This function delays the task - uses nanosleep()
 *
 * Parameters : 
 *              
 * Returns    : E_ErrorCodes
 *              
 ******************************************************************************/

static void delayExecution(unsigned int milliseconds)
{
#ifdef __LINUX__
    struct timespec tmReq;

    tmReq.tv_sec = (time_t)(milliseconds / 1000);
    tmReq.tv_nsec = (milliseconds % 1000) * 1000 * 1000;

    // we're not interested in remaining time nor in return value
    (void)nanosleep(&tmReq, NULL);
#endif
}


/******************************************************************************
 *
 * Function   : issuePncCommand
 *              
 * Description: The function issues one command line for exection
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool issuePncCommand(char *line)
{
    bool rc = true;

    if (getPncShellCmdFlag() == true)
    {
        printf("%s\n", line);
    }
#ifdef __LINUX__
    system(line);
#endif

    return rc;
}


/******************************************************************************
 *
 * Function   : generateLu
 *              
 * Description: The function prepares and generates shell command(s) for TCAM LU
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateLu(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char tmpbuf[500];
    bool rc = true;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    // "echo 0x%x > t_lookup
    sprintf(tmpbuf, cmdFormat, pPncEntry->tcam.lu);
    strcat(buf, tmpbuf);

    rc = issuePncCommand(buf);

    return rc;
}



/******************************************************************************
 *
 * Function   : generatePortId
 *              
 * Description: The function prepares and generates shell command(s) for TCAM port
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generatePortId(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char   tmpbuf[500];
    bool   rc = true;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    // "echo 0 0x%x > t_port"
    sprintf(tmpbuf, cmdFormat, pPncEntry->tcamMask.portId);
    strcat(buf, tmpbuf);

    rc = issuePncCommand(buf);

    return rc;
}



/******************************************************************************
 *
 * Function   : generateTcamAddInfo
 *              
 * Description: The function prepares and generates shell command(s) for TCAM AddInfo
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateTcamAddInfo(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char  tmpbuf[500];
    bool  rc = true;
    int   indx;
    bool  found = false;

    // "echo %i > t_ainfo_%i"   Note '<= MAX_BXINDX' since MAX_BXINDX is the maximum index 
    for (indx = 0; indx <= MAX_BXINDX; indx++)
    {
        if ((pPncEntry->tcamMask.addInfo & bitAra[indx]) != 0)
        {
            found = true;
            sprintf(tmpbuf, cmdFormat, indx, ((pPncEntry->tcam.addInfo & bitAra[indx]) == 0) ? 0: 1);
            // If string is started add a command separator
            if (buf[0] != 0)
            {
                strcat(buf, cmdTerminator);
            }

            strcat(buf, tmpbuf);
        }
    }

    if (found == true)
    {
        rc = issuePncCommand(buf);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : generatePacketAndMaskByte
 *              
 * Description: The function prepares and generates shell command(s) for single TCAM packet
 *              value and maskValue
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generatePacketAndMaskByte(char *buf, char *cmdFormat, PncEntry_S *pPncEntry, int indx)
{
    char tmpbuf[500];
    bool rc = true;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    // "echo %i 0x%x > t_offset_byte%s %i 0x%x > t_offset_mask"
    sprintf(tmpbuf, cmdFormat, indx, pPncEntry->tcam.packet[indx], cmdTerminator, indx, pPncEntry->tcamMask.packet[indx]);
    strcat(buf, tmpbuf);

    rc = issuePncCommand(buf);

    return rc;
}



/******************************************************************************
 *
 * Function   : generatePacket
 *              
 * Description: The function prepares and generates shell command(s) for TCAM packet
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generatePacket(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char  bufcopy[500];
    bool  rc = true;
    int   indx;

    // "echo %i 0x%x > t_offset_byte%s %i 0x%x > t_offset_mask"
    for (indx = 0; indx < pPncEntry->currTcamPktIndx; indx++)
    {
        // Copy start of command 
        strcpy(bufcopy, buf);
        if ((rc = generatePacketAndMaskByte(bufcopy, cmdFormat, pPncEntry, indx)) == false)
        {
            break;
        }
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : generateNxtLuId
 *              
 * Description: The function prepares and generates shell command(s) for SRAM Next LU
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateNxtLuId(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char tmpbuf[500];
    bool rc = true;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    // "echo 0x%x > s_lookup"
    sprintf(tmpbuf, cmdFormat, pPncEntry->sram.nextLuId);
    strcat(buf, tmpbuf);

    rc = issuePncCommand(buf);

    return rc;
}



/******************************************************************************
 *
 * Function   : generateSramAddInfoUpdt
 *              
 * Description: The function prepares and generates shell command(s) for SRAM Addinfo Update
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateSramAddInfoUpdt(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char  tmpbuf[500];
    bool  rc = true;
    int   indx;
    bool  found = false;

    // "echo 0x%x > s_ainfo"
    for (indx = 0; indx <= MAX_BXINDX; indx++)
    {
        if ((pPncEntry->sram.addInfoUpdtMask & bitAra[indx]) == 0 && (pPncEntry->sram.addInfoUpdtValue & bitAra[indx]) != 0)
        {
            found = true;
            sprintf(tmpbuf, cmdFormat, indx);
            // If string is started add a command separator
            if (buf[0] != 0)
            {
                strcat(buf, cmdTerminator);
            }

            strcat(buf, tmpbuf);
        }
    }

    if (found == true)
    {
        rc = issuePncCommand(buf);
    }

    return rc;
}

/******************************************************************************
 *
 * Function   : generateSramAddInfoUpdtExp
 *              
 * Description: The function prepares and generates shell command(s) for SRAM Addinfo Update
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateSramAddInfoUpdtExp(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char  tmpbuf[500];
    bool  rc = true;

    if (getFPGAFormatFlag() == true) 
    {
        rc = generateSramAddInfoUpdt(buf, "echo 0x%x > s_ainfo", pPncEntry);
    }
    else
    {
        // If string is started add a command separator
        if (buf[0] != 0)
        {
            strcat(buf, cmdTerminator);
        }

        // "echo 0x%x 0x%x > s_ainfo"
        sprintf(tmpbuf, cmdFormat, pPncEntry->sram.addInfoUpdtValue, ~(pPncEntry->sram.addInfoUpdtMask) & DEF_ADDINFOUPDATE_MASK);
        strcat(buf, tmpbuf);

        rc = issuePncCommand(buf);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : 
 *              
 * Description: The function prepares and generates shell command(s) for SRAM LU done
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateLuDo(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    bool rc = true;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    // "echo 1 > s_lookup_done"
    if (pPncEntry->sram.luDone != 0)
    {
        strcat(buf, cmdFormat);
    
        rc = issuePncCommand(buf);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : generateNxtLUOffInd
 *              
 * Description: The function prepares and generates shell command(s) for SRAM 
 *              Nxt LU Off Ind
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateNxtLUOffInd(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char tmpbuf[500];
    bool rc = true;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    // 22-Sep-09 Skip command when queue = 0
    if (pPncEntry->sram.nextLuOffsetInd != 0)
    {
        // "echo 0x%x > s_next_lookup_shift"
        sprintf(tmpbuf, cmdFormat, pPncEntry->sram.nextLuOffsetInd);
        strcat(buf, tmpbuf);
    
        rc = issuePncCommand(buf);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : generateQue
 *              
 * Description: The function prepares and generates shell command(s) for SRAM Que
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateQue(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char tmpbuf[500];
    bool rc = true;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    // 22-Sep-09 Skip command when queue = 0
    if (pPncEntry->sram.que != 0)
    {
        // "echo 0x%x > s_rxq"
        sprintf(tmpbuf, cmdFormat, pPncEntry->sram.que);
        strcat(buf, tmpbuf);
    
        rc = issuePncCommand(buf);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : generateShiftUpdt
 *              
 * Description: The function prepares and generates shell command(s) for SRAM Shift Update
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateShiftUpdt(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char tmpbuf[500];
    bool rc = true;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    // 22-Sep-09 Skip command when queue = 0
    if (EXTRACT_VAL_SHIFTUPDT(pPncEntry->sram.shiftUpdate) != 0)
    {
        // "echo %i 0x%x > s_shift_update"
        sprintf(tmpbuf, cmdFormat, EXTRACT_INDX_SHIFTUPDT(pPncEntry->sram.shiftUpdate), 
                EXTRACT_VAL_SHIFTUPDT(pPncEntry->sram.shiftUpdate));
        strcat(buf, tmpbuf);
    
        rc = issuePncCommand(buf);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : generateOneByteRI
 *              
 * Description: The function prepares and generates shell command(s) for SRAM RI byte
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateOneByteRI(char *buf, char *cmdFormat, UINT8 value, UINT8 mask, int startBitIndx)
{
    char  tmpbuf[500];
    bool  rc = true;     // takes care of nothing found
    int   indx;
    bool  found = false;

    // "echo 0x%x > s_rinfo"
    for (indx = 0; indx < sizeof(bitAra)/sizeof(bitAra[0]); indx++)
    {
        // 22-Sep-09 Masked non-sero bits
        if ((mask & bitAra[indx]) == 0 && (value & bitAra[indx]) != 0)
        {
            found = true;
            sprintf(tmpbuf, cmdFormat, (startBitIndx + indx));
            // If string is started add a command separator
            if (buf[0] != 0)
            {
                strcat(buf, cmdTerminator);
            }

            strcat(buf, tmpbuf);
        }
    }

    if (found == true)
    {
        rc = issuePncCommand(buf);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : generateRI
 *              
 * Description: The function prepares and generates shell command(s) for SRAM RI
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateRI(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char  bufcopy[500];
    bool  rc = true;
    int   indx;

    for (indx = 0; indx < RI_SIZE; indx++)
    {
        // Copy start of command 
        strcpy(bufcopy, buf);
        if ((rc = generateOneByteRI(bufcopy, cmdFormat, 
                                    pPncEntry->sram.resultInfoAndMask[indx], 
                                    pPncEntry->sram.resultInfoAndMask[RI_SIZE + indx], 
                                    8*(RI_SIZE-1-indx))) == false)
        {
            break;
        }
    }

    return rc;
}

/******************************************************************************
 *
 * Function   : generateRIexp
 *              
 * Description: The function prepares and generates shell command(s) for SRAM RI
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateRIexp(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char  tmpbuf[500];
    bool  rc = true;
    int   indx;
    UINT16 pairsAra[] = { 0x3, 0xC, 0x30, 0xC0, 0x300, 0xC00 };
    UINT8 value;
    bool  found = false;

    for (indx = 0; indx < RI_EXTRA_BITS_PAIRS; indx++)
    {
        value = (pPncEntry->sram.u.sram_exp.gen2 & pairsAra[indx]) >> (2 * indx);

        if (value != 0) {
            found = true;

            if (buf[0] != 0)
                strcat(buf, cmdTerminator);

            // "echo 0x%x 0x%x > s_rinfo_extra" 
            sprintf(tmpbuf, cmdFormat, value, indx);
            strcat(buf, tmpbuf);
       }
    }

    if (found == true) 
        rc = issuePncCommand(buf);

    return rc;
}


/******************************************************************************
 *
 * Function   : generateGemTxp2
 *              
 * Description: The function prepares and generates shell command(s) for 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateGemTxp2(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char tmpbuf[500];
    bool rc = true;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    // "echo 0x%x > s_flowid_hi"
    if (pPncEntry->sram.u.sram_reg.flowGemMask == 0)
    {
        sprintf(tmpbuf, cmdFormat, MAKE_GEMTXP2(pPncEntry->sram.u.sram_reg.txp2, pPncEntry->sram.u.sram_reg.gem));
        strcat(buf, tmpbuf);

        rc = issuePncCommand(buf);
    }

    return rc;
}



/******************************************************************************
 *
 * Function   : generateMod
 *              
 * Description: The function prepares and generates shell command(s) for 
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateMod(char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char tmpbuf[500];
    bool rc = true;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    // "echo 0x%x > s_flowid_lo"
    if (pPncEntry->sram.u.sram_reg.flowModMask == 0)
    {
        sprintf(tmpbuf, cmdFormat, pPncEntry->sram.u.sram_reg.mod);
        strcat(buf, tmpbuf);
        rc = issuePncCommand(buf);
    }

    return rc;
}


/******************************************************************************
 *
 * Function   : generateModExp
 *              
 * Description: The function prepares and generates shell command(s)
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateModExp (char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char    tmpbuf[500];
    bool    rc = true;
    bool    found = false;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    if (pPncEntry->sram.u.sram_exp.flowModLMask == 0)
    {
        // "echo 0x%x 0x%x > s_flowid_nibble"
        sprintf(tmpbuf, cmdFormat, (pPncEntry->sram.u.sram_exp.mod_l & 0xF), FLOWID_EXP_MOD_NIBBLE_START);
        strcat(buf, tmpbuf);
        found = true;
    }

    if (pPncEntry->sram.u.sram_exp.flowModMMask == 0)
    {
        if (buf[0] != 0)
        {
            strcat(buf, cmdTerminator);
        }
       
        // "echo 0x%x 0x%x > s_flowid_nibble"
        sprintf(tmpbuf, cmdFormat, (pPncEntry->sram.u.sram_exp.mod_m & 0xF), FLOWID_EXP_MOD_NIBBLE_START + 1);
        strcat(buf, tmpbuf);
        found = true;
    }

    if (pPncEntry->sram.u.sram_exp.flowModHMask == 0)
    {
        if (buf[0] != 0)
        {
            strcat(buf, cmdTerminator);
        }

        // "echo 0x%x 0x%x > s_flowid_nibble"
        sprintf(tmpbuf, cmdFormat, (pPncEntry->sram.u.sram_exp.mod_h & 0x3), FLOWID_EXP_MOD_NIBBLE_START + 2);
        strcat(buf, tmpbuf);
        found = true;
    }

    if (found == true) 
        rc = issuePncCommand(buf);

    return rc;
}


/******************************************************************************
 *
 * Function   : generateGemExp
 *              
 * Description: The function prepares and generates shell command(s)
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateGemExp (char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char    tmpbuf[500];
    bool    rc = true;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    if (pPncEntry->sram.u.sram_exp.flowGemMask == 0)
    {
        // "echo 0x%x 0x%x > s_flowid_nibble"
        sprintf(tmpbuf, cmdFormat, (pPncEntry->sram.u.sram_exp.gem & 0xF), FLOWID_EXP_GEM_NIBBLE_START);
        strcat(buf, tmpbuf);

        strcat(buf, cmdTerminator);
       
        // "echo 0x%x 0x%x > s_flowid_nibble"
        sprintf(tmpbuf, cmdFormat, (pPncEntry->sram.u.sram_exp.gem & 0xF0) >> 4, FLOWID_EXP_GEM_NIBBLE_START + 1);
        strcat(buf, tmpbuf);

        strcat(buf, cmdTerminator);

        // "echo 0x%x 0x%x > s_flowid_nibble" 
        sprintf(tmpbuf, cmdFormat, (pPncEntry->sram.u.sram_exp.gem & 0xF00) >> 8, FLOWID_EXP_GEM_NIBBLE_START + 2);
        strcat(buf, tmpbuf);

        rc = issuePncCommand(buf);
    }

    return rc;
}

/******************************************************************************
 *
 * Function   : generateTxp2Exp
 *              
 * Description: The function prepares and generates shell command(s)
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateTxp2Exp (char *buf, char *cmdFormat, PncEntry_S *pPncEntry)
{
    char    tmpbuf[500];
    bool    rc = true;

    // If string is started add a command separator
    if (buf[0] != 0)
    {
        strcat(buf, cmdTerminator);
    }

    if (pPncEntry->sram.u.sram_exp.flowTxp2Mask == 0)
    {
        // "echo 0x%x 0x%x > s_flowid_nibble"
        sprintf(tmpbuf, cmdFormat, (pPncEntry->sram.u.sram_exp.txp2 & 0xF), FLOWID_EXP_TXP2_NIBBLE_START);
        strcat(buf, tmpbuf);
        rc = issuePncCommand(buf);
    }

    return rc;
}



ShellCommand_S shellCommandAra[] =
{
    {"LU",                 generateLu,                "echo 0x%x > t_lookup"            },
    {"PortId",             generatePortId,            "echo 0 0x%x > t_port"            },
    {"AddInfo",            generateTcamAddInfo,       "echo 0x%x > t_ainfo_%i"          },
    {"Packet",             generatePacket,            "echo 0x%x 0x%02x > t_offset_byte%secho 0x%x 0x%02x > t_offset_mask" },
    {"NxtLuId",            generateNxtLuId,           "echo 0x%x > s_lookup"            },
    {"AddInfo Updt",       generateSramAddInfoUpdt,   "echo 0x%x > s_ainfo"             },
    {"LU Done",            generateLuDo,              "echo 1 > s_lookup_done"          },
    {"Nxt LU Off Ind",     generateNxtLUOffInd,       "echo 0x%x > s_next_lookup_shift" },
    {"Que",                generateQue,               "echo 0x%x > s_rxq"               },
    {"Shift update",       generateShiftUpdt,         "echo 0x%x 0x%x > s_shift_update" },
    {"RI",                 generateRI,                "echo 0x%x > s_rinfo"             },
    {"GemTxp2",            generateGemTxp2,           "echo 0x%x > s_flowid_hi"         },
    {"Mod",                generateMod,               "echo 0x%x > s_flowid_lo"         },
//    {"Key Type",           generateKeyType},
};


ShellCommand_S shellCommandAraExp[] =
{
    {"LU",                 generateLu,                "echo 0x%x > t_lookup"            },
    {"PortId",             generatePortId,            "echo 0 0x%x > t_port"            },
    {"AddInfo",            generateTcamAddInfo,       "echo 0x%x > t_ainfo_%i"          },
    {"Packet",             generatePacket,            "echo 0x%x 0x%02x > t_offset_byte%secho 0x%x 0x%02x > t_offset_mask" },
    {"NxtLuId",            generateNxtLuId,           "echo 0x%x > s_lookup"            },
    {"AddInfo Updt",       generateSramAddInfoUpdtExp,"echo 0x%x 0x%x > s_ainfo"        },
    {"LU Done",            generateLuDo,              "echo 1 > s_lookup_done"          },
    {"Nxt LU Off Ind",     generateNxtLUOffInd,       "echo 0x%x > s_next_lookup_shift" },
    {"Que",                generateQue,               "echo 0x%x > s_rxq"               },
    {"Shift update",       generateShiftUpdt,         "echo 0x%x 0x%x > s_shift_update" },
    {"RI",                 generateRI,                "echo 0x%x > s_rinfo"             },
    {"RIExp",              generateRIexp,             "echo 0x%x 0x%x > s_rinfo_extra"  },
    {"ModExp",             generateModExp,            "echo 0x%x 0x%x > s_flowid_part"},
    {"GemExp",             generateGemExp,            "echo 0x%x 0x%x > s_flowid_part"},
    {"Txp2Exp",            generateTxp2Exp,           "echo 0x%x 0x%x > s_flowid_part"},
};



/******************************************************************************
 *
 * Function   : generateSingleRowShellCommands
 *              
 * Description: The function generates shell commands for single PnC row
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool generateSingleRowShellCommands(PncEntry_S *pPnCEntry)
{
    // Each shell command must be prefaced with "cd /sys/class/misc/pnc/tcam/" or
    // alternately must run PnCtool from this directory, but then the params and pnc
    // files must be in some other location
    // 1. Get a clean row - "sw_clear"
    // 2. For each piece of info that can be written
    //    "cd /sys/class/misc/pnc/tcam/; <command-1>; <command-2>;..<command-n>"
    //    where <command-x> are commands to load data into row.
    //    Could also add a sw_dump command to see the kernel's version of the content 
    // 3. Write row to hardware - "hw_write <row index>"

    ShellCommand_S *pShellCommand;
    char           buf[500];
    int            indx;
    int            sizeAra;
    bool           rc = true;

    if (issuePncCommand(swClearCmd) == true)
    {
        if (getExpandedSramFormatFlag() == true){
            sizeAra = sizeof(shellCommandAraExp)/sizeof(shellCommandAraExp[0]);
            pShellCommand = shellCommandAraExp;
        }
        else{
            sizeAra = sizeof(shellCommandAra)/sizeof(shellCommandAra[0]);
            pShellCommand = shellCommandAra;
        }

        for (indx = 0; indx < sizeAra; indx++)
        {
            // Clear the buffer that accumulates PnC commands
            //strcpy(buf, cdcmd);
            buf[0] = 0;

            if (pShellCommand->fieldHandler != 0 && pShellCommand->cmdFormat != 0)
            {
                if ((pShellCommand->fieldHandler)(buf, pShellCommand->cmdFormat, pPnCEntry) == false)
                {
                    printf("%s: ***ERROR command generation failed for line with row index %i, %s field\n", 
                           __FUNCTION__, pPnCEntry->ind, pShellCommand->field);
                    rc = false;
                    break;
                }
            }

            pShellCommand++;
        }

        if (rc == true)
        {
            sprintf(buf, hwWriteCmd, pPnCEntry->ind);
            rc = issuePncCommand(buf);
        }
    }
    else
    {
        printf("%s: ***ERROR issuePncCommand() failed for line with row index %i, command: %s\n", 
               __FUNCTION__, pPnCEntry->ind, swClearCmd);
        rc = false;
    }

    return rc;
}

/******************************************************************************
 *
 * Function   : generatePncShellCommands
 *              
 * Description: The function generates shell commands for configuring the PnC rows
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

void generatePncShellCommands()
{
    PncEntry_S *pPnCEntry = findFirstUsedPncEntry();

    if (getPncShellCmdFlag() == true)
    {
        printf("***NOTE pncExecutionDelay is %d\n", get_execution_delay());
    }

    if (chdir(chdirfolder) == 0)
    {
        if (issuePncCommand(cdCmd) == true)
        {
            if (getNoInvalidateAllFlag() == false)
            {
                if (issuePncCommand(hwInvalidateAllCmd) == false)
                {
                    printf("%s: ***ERROR hwInvalidateAllCmd failed ('%s')\n", __FUNCTION__, hwInvalidateAllCmd);
                    return;
                }
            }

            while (pPnCEntry != 0)
            {
                if (getPncShellCmdFlag() == true)
                {
                    printf("%s: Row Ind %i\n", __FUNCTION__, pPnCEntry->ind);
                }
        
                if (generateSingleRowShellCommands(pPnCEntry) == false)
                {
                    printf("%s: ***ERROR system() failed for row %i\n", __FUNCTION__, pPnCEntry->ind);
                    if (getKeepGoingFlag() == false)
                    {
                        break;
                    }
                }

                delayExecution(get_execution_delay());
                pPnCEntry = findNextUsedPncEntry(pPnCEntry);
            }
        }
        else
        {
            printf("%s: ***ERROR cdCmd failed ('%s')\n", __FUNCTION__, cdCmd);
        }
    }
    else
    {
        printf("%s: ***ERROR chdir() failed for directory '%s'\n", __FUNCTION__, chdirfolder);
    }
}
#endif
