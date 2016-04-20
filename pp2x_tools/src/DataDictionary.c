/******************************************************************************/
/**                                                                          **/
/**  COPYRIGHT (C) 2009, Marvell Semiconductors Inc.                         **/ 
/**--------------------------------------------------------------------------**/
/******************************************************************************/
/******************************************************************************/
/**                                                                          **/
/**  MODULE      : PnC Tool                                                  **/
/**                                                                          **/
/**  FILE        : DataDictionary.c                                          **/
/**                                                                          **/
/**  DESCRIPTION : This file contains params sheet DB treatment              **/
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

#include "ezxml.h"
#include "xml_params.h"
#include "DataDictionary.h"
#include "PncGlobals.h"


static DdEntry ddEntryAra[MAX_DICTENTRIES];

Dictionary paramsDictionary =
{
    ddEntryAra, sizeof(ddEntryAra)/sizeof(ddEntryAra[0])
};



/******************************************************************************
 *
 * Function   : printDictionary
 *              
 * Description: The function prints the Dictionary
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/

void printDictionary()
{
    DdEntry *pDdEntry;
    int     indx = 0;
    int     entries = 0;

	if (getDebugFlag() == false)
		return;

    printf("Dictionary\n----------\n");
    for (indx = 0; indx < paramsDictionary.dictSize; indx++)
    {
        pDdEntry = &paramsDictionary.pentryAra[indx];

        if (pDdEntry->inuse == true)
        {
            printf("%-32s: %s\n", pDdEntry->name, pDdEntry->value);
            entries++;
        }
    }

    if (entries == 0)
    {
        printf("*** Dictionary is empty\n");
    }
    printf("\nDictionary print End\n\n");
}



/******************************************************************************
 *
 * Function   : findMatchingEntry
 *              
 * Description: The function finds a Data Dictionary entry with matching name
 *              
 * Parameters :  
 *
 * Returns    : DdEntry * or 0
 *              
 ******************************************************************************/

DdEntry *findMatchingEntry(char *name)
{
    DdEntry *pDdEntry;
    int     indx = 0;

    for (indx = 0; indx < paramsDictionary.dictSize; indx++)
    {
        pDdEntry = &paramsDictionary.pentryAra[indx];

        if (pDdEntry->inuse == true)
        {
            if (strcmp((char *)pDdEntry->name, name) == 0)
            {
                return pDdEntry;
            }
        }
    }
    return 0;
}



/******************************************************************************
 *
 * Function   : findUnusedEntry
 *              
 * Description: The function finds an unused Data Dictionary entry
 *              
 * Parameters :  
 *
 * Returns    : DdEntry * or 0
 *              
 ******************************************************************************/

DdEntry *findUnusedEntry()
{
    DdEntry *pDdEntry;
    int     indx = 0;

    for (indx = 0; indx < paramsDictionary.dictSize; indx++)
    {
        pDdEntry = &paramsDictionary.pentryAra[indx];
        if (pDdEntry->inuse == false)
        {
            return pDdEntry;
        }
    }
    return NULL;
}


/******************************************************************************
 *
 * Function   : isLineAComment
 *              
 * Description: The function checks if line is comment - starts with "//"
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool isLineAComment(unsigned char *token)
{
    if (token[0] == FORWARDSLASH_CHAR && token[1] == FORWARDSLASH_CHAR)
    {
        return true;
    }
    return false;
}


#define SPACE         0x20


#if 0
/******************************************************************************
 *
 * Function   : getDictToken
 *              
 * Description: The function returns a token from the given string 
 *              
 * Parameters :  
 *
 * Returns    : void
 *              
 ******************************************************************************/
static void getDictToken(unsigned char *token, unsigned char **line)
{
    int           indx = 0;
    unsigned char *tempLine = *line;
    int           length;

    while (1)
    {
        if (tempLine[indx] == '\t' || tempLine[indx] == '\n' || tempLine[indx] == '\r')
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

    // Remove trailing spaces
    length = strlen(token);

    for (indx = length-1; indx >= 0; indx--)
    {
        if (token[indx] == SPACE)
        {
            token[indx] = 0;
        }
        else
        {
            break;
        }
    }
}
#endif


/******************************************************************************
 *
 * Function   : insertEntry
 *              
 * Description: The function inserts an entry into the Data Dictionary
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool insertEntry(char *name, char *value)
{
	DdEntry        *pDdEntry;
	bool           rc = false;

	if (strlen((char *)name) != 0)
	{
		if ((pDdEntry = findUnusedEntry()) != 0)
		{
			if (strlen(name)  < sizeof(pDdEntry->name))
			{
				pDdEntry->inuse = true;

           			// Copy in name and value
				strcpy((char *)pDdEntry->name, (char *)name);
				strcpy((char *)pDdEntry->value, (char *)value);
				rc = true;
	 		}
			else
			{
				printf("%s: ***ERROR One of name/value strings is too large. Recvd length %i, Max length %i\n", 
					__FUNCTION__, strlen(name), sizeof(pDdEntry->name)-1);
			}
		}
	 	else
		{
			printf("%s: ***ERROR findUnusedEntry() failed. Max dictionary entries = %i\n", __FUNCTION__, MAX_DICTENTRIES);
	        }
	}
	else
       {
		// Comment is OK
            	rc = true;
	}

	return rc;
}


#define SPACE_CHAR          ' '
#define EOL_CHAR            '\n'

#if 0
/******************************************************************************
 *
 * Function   : isLineEmpty
 *              
 * Description: The function checks if line is empty i.e maybe spaces and \n
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

static bool isLineEmpty(char *line)
{
    char *locstr = 0;
    int  indx;
    int  length = strlen(line);
    bool rc = true;

    
    for (indx = 0; indx < length; indx++)
    {
        if (isgraph(line[indx]) != 0)
        {
            locstr = &line[indx];
            break;
        }
    }

    if (locstr != 0)
    {
        rc = false;
    }
    return rc;
}
#endif


/******************************************************************************
 *
 * Function   : prepareDataDictionary
 *              
 * Description: The function prepares the Data Dictionary from the given file
 *              
 * Parameters :  
 *
 * Returns    : bool
 *              
 ******************************************************************************/

bool prepareDataDictionary(char *filename)
{
	ezxml_t     	xmlHead = NULL;
	ezxml_t     	xmlDictionary;
	ezxml_t     	xmlElement;
	char  	       *xmlName;
	char	       *xmlValue;

	xmlHead = ezxml_parse_file(filename);
	if (xmlHead == NULL){
		printf("%s: Failed to find %s\n", __FUNCTION__, filename);
        	return false;
	}

	xmlDictionary = ezxml_get(xmlHead, WORKSHEET_DICTIONARY, -1);
	if (xmlDictionary == NULL){
		printf("%s: Failed to get %s\n", __FUNCTION__, WORKSHEET_DICTIONARY);
		ezxml_free(xmlHead);
		return false;
    	}

	xmlElement = ezxml_child(xmlDictionary, TABLE_ENTRY);
	if (xmlElement == NULL){
		printf("%s: Failed to get %s\n", __FUNCTION__, TABLE_ENTRY);
		ezxml_free(xmlHead);
		return false;
    	}
	
	for (; xmlElement; xmlElement = ezxml_next(xmlElement)) {
		xmlName = ezxml_attr(xmlElement, DICT_NAME);
		if (xmlName == NULL) {
			printf("%s: Failed to get %s\n", __func__, DICT_NAME);
			ezxml_free(xmlHead);
			return false;
		}
	
		xmlValue = ezxml_attr(xmlElement, DICT_VALUE);
		if (xmlValue == NULL) {
			printf("%s: Failed to get %s\n", __func__, DICT_VALUE);
			ezxml_free(xmlHead);
			return false;
		}

		if (insertEntry(xmlName, xmlValue) == false) {
			printf("%s: ***ERROR: failed to insert the entry %s %s\n", __FUNCTION__, xmlName, xmlValue);
			ezxml_free(xmlHead);
	 		return false;
		}
	}

	ezxml_free(xmlHead);

	return true;
}


