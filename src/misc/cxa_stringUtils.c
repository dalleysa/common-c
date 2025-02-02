/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
#include "cxa_stringUtils.h"


// ******** includes ********
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#include <cxa_assert.h>
#include <cxa_numberUtils.h>

#ifdef __XC
    // microchip
    #include <strtok.h>
#endif


// ******** local macro definitions ********


// ******** local type definitions ********
typedef struct
{
	cxa_stringUtils_dataType_t dataType;
	const char* string;
}dataType_string_mapEntry_t;


// ******** local function prototypes ********
#ifndef __XC
double strtod (const char* str, char** endptr);     // disable for pic32
#endif


// ********  local variable declarations *********
static dataType_string_mapEntry_t DT_STRING_MAP[] =
{
		{CXA_STRINGUTILS_DATATYPE_DOUBLE,  "double"},
		{CXA_STRINGUTILS_DATATYPE_INTEGER, "integer"},
		{CXA_STRINGUTILS_DATATYPE_STRING,  "string"},
		{CXA_STRINGUTILS_DATATYPE_UNKNOWN, "unknown"}
};


// ******** global function implementations ********
bool cxa_stringUtils_startsWith(const char* targetStringIn, const char* prefixStringIn)
{
	size_t targetStringLen_bytes = (targetStringIn != NULL) ? strlen(targetStringIn) : 0;
	size_t prefixStringLen_bytes = (prefixStringIn != NULL) ? strlen(prefixStringIn) : 0;

	return cxa_stringUtils_startsWith_withLengths(targetStringIn, targetStringLen_bytes, prefixStringIn, prefixStringLen_bytes);
}


bool cxa_stringUtils_startsWith_withLengths(const char* targetStringIn, size_t targetStringLen_bytesIn, const char* prefixStringIn, size_t prefixStringLen_bytesIn)
{
	// make sure we have enough chars in our target string
	if( prefixStringLen_bytesIn > targetStringLen_bytesIn ) return false;

	// iterate our characters
	for( size_t i = 0; i < prefixStringLen_bytesIn; i++ )
	{
		if( targetStringIn[i] != prefixStringIn[i] ) return false;
	}

	// if we made it here, target string starts with prefix string
	return true;
}


bool cxa_stringUtils_endsWith_withLengths(const char* targetStringIn, size_t targetStringLen_bytesIn, const char* suffixStringIn)
{
	// if one is false and the other isn't, we can't compare
	if( (targetStringIn == NULL) != (suffixStringIn == NULL) ) return false;
	// if they're both NULL, they're equal
	if( (targetStringIn == NULL) && (suffixStringIn == NULL) ) return true;

	// make sure we have enough chars in our target string
	size_t suffixStringLen_bytes = strlen(suffixStringIn);
	if( suffixStringLen_bytes > targetStringLen_bytesIn ) return false;

	// iterate our characters
	for( size_t i = 0; i < suffixStringLen_bytes; i++ )
	{
		if( targetStringIn[targetStringLen_bytesIn-1-i] != suffixStringIn[suffixStringLen_bytes-1-i] ) return false;
	}

	// if we made it here, target string starts with prefix string
	return true;
}


bool cxa_stringUtils_contains(const char* targetStringIn, const char* elementIn)
{
	if( (targetStringIn == NULL) || (elementIn == NULL) ) return false;

	return cxa_stringUtils_contains_withLengths(targetStringIn, strlen(targetStringIn), elementIn, strlen(elementIn));
}


bool cxa_stringUtils_contains_withLengths(const char* targetStringIn, size_t targetStringLen_bytesIn, const char* elementIn, size_t elementLen_bytesIn)
{
	if( (targetStringIn == NULL) || (elementIn == NULL) ) return false;

	return (cxa_stringUtils_indexOfFirstOccurence_withLengths(targetStringIn, targetStringLen_bytesIn, elementIn, elementLen_bytesIn) >= 0);
}


bool cxa_stringUtils_concat(char *targetStringIn, const char *sourceStringIn, size_t maxSizeTarget_bytesIn)
{
	cxa_assert(targetStringIn);
	cxa_assert(sourceStringIn);

	// get the current size of the target
	size_t targetLen_bytes;
	if( !cxa_stringUtils_strlen(targetStringIn, maxSizeTarget_bytesIn, &targetLen_bytes) ) return false;
	// ensure that we have space for the null term
	if( targetLen_bytes > (maxSizeTarget_bytesIn-1) ) return false;

	// now see if we have room for the new string
	size_t maxBufferSize_sourceString_bytes = maxSizeTarget_bytesIn - targetLen_bytes;
	size_t sourceLen_bytes;
	if( !cxa_stringUtils_strlen(sourceStringIn, maxBufferSize_sourceString_bytes, &sourceLen_bytes) ) return false;

	// we apparently have room for the string...do the concatenation
	for( size_t i = 0; i < sourceLen_bytes; i++ )
	{
		targetStringIn[targetLen_bytes+i] = sourceStringIn[i];
	}

	// null term
	targetStringIn[targetLen_bytes+sourceLen_bytes] = 0;

	return true;
}


bool cxa_stringUtils_concat_formattedString(char *targetStringIn, size_t maxSizeTarget_bytesIn, const char *fmtIn, ...)
{
	cxa_assert(targetStringIn);
	cxa_assert(fmtIn);

	// get the current size of the target
	size_t targetLen_bytes;
	if( !cxa_stringUtils_strlen(targetStringIn, maxSizeTarget_bytesIn, &targetLen_bytes) ) return false;

	size_t maxFormattedStringLength_bytes = maxSizeTarget_bytesIn - targetLen_bytes;
	char tmpBuffer[maxFormattedStringLength_bytes];

	// create our formatted string
	va_list varArgs;
	va_start(varArgs, fmtIn);
	int numCharsPrinted = vsnprintf(tmpBuffer, maxFormattedStringLength_bytes, fmtIn, varArgs);
	if( numCharsPrinted >= maxFormattedStringLength_bytes ) return false;
	tmpBuffer[maxFormattedStringLength_bytes-1] = 0;
	va_end(varArgs);

	// if we made it here, we apparently have room for the string...do the concatenation
	for( size_t i = 0; i < numCharsPrinted; i++ )
	{
		targetStringIn[targetLen_bytes+i] = tmpBuffer[i];
	}

	// null term
	targetStringIn[targetLen_bytes+numCharsPrinted] = 0;

	return true;
}


bool cxa_stringUtils_concat_withLengths(char *targetStringIn, size_t maxSizeTarget_bytesIn, const char *sourceStringIn, size_t sourceStringLen_bytesIn)
{
	cxa_assert(targetStringIn);
	cxa_assert(sourceStringIn);

	// get the current size of the target
	size_t targetLen_bytes;
	if( !cxa_stringUtils_strlen(targetStringIn, maxSizeTarget_bytesIn, &targetLen_bytes) ) targetLen_bytes = maxSizeTarget_bytesIn;
	// ensure that we have space for the null term
	if( targetLen_bytes > (maxSizeTarget_bytesIn-1) ) return false;

	// now see if we have room for the new string (and null term)
	size_t maxBufferSize_sourceString_bytes = maxSizeTarget_bytesIn - targetLen_bytes;
	if( maxBufferSize_sourceString_bytes < (sourceStringLen_bytesIn+1) ) return false;

	// we apparently have room for the string...do the concatenation
	for( size_t i = 0; i < sourceStringLen_bytesIn; i++ )
	{
		targetStringIn[targetLen_bytes+i] = sourceStringIn[i];
	}

	// null term
	targetStringIn[targetLen_bytes+sourceStringLen_bytesIn] = 0;

	return true;
}


bool cxa_stringUtils_copy(char *targetStringIn, const char* sourceStringIn, size_t sizeofTargetStringIn)
{
	cxa_assert(targetStringIn);

	bool wasNullTermd = false;
	for( size_t i = 0; i < sizeofTargetStringIn; i++ )
	{
		targetStringIn[i] = sourceStringIn[i];
		if( sourceStringIn[i] == 0 )
		{
			wasNullTermd = true;
			break;
		}
	}

	if( !wasNullTermd ) targetStringIn[sizeofTargetStringIn-1] = 0;

	return wasNullTermd;
}


bool cxa_stringUtils_strlen(const char *targetStringIn, size_t maxSize_bytesIn, size_t* stringLen_bytesOut)
{
	cxa_assert(targetStringIn);

	for( size_t i = 0; i < maxSize_bytesIn; i++ )
	{
		if( targetStringIn[i] == 0 )
		{
			if( stringLen_bytesOut != NULL ) *stringLen_bytesOut = i;
			return true;
		}
	}

	return false;
}


bool cxa_stringUtils_equals(const char* str1In, const char* str2In)
{
	return cxa_stringUtils_equals_withLengths(str1In, strlen(str1In), str2In, strlen(str2In));
}


bool cxa_stringUtils_equals_withLengths(const char* str1In, size_t str1Len_bytes, const char* str2In, size_t str2Len_bytes)
{
	// if one is NULL and the other isn't, we can't compare
	if( (str1In == NULL) != (str2In == NULL) ) return false;
	// if they're both NULL, they're equal
	if( (str1In == NULL) && (str2In == NULL) ) return true;

	// must have the same length
	if( str1Len_bytes != str2Len_bytes ) return false;

	// if we made it here, they have the same length...we're safe
	// to use strcmp
	return (strncmp(str1In, str2In, str1Len_bytes) == 0);
}


bool cxa_stringUtils_equals_ignoreCase(const char* str1In, const char* str2In)
{
	// if one is false and the other isn't, we can't compare
	if( (str1In == NULL) != (str2In == NULL) ) return false;
	// if they're both NULL, they're equal
	if( (str1In == NULL) && (str2In == NULL) ) return true;

	// must have the same length
	if( strlen(str1In) != strlen(str2In) ) return false;

	// now iterate over each character
	for( size_t i = 0; i < strlen(str1In); i++ )
	{
		if( tolower((int)str1In[i]) != tolower((int)str2In[i]) ) return false;
	}

	// if we made it here, the strings were identical
	return true;
}


size_t cxa_stringUtils_countOccurences(const char* targetStringIn, const char* elementIn)
{
	size_t strLength_target_bytes = strlen(targetStringIn);
	size_t strLength_element_bytes = strlen(elementIn);

	return cxa_stringUtils_countOccurences_withLengths(targetStringIn, strLength_target_bytes, elementIn, strLength_element_bytes);
}


size_t cxa_stringUtils_countOccurences_withLengths(const char* targetStringIn, size_t targetStringLen_bytesIn, const char* elementIn, size_t elementStringLen_bytesIn)
{
	cxa_assert(targetStringIn);
	cxa_assert(elementIn);

	if( elementStringLen_bytesIn > targetStringLen_bytesIn ) return 0;

	size_t retVal = 0;
	for( size_t i = 0; i < targetStringLen_bytesIn - elementStringLen_bytesIn; i++ )
	{
		if( cxa_stringUtils_equals_withLengths(&targetStringIn[i], elementStringLen_bytesIn, elementIn, elementStringLen_bytesIn) ) retVal++;
	}

	return retVal;
}


ssize_t cxa_stringUtils_indexOfFirstOccurence_withLengths(const char* targetStringIn, size_t targetStringLen_bytesIn, const char* elementIn, size_t elementLen_bytesIn)
{
	if( (targetStringIn == NULL) || (elementIn == NULL) ) return -2;

	for( size_t i = 0; i < targetStringLen_bytesIn; i++ )
	{
		if( (i + elementLen_bytesIn) > targetStringLen_bytesIn ) break;

		if( strncmp(&targetStringIn[i], elementIn, elementLen_bytesIn) == 0 ) return i;
	}

	return -1;
}


char* cxa_stringUtils_getLastCharacters(const char* targetStringIn, size_t numCharsIn)
{
	cxa_assert(targetStringIn);

	size_t strLength_bytes = strlen(targetStringIn);
	if( numCharsIn > strLength_bytes ) return NULL;

	return (char*)&targetStringIn[strLength_bytes - numCharsIn];
}


bool cxa_stringUtils_replaceFirstOccurence(const char *targetStringIn, const char *stringToReplaceIn, const char *replacementStringIn)
{
	cxa_assert(targetStringIn);
	cxa_assert(stringToReplaceIn);
	cxa_assert(replacementStringIn);

	size_t targetStringLen_bytes = strlen(targetStringIn);
	size_t stringToReplaceLen_bytes = strlen(stringToReplaceIn);
	size_t replacementStringLen_bytes = strlen(replacementStringIn);

	return cxa_stringUtils_replaceFirstOccurence_withLengths(targetStringIn, targetStringLen_bytes,
															 stringToReplaceIn, stringToReplaceLen_bytes,
															 replacementStringIn, replacementStringLen_bytes);
}


bool cxa_stringUtils_replaceFirstOccurence_withLengths(const char *targetStringIn, size_t targetStringLen_bytesIn,
													   const char *stringToReplaceIn, size_t stringToReplaceLen_bytesIn,
													   const char *replacementStringIn, size_t replacementStringLen_bytesIn)
{
	cxa_assert(targetStringIn);
	cxa_assert(stringToReplaceIn);
	cxa_assert(replacementStringIn);

	// make sure our sizes are appropriate
	if( (targetStringLen_bytesIn < stringToReplaceLen_bytesIn) ||
		(stringToReplaceLen_bytesIn == 0) ||
		(replacementStringLen_bytesIn > stringToReplaceLen_bytesIn) ) return false;

	// find the first occurrence
	ssize_t indexOfFirstOccurence = cxa_stringUtils_indexOfFirstOccurence_withLengths(targetStringIn, targetStringLen_bytesIn, stringToReplaceIn, stringToReplaceLen_bytesIn);
	if( indexOfFirstOccurence < 0 ) return false;
	char* firstOccurrence = (char*)&(targetStringIn[indexOfFirstOccurence]);

	// got the first occurrence...adjust the string size first (if needed...can only be smaller)
	size_t adjSize_bytes = stringToReplaceLen_bytesIn - replacementStringLen_bytesIn;
	if( adjSize_bytes > 0 )
	{
		// need to remove some bytes
		memmove((void*)firstOccurrence, (void*)(firstOccurrence+adjSize_bytes),
				targetStringLen_bytesIn - (firstOccurrence - targetStringIn) - adjSize_bytes);
	}

	// now do the actual replacement
	for( size_t i = 0; i < replacementStringLen_bytesIn; i++ )
	{
		firstOccurrence[i] = replacementStringIn[i];
	}
	return true;
}


void cxa_stringUtils_trim(char *const targetStringIn)
{
	cxa_assert(targetStringIn);

	size_t strLen_bytes = strlen(targetStringIn);
	for( size_t i = 0; i < strLen_bytes; i++ )
	{
		size_t currIndex = strLen_bytes - i - 1;
		if( isspace((int)targetStringIn[currIndex]) )
		{
			targetStringIn[currIndex] = 0;
		}
		else
		{
			// we're done
			break;
		}
	}
}


bool cxa_stringUtils_bytesToHexString(uint8_t* bytesIn, size_t numBytesIn, bool transposeIn, char* hexStringOut, size_t maxLenHexString_bytesIn)
{
	cxa_assert(bytesIn);
	cxa_assert(hexStringOut);

	hexStringOut[0] = 0;

	for( size_t i = 0; i < numBytesIn; i++ )
	{
		char currHexByteStr[3];
		snprintf(currHexByteStr, sizeof(currHexByteStr), "%02X", bytesIn[(transposeIn ? (numBytesIn - i - 1) : i)]);

		if( !cxa_stringUtils_concat(hexStringOut, currHexByteStr, maxLenHexString_bytesIn) ) return false;
	}

	return true;
}


bool cxa_stringUtils_hexStringToBytes(const char *const hexStringIn, size_t numBytesIn, bool transposeIn, uint8_t* bytesOut)
{
	cxa_assert(hexStringIn);

	size_t strLength_bytes = strlen(hexStringIn);
	if( (strLength_bytes / 2) < numBytesIn ) return false;

	memset(bytesOut, 0, numBytesIn);

	for( size_t i = 0; i < strLength_bytes; i++ )
	{
		uint8_t currChar = hexStringIn[i];

		// keep all alpha characters upper case
		if( islower(currChar) ) currChar = toupper(currChar);

		// convert to a value
		uint8_t currVal = 0;
		if( (48 <= currChar) && (currChar <= 57) )
		{
			currVal = currChar - 48;				// 0-9
		}
		else if( (65 <= currChar) && (currChar <= 70) )
		{
			currVal = (currChar - 65) + 10;			// A-F
		}
		else
		{
			return false;
		}

		size_t currByte = i / 2;
		if( transposeIn ) currByte = numBytesIn - currByte - 1;
		if( i % 2 )
		{
			// lower nibble
			bytesOut[currByte] |= (currVal << 0);
		}
		else
		{
			// upper nibble
			bytesOut[currByte] |= (currVal << 4);
		}
	}

	return true;
}


bool cxa_stringUtils_ipStringToUint32(const char *const ipStringIn, uint32_t *const ipBytesOut)
{
	if( ipStringIn == NULL ) return false;

	cxa_stringUtils_parseResult_t parseResult;
	char* tokSavePtr;
	char* targetStr;

	if( ((targetStr = strtok_r((char*)ipStringIn, ".", &tokSavePtr)) == NULL) ||
		!cxa_stringUtils_parseString(targetStr, &parseResult) ||
		(parseResult.dataType != CXA_STRINGUTILS_DATATYPE_INTEGER) ||
		(parseResult.val_uint > 255) ) return false;
	uint8_t byte0 = parseResult.val_uint;

	if( ((targetStr = strtok_r(NULL, ".", &tokSavePtr)) == NULL) ||
		!cxa_stringUtils_parseString(targetStr, &parseResult) ||
		(parseResult.dataType != CXA_STRINGUTILS_DATATYPE_INTEGER) ||
		(parseResult.val_uint > 255) ) return false;
	uint8_t byte1 = parseResult.val_uint;

	if( ((targetStr = strtok_r(NULL, ".", &tokSavePtr)) == NULL) ||
		!cxa_stringUtils_parseString(targetStr, &parseResult) ||
		(parseResult.dataType != CXA_STRINGUTILS_DATATYPE_INTEGER) ||
		(parseResult.val_uint > 255) ) return false;
	uint8_t byte2 = parseResult.val_uint;

	if( ((targetStr = strtok_r(NULL, ".", &tokSavePtr)) == NULL) ||
		!cxa_stringUtils_parseString(targetStr, &parseResult) ||
		(parseResult.dataType != CXA_STRINGUTILS_DATATYPE_INTEGER) ||
		(parseResult.val_uint > 255) ) return false;
	uint8_t byte3 = parseResult.val_uint;

	if( ipBytesOut != NULL ) *ipBytesOut = ((uint32_t)byte3 << 24) | ((uint32_t)byte2 << 16) | ((uint32_t)byte1 << 8) | ((uint32_t)byte0 << 0);

	return true;
}


bool cxa_stringUtils_parseString(char *const strIn, cxa_stringUtils_parseResult_t* parseResultOut)
{
	if( strIn == NULL ) return false;

	#if !defined(errno_defined) && !defined(errno)
	int errno = 0;
	#endif

	// first things first...see if the string contains a period
	bool parseSuccess = false;
	if( cxa_stringUtils_contains(strIn, ".") )
	{
		// this may be a double or a string
		char* p = strIn;
		errno = 0;
		double val = strtod(strIn, &p);
		if( (errno != 0) || (strIn == p) || (*p != 0) )
		{
			// couldn't parse a double...must be a string
			parseSuccess = true;
			if( parseResultOut != NULL )
			{
				parseResultOut->dataType = CXA_STRINGUTILS_DATATYPE_STRING;
				parseResultOut->val_string = strIn;
			}
		}
		else
		{
			// we parsed a double!
			parseSuccess = true;
			if( parseResultOut != NULL )
			{
				parseResultOut->dataType = CXA_STRINGUTILS_DATATYPE_DOUBLE;
				parseResultOut->val_double = val;
			}
		}
	}
	else
	{
		// this may be a signed integer or a string
		char* p = strIn;
		errno = 0;
		long val = strtol(strIn, &p, 10);
		if( (errno != 0) || (strIn == p) || (*p != 0) )
		{
			// couldn't parse an integer...must be a string
			parseSuccess = true;
			if( parseResultOut != NULL )
			{
				parseResultOut->dataType = CXA_STRINGUTILS_DATATYPE_STRING;
				parseResultOut->val_string = strIn;
			}
		}
		else
		{
			// we got an integer!
			parseSuccess = true;
			if( parseResultOut != NULL )
			{
				parseResultOut->dataType = CXA_STRINGUTILS_DATATYPE_INTEGER;
				parseResultOut->val_int = val;
			}
		}
	}

	return parseSuccess;
}


const char* cxa_stringUtils_getStringForDataType(cxa_stringUtils_dataType_t dataTypeIn)
{
	for(size_t i = 0; i < (sizeof(DT_STRING_MAP)/sizeof(*DT_STRING_MAP)); i++ )
	{
		if( DT_STRING_MAP[i].dataType == dataTypeIn ) return DT_STRING_MAP[i].string;
	}

	return DT_STRING_MAP[(sizeof(DT_STRING_MAP)/sizeof(*DT_STRING_MAP))-1].string;
}


// ******** local function implementations ********
