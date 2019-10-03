/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
 
/**
 * @file
 * This file contains an implementation of a statically allocated, fixed-max-length buffer/
 * array that is specifically targetted at byte-level manipulation. If you need to store
 * binary data and access it as bytes, 16-bit integers, 32-bit integers, this is the
 * appropriate class.
 *
 * @note This object should work across all architecture-specific implementations
 *
 *
 * #### Example Usage: ####
 *
 * @code
 * cxa_fixedByteBuffer_t myFbb;
 * uint8_t myFbb_buffer[16];			// where the data is actually stored
 *
 * // initialize the buffer with a maximum of 16 elements
 * cxa_fixedByteBuffer_init(&myFbb, (void*)myBuffer_buffer, sizeof(myBuffer_buffer));
 * // OR more simply:
 * cxa_fixedByteBuffer_initStd(&myFbb, myBuffer_buffer);
 *
 * ...
 *
 * // add two new values to the buffer (0x47 and 0x65)
 * cxa_fixedByteBuffer_append_uint8(&myFbb, 0x47);
 * cxa_fixedByteBuffer_append_uint8(&myFbb, 0x65);
 *
 * ...
 *
 * // now retrieve the two values out of the buffer treating them as parts of a 16-bit integer
 * uint16_t foo;
 * if( !cxa_fixedByteBuffer_get_uint16LE(&myFbb, 0, foo) )
 * {
 *    // failed to get uint16_t
 * }
 * // now, foo = 0x6547
 * @endcode
 */
#ifndef CXA_FIXED_BYTE_BUFFER_H_
#define CXA_FIXED_BYTE_BUFFER_H_


// ******** includes ********
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <cxa_config.h>
#include <cxa_array.h>


// ******** global macro definitions ********
/**
 * @public
 * @brief Shortcut to initialize a fixedByteBuffer with a buffer of an explicit data type
 *
 * @code
 * cxa_fixedByteBuffer_t myFbb;
 * double myBuffer[100];
 *
 * cxa_fixedByteBuffer_initStd(&myArray, myBuffer);
 * // equivalent to
 * cxa_fixedByteBuffer_init(&myFbb, (void)myBuffer, sizeof(myBuffer));
 * @endcode
 *
 * @param[in] fbbIn pointer to the pre-allocated fixedByteBuffer object
 * @param[in] bufferIn pointer to the declared c-style array which
 * 		will contain the data for the fixedByteBuffer
 */
#define cxa_fixedByteBuffer_initStd(fbbIn, bufferIn)						cxa_fixedByteBuffer_init((fbbIn), ((void*)(bufferIn)), sizeof(bufferIn))

#define cxa_fixedByteBuffer_append_uint8(fbbIn, uint8In)					cxa_fixedByteBuffer_append((fbbIn), (uint8_t[]){(uint8In)}, 1)
#define cxa_fixedByteBuffer_append_uint16LE(fbbIn, uint16In)				cxa_fixedByteBuffer_append((fbbIn), (uint8_t[]){((uint8_t)((((uint16_t)(uint16In)) & 0x00FF) >> 0)), ((uint8_t)((((uint16_t)(uint16In)) & 0xFF00) >> 8))}, 2)
#define cxa_fixedByteBuffer_append_uint16BE(fbbIn, uint16In)				cxa_fixedByteBuffer_append((fbbIn), (uint8_t[]){((uint8_t)((((uint16_t)(uint16In)) & 0xFF00) >> 8)), ((uint8_t)((((uint16_t)(uint16In)) & 0x00FF) >> 0))}, 2)
#define cxa_fixedByteBuffer_append_uint32LE(fbbIn, uint32In)				cxa_fixedByteBuffer_append((fbbIn), (uint8_t[]){((uint8_t)((((uint32_t)(uint32In)) & 0x000000FF) >> 0)), ((uint8_t)((((uint32_t)(uint32In)) & 0x0000FF00) >> 8)), ((uint8_t)((((uint32_t)(uint32In)) & 0x00FF0000) >> 16)), ((uint8_t)((((uint32_t)(uint32In)) & 0xFF000000) >> 24)) }, 4)
#define cxa_fixedByteBuffer_append_uint32BE(fbbIn, uint32In)				cxa_fixedByteBuffer_append((fbbIn), (uint8_t[]){((uint8_t)((((uint32_t)(uint32In)) & 0xFF000000) >> 24)), ((uint8_t)((((uint32_t)(uint32In)) & 0x00FF0000) >> 16)), ((uint8_t)((((uint32_t)(uint32In)) & 0x0000FF00) >> 8)), ((uint8_t)((((uint32_t)(uint32In)) & 0x000000FF) >> 0)) }, 4)
//#define cxa_fixedByteBuffer_append_float(fbbIn, floatIn)					cxa_fixedByteBuffer_append((fbbIn), (uint8_t*)&(floatIn), 4)
#define cxa_fixedByteBuffer_append_cString(fbbIn, strIn)					cxa_fixedByteBuffer_append((fbbIn), (uint8_t*)(strIn), (strlen(strIn)+1))

#define cxa_fixedByteBuffer_append_lengthPrefixedCString_uint16BE(fbbIn, strIn, includeNullTermIn)	\
	cxa_fixedByteBuffer_append_lengthPrefixedField_uint16BE((fbbIn), (uint8_t*)(strIn), strlen((const char *)strIn) + ((includeNullTermIn) ? 1 : 0))

#define cxa_fixedByteBuffer_remove_uint8(fbbIn, indexIn)					cxa_fixedByteBuffer_remove((fbbIn), (indexIn), 1)
#define cxa_fixedByteBuffer_remove_uint16(fbbIn, indexIn)					cxa_fixedByteBuffer_remove((fbbIn), (indexIn), 2)
#define cxa_fixedByteBuffer_remove_uint32(fbbIn, indexIn)					cxa_fixedByteBuffer_remove((fbbIn), (indexIn), 4)
#define cxa_fixedByteBuffer_remove_float(fbbIn, indexIn)					cxa_fixedByteBuffer_remove((fbbIn), (indexIn), 4)

#define cxa_fixedByteBuffer_get_uint8(fbbIn, indexIn, uint8Out)				cxa_fixedByteBuffer_get((fbbIn), (indexIn), false, (uint8_t*)&(uint8Out), 1)
#define cxa_fixedByteBuffer_get_uint16LE(fbbIn, indexIn, uint16Out)			cxa_fixedByteBuffer_get((fbbIn), (indexIn), false, (uint8_t*)&(uint16Out), 2)
#define cxa_fixedByteBuffer_get_uint32LE(fbbIn, indexIn, uint32Out)			cxa_fixedByteBuffer_get((fbbIn), (indexIn), false, (uint8_t*)&(uint32Out), 4)
//#define cxa_fixedByteBuffer_get_float(fbbIn, indexIn, floatOut)				cxa_fixedByteBuffer_get((fbbIn), (indexIn), false, (uint8_t*)&(floatOut), 4)

#define cxa_fixedByteBuffer_get_uint16BE(fbbIn, indexIn, uint16Out)			cxa_fixedByteBuffer_get((fbbIn), (indexIn), true, (uint8_t*)&(uint16Out), 2)
#define cxa_fixedByteBuffer_get_uint32BE(fbbIn, indexIn, uint32Out)			cxa_fixedByteBuffer_get((fbbIn), (indexIn), true, (uint8_t*)&(uint32Out), 4)
#define cxa_fixedByteBuffer_get_floatBE(fbbIn, indexIn, floatOut)			cxa_fixedByteBuffer_get((fbbIn), (indexIn), true, (uint8_t*)&(floatOut), 4)

#define cxa_fixedByteBuffer_replace_uint8(fbbIn, indexIn, uint8In)			cxa_fixedByteBuffer_replace((fbbIn), (indexIn), (uint8_t[]){(uint8In)}, 1)
#define cxa_fixedByteBuffer_replace_uint16LE(fbbIn, indexIn, uint16In)		cxa_fixedByteBuffer_replace((fbbIn), (indexIn), (uint8_t[]){((uint8_t)((((uint16_t)(uint16In)) & 0x00FF) >> 0)), ((uint8_t)((((uint16_t)(uint16In)) & 0xFF00) >> 8))}, 2)
#define cxa_fixedByteBuffer_replace_uint32LE(fbbIn, indexIn, uint32In)		cxa_fixedByteBuffer_replace((fbbIn), (indexIn), (uint8_t[]){ ((uint8_t)((((uint32_t)(uint32In)) & 0x000000FF) >> 0)), ((uint8_t)((((uint32_t)(uint32In)) & 0x0000FF00) >> 8)), ((uint8_t)((((uint32_t)(uint32In)) & 0x00FF0000) >> 16)), ((uint8_t)((((uint32_t)(uint32In)) & 0xFF000000) >> 24)) }, 4)
#define cxa_fixedByteBuffer_replace_float(fbbIn, indexIn, floatIn)			cxa_fixedByteBuffer_replace((fbbIn), (indexIn), (uint8_t*)&(floatIn), 4)

#define cxa_fixedByteBuffer_insert_uint8(fbbIn, indexIn, uint8In)			cxa_fixedByteBuffer_insert((fbbIn), (indexIn), (uint8_t[]){(uint8In)}, 1)
#define cxa_fixedByteBuffer_insert_uint16(fbbIn, indexIn, uint16In)			cxa_fixedByteBuffer_insert((fbbIn), (indexIn), (uint8_t[]){((uint8_t)((((uint16_t)(uint16In)) & 0x00FF) >> 0)), ((uint8_t)((((uint16_t)(uint16In)) & 0xFF00) >> 8))}, 2)
#define cxa_fixedByteBuffer_insert_uint32(fbbIn, indexIn, uint32In)			cxa_fixedByteBuffer_insert((fbbIn), (indexIn), (uint8_t[]){ ((uint8_t)((((uint32_t)(uint32In)) & 0x000000FF) >> 0)), ((uint8_t)((((uint32_t)(uint32In)) & 0x0000FF00) >> 8)), ((uint8_t)((((uint32_t)(uint32In)) & 0x00FF0000) >> 16)), ((uint8_t)((((uint32_t)(uint32In)) & 0xFF000000) >> 24)) }, 4)
#define cxa_fixedByteBuffer_insert_float(fbbIn, indexIn, floatIn)			cxa_fixedByteBuffer_insert((fbbIn), (indexIn), (uint8_t*)&(floatIn), 4)
#define cxa_fixedByteBuffer_insert_cString(fbbIn, indexIn, strIn)			cxa_fixedByteBuffer_insert((fbbIn), (indexIn), (uint8_t*)(strIn), (strlen(strIn)+1))


// ******** global type definitions *********
/**
 * @public
 * @brief "Forward" declaration of the cxa_fixedByteBuffer_t object
 */
typedef struct cxa_fixedByteBuffer cxa_fixedByteBuffer_t;


/**
 * @private
 */
struct cxa_fixedByteBuffer
{
	cxa_array_t bytes;
};


// ******** global function prototypes ********
/**
 * @public
 * @brief Initializes the pre-allocated fixedByteBuffer using the specified
 *		buffer to store elements.
 *
 * @param[in] fbbIn pointer to the pre-allocated fixedByteBuffer object
 * @param[in] bufferLocIn pointer to the pre-allocated chunk of memory that will
 * 		be used to store elements in the buffer
 * @param[in] bufferMaxSize_bytesIn the maximum size of the chunk of memory (buffer) in bytes
 */
void cxa_fixedByteBuffer_init(cxa_fixedByteBuffer_t *const fbbIn, void *const bufferLocIn, const size_t bufferMaxSize_bytesIn);


/**
 * @public
 * @brief Initializes the pre-allocated fixedByteBuffer using the
 * 		specified memory (which already contains data).
 *
 * @param[in] fbbIn pointer to the pre-allocated fixedByteBuffer object
 * @param[in] currNumElemsIn the number of bytes currently in the
 *		specified buffer. The resulting buffer will be initialized such that
 *		::cxa_fixedByteBuffer_getSize_bytes will equal currNumElemsIn.
 * @param[in] bufferLocIn pointer to the pre-allocated chunk of memory that will
 * 		be used to store bytes in the array (the buffer)
 * @param[in] bufferMaxSize_bytesIn the maximum size of the chunk of memory (buffer) in bytes
 */
void cxa_fixedByteBuffer_init_inPlace(cxa_fixedByteBuffer_t *const fbbIn, const size_t currNumElemsIn, void *const bufferLocIn, const size_t bufferMaxSize_bytesIn);


/**
 * @public
 * @brief Initializes a new subBuffer whose bytes are stored within another fixedByteBuffer. This
 * 		function requires an explicit startIndex and length.
 * 		Note: both the parentFbb and the subFbb will be backed by the same data, modification
 * 			of byte values in one will be reflected in the other. Care should be made to avoid
 * 			structure-modifying actions (such as appending, inserting or removing)
 *
 * @param[in] subFbbIn pointer to the pre-allocated fixedByteBuffer object (the new fbb)
 * @param[in] parentFbbIn pointer to the pre-initialized parent fixedByteBuffer (currently holding the target bytes)
 * @param[in] startIndexIn the desired offset of the subFbb within the parent (eg. 2 -> index 0 of subFbb == index 2 of parentFbb)
 * @param[in] numBytesIn the desired size of the subFbb (in bytes). The current and max size of the subFbb will
 * 		be set to this value. Care must be taken to ensure that startIndexIn + numBytesIn is not out of bounds
 * 		of the parentFbb
 */
void cxa_fixedByteBuffer_init_subBufferFixedSize(cxa_fixedByteBuffer_t *const subFbbIn, cxa_fixedByteBuffer_t *const parentFbbIn, const size_t startIndexIn, size_t numBytesIn);


/**
 * @public
 * @brief Initializes a new subBuffer whose bytes are stored within another fixedByteBuffer. This
 * 		function will size the subFbb to use all bytes in the parent starting at the specified
 * 		index (eg. parentSize=4, startIndex=2 -> subFbbSize=2).
 * 		Note: both the parentFbb and the subFbb will be backed by the same data, modification
 * 			of byte values in one will be reflected in the other. Care should be made to avoid
 * 			structure-modifying actions (such as appending, inserting or removing)
 *
 * @param[in] subFbbIn pointer to the pre-allocated fixedByteBuffer object (the new fbb)
 * @param[in] parentFbbIn pointer to the pre-initialized parent fixedByteBuffer (currently holding the target bytes)
 * @param[in] startIndexIn the desired offset of the subFbb within the parent (eg. 2 -> index 0 of subFbb == index 2 of parentFbb)
 */
void cxa_fixedByteBuffer_init_subBufferRemainingElems(cxa_fixedByteBuffer_t *const subFbbIn, cxa_fixedByteBuffer_t *const parentFbbIn, const size_t startIndexIn);


/**
 * @public
 * @brief Initializes a new subBuffer whose bytes are stored within another fixedByteBuffer. This
 * 		function is similar to 'cxa_fixedByteBuffer_init_subBufferRemainingElems' except that
 * 		the subBuffer can be located anywhere within the parent buffer, even in "empty" space.
 * 		Note: both the parentFbb and subFbb will be backed by the same data, modification
 * 			of byte values in one will be reflected in the other. Structure-modifying actions
 * 			(append, insert, remove) in one will NOT update the meta information of the other
 * 			(eg. size, freeSize, etc). The two buffers will essentially be independent but
 * 			backed in the same memory.
 *
 * @param[in] subFbbIn pointer to the pre-allocated fixedByteBuffer object (the new fbb)
 * @param[in] parentFbbIn pointer to the pre-initialized parent fixedByteBuffer (currently holding the target bytes)
 * @param[in] startIndexIn the desired offset of the subFbb within the parent (eg. 2 -> index 0 of subFbb == index 2 of parentFbb)
 */
void cxa_fixedByteBuffer_init_subBufferParentMaxSize(cxa_fixedByteBuffer_t *const subFbbIn, cxa_fixedByteBuffer_t *const parentFbbIn, const size_t startIndexIn);


/**
 * @public
 * @brief Appends (copies) the number of bytes at the specified pointer to the end of the fixed byte buffer
 *
 * @param[in] fbbIn pointer to the pre-initialized cxa_fixedByteBuffer_t object
 * @param[in] ptrIn pointer to the location of the bytes to copy to the end of our buffer
 * @param[in] numBytesIn the number of bytes, starting at ptrIn to copy to the end of our buffer
 *
 * @return true if the append was successful, false if not (eg. full)
 */
bool cxa_fixedByteBuffer_append(cxa_fixedByteBuffer_t *const fbbIn, uint8_t *const ptrIn, const size_t numBytesIn);


/**
 * @public
 * @brief Appends a length-prefixed field to the fixed byte buffer.
 * 		First, 2-bytes of length data will be appended to the buffer in BigEndian. The length field contains
 * 		the number of bytes following and is equal to 'numBytesIn'. Following the length prefix, the number
 * 		of bytes starting at ptrIn will be copied to the end of the fixed byte buffer. The total number of bytes
 * 		appended to the buffer will be ('numBytesIn' + 2)
 *
 * @param[in] fbbIn pointer to the pre-initialized cxa_fixedByteBuffer_t object
 * @param[in] ptrIn pointer to the location of the bytes to copy to the end of our buffer
 * @param[in] numBytesIn the number of bytes, starting at ptrIn to copy to the end of our buffer
 *
 * @return true if the append was successful, false if not (eg. full)
 */
bool cxa_fixedByteBuffer_append_lengthPrefixedField_uint16BE(cxa_fixedByteBuffer_t *const fbbIn, uint8_t *const ptrIn, const uint16_t numBytesIn);


/**
 * @public
 * @brief Similar to the 'cxa_array_append_empty'...appends the
 * 		given number of bytes to the buffer (increasing the
 * 		size of the buffer), but doesn't actually copy data
 * 		into the buffer.
 *
 * Instead, this function returns a pointer to the start of
 * the newly created "block" in the buffer.This allows you
 * to minimize copy operations by initializing the element
 * "in-place" within the buffer.
 *
 * @param[in] fbbIn pointer to the pre-initialized cxa_fixedByteBuffer_t object
 * @param[in] numBytesIn the number of empty bytes to append to the buffer
 *
 * @return a pointer to the new "block" location within the
 * 		buffer object OR NULL on error (buffer is likely full)
 */
void* cxa_fixedByteBuffer_append_emptyBytes(cxa_fixedByteBuffer_t *const fbbIn, const size_t numBytesIn);


/**
 * @public
 * @brief Appends the contents of the source buffer to the target buffer.
 *
 * @param[in] fbbIn pointer to the pre-initialized cxa_fixedByteBuffer_t object
 *
 * @return true if the contents were copied, false if there is not enough space
 */
bool cxa_fixedByteBuffer_append_fbb(cxa_fixedByteBuffer_t *const fbbIn, cxa_fixedByteBuffer_t *const sourceFbbIn);


/**
 * @public
 * @brief Removes the specified elements from the buffer (moving all following elements down)
 *
 * @param[in] fbbIn pointer to the pre-initialized cxa_fixedByteBuffer_t object
 * @param[in] indexIn the first index that should be removed
 * @param[in] numBytesIn the number of bytes following 'indexIn' that should be removed
 *
 * @return true if the remove was successful, false if the weren't enough bytes in the buffer. If
 * 		the remove fails, _NO_ bytes should have been removed from the buffer
 */
bool cxa_fixedByteBuffer_remove(cxa_fixedByteBuffer_t *const fbbIn, const size_t indexIn, const size_t numBytesIn);


/**
 * @public
 * @brief Removes a NULL-terminated 'cString' which starts at the specified index (moving
 * 		all following elements down)
 *
 * @param[in] fbbIn pointer to the pre-initialized cxa_fixedByteBuffer_t object
 * @param[in] indexIn the index of the start of the 'cString'
 *
 * @return true if the remove was successful, false if the length of the supposed string exceeds
 * 		the size of the fixedByteBuffer (eg. the string wasn't properly terminated)
 */
bool cxa_fixedByteBuffer_remove_cString(cxa_fixedByteBuffer_t *const fbbIn, const size_t indexIn);


/**
 * @public
 * @brief Returns a pointer to the specified index, contained within the fixedByteBuffer
 *
 * @param[in] fbbIn pointer to the pre-initialized cxa_fixedByteBuffer_t object
 * @param[in] the index of the desired element in the fixedByteBuffer
 *
 * @return pointer to the specified element/index, NULL if the index is out-of-bounds.
 */
uint8_t* cxa_fixedByteBuffer_get_pointerToIndex(cxa_fixedByteBuffer_t *const fbbIn, const size_t indexIn);
bool cxa_fixedByteBuffer_get(cxa_fixedByteBuffer_t *const fbbIn, const size_t indexIn, bool transposeIn, uint8_t *const valOut, const size_t numBytesIn);
bool cxa_fixedByteBuffer_get_cString(cxa_fixedByteBuffer_t *const fbbIn, const size_t indexIn, char *const stringOut, size_t maxOutputSize_bytes);
bool cxa_fixedByteBuffer_get_cString_inPlace(cxa_fixedByteBuffer_t *const fbbIn, const size_t indexIn, char** stringOut, size_t *strLen_bytesOut);
bool cxa_fixedByteBuffer_get_lengthPrefixedField_uint16BE(cxa_fixedByteBuffer_t *const fbbIn, const size_t indexIn, uint8_t** ptrOut, uint16_t* numBytesOut);
bool cxa_fixedByteBuffer_get_lengthPrefixedCString_uint16BE(cxa_fixedByteBuffer_t *const fbbIn, const size_t indexIn, char** ptrOut, uint16_t* numBytesOut, bool *isNullTerminatedOut);

bool cxa_fixedByteBuffer_replace(cxa_fixedByteBuffer_t *const fbbIn, const size_t indexIn, uint8_t *const ptrIn, const size_t numBytesIn);
bool cxa_fixedByteBuffer_replace_cString(cxa_fixedByteBuffer_t *const fbbIn, const size_t indexIn, char *const stringIn);

bool cxa_fixedByteBuffer_insert(cxa_fixedByteBuffer_t *const fbbIn, const size_t indexIn, uint8_t *const ptrIn, const size_t numBytesIn);

/**
 * @public
 * @brief Returns a pointer to the starting address of the data store for this buffer.
 * 		  Commonly used by logging functions.
 *
 * @param[in] fbbIn pointer to the pre-initialized cxa_fixedByteBuffer_t object
 *
 * @return pointer to the data store for this buffer
 */
uint8_t* cxa_fixedByteBuffer_get_pointerToStartOfData(cxa_fixedByteBuffer_t *const fbbIn);

/**
 * @public
 * @brief Determines the current number of bytes in the buffer
 *
 * @param[in] fbbIn pointer to the pre-initialized fixedByteBuffer object
 *
 * @return the number of bytes in the given buffer
 */
size_t cxa_fixedByteBuffer_getSize_bytes(cxa_fixedByteBuffer_t *const fbbIn);


/**
 * @public
 * @brief Determines the maximum number of bytes that can be contained within this buffer
 *
 * @param[in] fbbIn pointer to the pre-initialized fixedByteBuffer object
 *
 * @return the maximum number of bytes that can be contained in this buffer
 */
size_t cxa_fixedByteBuffer_getMaxSize_bytes(cxa_fixedByteBuffer_t *const fbbIn);


/**
 * @public
 * @brief Determines the number of unused bytes remaining in the buffer
 *
 * @param[in] fbbIn pointer to the pre-initialized fixedByteBuffer object
 *
 * @return the number of unused/free bytes in the buffer
 */
size_t cxa_fixedByteBuffer_getFreeSize_bytes(cxa_fixedByteBuffer_t *const fbbIn);


/**
 * @public
 * @brief Determines if this buffer is currently full (::cxa_fixedByteBuffer_getCurrSize == ::cxa_fixedByteBuffer_getMaxSize)
 *
 * @param[in] fbbIn pointer to the pre-initialized fixedByteBuffer object
 *
 * @return true if this buffer is full, false if not
 */
bool cxa_fixedByteBuffer_isFull(cxa_fixedByteBuffer_t *const fbbIn);


/**
 * @public
 * @brief Determines if this buffer is currently empty (no bytes)
 *
 * @param[in] fbbIn pointer to the pre-initialized fixedByteBuffer object
 *
 * @return true if there are no bytes in this buffer, false if there are
 */
bool cxa_fixedByteBuffer_isEmpty(cxa_fixedByteBuffer_t *const fbbIn);


/**
 * @public
 * @brief Clears the buffer, discarding any elements current contained
 * within it. This function will result in an empty buffer, but the
 * underlying memory may still contain the original data.
 *
 * @param[in] fbbIn pointer to the pre-initialized fixedByteBuffer object
 */
void cxa_fixedByteBuffer_clear(cxa_fixedByteBuffer_t *const fbbIn);


#endif // CXA_FIXED_BYTE_BUFFER_H_
