/**
 * @file
 *
 * @note This object should work across all architecture-specific implementations
 *
 *
 * #### Example Usage: ####
 *
 * @code
 * @endcode
 *
 *
 * @copyright 2017 opencxa.org
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Christopher Armenio
 */
#ifndef CXA_NVS_MANAGER_H_
#define CXA_NVS_MANAGER_H_


// ******** includes ********
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// ******** global macro definitions ********


// ******** global type definitions *********


// ******** global function prototypes ********
bool cxa_nvsManager_doesKeyExist(const char *const keyIn);

bool cxa_nvsManager_get_cString(const char *const keyIn, char *const valueOut, size_t maxOutputSize_bytes);
bool cxa_nvsManager_set_cString(const char *const keyIn, char *const valueIn);

bool cxa_nvsManager_get_uint32(const char *const keyIn, uint32_t *const valueOut);
bool cxa_nvsManager_set_uint32(const char *const keyIn, uint32_t valueIn);

bool cxa_nvsManager_get_blob(const char *const keyIn, uint8_t *const valueOut, size_t maxOutputSize_bytesIn, size_t *const actualOutputSize_bytesOut);
bool cxa_nvsManager_set_blob(const char *const keyIn, uint8_t *const valueIn, size_t blobSize_bytesIn);

bool cxa_nvsManager_erase(const char *const keyIn);

bool cxa_nvsManager_commit(void);


#endif /* CXA_NVS_MANAGER_H_ */
