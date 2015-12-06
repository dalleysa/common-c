/**
 * @file
 * @copyright 2015 opencxa.org
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
#ifndef CXA_MQTT_RPC_NODE_BRIDGE_H_
#define CXA_MQTT_RPC_NODE_BRIDGE_H_


// ******** includes ********
#include <cxa_array.h>
#include <cxa_ioStream.h>
#include <cxa_logger_header.h>
#include <cxa_mqtt_rpc_node.h>
#include <cxa_protocolParser_mqtt.h>
#include <cxa_config.h>


// ******** global macro definitions ********
#ifndef CXA_MQTT_RPC_NODE_BRIDGE_CLIENTID_MAXLEN_BYTES
	#define CXA_MQTT_RPC_NODE_BRIDGE_CLIENTID_MAXLEN_BYTES			17
#endif
#ifndef CXA_MQTT_RPC_NODE_BRIDGE_MAPPEDNAME_MAXLEN_BYTES
	#define CXA_MQTT_RPC_NODE_BRIDGE_MAPPEDNAME_MAXLEN_BYTES		9
#endif



// ******** global type definitions *********
typedef struct cxa_mqtt_rpc_node_bridge cxa_mqtt_rpc_node_bridge_t;

/**
 * @public
 */
typedef char* (*cxa_mqtt_rpc_node_bridge_cb_authenticateClient_t)(char *const clientIdIn, size_t clientIdLen_bytes,
																	char *const usernameIn, size_t usernameLen_bytesIn,
																	uint8_t *const passwordIn, size_t passwordLen_bytesIn,
																	void *userVarIn);


/**
 * @private
 */
struct cxa_mqtt_rpc_node_bridge
{
	cxa_mqtt_rpc_node_t super;

	bool isSingle;
	cxa_protocolParser_mqtt_t mpp;

	cxa_mqtt_rpc_node_bridge_cb_authenticateClient_t cb_auth;
	void* userVar_auth;
};


// ******** global function prototypes ********
/**
 * @protected
 */
void cxa_mqtt_rpc_node_bridge_init(cxa_mqtt_rpc_node_bridge_t *const nodeIn, cxa_mqtt_rpc_node_t *const parentNodeIn, char *const nameIn,
										 cxa_ioStream_t *const iosIn, cxa_timeBase_t *const timeBaseIn,
										 cxa_mqtt_rpc_node_bridge_cb_authenticateClient_t cb_authIn, void* authCbUserVarIn);

void cxa_mqtt_rpc_node_bridge_update(cxa_mqtt_rpc_node_bridge_t *const nodeIn);

#endif // CXA_MQTT_RPC_NODEBRIDGE_H_
