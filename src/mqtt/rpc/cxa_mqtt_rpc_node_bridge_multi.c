/**
 * Copyright 2015 opencxa.org
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Christopher Armenio
 */
#include "cxa_mqtt_rpc_node_bridge_multi.h"


// ******** includes ********
#include <string.h>
#include <cxa_assert.h>
#include <cxa_mqtt_messageFactory.h>
#include <cxa_mqtt_message_connect.h>
#include <cxa_mqtt_message_connack.h>
#include <cxa_mqtt_message_pingResponse.h>
#include <cxa_mqtt_message_publish.h>
#include <cxa_mqtt_rpc_node_root.h>
#include <cxa_stringUtils.h>

#define CXA_LOG_LEVEL		CXA_LOG_LEVEL_TRACE
#include <cxa_logger_implementation.h>


// ******** local macro definitions ********


// ******** local type definitions ********


// ******** local function prototypes ********
static cxa_mqtt_rpc_node_bridge_authorization_t bridgeAuthCb(char *const clientIdIn, size_t clientIdLen_bytes,
															 char *const usernameIn, size_t usernameLen_bytesIn,
															 uint8_t *const passwordIn, size_t passwordLen_bytesIn,
															 void *userVarIn);

static bool rpcCb_catchall(cxa_mqtt_rpc_node_t *const nodeIn, char *const remainingTopicIn, size_t remainingTopicLen_bytes, cxa_mqtt_message_t *const msgIn, void* userVarIn);
static void scm_handlePublish(cxa_mqtt_rpc_node_bridge_t *const superIn, cxa_mqtt_message_t *const msgIn);


// ********  local variable declarations *********


// ******** global function implementations ********
void cxa_mqtt_rpc_node_bridge_multi_init(cxa_mqtt_rpc_node_bridge_multi_t *const nodeIn, cxa_mqtt_rpc_node_t *const parentNodeIn,
										 cxa_protocolParser_mqtt_t *const mppIn,
										 const char *nameFmtIn, ...)
{
	cxa_assert(nodeIn);
	cxa_assert(parentNodeIn);
	cxa_assert(mppIn);
	cxa_assert(nameFmtIn);

	// initialize our super class
	va_list varArgs;
	va_start(varArgs, nameFmtIn);
	cxa_mqtt_rpc_node_bridge_vinit(&nodeIn->super, parentNodeIn, mppIn, scm_handlePublish, bridgeAuthCb, (void*)nodeIn, nameFmtIn, varArgs);
	va_end(varArgs);
	cxa_mqtt_rpc_node_setCatchAll(&nodeIn->super.super, rpcCb_catchall, (void*)nodeIn);

	// set some defaults
	nodeIn->cb_localAuth = NULL;
	nodeIn->localAuthUserVar = NULL;

	// setup our remote nodes
	cxa_array_initStd(&nodeIn->remoteNodes, nodeIn->remoteNodes_raw);
}


void cxa_mqtt_rpc_node_bridge_multi_setAuthCb(cxa_mqtt_rpc_node_bridge_multi_t *const nodeIn, cxa_mqtt_rpc_node_bridge_multi_cb_authenticateClient_t authCbIn, void *const userVarIn)
{
	cxa_assert(nodeIn);

	nodeIn->cb_localAuth = authCbIn;
	nodeIn->localAuthUserVar = userVarIn;
}


void cxa_mqtt_rpc_node_bridge_multi_clearRemoteNodes(cxa_mqtt_rpc_node_bridge_multi_t *const nodeIn)
{
	cxa_assert(nodeIn);

	cxa_array_clear(&nodeIn->remoteNodes);
}


// ******** local function implementations ********
static cxa_mqtt_rpc_node_bridge_authorization_t bridgeAuthCb(char *const clientIdIn, size_t clientIdLen_bytes,
															 char *const usernameIn, size_t usernameLen_bytesIn,
															 uint8_t *const passwordIn, size_t passwordLen_bytesIn,
															 void *userVarIn)
{
	cxa_mqtt_rpc_node_bridge_multi_t* nodeIn = (cxa_mqtt_rpc_node_bridge_multi_t*)userVarIn;
	cxa_assert(nodeIn);

	// we need an authorization callback first...
	if( nodeIn->cb_localAuth == NULL ) return CXA_MQTT_RPC_NODE_BRIDGE_AUTH_IGNORE;

	// check out our current remote nodes to see if someone is connecting again (reboot maybe?)
	cxa_array_iterate(&nodeIn->remoteNodes, currRemoteNode, cxa_mqtt_rpc_node_bridge_multi_remoteNodeEntry_t)
	{
		if( currRemoteNode ) continue;

		if( cxa_stringUtils_equals(currRemoteNode->clientId, clientIdIn) )
		{
			cxa_logger_log_untermString(&nodeIn->super.super.logger, CXA_LOG_LEVEL_DEBUG, "reauth attempt for '", clientIdIn, clientIdLen_bytes, "'");
			return CXA_MQTT_RPC_NODE_BRIDGE_AUTH_ALLOW;
		}
	}

	// make sure we have space
	if( cxa_array_isFull(&nodeIn->remoteNodes) )
	{
		cxa_logger_warn(&nodeIn->super.super.logger, "too many remote dropping");
		return CXA_MQTT_RPC_NODE_BRIDGE_AUTH_IGNORE;
	}

	// get our new entry ready to record the remote client
	cxa_mqtt_rpc_node_bridge_multi_remoteNodeEntry_t newEntry;
	newEntry.clientId[0] = 0;
	newEntry.mappedName[0] = 0;

	// make sure the client ID OK
	if( clientIdLen_bytes >= sizeof(newEntry.clientId) )
	{
		cxa_logger_warn(&nodeIn->super.super.logger, "remote clientId too long");
		return CXA_MQTT_RPC_NODE_BRIDGE_AUTH_IGNORE;
	}
	cxa_stringUtils_concat_withLengths(newEntry.clientId, sizeof(newEntry.clientId), clientIdIn, clientIdLen_bytes);

	// call _our_ authorization callback
	cxa_mqtt_rpc_node_bridge_authorization_t retVal = nodeIn->cb_localAuth(clientIdIn, clientIdLen_bytes,
																		   usernameIn, usernameLen_bytesIn,
																		   passwordIn, passwordLen_bytesIn,
																		   newEntry.mappedName, sizeof(newEntry.mappedName),
																		   nodeIn->localAuthUserVar);
	if( retVal != CXA_MQTT_RPC_NODE_BRIDGE_AUTH_ALLOW ) return retVal;

	// if we made it here, we are allowing it (assuming we have space)
	cxa_assert( cxa_array_append(&nodeIn->remoteNodes, &newEntry) );

	return CXA_MQTT_RPC_NODE_BRIDGE_AUTH_ALLOW;
}


static bool rpcCb_catchall(cxa_mqtt_rpc_node_t *const superIn, char *const remainingTopicIn, size_t remainingTopicLen_bytes, cxa_mqtt_message_t *const msgIn, void* userVarIn)
{
	cxa_mqtt_rpc_node_bridge_multi_t* nodeIn = (cxa_mqtt_rpc_node_bridge_multi_t*)superIn;
	cxa_assert(nodeIn);

	cxa_logger_log_untermString(&nodeIn->super.super.logger, CXA_LOG_LEVEL_TRACE, "catchall: '", remainingTopicIn, remainingTopicLen_bytes, "'");

	// iterate through our mapped nodes to see if we match a node we know...
	cxa_array_iterate(&nodeIn->remoteNodes, currRemNode, cxa_mqtt_rpc_node_bridge_multi_remoteNodeEntry_t)
	{
		if( currRemNode == NULL ) continue;

		size_t currMappedNameLen_bytes = strlen(currRemNode->mappedName);
		if( (currMappedNameLen_bytes <= remainingTopicLen_bytes) && cxa_stringUtils_startsWith(remainingTopicIn, currRemNode->mappedName) )
		{
			// we have a match!
			cxa_logger_trace(&nodeIn->super.super.logger, "found match: '%s'", currRemNode->mappedName);

			// advance to the end of our mapped name (should be a path separator)
			char* newTopicName = remainingTopicIn + currMappedNameLen_bytes;
			size_t newTopicNameLen_bytes = remainingTopicLen_bytes - currMappedNameLen_bytes;

			// now, we need to massage the topic of this message too look something like:
			// <foo's clientId>/::bar

			if( (newTopicNameLen_bytes < 1) || (*newTopicName != '/') ||
					!cxa_mqtt_message_publish_topicName_trimToPointer(msgIn, newTopicName) ||
					!cxa_mqtt_message_publish_topicName_prependCString(msgIn, currRemNode->clientId) )
			{
				cxa_logger_warn(&nodeIn->super.super.logger, "error remapping topic name, dropping");
				return true;		// return true because we _should_ have handled this
			}

			// send the message
			if( !cxa_protocolParser_writePacket(&nodeIn->super.mpp->super, cxa_mqtt_message_getBuffer(msgIn)) )
			{
				cxa_logger_warn(&nodeIn->super.super.logger, "error forwarding message");
			}

			return true;
		}
	}

	// if we made it here, we found no matching remote nodes
	cxa_logger_log_untermString(&nodeIn->super.super.logger, CXA_LOG_LEVEL_WARN, "couldn't handle '", remainingTopicIn, remainingTopicLen_bytes, "'");
	return false;
}


static void scm_handlePublish(cxa_mqtt_rpc_node_bridge_t *const superIn, cxa_mqtt_message_t *const msgIn)
{
	cxa_mqtt_rpc_node_bridge_multi_t* nodeIn = (cxa_mqtt_rpc_node_bridge_multi_t*)superIn;
	cxa_assert(nodeIn);
	cxa_assert(msgIn);

	// get our topic name and length
	char* topicName;
	uint16_t topicLen_bytes;
	if( !cxa_mqtt_message_publish_getTopicName(msgIn, &topicName, &topicLen_bytes) ) return;
	cxa_logger_log_untermString(&nodeIn->super.super.logger, CXA_LOG_LEVEL_TRACE, "got PUBLISH '", topicName, topicLen_bytes, "'");

	// ensure that our publish topic actually matches one of our remote nodes
	cxa_mqtt_rpc_node_bridge_multi_remoteNodeEntry_t* targetRne = NULL;
	cxa_array_iterate(&nodeIn->remoteNodes, currRemNode, cxa_mqtt_rpc_node_bridge_multi_remoteNodeEntry_t)
	{
		if( currRemNode == NULL ) continue;

		if( cxa_stringUtils_contains_withLengths(topicName, topicLen_bytes, currRemNode->clientId, strlen(currRemNode->clientId)) )
		{
			targetRne = currRemNode;
			break;
		}
	}
	if( targetRne == NULL ) return;

	// remember whether this is a request
	bool isResponse = cxa_stringUtils_startsWith(topicName, CXA_MQTT_RPCNODE_RESP_PREFIX);

	// we'll need to do some unmapping to get the topic name inline with our node structure
	char* startOfClientId;
	if( (startOfClientId = strstr(topicName, targetRne->clientId)) == NULL ) return;

	// ok...get rid of everything up to, and including, the clientId (+1 is for separator)
	if( !cxa_mqtt_message_publish_topicName_trimToPointer(msgIn, startOfClientId+strlen(targetRne->clientId)+1) ) return;

	// now we need to prepend our node structure
	cxa_mqtt_rpc_node_t* currNode = &nodeIn->super.super;
	while( currNode != NULL )
	{
		if( !cxa_mqtt_message_publish_topicName_prependCString(msgIn, "/") ||
			!cxa_mqtt_message_publish_topicName_prependCString(msgIn, currNode->name) ) return;

		// if we're the root node, we need to prepend our root prefix (if it exists)
		if( currNode->isRootNode )
		{
			if( !cxa_mqtt_message_publish_topicName_prependCString(msgIn, cxa_mqtt_rpc_node_root_getPrefix((cxa_mqtt_rpc_node_root_t*)currNode)) ) return;
		}
		currNode = currNode->parentNode;
	}

	// if we made it here, we should have a proper topic name (potentially without the response prefix)
	if( isResponse && !cxa_mqtt_message_publish_topicName_prependCString(msgIn, CXA_MQTT_RPCNODE_RESP_PREFIX) ) return;

	// now that everything is unmapped...toss to the root node for handling
	cxa_mqtt_rpc_node_root_t* rootNode = cxa_mqtt_rpc_node_getRootNode(&nodeIn->super.super);
	if( rootNode != NULL ) cxa_mqtt_rpc_node_root_handleInternalPublish(rootNode, msgIn);
}
