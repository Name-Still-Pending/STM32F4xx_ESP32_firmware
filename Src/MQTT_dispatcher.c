/*
 * MQTT_dispatcher.c
 *
 *  Created on: Dec 29, 2023
 *      Author: poti
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpointer-sign"


#include "MQTT_dispatcher.h"
#include "esp32_manager.h"
#include <string.h>
#include "message_buffer.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"
#include <stdlib.h>
#include "portable.h"
#include <stdio.h>
#include <errno.h>

//
// private defines
//

#define MQTT_FLAG_INITIALIZED	(1 << 0)
#define MQTT_FLAG_CONNECTED		(1 << 1)
#define MQTT_FLAG_FREE			(1 << 2)
#define MQTT_FLAG_PUB_OK			(1 << 3)
#define MQTT_FLAG_PUB_FAIL		(1 << 4)

#define TOPIC_BITFIELD_SIZE BITFIELD_SIZE(MQTT_MAX_TOPICS)
#define SUBSCRIBER_BITFIELD_SIZE BITFIELD_SIZE(MQTT_MAX_SUBSCRIBERS)
#define SET_DEADLINE(timeout) TickType_t deadline = (timeout) + xTaskGetTickCount()
#define GET_TIMEOUT (deadline - xTaskGetTickCount())

//
// private typedefs
//

typedef struct{
	int8_t *name;
	uint8_t subscriber_bitfield[SUBSCRIBER_BITFIELD_SIZE];
}MqttTopic_t;

typedef struct{
	int32_t id;
	QueueHandle_t queue;
	uint8_t topic_bitfield[TOPIC_BITFIELD_SIZE];
	uint32_t flags;
}MqttSubscriber_t;



//
// private function prototypes
//

static void mqtt_dispatcher_init_task(void*);
static void mqtt_dispatcher_task(void*);
static Response_t mqtt_redirect_callback(uint8_t *msg, int16_t len);
static MqttResponse_t parse_input(uint8_t *msg, int16_t len);

static BaseType_t subscriber_init(MqttSubscriber_t *handle, int32_t id, uint32_t flags);
static BaseType_t subscriber_deinit(MqttSubscriber_t *handle);
static MqttResponse_t topic_init(MqttTopic_t *handle, const int8_t *name, int16_t qos, TickType_t timeout);
static MqttResponse_t topic_deinit(MqttTopic_t *handle, TickType_t timeout);
static BaseType_t topic_has_subscribers(MqttTopic_t *handle);
static BaseType_t topic_set_subscriber(MqttTopic_t *topic, MqttSubscriber_t *sub, uint8_t v);
static BaseType_t subscriber_set_topic(MqttSubscriber_t *sub, MqttTopic_t *topic, uint8_t v);
static MqttTopic_t *get_topic(const int8_t *name);
static MqttSubscriber_t *get_next_topic_subscriber(MqttTopic_t *topic);
static BaseType_t parse_message(uint8_t *msg, int16_t len, MqttMessage_t *buffer);
static void subscriber_send_message(MqttSubscriber_t *sub, MqttMessage_t *msg);


//
// private variables
//

static StaticEventGroup_t 	mqtt_flags_static;
static EventGroupHandle_t 	mqtt_flags_handle;
static MessageBufferHandle_t	mqtt_in_message_buffer_handle;

static TaskHandle_t mqtt_dispatcher_handle;
static TaskHandle_t mqtt_dispatcher_init_handle;

static STATIC_MESSAGE_REDIRECT_STRUCT(mqtt_redirect, "MQTT", mqtt_redirect_callback);

// message dispatch variables

static StaticSemaphore_t	subMutex_static;
static SemaphoreHandle_t	subMutex_handle;

static MqttTopic_t			topics[MQTT_MAX_TOPICS]				= {0};
static MqttSubscriber_t		subscribers[MQTT_MAX_SUBSCRIBERS];

//
// public function definitions
//

BaseType_t mqtt_init(){
#define NULL_CHECK(handle)if(!handle) return pdFAIL;
	mqtt_flags_handle 				= xEventGroupCreateStatic(&mqtt_flags_static);
	NULL_CHECK(mqtt_flags_handle)
	mqtt_in_message_buffer_handle 	= xMessageBufferCreate(MQTT_IN_BUFFER_SIZE);
	NULL_CHECK(mqtt_in_message_buffer_handle)
	subMutex_handle					= xSemaphoreCreateMutexStatic(&subMutex_static);
	NULL_CHECK(subMutex_handle)

	for(int32_t i = 0; i < MQTT_MAX_SUBSCRIBERS; ++i){
		subscribers[i].id = -1;
		subscribers[i].queue = NULL;
	}

	BaseType_t resp;
	resp = xTaskCreate(mqtt_dispatcher_task, "MQTT_dispatcher", 1024, NULL, 2, &mqtt_dispatcher_handle);
	if(resp != pdPASS) return resp;
	resp = xTaskCreate(mqtt_dispatcher_init_task, "MQTT_dispatcher_init", 256, NULL, 1, &mqtt_dispatcher_init_handle);
	if(resp != pdPASS) return resp;

	return pdPASS;
}

MqttResponse_t mqtt_pub(uint8_t *topic, uint8_t *msg, uint8_t qos, uint8_t retain, TickType_t timeout){
	SET_DEADLINE(timeout);
	EventBits_t flags = xEventGroupWaitBits(mqtt_flags_handle, MQTT_FLAG_CONNECTED, pdFALSE, pdTRUE, timeout);
	if(!(flags & MQTT_FLAG_CONNECTED)) return MQTT_TIMEOUT_NOCONN;

	uint8_t cmdBuf[256] = {0};
	size_t len = sprintf(cmdBuf, "AT+MQTTPUB=0,\"%s\",\"%s\",%d,%d\r\n", topic, msg, qos, retain);
	Response_t esp32Resp = esp32_command(cmdBuf, len, AT_RESP_FINAL, GET_TIMEOUT);
	if(esp32Resp == AT_SEND_TIMEOUT) return MQTT_TIMEOUT_BUSY;

	return esp32Resp == AT_RESP_OK ? MQTT_OK : MQTT_PUB_FAIL;
}

MqttResponse_t mqtt_pub_raw(uint8_t *topic, void *msg, uint8_t len, uint8_t qos, uint8_t retain, TickType_t timeout){
	SET_DEADLINE(timeout);
	EventBits_t flags = xEventGroupWaitBits(mqtt_flags_handle, MQTT_FLAG_CONNECTED, pdFALSE, pdTRUE, timeout);
	if(!(flags & MQTT_FLAG_CONNECTED)) return MQTT_TIMEOUT_NOCONN;

	int8_t cmdBuf[256] = {0};
	int32_t cmdLen = sprintf(cmdBuf, "AT+MQTTPUBRAW=0,\"%s\",%d,%d,%d\r\n", topic, len, (int16_t)qos, retain);

	Response_t esp32Resp = esp32_command_long(cmdBuf, cmdLen, msg, len, GET_TIMEOUT);
	if(esp32Resp != AT_RESP_OK)
		return MQTT_PUB_FAIL;

	return MQTT_OK;
}

MqttResponse_t mqtt_add_subscriber(MqttSubHandle_t *handle, uint32_t flags, TickType_t timeout){
	if(!xSemaphoreTake(subMutex_handle, timeout)) return MQTT_TIMEOUT_BUSY;
	MqttResponse_t retval;

	do{
		// find first available subscriber spot
		int32_t subIndex = -1;
		for(int32_t i = 0; i < MQTT_MAX_SUBSCRIBERS; ++i)
			if(subscribers[i].id < 0){
				subIndex = i;
				break;
			}
		if(subIndex < 0){
			retval = MQTT_SUB_FAIL_TOO_MANY_SUBS;
			break;
		}

		if(!subscriber_init(&subscribers[subIndex], subIndex, flags)){
			retval = MQTT_SUB_FAIL;
			break;
		}
		*handle = &subscribers[subIndex];

	}while(0);

	xSemaphoreGive(subMutex_handle);
	return retval;
}

MqttResponse_t mqtt_remove_subscriber(MqttSubHandle_t *handle, TickType_t timeout){
	if(!xSemaphoreTake(subMutex_handle, timeout)) return MQTT_TIMEOUT_BUSY;
	MqttResponse_t retval = MQTT_OK;

	do{
		retval = subscriber_deinit((MqttSubscriber_t*)*handle);
		if(retval != MQTT_OK) break;

		*handle = NULL;

	}while(0);

	xSemaphoreGive(subMutex_handle);
	return retval;
}

MqttResponse_t mqtt_subscribe_topic(MqttSubHandle_t handle, MqttTopicHandle_t *topicHandle, const int8_t *topic, TickType_t timeout){
	SET_DEADLINE(timeout);
	if(!xSemaphoreTake(subMutex_handle, timeout)) return MQTT_TIMEOUT_BUSY;


	MqttResponse_t retval = MQTT_OK;
	MqttTopic_t *topHandle = get_topic(topic);

	do{
		if(topHandle == NULL){
			int32_t i = 0;
			for(; i < MQTT_MAX_TOPICS; ++i){
				if(topics[i].name == NULL) break;
			}
			if(i == MQTT_MAX_TOPICS) {
				retval = MQTT_SUB_FAIL_TOPICS_FULL;
				break;
			}

			retval = topic_init(&topics[i], topic, MQTT_QOS, GET_TIMEOUT);
			if(retval != MQTT_OK) break;

			topHandle = &topics[i];
		}

		MqttSubscriber_t *subHandle = (MqttSubscriber_t*)handle;

		topic_set_subscriber(topHandle, subHandle, 1);
		subscriber_set_topic(subHandle, topHandle, 1);
	}while(0);

	xSemaphoreGive(subMutex_handle);
	if(topicHandle != NULL && retval == MQTT_OK) *topicHandle = topHandle;
	return retval;
}

MqttResponse_t mqtt_unsubscribe_topic(MqttSubHandle_t handle, MqttTopicHandle_t *topic, TickType_t timeout){
	SET_DEADLINE(timeout);
	if(!xSemaphoreTake(subMutex_handle, timeout)) return MQTT_TIMEOUT_BUSY;

	MqttResponse_t retval = MQTT_OK;
	MqttSubscriber_t *subHandle = (MqttSubscriber_t*)handle;
	MqttTopic_t *topHandle = (MqttTopic_t*)*topic;

	do{
		if(!topic_set_subscriber(topHandle, subHandle, 0)){
			retval = MQTT_UNSUB_FAIL_BAD_TOPIC;
			break;
		}

		if(!topic_has_subscribers(topHandle)){
			retval = topic_deinit(topHandle, GET_TIMEOUT);
			if(retval) {
				topic_set_subscriber(topHandle, subHandle, 1);
				break;
			}
		}

		subscriber_set_topic(subHandle, topHandle, 0);
		*topic = NULL;
	}while(0);

	xSemaphoreGive(subMutex_handle);
	return retval;
}

inline BaseType_t mqtt_poll(MqttSubHandle_t handle, MqttMessage_t *message, TickType_t timeout){
	return xQueueReceive(((MqttSubscriber_t*)handle)->queue, message, timeout);
}


//
// private function definitions
//

static void mqtt_dispatcher_init_task(void*){
	SET_DEADLINE(pdMS_TO_TICKS(60000));
	Response_t resp = 0;
	BaseType_t error = 0;


#define OK_CHECK if(resp != AT_RESP_OK){ \
					error = 1;			\
					break;				\
				}						\

	do{
		if(!esp32_status_wait(STATUS_INITIALIZED, pdTRUE, GET_TIMEOUT)){
			error = 1;
			break;
		}

		if(!esp32_capture(GET_TIMEOUT)){
			error = 1;
			break;
		}

		BaseType_t rResp = esp32_add_redirect_handler(&mqtt_redirect, GET_TIMEOUT);
		if(rResp != MSGRED_OP_OK){
			error = 1;
			break;
		}

		// set MQTT user config
		resp = AT_command_static("AT+MQTTUSERCFG=0,1,\"" MQTT_CLIENT_NAME "\",\"" MQTT_LOGIN_USERNAME "\",\"" MQTT_LOGIN_PASSWORD "\",0,0,\"\"\r\n", AT_RESP_FINAL, GET_TIMEOUT);
		OK_CHECK;

		// set MQTT conn config
		resp = AT_command_static("AT+MQTTCONNCFG=0," xstr(MQTT_KEEPALIVE) ",0,\"" MQTT_LWT_TOPIC "\",\"" MQTT_LWT_MESSAGE "\",2,0\r\n", AT_RESP_FINAL, GET_TIMEOUT);
		OK_CHECK;

		// MQTT connect
		do{
			resp = AT_command_static("AT+MQTTCONN=0,\"" MQTT_BROKER_IP "\"," xstr(MQTT_BROKER_PORT) ",1\r\n", AT_RESP_FINAL, GET_TIMEOUT);
		}while(resp == AT_RESP_ERROR);
		OK_CHECK;
	}while(0);

#undef OK_CHECK

	esp32_release();

	while(error){
		vTaskDelay(pdMS_TO_TICKS(1000));
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	}
	xEventGroupSetBits(mqtt_flags_handle, MQTT_FLAG_INITIALIZED);
	vTaskDelay(pdMS_TO_TICKS(1000));

	//TODO Fix hard fault on task delete
#if 1
	vTaskSuspend(NULL);
#else
	vTaskDelete(NULL);
#endif
}

static void mqtt_dispatcher_task(void*){
	size_t msgLen = 0;
	uint8_t msgBuf[MQTT_MAX_MESSAGE_LEN] = {0};

	while(1){
		msgLen = 0;
		do{
			msgLen = xMessageBufferReceive(mqtt_in_message_buffer_handle, msgBuf, MQTT_MAX_MESSAGE_LEN, pdMS_TO_TICKS(1000));
		}while(!msgLen);

		// parse message
		MqttResponse_t mqttResp = parse_input(msgBuf, msgLen);
		MqttMessage_t buffer;
		MqttSubscriber_t *sub;

		switch(mqttResp){
		case MQTT_CONNECTED:
			xEventGroupSetBits(mqtt_flags_handle, MQTT_FLAG_CONNECTED);
			break;
		case MQTT_DISCONNECTED:
			xEventGroupClearBits(mqtt_flags_handle, MQTT_FLAG_CONNECTED);
			break;
		case MQTT_PUB_OK:
			esp32_confirm_transmission(pdTRUE);
			break;
		case MQTT_PUB_FAIL:
			esp32_confirm_transmission(pdFALSE);
			break;
		case MQTT_SUBRECV:
			if(!xSemaphoreTake(subMutex_handle, pdMS_TO_TICKS(10000))) break;
			if(parse_message(msgBuf, msgLen, &buffer)){
				while((sub = get_next_topic_subscriber((MqttTopic_t*)buffer.topic)) != NULL){
					subscriber_send_message(sub, &buffer);
				}
			}
			xSemaphoreGive(subMutex_handle);
			break;
		default:
		}
	}

	vTaskDelete(NULL);
}

static inline Response_t mqtt_redirect_callback(uint8_t *msg, int16_t len){
	if(!xMessageBufferSend(mqtt_in_message_buffer_handle, msg, len + 1, pdMS_TO_TICKS(500))) return OTHR_REDIRECT_TIMEOUT;

	return OTHR_REDIRECT_DONE;
}

static MqttResponse_t parse_input(uint8_t *msg, int16_t len){
	if(len >= 6 && !strncmp(msg, "PUB:", 4)){
		if(!strncmp(&msg[4], "OK", 2))					return MQTT_PUB_OK;
		if(len >= 8 && !strncmp(&msg[4], "FAIL", 4))	return MQTT_PUB_FAIL;
														return MQTT_UNKNOWN;
	}
	if(len >= 9 && !strncmp(msg, "CONNECTED", 9))		return MQTT_CONNECTED;
	if(len >= 12 && !strncmp(msg, "DISCONNECTED", 12))  return MQTT_DISCONNECTED;
	if(len >= 7 && !strncmp(msg, "SUBRECV", 7))			return MQTT_SUBRECV;

														return MQTT_UNKNOWN;
}

static BaseType_t subscriber_init(MqttSubscriber_t *handle, int32_t id, uint32_t flags){
	handle->queue = xQueueCreate(MQTT_MAX_MESSAGES_QUEUED, sizeof(MqttMessage_t));
	if(handle->queue == NULL) return pdFAIL;
	handle->id = id;
	handle->flags = flags;
	memset(handle->topic_bitfield, 0, TOPIC_BITFIELD_SIZE);
	return pdPASS;
}

static BaseType_t subscriber_deinit(MqttSubscriber_t *handle){
	vQueueDelete(handle->queue);
	handle->queue = NULL;
	handle->id = -1;
	handle->flags = MQTT_SUB_DEFAULT;
	memset(handle->topic_bitfield, 0, TOPIC_BITFIELD_SIZE);
	return pdPASS;
}

static MqttResponse_t topic_init(MqttTopic_t *handle, const int8_t *name, int16_t qos, TickType_t timeout){
	TickType_t deadline = xTaskGetTickCount() + timeout;

	int32_t nameLen = strlen(name);
	handle->name = (int8_t*) pvPortMalloc(nameLen + 1);
	if(handle->name == NULL) return MQTT_SUB_FAIL;

	uint8_t cmd[256];
	int len = sprintf(cmd, "AT+MQTTSUB=0,\"%s\",%d\r\n", name, qos);

	EventBits_t flags = xEventGroupWaitBits(mqtt_flags_handle, MQTT_FLAG_CONNECTED, pdFALSE, pdTRUE, deadline - xTaskGetTickCount());
	if(!(flags & MQTT_FLAG_CONNECTED)){
		vPortFree(handle->name);
		return MQTT_TIMEOUT_NOCONN;
	}

	Response_t resp = esp32_command(cmd, len, AT_RESP_UNKNOWN, deadline - xTaskGetTickCount());
	if(resp != AT_RESP_OK){
		vPortFree(handle->name);
		return MQTT_SUB_FAIL;
	}

	memset(handle->subscriber_bitfield, 0, SUBSCRIBER_BITFIELD_SIZE);
	strcpy(handle->name, name);

	return MQTT_OK;
}

static MqttResponse_t topic_deinit(MqttTopic_t *handle, TickType_t timeout){
	TickType_t deadline = xTaskGetTickCount() + timeout;
	if(topic_has_subscribers(handle)) return MQTT_SUB_FAIL;

	int8_t cmd[256];
	int len = sprintf(cmd, "AT+MQTTUNSUB=0\"%s\"\r\n", handle->name);

	EventBits_t flags = xEventGroupWaitBits(mqtt_flags_handle, MQTT_FLAG_CONNECTED, pdFALSE, pdTRUE, deadline - xTaskGetTickCount());
	if(!(flags & MQTT_FLAG_CONNECTED)){
		return MQTT_TIMEOUT_NOCONN;
	}

	Response_t resp = esp32_command(cmd, len, AT_RESP_UNKNOWN, deadline - xTaskGetTickCount());
	if(resp != AT_RESP_OK){
		return MQTT_SUB_FAIL;
	}

	vPortFree(handle->name);

	return MQTT_OK;
}

static BaseType_t topic_has_subscribers(MqttTopic_t *handle){
	for(int32_t i = 0; i < TOPIC_BITFIELD_SIZE; ++i) if(handle->subscriber_bitfield[i]) return pdTRUE;
	return pdFALSE;
}

static BaseType_t topic_set_subscriber(MqttTopic_t *topic, MqttSubscriber_t *sub, uint8_t v){
	int32_t i = ((uint32_t)subscribers - (uint32_t)sub) / sizeof(MqttSubscriber_t);
	div_t d = div(i, 8);
	if(!!v == !!(topic->subscriber_bitfield[d.quot] & (1 << d.rem)))
		return pdFALSE;
	if(v) topic->subscriber_bitfield[d.quot] |= (1 << d.rem);
	else  topic->subscriber_bitfield[d.quot] &= ~(1 << d.rem);

	return pdTRUE;
}

static BaseType_t subscriber_set_topic(MqttSubscriber_t *sub, MqttTopic_t *topic, uint8_t v){
	int32_t i = ((uint32_t)topics - (uint32_t)topic) / sizeof(MqttTopic_t);
	div_t d = div(i, 8);
	if(!!v == !!(sub->topic_bitfield[d.quot] & (1 << d.rem)))
		return pdFALSE;
	if(v) sub->topic_bitfield[d.quot] |= (1 << d.rem);
	else  sub->topic_bitfield[d.quot] &= ~(1 << d.rem);

	return pdTRUE;
}

static MqttTopic_t *get_topic(const int8_t *name){
	for(int32_t i = 0; i < MQTT_MAX_TOPICS; ++i){
		if(topics[i].name != NULL && !strcmp(name, topics[i].name)) return &topics[i];
	}
	return NULL;
}

static MqttSubscriber_t *get_next_topic_subscriber(MqttTopic_t *topic){
	static int16_t offset = 0, bi = 0;
	static uint8_t byte = 0;
	MqttSubscriber_t *retval = NULL;
	for(; bi < SUBSCRIBER_BITFIELD_SIZE && !retval; ++bi, offset = 0){
		if(!byte) byte = topic->subscriber_bitfield[bi];
		for(; byte && !retval; byte >>= 1, ++offset){
			if(byte & 1){
				retval = &subscribers[bi * 8 + offset];
			}
		}
	}
	if(!retval)	bi = offset = byte = 0;
	return retval;
}

static BaseType_t parse_message(uint8_t *msg, int16_t len, MqttMessage_t *buffer){
	buffer->topic = NULL;
	buffer->len = -1;
	buffer->value[0] = '\0';
	int8_t *tokens[4] = {NULL};
	int16_t tokenCount = 0;
	tokens[0] = strtok(&msg[11], "\"");
	while(tokens[tokenCount] && tokenCount < 3){
		if(tokens[tokenCount][0] != '\0')++tokenCount;
		tokens[tokenCount] = strtok(NULL, ",");
	}

	if(tokenCount != 3)
		return pdFALSE;

	if((buffer->topic = get_topic(tokens[0])) == NULL)
		return pdFALSE;

	buffer->len = strtol(tokens[1], NULL, 0);
	if(!buffer->len || errno == ERANGE){
		errno = 0;
		return pdFALSE;
	}

	memcpy(buffer->value, tokens[2], buffer->len);
	memset(&buffer->value[buffer->len], 0, MQTT_MAX_MESSAGE_LEN - buffer->len);

	return pdTRUE;
}

static void subscriber_send_message(MqttSubscriber_t *sub, MqttMessage_t *msg){
	if(uxQueueMessagesWaiting(sub->queue) == MQTT_MAX_MESSAGES_QUEUED){
		MqttMessage_t dummyBuf;
		if(sub->flags & MQTT_SUB_ON_QUEUE_FULL_DROP_OLD)	xQueueReceive(sub->queue, &dummyBuf, 0);
		else return;
	}
	xQueueSend(sub->queue, msg, 0);
}


#pragma GCC diagnostic pop
