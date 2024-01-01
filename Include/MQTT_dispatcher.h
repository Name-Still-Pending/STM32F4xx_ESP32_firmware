/*
 * MQTT_dispatcher.h
 *
 *  Created on: Dec 29, 2023
 *      Author: poti
 */

#ifndef WIFI_MQTT_FIRMWARE_INCLUDE_MQTT_DISPATCHER_H_
#define WIFI_MQTT_FIRMWARE_INCLUDE_MQTT_DISPATCHER_H_

#include "MQTT_dispatcher_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

//
// public defines
//

#define BITFIELD_SIZE(bits_num)		((bits_num) / 8 + !!((bits_num) % 8))
#define MQTT_SUBFLAGS_ON_QUEUE_FULL_BIT 0x1

//
// public typedefs
//

typedef enum {
	MQTT_OK							= (0),
	MQTT_PUB_OK						= (1 << 0),
	MQTT_PUB_FAIL					= (1 << 1),
	MQTT_SUBRECV					= (1 << 2),
	MQTT_CONNECTED					= (1 << 3),
	MQTT_DISCONNECTED				= (1 << 4),

	MQTT_SUB_FAIL_TOPICS_FULL		= (1 << 21),
	MQTT_UNSUB_FAIL_BAD_TOPIC		= (1 << 22),
	MQTT_UNSUB_FAIL_SUBBED			= (1 << 23),
	MQTT_SUB_FAIL					= (1 << 24),
	MQTT_SUB_FAIL_ALREADY_SUBBED	= (1 << 25),
	MQTT_UNSUB_FAIL_BAD_HANDLE		= (1 << 26),
	MQTT_SUB_FAIL_QUEUE_FAIL		= (1 << 27),
	MQTT_SUB_FAIL_TOO_MANY_SUBS		= (1 << 28),
	MQTT_TIMEOUT_NOCONN				= (1 << 29),
	MQTT_TIMEOUT_BUSY				= (1 << 30),
	MQTT_UNKNOWN					= (1 << 31)
}MqttResponse_t;

typedef enum {
	MQTT_SUB_DEFAULT				= (0),
	MQTT_SUB_ON_QUEUE_FULL_DROP_OLD = (1),
}MqttSubscriberFlags_t;

typedef void *MqttSubHandle_t;

typedef void *MqttTopicHandle_t;

typedef struct{
	MqttTopicHandle_t topic;
	int len;
	uint8_t value[MQTT_MAX_MESSAGE_LEN];
}MqttMessage_t;

//
// public function prototypes
//

BaseType_t mqtt_init();

MqttResponse_t mqtt_pub(int8_t *topicName, uint8_t *msg, uint8_t qos, uint8_t retain, TickType_t timeout);

MqttResponse_t mqtt_pub_raw(int8_t *topicName, void *msg, uint8_t len, uint8_t qos, uint8_t retain, TickType_t timeout);

MqttResponse_t mqtt_add_subscriber(MqttSubHandle_t *handle, uint32_t flags, TickType_t timeout);

MqttResponse_t mqtt_remove_subscriber(MqttSubHandle_t *handle, TickType_t timeout);

MqttResponse_t mqtt_subscribe_topic(MqttSubHandle_t handle, MqttTopicHandle_t *topicHandle, const int8_t *topicName, TickType_t timeout);

MqttResponse_t mqtt_unsubscribe_topic(MqttSubHandle_t handle, MqttTopicHandle_t *topicName, TickType_t timeout);

BaseType_t mqtt_poll(MqttSubHandle_t handle, MqttMessage_t *message, TickType_t timeout);

BaseType_t mqtt_to_latest(MqttSubHandle_t handle);

#endif /* WIFI_MQTT_FIRMWARE_INCLUDE_MQTT_DISPATCHER_H_ */
