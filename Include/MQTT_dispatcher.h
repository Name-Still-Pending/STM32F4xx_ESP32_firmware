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


/**
 * @brief Initializes all components of the MQTT dispatch system. Creates the MQTT_dispatcher and MQTT_dispatcher_init (timeout 60 s) tasks.
 * @note Uses ESP32 communication. eps32_manager must be successfully initialized.
 * @retval pdPASS on successful completion, pdFAIL otherwise
 */
BaseType_t mqtt_init();

/**
 * @brief Publishes an MQTT message.
 * @note Meant for text only, do not use commas. If you want to publish binary data or messages where the publish command would exceed 256 characters, use mqtt_pub_raw instead.
 * @param topic: MQTT topic to publish to
 * @param msg: Message to publish
 * @param qos: Quality of service level [0, 2]
 * @param retain: true / false, retain message.
 * @param timeout: Operation timeout.
 * @retval MQTT_OK on success, other on error.
 */
MqttResponse_t mqtt_pub(int8_t *topicName, uint8_t *msg, uint8_t qos, uint8_t retain, TickType_t timeout);

/**
 * @brief Publishes raw data to MQTT.
 * @param topic: MQTT topic to publish to
 * @param data: Data to publish.
 * @param len: Exact size (bytes) of published data.
 * @param qos: Quality of service level [0, 2]
 * @param retain: true / false, retain message.
 * @param timeout: Operation timeout.
 * @retval MQTT_OK on success, other on error.
 */
MqttResponse_t mqtt_pub_raw(int8_t *topicName, void *msg, uint8_t len, uint8_t qos, uint8_t retain, TickType_t timeout);

/**
 * @brief Initializes a subscriber and returns a handle.
 * @note Not thread-safe. Do not share subscribers across threads/tasks.
 * @note Make sure not to lose the handle.
 * @note The total number of subscribers globally cannot exceed the MQTT_MAX_SUBSCRIBERS predefine (see MQTT_dispatcher_config.h)
 * @param handle: Pointer to the variable where you want to store the subscriber handle.
 * @param flags: Subscriber behavior settings. Use MqttSubscriberFlags_t enum.
 * @param timeout: Operation timeout.
 * @retval MQTT_OK on success, other on error.
 */
MqttResponse_t mqtt_add_subscriber(MqttSubHandle_t *handle, uint32_t flags, TickType_t timeout);

/**
 * @brief Destroys a subscriber, freeing its slot.
 * @note Untested, may not work.
 * @param handle: Pointer to the variable, where the subscriber handle is stored. Said handle will be set to NULL on success.
 * @param timeout: Operation timeout.
 * @retval MQTT_OK on success, other on error.
 */
MqttResponse_t mqtt_remove_subscriber(MqttSubHandle_t *handle, TickType_t timeout);

/**
 * @brief Subscribes a subscriber to a topic.
 * @note The amount of unique topics across all subscribers globally is limited by MQTT_MAX_TOPICS predefine (see MQTT_dispatcher_config.h)
 * @param handle: Handle of the subscriber.
 * @param topicHandle: Pointer to the handle of the subscribed topic. If you do not need the handle (unadvised), you can pass NULL.
 * @param topic: Name of the topic.
 * @param timeout: Operation timeout.
 * @retval MQTT_OK on success, other on error.
 */
MqttResponse_t mqtt_subscribe_topic(MqttSubHandle_t handle, MqttTopicHandle_t *topicHandle, const int8_t *topicName, TickType_t timeout);

/**
 * @brief Unsubscribes a subscriber from a topic.
 * @note If there are no more subscriptions to the topic, it will be destroyed, freeing its slot.
 * @note Untested, probably doesn't work.
 * @param handle: Subscriber handle.
 * @param topic: Pointer to the topic handle, so that it can be set to NULL on success.
 * @param timeout: Operation timeout.
 * @retval MQTT_OK on success, other on error.
 */
MqttResponse_t mqtt_unsubscribe_topic(MqttSubHandle_t handle, MqttTopicHandle_t *topicName, TickType_t timeout);

/**
 * @brief Waits for a message to arrive.
 * @param handle: Subscriber handle.
 * @param message: Received message buffer.
 * @param timeout: Operation timeout.
 * @retval pdTRUE on message received, pdFALSE on timeout.
 */
BaseType_t mqtt_poll(MqttSubHandle_t handle, MqttMessage_t *message, TickType_t timeout);

/**
 * @brief Moves the queue to the latest message.
 * @note This could technically turn into an infinite loop, if the messages were written to the queue fast enough, but that should be impossible in practice.
 * @param handle: Subscriber handle
 * @retval The number of discarded messages.
 */
BaseType_t mqtt_to_latest(MqttSubHandle_t handle);

#endif /* WIFI_MQTT_FIRMWARE_INCLUDE_MQTT_DISPATCHER_H_ */
