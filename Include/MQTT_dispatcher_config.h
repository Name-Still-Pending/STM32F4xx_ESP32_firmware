/*
 * MQTT_dispatcher_config.h
 *
 *  Created on: Dec 29, 2023
 *      Author: poti
 */

#ifndef WIFI_MQTT_FIRMWARE_INCLUDE_MQTT_DISPATCHER_CONFIG_H_
#define WIFI_MQTT_FIRMWARE_INCLUDE_MQTT_DISPATCHER_CONFIG_H_

#include "main.h"

#ifndef MQTT_BROKER_IP
/**
 * @brief IP address of the MQTT broker (e.g. local IP of the host machine).
 */
#define MQTT_BROKER_IP "192.168.0.10"
#warning "MQTT broker IP not defined. Using default!"
#endif

#ifndef MQTT_BROKER_PORT
/**
 * @brief Port of the MQTT broker.
 */
#define MQTT_BROKER_PORT 1883
#warning "MQTT broker port not defined. Using default!"
#endif

#ifndef MQTT_LWT_TOPIC
/**
 * @brief Topic in which to publish the last will and testament (Disconnect message).
 */
#define MQTT_LWT_TOPIC "my_lwt_topic"
#endif

#ifndef MQTT_LWT_MESSAGE
/**
 * @brief Last will and testament (disconnect message).
 */
#define MQTT_LWT_MESSAGE "I_AM_DEAD"
#endif

#ifndef MQTT_CLIENT_NAME
/**
 * @brief Name of this MQTT client (must be unique to the broker).
 */
#define MQTT_CLIENT_NAME "ESP32_WiFi_module"
#endif

#ifndef MQTT_ANONYMOUS_LOGIN
/**
 * @brief Anonymous login, must be enabled on the broker.
 */
#define MQTT_ANONYMOUS_LOGIN 1
#endif

#ifndef MQTT_LOGIN_USERNAME
/**
 * @brief Username for the broker connection. Irrelevant if anonymous login is enabled.
 */
#define MQTT_LOGIN_USERNAME ""
#if !MQTT_ANONYMOUS_LOGIN
#warning "MQTT client username not set. Using default!"
#endif
#endif

#ifndef MQTT_LOGIN_PASSWORD
/**
 * @brief Password for the broker connection. Irrelevant if anonymous login is enabled.
 * @note It's recommended you use compiler options to define it.
 */
#define MQTT_LOGIN_PASSWORD ""
#if !MQTT_ANONYMOUS_LOGIN
#warning "MQTT client password not set. Using default!"
#endif
#endif

#ifndef MQTT_KEEPALIVE
/**
 * @brief Timeout (seconds) before the connection is terminated, if there's no traffic.
 */
#define MQTT_KEEPALIVE 120
#endif

#ifndef MQTT_IN_BUFFER_SIZE
/**
 * @brief Size (bytes) of the buffer through which the ESP32 manager redirects the messages to the MQTT dispatcher.
 */
#define MQTT_IN_BUFFER_SIZE 512
#endif

#ifndef MQTT_MAX_MESSAGE_LEN
/**
 * @brief Maximum size (bytes) of received MQTT message contents.
 * @note A smaller value will reduce memory usage.
 */
#define MQTT_MAX_MESSAGE_LEN 256
#endif

#ifndef MQTT_MAX_SUBSCRIBERS
/**
 * @brief Maximum amount of subscribers.
 */
#define MQTT_MAX_SUBSCRIBERS 8
#endif

#ifndef MQTT_MAX_TOPICS
/**
 * @brief Maximum amount of unique topic subscriptions.
 */
#define MQTT_MAX_TOPICS 8
#endif

#ifndef MQTT_MAX_MESSAGES_QUEUED
/**
 * @brief Maximum amount of messages a subscriber can have queued up at once.
 * @note if this number is exceeded, messages will be discarded according to the subscribers settings.
 */
#define MQTT_MAX_MESSAGES_QUEUED 8
#endif

#ifndef MQTT_QOS
/**
 * @brief Quality of service for topic subscriptions.
 */
#define MQTT_QOS 2
#endif

#ifndef MQTT_STATIC_MESSAGE_QUEUES
#define MQTT_STATIC_MESSAGE_QUEUES 0
#endif

#endif /* WIFI_MQTT_FIRMWARE_INCLUDE_MQTT_DISPATCHER_CONFIG_H_ */
