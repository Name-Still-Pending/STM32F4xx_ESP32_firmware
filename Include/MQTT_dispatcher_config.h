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
#define MQTT_BROKER_IP "192.168.0.10"
#warning "MQTT broker IP not defined. Using default!"
#endif

#ifndef MQTT_BROKER_PORT
#define MQTT_BROKER_PORT 1883
#warning "MQTT broker port not defined. Using default!"
#endif

#ifndef MQTT_LWT_TOPIC
#define MQTT_LWT_TOPIC "my_lwt_topic"
#endif

#ifndef MQTT_LWT_MESSAGE
#define MQTT_LWT_MESSAGE "I_AM_DEAD"
#endif

#ifndef MQTT_CLIENT_NAME
#define MQTT_CLIENT_NAME "ESP32_WiFi_module"
#endif

#ifndef MQTT_ANONYMOUS_LOGIN
#define MQTT_ANONYMOUS_LOGIN 1
#endif

#ifndef MQTT_LOGIN_USERNAME
#define MQTT_LOGIN_USERNAME ""
#if !MQTT_ANONYMOUS_LOGIN
#warning "MQTT client username not set. Using default!"
#endif
#endif

#ifndef MQTT_LOGIN_PASSWORD
#define MQTT_LOGIN_PASSWORD ""
#if !MQTT_ANONYMOUS_LOGIN
#warning "MQTT client password not set. Using default!"
#endif
#endif

#ifndef MQTT_KEEPALIVE
#define MQTT_KEEPALIVE 120
#endif

#ifndef MQTT_IN_BUFFER_SIZE
#define MQTT_IN_BUFFER_SIZE 512
#endif

#ifndef MQTT_MAX_MESSAGE_LEN
#define MQTT_MAX_MESSAGE_LEN 256
#endif

#ifndef MQTT_MAX_SUBSCRIBERS
#define MQTT_MAX_SUBSCRIBERS 8
#endif

#ifndef MQTT_MAX_TOPICS
#define MQTT_MAX_TOPICS 8
#endif

#ifndef MQTT_MAX_MESSAGES_QUEUED
#define MQTT_MAX_MESSAGES_QUEUED 8
#endif

#ifndef MQTT_QOS
#define MQTT_QOS 2
#endif

#ifndef SHAME
#define SHAME 1
#endif

#endif /* WIFI_MQTT_FIRMWARE_INCLUDE_MQTT_DISPATCHER_CONFIG_H_ */
