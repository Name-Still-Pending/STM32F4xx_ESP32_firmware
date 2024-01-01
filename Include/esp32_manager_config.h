/*
 * Wifi_MQTT_config.h
 *
 *  Created on: Dec 26, 2023
 *      Author: poti
 */

#ifndef WIFI_MQTT_FIRMWARE_INCLUDE_ESP32_MANAGER_CONFIG_H_
#define WIFI_MQTT_FIRMWARE_INCLUDE_ESP32_MANAGER_CONFIG_H_

#include "main.h"

#ifndef RX_MESSAGE_MAX_LEN
/**
 * @brief Maximum length of a single message, delimited by \r\n.
 */
#define RX_MESSAGE_MAX_LEN 256
#endif


#ifndef RX_BUFFER_SIZE
/**
 * @brief Size (bytes) of the RX buffer.
 */
#define RX_BUFFER_SIZE 1024
#endif

#ifndef USED_UART
/**
 * @brief UART device used for communication with the ESP32 unit.
 */
#define USED_UART USART2
#endif

//
// Connection settings
//

#ifndef WIFI_SSID
/**
 * WiFi network SSID.
 */
#define WIFI_SSID "wifi"
#warning "Wifi SSID not defined! Using default!"
#endif

#ifndef WIFI_PASS
/**
 * WiFi network password. It's recommended you use compiler options to define it.
 */
#define WIFI_PASS "password"
#warning "Wifi password not defined! Using default!"
#endif



// utils

#define str(x) #x
#define xstr(x) str(x)

#endif /* WIFI_MQTT_FIRMWARE_INCLUDE_ESP32_MANAGER_CONFIG_H_ */
