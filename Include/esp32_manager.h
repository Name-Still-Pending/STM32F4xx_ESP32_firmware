/*
 * Wifi_MQTT_firmware.h
 *
 *  Created on: Dec 26, 2023
 *      Author: poti
 */

#ifndef WIFI_MQTT_FIRMWARE_INCLUDE_ESP32_MANAGER_H_
#define WIFI_MQTT_FIRMWARE_INCLUDE_ESP32_MANAGER_H_

#include <esp32_manager_config.h>
#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "event_groups.h"

// public typedefs


typedef enum {
	 AT_RESP_NO_WAIT			= (0),
	 AT_RESP_OK 				= (1 << 0),
	 AT_RESP_ERROR 				= (1 << 1),
	 AT_RESP_SEND_OK			= (1 << 2),
	 AT_RESP_WIFI_CONNECTED	 	= (1 << 3),
	 AT_RESP_WIFI_GOT_IP		= (1 << 4),
	 AT_RESP_AT					= (1 << 5),
	 AT_RESP_UNKNOWN			= (1 << 6),
	 AT_RESP_BUSY				= (1 << 7),
	 AT_INVALID					= (1 << 8),
	 AT_RESP_OTHER				= (1 << 9),
	 AT_RESP_EMPTY				= (1 << 10),
	 AT_RESP_WIFI_DISCONNECT	= (1 << 11),
	 AT_RESP_SEND_FAIL			= (1 << 12),
	 AT_RESP_READY				= (1 << 13),

	 OTHR_REDIRECT_DONE			= (1 << 20),
	 OTHR_REDIRECT_INCOMPLETE	= (1 << 21),
	 OTHR_REDIRECT_NO_ACTION	= (1 << 22),
	 OTHR_REDIRECT_TIMEOUT		= (1 << 23),

	 RX_MESSAGE_TOO_LONG		= (1 << 31),
	 AT_SEND_TIMEOUT			= (1 << 30),
	 AT_TRANSMIT_FAIL			= (1 << 29)
}Response_t;

typedef Response_t(*MsgHandler_t)(uint8_t*, int16_t, TickType_t);

typedef struct{
	const uint8_t *const match;
	const int16_t matchLen;
	const MsgHandler_t callback;
	void *next;
}MessageRedirect_t;

// public defines



#define AT_RESP_IGNORE					(AT_RESP_EMPTY | AT_RESP_AT)
#define AT_RESP_FINAL					(AT_RESP_OK | AT_RESP_ERROR | AT_RESP_BUSY)

#define STATUS_INITIALIZED				(1 << 0)
#define STATUS_CONNECTED					(1 << 1)
#define STATUS_GOT_IP					(1 << 2)
#define STATUS_TRANSMISSION_START		(1 << 3)
#define STATUS_TRANSMISSION_FAIL			(1 << 4)
#define STATUS_TRANSMISSION_SUCCESS		(1 << 5)
#define STATUS_TRANSMISSION_END			(STATUS_TRANSMISSION_FAIL | STATUS_TRANSMISSION_SUCCESS)
#define STATUS_TIMEOUT					(1 << 6)

#define STATUS_ALL_GOOD					(0x7)

#define MSGRED_TIMEOUT					(1 << 2)
#define MSGRED_OP_OK						(1 << 0)
#define MSGRED_OP_FAIL					(1 << 1)


#define AT_command_static(cmd, accResp, timeout_ticks) esp32_command(cmd, sizeof(cmd) - 1, accResp, timeout_ticks);
#define esp32_try_capture(timeout, timeout_resp) if(!esp32_capture(timeout)) return timeout_resp;

#define EXTERN_TASK_HANDLES \
	TaskHandle_t RxTask_handle;		\

#define STATIC_MESSAGE_REDIRECT_STRUCT(name, match, callback) MessageRedirect_t name = {match, sizeof(match) - 1, callback, NULL}


//
// public function prototypes
//


BaseType_t 	esp32_init(UART_HandleTypeDef *uart);

Response_t 	esp32_command(uint8_t *cmd, int16_t len, Response_t acceptedResponse, TickType_t timeout);

Response_t 	esp32_send_raw(void *data, int16_t len, Response_t acceptedResponse, TickType_t timeout);

Response_t 	esp32_command_long(uint8_t *cmd, int16_t cmdLen, void *data, int16_t dataLen, TickType_t timeout);

Response_t 	esp32_response(Response_t acceptedResponse, TickType_t timeout);

uint16_t 	esp32_clear_response_queue();

BaseType_t	esp32_capture(TickType_t timeout);

BaseType_t 	esp32_release();

size_t 		esp32_get_other(void *buffer, size_t maxSize, TickType_t timeout);

BaseType_t  esp32_status_wait(EventBits_t bits, BaseType_t waitForAll, TickType_t timeout);

void		esp32_confirm_transmission(uint8_t success);

BaseType_t	esp32_add_redirect_handler(MessageRedirect_t *const handler, TickType_t timeout);

BaseType_t	esp32_remove_redirect_handler(MessageRedirect_t *const handler, TickType_t timeout);


#endif /* WIFI_MQTT_FIRMWARE_INCLUDE_ESP32_MANAGER_H_ */
