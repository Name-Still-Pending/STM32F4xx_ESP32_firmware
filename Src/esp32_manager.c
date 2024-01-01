/*
 * Wifi_MQTT_firmware.c
 *
 *  Created on: Dec 26, 2023
 *      Author: poti
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpointer-sign"

#include <esp32_manager.h>
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <limits.h>
#include "semphr.h"
#include <string.h>
#include "queue.h"
#include "usart_rx_queue.h"
#include "message_buffer.h"
#include "event_groups.h"

// private typedefs

typedef struct{
	TickType_t recieveWaitTicks;
	TickType_t processMsgTimeout;

}RxTaskParams_TypeDef;

// private defines

#define RX_STACK_DEPTH 100
#define TX_STACK_DEPTH 100

#define SET_DEADLINE(timeout) TickType_t deadline = (timeout) + xTaskGetTickCount()
#define GET_TIMEOUT (deadline - xTaskGetTickCount())

#define SET_TIMEOUT_FLAG xEventGroupSetBits(flags_handle, STATUS_TIMEOUT)
#define GET_TIMEOUT_FLAG (xEventGroupGetBits(flags_handle) & STATUS_TIMEOUT)
#define CLEAR_TIMEOUT_FLAG xEventGroupClearBits(flags_handle, STATUS_TIMEOUT)

//
// private variables
//

static UART_HandleTypeDef *UART = NULL;
extern TaskHandle_t RxTask_handle;
static TaskHandle_t init_task_handle;

static SemaphoreHandle_t 	TxMutex_handle;
static QueueHandle_t 		RxResponseQueue_handle;
static MessageBufferHandle_t RxOtherMessageBuffer_handle;
static EventGroupHandle_t 	flags_handle;
static StaticEventGroup_t 	flags_static;
static StaticSemaphore_t	redirectEditMutex_static;
static SemaphoreHandle_t	RedirectEditMutex_handle;

static uint8_t AT_manual_capture = 0;

static MessageRedirect_t *msgred_first = NULL;
static MessageRedirect_t *msgred_last = NULL;

//
// private function prototypes
//

static void esp32_rx_task(void *params);
static void esp32_init_task(void*);
static Response_t getResponseType(uint8_t *msg, int16_t len);
static Response_t processOther(uint8_t *msg, int16_t len, TickType_t timeout);

//
// public function definitions
//

/**
 * @brief starts the esp32_Rx and esp32_init tasks.
 * @note Must be called before osKernelStart()
 * @param uart - handle of the used UART device.
 * @returns pdPASS on success, other on error.
 */
BaseType_t esp32_init(UART_HandleTypeDef *uart){
	UART = uart;
	RxTaskParams_TypeDef rxParams = {0};
	rxParams.recieveWaitTicks = pdMS_TO_TICKS(500);
	rxParams.processMsgTimeout = pdMS_TO_TICKS(1000);
#define NULL_CHECK(handle)if(!handle) return pdFAIL;
	TxMutex_handle 				= xSemaphoreCreateMutex();
	NULL_CHECK(TxMutex_handle)
	RxResponseQueue_handle 		= xQueueCreate(20, sizeof(Response_t));
	NULL_CHECK(RxResponseQueue_handle)
	RxOtherMessageBuffer_handle 	= xMessageBufferCreate(256);
	NULL_CHECK(RxOtherMessageBuffer_handle)
	flags_handle					= xEventGroupCreateStatic(&flags_static);
	NULL_CHECK(flags_handle)
	RedirectEditMutex_handle	= xSemaphoreCreateMutexStatic(&redirectEditMutex_static);
	NULL_CHECK(RedirectEditMutex_handle)
#undef NULL_CHECK

	__HAL_UART_ENABLE_IT(UART, UART_IT_RXNE);

	BaseType_t resp;
	resp = xTaskCreate(esp32_rx_task, "esp32_Rx", RX_STACK_DEPTH, &rxParams, 1, &RxTask_handle);
	if(resp != pdPASS) return resp;
	resp = xTaskCreate(esp32_init_task, "esp32_init", 100, NULL, 1, &init_task_handle);
	if(resp != pdPASS) return resp;

	return pdPASS;
}

Response_t esp32_command(uint8_t *cmd, int16_t len, Response_t acceptedResponse, TickType_t timeout){
	SET_DEADLINE(timeout);
	if(len < 0) len = strlen(cmd);
	if(strcmp((int8_t*)&cmd[len - 2], "\r\n")) return AT_INVALID;
	if(len > 256) return AT_INVALID;

	if(acceptedResponse != AT_RESP_NO_WAIT) acceptedResponse |= AT_RESP_FINAL;
	return esp32_send_raw(cmd, len, acceptedResponse, GET_TIMEOUT);
}

Response_t esp32_send_raw(void *data, int16_t len, Response_t acceptedResponse, TickType_t timeout){
	SET_DEADLINE(timeout);
	BaseType_t resp;
	HAL_StatusTypeDef hStatus;
	if(!AT_manual_capture){
		resp = xSemaphoreTake(TxMutex_handle, timeout);
		if(!resp) return AT_SEND_TIMEOUT;
	}

	hStatus = HAL_UART_Transmit_IT(UART, data, len);
	if(hStatus != HAL_OK) return AT_TRANSMIT_FAIL;
	if(acceptedResponse == AT_RESP_NO_WAIT) return AT_RESP_NO_WAIT;

	return esp32_response(acceptedResponse, GET_TIMEOUT);
}

Response_t 	esp32_command_long(uint8_t *cmd, int16_t cmdLen, void *data, int16_t dataLen, TickType_t timeout){
	SET_DEADLINE(timeout);
	if(!esp32_capture(GET_TIMEOUT)) return AT_SEND_TIMEOUT;
	Response_t retval = AT_RESP_OK;
	do{
		retval = esp32_command(cmd, cmdLen, AT_RESP_FINAL, GET_TIMEOUT);
		if(retval != AT_RESP_OK) break;
		if(!(xEventGroupWaitBits(flags_handle, STATUS_TRANSMISSION_START, pdTRUE, pdTRUE, GET_TIMEOUT) & STATUS_TRANSMISSION_START)){
			retval = AT_SEND_TIMEOUT;
			break;
		}
		retval = esp32_send_raw(data, dataLen, AT_RESP_NO_WAIT, GET_TIMEOUT);
		if(retval != AT_RESP_NO_WAIT) break;

		switch(xEventGroupWaitBits(flags_handle, STATUS_TRANSMISSION_END, pdTRUE, pdFALSE, GET_TIMEOUT) & STATUS_TRANSMISSION_END){
		case STATUS_TRANSMISSION_SUCCESS:
			retval = AT_RESP_OK;
			break;
		case STATUS_TRANSMISSION_FAIL:
			retval = AT_RESP_SEND_FAIL;
			break;
		default:
			retval = AT_SEND_TIMEOUT;
		}

	}while(0);

	esp32_release();

	return retval;
}

Response_t esp32_response(Response_t acceptedResponse, TickType_t timeout){
	BaseType_t qResp;
	Response_t resp = 0;
	SET_DEADLINE(timeout);
	do{
		qResp = xQueueReceive(RxResponseQueue_handle, &resp, GET_TIMEOUT);
	}while(!(resp & acceptedResponse) && qResp);

	if(qResp != pdTRUE) {
		SET_TIMEOUT_FLAG;
		return AT_SEND_TIMEOUT;
	}

	if(!AT_manual_capture){
		xSemaphoreGive(TxMutex_handle);
	}

	return resp;
}

inline BaseType_t esp32_capture(TickType_t timeout){
	BaseType_t resp;
	resp = xSemaphoreTake(TxMutex_handle, timeout);
	if(resp != pdTRUE) return resp;
	AT_manual_capture = 1;
	return resp;
}

inline BaseType_t esp32_release(){
	if(!AT_manual_capture) return pdFALSE;
	AT_manual_capture = 0;
	return xSemaphoreGive(TxMutex_handle);
}

inline size_t esp32_get_other(void *buffer, size_t maxSize, TickType_t timeout){
	return xMessageBufferReceive(RxOtherMessageBuffer_handle, buffer, maxSize, timeout);
}

inline uint16_t esp32_clear_response_queue(){
	uint16_t retval = uxQueueMessagesWaiting(RxResponseQueue_handle);
	xQueueReset(RxResponseQueue_handle);
	return retval;
}

inline BaseType_t  esp32_status_wait(EventBits_t bits, BaseType_t waitForAll, TickType_t timeout){
	EventBits_t setBits = xEventGroupWaitBits(flags_handle, bits, pdFALSE, waitForAll, timeout) & bits;
	return (waitForAll && setBits == bits) || (!waitForAll && setBits);
}

inline void esp32_confirm_transmission(uint8_t success){
	xEventGroupSetBits(flags_handle, success ? STATUS_TRANSMISSION_SUCCESS : STATUS_TRANSMISSION_FAIL);
}

BaseType_t	esp32_add_redirect_handler(MessageRedirect_t *const handler, TickType_t timeout){
	if(!xSemaphoreTake(RedirectEditMutex_handle, timeout)) return pdFAIL;

	if(msgred_last == NULL){
		msgred_last = msgred_first = handler;
	}
	else{
		msgred_last->next = handler;
		msgred_last = handler;
	}
	xSemaphoreGive(RedirectEditMutex_handle);
	return pdPASS;
}

BaseType_t	esp32_remove_redirect_handler(MessageRedirect_t *const handler, TickType_t timeout){
	if(!xSemaphoreTake(RedirectEditMutex_handle, timeout)) return MSGRED_TIMEOUT;

	MessageRedirect_t **ptr = &msgred_first;

	while(*ptr != NULL && *ptr != handler)
		ptr = (MessageRedirect_t**)&((ptr[0]->next));

	BaseType_t retval;
	if(*ptr == NULL) retval = MSGRED_OP_FAIL;
	else{
		retval = MSGRED_OP_OK;
		(*ptr)->next = handler->next;
		if(handler == msgred_last) msgred_last = NULL;
	}

	xSemaphoreGive(RedirectEditMutex_handle);
	return retval;
}


// private function definitions

static void esp32_rx_task(void *params){
	RxTaskParams_TypeDef p = *((RxTaskParams_TypeDef*)params);

	xEventGroupClearBits(flags_handle, STATUS_ALL_GOOD);

	uint8_t msgBuf[RX_MESSAGE_MAX_LEN] = {0};

	const uint8_t match[] = "\r\n";
	uint8_t match_i = 0;
	int16_t msgLen = 0;
	uint8_t cont = 0;

	while(1){
		if(!cont){
			msgLen = 0;
			match_i = 0;
		}
		else cont = 0;
		// get message
		while(match_i < 2 && msgLen < RX_MESSAGE_MAX_LEN){
			if(!rx_queue_get(&msgBuf[msgLen])){
				xTaskNotifyWait(pdFALSE, 0xffffffff, NULL, p.recieveWaitTicks);
				continue;
			}
			if(msgBuf[0] == '>'){
				xEventGroupSetBits(flags_handle, STATUS_TRANSMISSION_START);
				continue;
			}
			if(match[match_i] == msgBuf[msgLen]) ++match_i;
			else match_i = 0;
			++msgLen;
		}

		// parse message type
		SET_DEADLINE(p.processMsgTimeout);
		Response_t resp;
		if(msgLen == RX_MESSAGE_MAX_LEN && match_i < 2){
			resp = RX_MESSAGE_TOO_LONG;
		}
		else{
			msgLen -= 2;
			*((uint16_t*)&msgBuf[msgLen]) = 0;
			resp = getResponseType(msgBuf, msgLen);
		}

		// process message
		BaseType_t qResp;

		switch(resp){
		case AT_RESP_EMPTY:
		case AT_RESP_AT:
		case AT_RESP_UNKNOWN:
			continue;

		case AT_RESP_SEND_OK:
			xEventGroupSetBits(flags_handle, STATUS_TRANSMISSION_SUCCESS);
			break;

		case AT_RESP_SEND_FAIL:
			xEventGroupSetBits(flags_handle, STATUS_TRANSMISSION_FAIL);
			break;

		case AT_RESP_WIFI_CONNECTED:
			xEventGroupSetBits(flags_handle, STATUS_CONNECTED);
			break;

		case AT_RESP_WIFI_GOT_IP:
			xEventGroupSetBits(flags_handle, STATUS_GOT_IP);
			break;

		case AT_RESP_WIFI_DISCONNECT:
			xEventGroupClearBits(flags_handle, STATUS_CONNECTED | STATUS_GOT_IP);
			break;

		case AT_RESP_OTHER:
			Response_t othrResp = processOther(&msgBuf[1], msgLen - 1, GET_TIMEOUT);
			if(othrResp == OTHR_REDIRECT_DONE) break;
			if(othrResp == OTHR_REDIRECT_INCOMPLETE){
				memcpy(&msgBuf[msgLen], "\r\n", 2);
				cont = 1;
				continue;
			}

			xMessageBufferSend(RxOtherMessageBuffer_handle, &msgBuf[1], msgLen - 1, GET_TIMEOUT);
			//TODO: handle timeout

		default:
			if(resp & (AT_RESP_OK | AT_RESP_ERROR) && GET_TIMEOUT_FLAG) {
				CLEAR_TIMEOUT_FLAG;
				continue; // ignores final response if timeout flag is set (command is no longer relevant)
			}
			qResp = xQueueSend(RxResponseQueue_handle, &resp, GET_TIMEOUT);
			if(qResp != pdTRUE) continue;

		}
	}

	vTaskDelete(init_task_handle);
}

static void esp32_init_task(void* params){
	SET_DEADLINE(pdMS_TO_TICKS(60000));
	Response_t resp = 0;
	uint8_t error = 0;

#define OK_CHECK if(resp != AT_RESP_OK){ \
					error = 1;			\
					break;				\
				}						\

	do{
		if(!esp32_capture(GET_TIMEOUT)){
			error = 1;
			break;
		}

		// wait for ready
		resp = AT_command_static("AT\r\n", AT_RESP_BUSY, GET_TIMEOUT);
		while(resp == AT_RESP_BUSY){
			vTaskDelay(pdMS_TO_TICKS(1000));
			esp32_clear_response_queue();
			resp = AT_command_static("AT\r\n", AT_RESP_BUSY | AT_RESP_READY, GET_TIMEOUT);
		}

		// restore defaults
		resp = AT_command_static("AT+RESTORE\r\n", AT_RESP_FINAL, GET_TIMEOUT);
		OK_CHECK;

		resp = esp32_response(AT_RESP_READY, pdMS_TO_TICKS(GET_TIMEOUT));
		if(resp != AT_RESP_READY){
			error = 1;
			break;
		}

		// set cwmode
		resp = AT_command_static("AT+CWMODE=1\r\n", AT_RESP_FINAL, GET_TIMEOUT);
		OK_CHECK;

		// set wifi config
		resp = AT_command_static("AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASS "\"\r\n", AT_RESP_FINAL, GET_TIMEOUT);
		OK_CHECK;


#undef OK_CHECK
	}while(0);

	esp32_release();

	while(error){
		vTaskDelay(pdMS_TO_TICKS(1000));
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	}

	xEventGroupSetBits(flags_handle, STATUS_INITIALIZED);
#if 1
	vTaskDelete(NULL);
#else
	vTaskSuspend(NULL);
#endif
}

static inline Response_t getResponseType(uint8_t *msg, int16_t len){
	if(len == 0) 												return AT_RESP_EMPTY;
	if(len >  2 && !strncmp(msg, "AT", 2))						return AT_RESP_AT;
	if(msg[0] == '+') 											return AT_RESP_OTHER;
	if(len >= 2 && !strcmp(msg, "OK")) 							return AT_RESP_OK;
	if(len >= 5){
		if(!strcmp(msg, "ERROR"))								return AT_RESP_ERROR;
		if(!strcmp(msg, "ready"))								return AT_RESP_READY;
	}
	if(len >= 4 && !strncmp(msg, "busy", 4))					return AT_RESP_BUSY;
	if(len >= 7 && !strncmp(msg, "SEND ", 5)){
		if(!strcmp(&msg[5], "OK"))								return AT_RESP_SEND_OK;
		if(!strcmp(&msg[5], "FAIL"))							return AT_RESP_SEND_FAIL;
																return AT_RESP_UNKNOWN;
	}
	if(len >= 11 && !strncmp(msg, "WIFI ", 5)){
		if(len >= 14 && !strcmp(&msg[5], "CONNECTED"))			return AT_RESP_WIFI_CONNECTED;
		if(len >= 11 && !strcmp(&msg[5], "GOT IP"))				return AT_RESP_WIFI_GOT_IP;
		if(len >= 15 && !strcmp(&msg[5], "DISCONNECT"))			return AT_RESP_WIFI_DISCONNECT;
																return AT_RESP_UNKNOWN;
	}

	return AT_RESP_UNKNOWN;
}

static Response_t processOther(uint8_t *msg, int16_t len, TickType_t timeout){
	SET_DEADLINE(timeout);
	if(!xSemaphoreTake(RedirectEditMutex_handle, timeout)) return OTHR_REDIRECT_TIMEOUT;

	Response_t retval = OTHR_REDIRECT_NO_ACTION;
	for(MessageRedirect_t *ptr = msgred_first; ptr != NULL && retval == OTHR_REDIRECT_NO_ACTION; ptr = (MessageRedirect_t*)ptr->next){
		if(len >= ptr->matchLen && !strncmp(msg, ptr->match, ptr->matchLen)){
			retval = ptr->callback(&msg[ptr->matchLen], len - ptr->matchLen, GET_TIMEOUT);
		}
	}

	xSemaphoreGive(RedirectEditMutex_handle);
	return retval;
}


#pragma GCC diagnostic pop











