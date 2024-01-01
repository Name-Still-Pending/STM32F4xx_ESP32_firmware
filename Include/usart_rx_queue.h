/*
 * usart_rx_queue.h
 *
 *  Created on: Dec 28, 2023
 *      Author: poti
 */

#ifndef WIFI_MQTT_FIRMWARE_SRC_USART_RX_QUEUE_H_
#define WIFI_MQTT_FIRMWARE_SRC_USART_RX_QUEUE_H_

#include <esp32_manager_config.h>
#include "FreeRTOS.h"
#include "task.h"

#define USART_QUEUE_RX_ISR_SNIPPET 		if(USED_UART->SR & USART_SR_RXNE){																		\
											rx_queue_put(USED_UART->DR);																		\
											USED_UART->SR &= ~USART_SR_RXNE;																	\
											BaseType_t higherPriorityTaskWoken = pdFALSE;														\
											xTaskNotifyFromISR(getTaskHandle(), count(), eSetValueWithOverwrite, &higherPriorityTaskWoken);\
											portYIELD_FROM_ISR(higherPriorityTaskWoken);														\
										}																										\


void rx_queue_put(uint8_t data);

uint8_t rx_queue_get(uint8_t *out);

uint32_t count();

TaskHandle_t getTaskHandle();

#endif /* WIFI_MQTT_FIRMWARE_SRC_USART_RX_QUEUE_H_ */
