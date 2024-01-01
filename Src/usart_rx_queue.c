/*
 * usart_rx_queue.c
 *
 *  Created on: Dec 28, 2023
 *      Author: poti
 */


#include "usart_rx_queue.h"

#define INCREMENT_CLAMP(var, max) {if(++var >= max) var = 0;}

static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint32_t rd_i = 0;
static uint32_t wr_i = 0;

extern TaskHandle_t RxTask_handle;


inline void rx_queue_put(uint8_t data){
	rx_buffer[wr_i] = data;
	INCREMENT_CLAMP(wr_i, RX_BUFFER_SIZE);
}

inline uint8_t rx_queue_get(uint8_t *out){
	if(wr_i == rd_i) return 0;
	*out = rx_buffer[rd_i];
	INCREMENT_CLAMP(rd_i, RX_BUFFER_SIZE);
	return 1;
}

inline uint32_t count(){
	if(rd_i > wr_i) return wr_i - rd_i + RX_BUFFER_SIZE;
	return wr_i - rd_i;
}

inline TaskHandle_t getTaskHandle(){
	return RxTask_handle;
}

