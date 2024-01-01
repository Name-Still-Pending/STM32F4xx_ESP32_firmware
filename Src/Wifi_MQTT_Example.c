/*
 * Wifi_MQTT_Example.c
 *
 *  Created on: Dec 27, 2023
 *      Author: poti
 */


#include <esp32_manager.h>
#include "Wifi_MQTT_Example.h"
#include "task.h"
#include "MQTT_dispatcher.h"

static TaskHandle_t exampleTask_handle;

static void exampleTask(void*);

void createExampleTask(){
	xTaskCreate(exampleTask, "Wifi_Example", sizeof(MqttMessage_t) + 8, NULL, 3, &exampleTask_handle);
}


void exampleTask(void*){
//	const char buffer[] = "Test";
	MqttSubHandle_t sub = NULL;
	MqttResponse_t mqttResp;
	MqttTopicHandle_t topic;
	mqtt_add_subscriber(&sub, MQTT_SUB_ON_QUEUE_FULL_DROP_OLD, pdMS_TO_TICKS(60000));
	mqttResp = mqtt_subscribe_topic(sub, &topic, "test_out0", pdMS_TO_TICKS(60000));
	while(1){
		MqttMessage_t msg;
		while(!mqtt_poll(sub, &msg, pdMS_TO_TICKS(1000)));
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
		if(mqtt_pub_raw("test_in0", msg.value, msg.len, 2, 0, pdMS_TO_TICKS(1000)) == MQTT_OK){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		}
	}

	vTaskDelete(NULL);
}
