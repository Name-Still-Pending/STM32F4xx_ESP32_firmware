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

/**
 * This is a very basic example of MQTT dispatcher API use.
 */

static TaskHandle_t exampleTask_handle;

static void exampleTask(void*);

/**
 * Call this function in your main() to use this example.
 */
void createExampleTasks(){
	xTaskCreate(exampleTask, "Wifi_Example_1", sizeof(MqttMessage_t) + 8, "test_in0", 1, &exampleTask_handle);
	xTaskCreate(exampleTask, "Wifi_Example_2", sizeof(MqttMessage_t) + 8, "test_in1", 1, &exampleTask_handle);
}


static void exampleTask(void* params){
	MqttSubHandle_t sub = NULL;

	/**
	 * Storing topic handles is useful for checking which topic sent the message,
	 * and necessary if you want to unsubscribe without deleting the subscriber.
	 */
	MqttTopicHandle_t topic0, topic1;

	/**
	 * Be generous with timeouts in the beginning, as ESP32 manager and MQTT dispatcher take a while to initialize.
	 * Also, you should check the return values, which I didn't do here solely because I'm too lazy.
	 *
	 * MQTT_SUB_ON_QUEUE_FULL_DROP_OLD, makes it so that, when the queue is full, old messages (first in queue) are discarded
	 * instead of new ones (which is default as  set by MATT_SUB_DEFAULT).
	 */
	mqtt_add_subscriber(&sub, MQTT_SUB_ON_QUEUE_FULL_DROP_OLD, pdMS_TO_TICKS(60000));
	mqtt_subscribe_topic(sub, &topic0, "test_out0", pdMS_TO_TICKS(60000));
	mqtt_subscribe_topic(sub, &topic1, "test_out1", pdMS_TO_TICKS(60000));
	while(1){
		MqttMessage_t msg;

		/**
		 * A basic polling loop. It will block for as long as there are no messages and is impossible to exit.
		 * It makes sense to set a longer timeout if you don't expect to receive messages very often.
		 * If timeout is set to 0, you will receive a message only if it's already in the queue.
		 * If messages come faster than you can process them and they aren't all critical, use mqtt_to_latest.
		 */
		while(!mqtt_poll(sub, &msg, pdMS_TO_TICKS(1000)));

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);


		/**
		 * Publishing usually takes a little less than a second. Since simultaneous operations aren't possible
		 * due to ESP32 module limitations, it's easy to timeout if you're too tight with time restrictions or if
		 * things happen all at once or in rapid succession.
		 *
		 * Anyway, you should avoid timeouts, their handling is untested and likely faulty.
		 *
		 * If you intend to send raw data or long messages, use mqtt_pub_raw instead.
		 */
		if(mqtt_pub((char*)params, msg.value, 2, 0, pdMS_TO_TICKS(5000)) == MQTT_OK){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); // Light blue LED on success.
		}
		else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Light red LED on error.
		}
	}

	vTaskDelete(NULL);
}
