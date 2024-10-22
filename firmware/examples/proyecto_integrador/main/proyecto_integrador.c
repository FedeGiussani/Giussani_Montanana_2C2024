/*! @mainpage Linterna operada con gestos
 *
 * @section genDesc General Description
 * Aplicacion que controla una linterna operada mediante gestos.
 *
 * @section hardConn Hardware Connection
 *
 * |    Peripheral  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	PIN_X	 	| 	GPIO_X		|
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 18/10/2024 | Document creation		                         |
 *
 * @author Giussani Federico (federico.giussani@ingenieria.uner.edu.ar)
 * 		   Montanana Agustina (agustina.montanana@ingenieria.uner.edu.ar)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_mcu.h"
#include "gpio_mcu.h"
#include "led.h"
#include "apds9960.h"
#include "pwm_mcu.h"
/*==================[macros and definitions]=================================*/
#define CONFIG_ACC_PERIOD 50
/*==================[internal data definition]===============================*/
float frec=1000;
float Duty_cycle=0.0;
float percent = 20.0;
/*==================[internal functions declaration]=========================*/
TaskHandle_t gesture_process_event_handle = NULL;


void pint_intr_callback(void)
{
	/* Toggle the state of LED_3 */
	LedToggle(LED_3);
	/* Notify the hrm_process_event_task to run */
	vTaskNotifyGiveFromISR(gesture_process_event_handle, pdFALSE);
}

/**
 * @brief HRM main process task	
 */
static void gesture_task(void *pvParameter)
{
	while (true)
	{
		LedToggle(LED_2);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    /* La tarea espera en este punto hasta recibir una notificación */
		if (APDS9960_isGestureAvailable())
		{
			switch (APDS9960_readGesture())
			{
			case DIR_UP:
				printf("up\r\n");
				Duty_cycle = Duty_cycle + percent;
				PWMSetDutyCycle(PWM_0, Duty_cycle);
				break;
			case DIR_DOWN:
				printf("down\r\n");
				Duty_cycle = Duty_cycle - percent;
				PWMSetDutyCycle(PWM_0, Duty_cycle);
				break;
			case DIR_LEFT:;
				printf("left\r\n");
				PWMOff(PWM_0);
				break;
			case DIR_RIGHT:
				printf("right\r\n");
				PWMOn(PWM_0);
				break;
			case DIR_NEAR:
				printf("near\r\n");
				break;
			case DIR_FAR:
				printf("far\r\n");
				break;
			default:
				printf("NONE\r\n");
			}
		}
	}
}

/*==================[external functions definition]==========================*/
void app_main(void){
	GPIOInit(GPIO_1, GPIO_INPUT);
	GPIOInit(GPIO_2, GPIO_OUTPUT);
	GPIOActivInt(GPIO_1, pint_intr_callback, 0, NULL);
	LedsInit();
	printf("Init APDS9960 test.\r\n");
	I2C_initialize(100000);
    if(!APDS9960_initialize())
	{
		printf("APDS9960 initialize failed.\r\n");
	};
	APDS9960_enableGestureSensor(true);
	PWMInit(PWM_0, GPIO_2, frec);
	printf("Init PWM.\r\n");
    xTaskCreate(&gesture_task, "GESTURE LOOP", 4096, NULL, 5, &gesture_process_event_handle);
}
/*==================[end of file]============================================*/