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
#include "hc_sr04.h"
#include "timer_mcu.h"
/*==================[macros and definitions]=================================*/
/**
 * @def REFRESCO_MEDICION
 * @brief Intervalo de refresco para la tarea de medición de distancia (en ms).
 */
#define REFRESCO_MEDICION 1000000

/*==================[internal data definition]===============================*/
/**
 * @brief Frecuencia de la señal PWM en Hz.
 */
uint16_t frec=1000;
/**
 * @brief Ciclo de trabajo actual del PWM (en porcentaje).
 */
uint16_t Duty_cycle=0;
/**
 * @brief Incremento o decremento del ciclo de trabajo (en porcentaje).
 */
uint16_t percent = 20;

/**
 * @brief Handle para la tarea de medición de distancia.
 */
TaskHandle_t Medir_task_handle = NULL;

/**
 * @brief Manejador de la tarea para el procesamiento de gestos.
 * 
 * Este manejador se utiliza para notificar la tarea cuando ocurre un evento de gesto.
 */
TaskHandle_t gesture_process_event_handle = NULL;

/**
 * @brief Variable para almacenar la distancia medida por el sensor ultrasónico (en cm).
 */
uint16_t lectura_actual = 0;

uint16_t lectura_anterior = 100;

/*==================[internal functions declaration]=========================*/
/**
 * @fn void FuncTimerMedir(void *param)
 * @brief Función llamada por un temporizador para notificar a la tarea de medición de distancia.
 * 
 * Envía una notificación a la tarea encargada de realizar la medición de distancia para que se ejecute.
 * 
 * @param param Parámetro no utilizado.
 * @return 
 */
void FuncTimerMedir(void)
{
	LedToggle(LED_1);
    vTaskNotifyGiveFromISR(Medir_task_handle, pdFALSE); /* Envía una notificación a la tarea asociada a medir*/
}

/**
 * @fn OperarConDistancia
 * @brief 
 * 
 * 
 * @return 
 */
void OperarConDistancia()
{
    while (true)
    {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /*la tarea espera en este punto hasta recibir la notificacion*/
		// aca se realizan las tareas de encender leds dependiendo la distancia y encender el display
		
		lectura_actual=HcSr04ReadDistanceInCentimeters();

		if ((lectura_actual < 30) & (lectura_anterior > 30))
		{
			Duty_cycle = Duty_cycle/2;
		}
		else if ((lectura_actual < 30) & (lectura_anterior < 30))
		{
			break;
		}
		else if ((lectura_actual > 30) & (lectura_anterior < 30))
		{
			if(Duty_cycle>50)
			{
				Duty_cycle=100;
			}
			else
			{
				Duty_cycle=Duty_cycle*2;
			}
		}
		else if ((lectura_actual > 30) & (lectura_anterior > 30))
		{
			break;
		}
		lectura_anterior=lectura_actual;
    }
}

/**
 * @fn pint_intr_callback
 * @brief Callback de interrupción de la señal PINT.
 * 
 * Esta función se ejecuta cuando se detecta una interrupción en el pin PINT. 
 * Se utiliza para alternar el estado de un LED y notificar a la tarea de procesamiento de gestos.
 * @return 
 */
void pint_intr_callback(void)
{
	/* Toggle the state of LED_3 */
	LedToggle(LED_3);
	/* Notify the hrm_process_event_task to run */
	vTaskNotifyGiveFromISR(gesture_process_event_handle, pdFALSE);
}

/**
 * @fn gesture_task
 * @brief Tarea principal para el procesamiento de gestos.
 * 
 * Esta tarea se encarga de monitorear continuamente los gestos detectados por el sensor APDS9960 
 * y ajustar el ciclo de trabajo del PWM o realizar otras acciones según el gesto.
 * 
 * @param pvParameter Parámetro de entrada para la tarea (no utilizado).
 * @return 
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
/**
 * @fn app_main
 * @brief Función principal de la aplicación.
 * 
 * Inicializa los periféricos GPIO, LEDs, I2C, el sensor de gestos APDS9960 y el PWM. 
 * También crea la tarea encargada del procesamiento de gestos.
 */
void app_main(void){
	LedsInit();
	GPIOInit(GPIO_1, GPIO_INPUT);
	GPIOInit(GPIO_9, GPIO_OUTPUT);
	HcSr04Init(GPIO_3, GPIO_2);

	/* Inicialización de timer medicion */
    timer_config_t timer_medicion = {
        .timer = TIMER_A,
        .period = REFRESCO_MEDICION,
        .func_p = FuncTimerMedir,
        .param_p = NULL};
    TimerInit(&timer_medicion);

	GPIOActivInt(GPIO_1, pint_intr_callback, 0, NULL);
	printf("Init APDS9960 test.\r\n");
	I2C_initialize(100000);
    if(!APDS9960_initialize())
	{
		printf("APDS9960 initialize failed.\r\n");
	};
	APDS9960_enableGestureSensor(true);
	PWMInit(PWM_0, GPIO_9, frec);
	printf("Init PWM.\r\n");

    xTaskCreate(&gesture_task, "GESTURE LOOP", 4096, NULL, 5, &gesture_process_event_handle);
	xTaskCreate(&OperarConDistancia, "medir", 4096, NULL, 5, &Medir_task_handle);

	/*Inicio del conteo de timers*/
    TimerStart(timer_medicion.timer);
}
/*==================[end of file]============================================*/