/*! @mainpage Linterna operada con gestos
 *
 * @section genDesc General Description
 * Aplicacion que controla una linterna operada mediante gestos y distancia. Se enciende, se apaga, 
 * aumenta y disminuye su intensidad segun el gesto detectado por el sensor. También
 * varía su intensidad de acuerdo a la distancia medida por el sensor de ultrasonido.
 *
 * @section hardConn Hardware Connection
 *
 * |    HC-SR04     |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	Vcc 	    |	5V      	|
 * | 	Echo		| 	GPIO_3		|
 * | 	Trig	 	| 	GPIO_2		|
 * | 	Gnd 	    | 	GND     	|
 * 
 * | APDS9960(I2C)  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	Vcc 	    |	5V      	|
 * | 	SDA	    	| 	GPIO_6		|
 * | 	SCL 	 	| 	GPIO_7		|
 * | 	Gnd 	    | 	GND     	|
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
 * @brief Intervalo de refresco para la tarea de medición de distancia (en us).
 */
#define REFRESCO_MEDICION 100000

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

/**
 * @brief Variable para almacenar la ultima lectura de distancia medida por el sensor ultrasónico (en cm).
 */
uint16_t lectura_anterior = 100;

/*==================[internal functions declaration]=========================*/
/**
 * @fn void FuncTimerMedir(void)
 * @brief Función llamada por un temporizador para notificar a la tarea de medición de distancia.
 * 
 * Envía una notificación a la tarea encargada de realizar la medición de distancia para que se ejecute.
 * 
 * @return 
 */
void FuncTimerMedir(void)
{
	LedToggle(LED_1);
    vTaskNotifyGiveFromISR(Medir_task_handle, pdFALSE); /* Envía una notificación a la tarea asociada a medir*/
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
 * @fn OperarConDistancia()
 * @brief Tarea principal que se encarga de recibir la medida tomada por el sensor de distancia y operar con ella.
 * 
 * Si la distancia que toma el sensor es menor a 30 cm y la ultima lectura era mayor a 30 cm cambia el ciclo de trabajo del PWM a la mitad.
 * Si la distancia que toma el sensor es mayor a 30 cm y la ultima lectura era menor a 30 cm cambia el ciclo de trabajo del PWM al doble.
 * Si la distancia actual y la tomada anteriormente esta dentro de la ventana de mas o menos de 30 cm no varía el ciclo de trabajo del PWM.
 * 
 * @return 
 */
void OperarConDistancia()
{
    while (true)
    {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /*la tarea espera en este punto hasta recibir la notificacion*/
		
		lectura_actual=HcSr04ReadDistanceInCentimeters();

		if ((lectura_actual < 30) && (lectura_anterior > 30))
		{
			printf("cercano\r\n");
			Duty_cycle = Duty_cycle/2;
			PWMSetDutyCycle(PWM_0, Duty_cycle);
		}
		else if ((lectura_actual < 30) && (lectura_anterior < 30))
		{
            continue;
		}
		else if ((lectura_actual > 30) && (lectura_anterior < 30))
		{
			printf("lejano\r\n");
			Duty_cycle = (Duty_cycle > 50) ? 100 : Duty_cycle * 2;
			PWMSetDutyCycle(PWM_0, Duty_cycle);
			
		}
		else if((lectura_actual > 30) && (lectura_anterior > 30))
		{
            continue;
		}
		lectura_anterior=lectura_actual;
    }
}

/**
 * @fn gesture_task
 * @brief Tarea principal para el procesamiento de gestos.
 * 
 * Esta tarea se encarga de monitorear continuamente los gestos detectados por el sensor APDS9960 
 * y ajustar el ciclo de trabajo del PWM.
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
 * @fn app_main(void)
 * @brief Función principal de la aplicación.
 * 
 * Esta función realiza las siguientes inicializaciones y configuraciones:
 * - Inicializa los LEDs y los GPIO necesarios para la interacción con periféricos.
 * - Configura y activa el sensor ultrasónico y el sensor de gestos APDS9960.
 * - Configura el temporizador `timer_medicion` para la medición de distancia.
 * - Inicia el módulo PWM para el control de salida.
 * - Crea las tareas `gesture_task` y `OperarConDistancia` para el procesamiento
 *   de gestos y la medición de distancia respectivamente.
 * - Activa la interrupción de GPIO para el control de eventos.
 *
 * @return 
 */
void app_main(void) {
    // Inicialización de LEDs y GPIO
    LedsInit();
    GPIOInit(GPIO_1, GPIO_INPUT); // Configuración de GPIO_1 como entrada.
    GPIOInit(GPIO_9, GPIO_OUTPUT); // Configuración de GPIO_9 como salida.
    
    // Inicialización del sensor ultrasónico
    HcSr04Init(GPIO_3, GPIO_2);

    //Configuración del temporizador para la medición de distancia
    timer_config_t timer_medicion = {
        .timer = TIMER_A, // Temporizador seleccionado.
        .period = REFRESCO_MEDICION, // Período de refresco de medición.
        .func_p = FuncTimerMedir, // Función callback para el temporizador.
        .param_p = NULL // Parámetro opcional para la función callback.
    };
    TimerInit(&timer_medicion);

    // Configuración de interrupción para GPIO
    GPIOActivInt(GPIO_1, pint_intr_callback, 0, NULL);
    printf("Init APDS9960 test.\r\n");

    // Inicialización de I2C y sensor APDS9960
    I2C_initialize(100000); // Inicialización de I2C a 100 kHz.
    if (!APDS9960_initialize()) 
	{
		printf("APDS9960 initialize failed.\r\n"); // Error de inicialización del sensor APDS9960.
    }
    APDS9960_enableGestureSensor(true); // Habilita el sensor de gestos APDS9960.
    
    // Configuración de PWM en GPIO_9
    PWMInit(PWM_0, GPIO_9, frec); // Inicialización del PWM con frecuencia especificada en `frec`. 
    printf("Init PWM.\r\n");

    // Creación de tareas en FreeRTOS
    xTaskCreate(&gesture_task, "GESTURE LOOP", 4096, NULL, 5, &gesture_process_event_handle); // Tarea para el procesamiento de gestos. 
    xTaskCreate(&OperarConDistancia, "medir", 4096, NULL, 5, &Medir_task_handle); // Tarea para la medición de distancia. 

    // Inicio del conteo de temporizadores
    TimerStart(timer_medicion.timer); // Inicia el temporizador para la medición.
}
/*==================[end of file]============================================*/