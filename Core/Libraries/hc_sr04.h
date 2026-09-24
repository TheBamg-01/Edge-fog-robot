/*
 * hc_sr04.h
 *
 *  Created on: Oct 11, 2025
 *      Author: HP
 *
 *  Librería de abstracción para la gestión de sonares HC-SR04 mediante Input Capture.
 *  Proporciona una interfaz orientada a objetos para instanciar múltiples sensores
 *  sin colisiones de memoria ni bloqueos de CPU.
 */
#ifndef INC_HC_SR04_H_
#define INC_HC_SR04_H_

#include "main.h"

// Definimos una estructura para manejar cada sensor individualmente.
// Este encapsulamiento aísla la configuración del hardware físico y la memoria temporal
// de cada sonar, permitiendo escalar a N sensores (Frontal, Lateral, etc.) sin interferencias.
typedef struct {
	// Configuración Hardware (Mapeo de periféricos)
	GPIO_TypeDef* TRIG_Port;    // Puerto físico del microcontrolador conectado al pin TRIG
	uint16_t            TRIG_Pin; // Pin específico para emitir el pulso de excitación
	TIM_HandleTypeDef* TIM_Handle; // Referencia al temporizador de hardware (Ej: htim1)
	uint32_t            TIM_Channel; // Canal del timer configurado en modo Input Capture (Ej: TIM_CHANNEL_1)

	// Variables de estado (Internas del ciclo de medición)
	// El calificador 'volatile' es una directiva arquitectónica estricta aquí: garantiza que el
	// compilador no optimice estas variables en registros del CPU, ya que su valor muta
	// asíncronamente y en segundo plano impulsado por las interrupciones del hardware (ISR).
	volatile uint32_t   t_ini;          // Marca de tiempo (ticks) en la que salió la onda de sonido
	volatile uint32_t   t_end;          // Marca de tiempo (ticks) en la que rebotó el eco
	volatile uint8_t    flag_captured;  // Máquina de estados: 0 = Esperando flanco de subida, 1 = Esperando bajada
	volatile uint16_t   distance_cm;    // Distancia final calculada (Lista para ser leída de forma segura por el RTOS)
} HCSR04_Sensor_t;

// Función de inicialización
// Vincula el hardware inicial configurado por STM32CubeMX con la estructura de control lógico
void HCSR04_Init_Sensor(HCSR04_Sensor_t* sensor, GPIO_TypeDef* port, uint16_t pin, TIM_HandleTypeDef* tim, uint32_t channel);

// Función de disparo y lectura
void HCSR04_Trigger(HCSR04_Sensor_t* sensor);
uint16_t HCSR04_Read(HCSR04_Sensor_t* sensor);

#endif /* INC_HC_SR04_H_ */
