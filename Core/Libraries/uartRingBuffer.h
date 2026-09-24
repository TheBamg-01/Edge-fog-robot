/*
 * uartRingBuffer.h
 *
 *  Created on: Oct 11, 2025
 *      Author: HP
 *
 *  Capa de abstracción para la gestión de buffers circulares (Ring Buffers) asíncronos.
 *  Arquitectura Multi-instancia: Permite crear colas de memoria independientes para
 *  múltiples periféricos UART (ej. LoRa, GPS, Telemetría) sin cruce de datos.
 *  Protege al sistema operativo (RTOS) de bloqueos al desacoplar la velocidad
 *  física de transmisión/recepción de la velocidad de procesamiento del CPU.
 */

#ifndef UARTRINGBUFFER_H_
#define UARTRINGBUFFER_H_

#include "stm32f4xx_hal.h"
#include <string.h>

// Tamaño de la memoria asignada por cada buffer (Rx y Tx).
// Debe ser potencia de 2 para máxima eficiencia, pero aquí se utiliza aritmética
// de módulo estándar para flexibilidad. 512 bytes evita desbordamientos (overrun)
// en ráfagas largas de datos NMEA del GPS o tramas JSON de LoRa.
#define UART_BUFFER_SIZE 512

// Estructura interna de bajo nivel para el control de la memoria circular
typedef struct
{
  unsigned char buffer[UART_BUFFER_SIZE]; // Arreglo contiguo en memoria RAM

  // Modificadores 'volatile' obligatorios:
  // Obligan al compilador a leer siempre desde la RAM y no desde los registros del CPU,
  // ya que estos índices son modificados asíncronamente en background por la Interrupción (ISR).
  volatile unsigned int head; // Puntero de escritura (Donde se inserta el nuevo dato)
  volatile unsigned int tail; // Puntero de lectura (Donde se extrae el dato más antiguo)
} ring_buffer_t;

// ESTRUCTURA PRINCIPAL: Objeto de Contexto (Context Handle)
// Encapsula el hardware y la memoria para instanciar un canal de comunicación completo.
typedef struct
{
	UART_HandleTypeDef *huart;         // Enlace al periférico de hardware de silicio (ej. huart1)
	ring_buffer_t      rx_buffer;      // Cola circular dedicada a la recepción (RX)
	ring_buffer_t      tx_buffer;      // Cola circular dedicada a la transmisión (TX)
} UART_RingBuffer_t;


// --- API del Módulo (Prototipos Thread-Safe) ---

/* Vincula el hardware con las estructuras de memoria y habilita las interrupciones base */
void Ringbuf_Init(UART_RingBuffer_t *rb, UART_HandleTypeDef *huart);

/* Extracción atómica de un byte desde el buffer RX (Uso interno y externo) */
int Uart_Read(UART_RingBuffer_t *rb);

/* Inserción atómica de un byte hacia el buffer TX con protección anti-bloqueo */
void Uart_Write(UART_RingBuffer_t *rb, int c);

/* Transfiere una cadena completa al anillo de TX y dispara la interrupción de hardware */
void Uart_SendString(UART_RingBuffer_t *rb, const char *s);

/* Calcula matemáticamente cuántos bytes sin leer residen actualmente en la RAM */
int IsDataAvailable(UART_RingBuffer_t *rb);

/* Purga de memoria estricta para evitar datos fantasma (Zero-filling) */
void Uart_Flush(UART_RingBuffer_t *rb);

/* Analizador léxico no bloqueante: Extrae tramas completas delimitadas por '\n' */
int Ringbuf_Read_Line(UART_RingBuffer_t *rb, char *buffertocopyinto, int max_len);

/* Rutina de Servicio de Interrupción (ISR): Manipulación directa de registros de silicio */
void Uart_Isr(UART_RingBuffer_t *rb);

#endif /* UARTRINGBUFFER_H_ */
