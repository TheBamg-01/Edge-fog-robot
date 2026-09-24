/*
 * rylr998.h
 *
 *  Created on: Oct 15, 2025
 *      Author: HP
 *
 *  Capa de abstracción (HAL) para el transceptor LoRa RYLR998.
 *  Maneja el estado del hardware de radio y encapsula la comunicación
 *  de comandos AT por puerto serial (UART), apoyándose en un buffer circular
 *  para prevenir la pérdida de paquetes (Overrun) en transmisiones asíncronas.
 */
#ifndef INC_RYLR998_H_
#define INC_RYLR998_H_

#include "main.h"
#include "uartRingBuffer.h" // Dependencia crítica para la recepción no bloqueante

// ── ESTRUCTURAS DE DATOS ───────────────────────────────────────────────

// Estructura auxiliar para aislar las coordenadas extraídas de la telemetría remota
typedef struct {
    float latitude;
    float longitude;
} Coordinates_t;

// Objeto de Contexto del Transceptor (Hardware Handle)
// Agrupa los punteros físicos y los buffers de memoria de una instancia LoRa específica.
// Permite escalar el sistema si en el futuro se añaden múltiples radios al mismo robot.
typedef struct {
    UART_HandleTypeDef  *huart;      // Puntero al periférico UART del microcontrolador (Ej. huart6)
    UART_RingBuffer_t   *rb;         // Puntero al gestor de memoria circular gestionado por interrupciones (DMA/ISR)
    Coordinates_t       received_coords; // Última coordenada válida recibida y parseada
    char                received_data[256]; // Buffer de retención seguro para el payload genérico decodificado
} RYLR998_t;

// ── API DEL MÓDULO ─────────────────────────────────────────────────────

// Rutina de aprovisionamiento y configuración de red de la antena
void rylr998_init(RYLR998_t *lora, UART_HandleTypeDef *huart, UART_RingBuffer_t *rb, int address, int network_id);

// Transmisión y recepción de cargas útiles (Payloads)
void rylr998_send_data(RYLR998_t *lora, int address, const char *data);
int  rylr998_receive_data(RYLR998_t *lora);

#endif /* INC_RYLR998_H_ */
