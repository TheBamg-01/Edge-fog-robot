/*
 * rylr998.c
 *
 *  Created on: Oct 15, 2025
 *      Author: HP
 *
 *  Implementación del controlador del módulo LoRa RYLR998.
 *  Interactúa con la interfaz serial enviando comandos AT estructurados y
 *  construye un analizador léxico (Parser) seguro para decodificar las
 *  respuestas entrantes, protegiendo al RTOS de desbordamientos de memoria.
 */

#include "rylr998.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cmsis_os.h"

// ── INIT: Aprovisionamiento de Hardware ──────────────────────────────────
// Vincula los periféricos al objeto estructurado y aplica la configuración de red.
// El módulo RYLR998 requiere tiempos muertos (Delays) generosos después de cada comando
// para guardar los parámetros en su memoria Flash/EEPROM interna antes de aceptar otro.
void rylr998_init(RYLR998_t *lora, UART_HandleTypeDef *huart, UART_RingBuffer_t *rb, int address, int network_id)
{
    lora->huart = huart; // Asignación del periférico de transmisión
    lora->rb = rb;       // Asignación de la memoria circular de recepción

    printf("Inicializando modulo LoRa...\r\n");
    char cmd_buffer[50]; // Buffer temporal exclusivo para armar comandos AT

    // Configurar dirección única del nodo en la malla (Ej. Robot = 2, Base = 1)
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+ADDRESS=%d\r\n", address);
    Uart_SendString(lora->rb, cmd_buffer);
    osDelay(500); // 500ms de holgura para escritura en silicio

    // Configurar identificador de red (Capa de aislamiento para no chocar con otras redes LoRa cercanas)
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+NETWORKID=%d\r\n", network_id);
    Uart_SendString(lora->rb, cmd_buffer);
    osDelay(500);

    // Configurar banda de frecuencia (915 MHz para la región de América/ITU Región 2)
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+BAND=915000000\r\n");
    Uart_SendString(lora->rb, cmd_buffer);
    osDelay(500);

    printf("Modulo LoRa inicializado.\r\n");
    // Purgamos basura inicial del arranque para dejar el receptor limpio
    Uart_Flush(lora->rb);
}

// ── POLLING DE SINCRONIZACIÓN ──────────────────────────────────────────
// Máquina de estados de espera semi-bloqueante. Utiliza el tick del RTOS para
// un timeout seguro, evitando que el hilo colapse infinitamente si el módulo
// de radio pierde energía, se quema o el cable RX se desconecta.
static int wait_for_ok(RYLR998_t *lora, uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick(); // Marca de inicio
    char line_buffer[64];

    // Bucle vigilante delimitado por la ventana de tiempo (Timeout)
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        // Extraemos líneas del anillo a medida que llegan por interrupción
        if (Ringbuf_Read_Line(lora->rb, line_buffer, sizeof(line_buffer))) {
            if (strncmp(line_buffer, "+OK", 3) == 0) {
                return 1; // Transacción confirmada por el módulo
            }
            if (strncmp(line_buffer, "+ERR", 4) == 0) {
                //printf("LORA ERROR: %s\r\n", line_buffer);
                return 0; // ❌ Comando rechazado por sintaxis o estado del módulo
            }
        }
        osDelay(10); // Cedemos la CPU al RTOS brevemente para no asfixiar otras tareas
    }

    //printf("LORA TIMEOUT: No se recibió +OK\r\n");
    return 0; // ❌ El tiempo se agotó sin respuesta (Falla física o de enlace)
}

// ── TRANSMISIÓN DE CARGA ÚTIL ──────────────────────────────────────────
// Empaqueta dinámicamente un string en el formato estricto AT+SEND=Dirección,Longitud,Datos
void rylr998_send_data(RYLR998_t *lora, int address, const char *data)
{
    char tx_command[256]; // Buffer ensanchado para soportar rutas complejas o JSONs pequeños

    // Limpiar buffer RX antes de enviar (evitar basura)
    //Uart_Flush(lora->rb);

    // Construcción del frame AT calculando la longitud exacta requerida por el módulo
    int payload_len = strlen(data);
    snprintf(tx_command, sizeof(tx_command), "AT+SEND=%d,%d,%s\r\n", address, payload_len, data);

    // Disparamos la trama por hardware
    Uart_SendString(lora->rb, tx_command);

    // Bloqueamos el hilo temporalmente (hasta 1 seg) esperando el Acuse de Recibo local (ACK)
   if (!wait_for_ok(lora, 1000)) {

	   //printf("WARN: Envío LoRa falló o sin confirmación\r\n");
    }

    // Retardo estructural que garantiza que la modulación de radiofrecuencia (Chirp Spread Spectrum)
    // termine de transmitirse por el aire antes de que el procesador intente otra acción intensiva.
    osDelay(100);
}

// ── PARSER DE RECEPCIÓN ────────────────────────────────────────────────
// Analizador léxico para decodificar tramas entrantes. Esta es la frontera
// entre el exterior físico y la memoria del sistema, por lo que incluye barreras
// de protección rígidas contra punteros desalineados y desbordamientos de memoria (Buffer Overflow).
int rylr998_receive_data(RYLR998_t *lora)
{
    char line_buffer[256];

    // Intentar extraer una cadena completa (delimitada por \r\n) del búfer circular
    if (Ringbuf_Read_Line(lora->rb, line_buffer, sizeof(line_buffer)))
    {
        // Filtro rápido: Ignoramos ecos locales del módulo
        if (strncmp(line_buffer, "+OK", 3) == 0) return 0;
        if (strncmp(line_buffer, "+ERR", 4) == 0) return 0;

        // Validar si la trama corresponde a una recepción aérea de otro nodo
        if (strncmp(line_buffer, "+RCV=", 5) == 0)
        {
            // Formato esperado de la trama externa: +RCV=Addr,Length,Data,RSSI,SNR

            // 1. Obtener Length (Longitud declarada de los datos)
            // Aritmética de punteros: Buscamos la primera coma separadora
            char *first_comma = strchr(line_buffer, ',');
            if (!first_comma) return 0; // Trama malformada, abortar

            int data_len = atoi(first_comma + 1); // Extraemos la longitud nominal

            // 2. Encontrar el inicio exacto de la carga útil (Data)
            // Buscamos la segunda coma a partir de la primera
            char *second_comma = strchr(first_comma + 1, ',');
            if (!second_comma) return 0; // Trama truncada, abortar

            char *data_start = second_comma + 1; // El payload inicia un byte después de la coma

            // 3. CLAMP DE SEGURIDAD DE MEMORIA (Crítico)
            // Previene que un paquete corrupto o un atacante sature el buffer de destino
            if (data_len >= sizeof(lora->received_data)) {
                data_len = sizeof(lora->received_data) - 1; // Truncamos al límite seguro
            }

            // Validar coherencia espacial de la longitud declarada
            if (data_len <= 0 || data_len > 240) {
                //printf("WARN: data_len inválido: %d\r\n", data_len);
                return 0; // Rechazar paquete corrupto
            }

            // 4. Extracción Segura
            // Limpiamos el buffer de destino garantizando que no haya bytes fantasma de la iteración anterior
            memset(lora->received_data, 0, sizeof(lora->received_data));

            // Copiamos EXACTAMENTE 'data_len' bytes desde el puntero inicial
            strncpy(lora->received_data, data_start, data_len);

            // Forzamos el carácter nulo finalizador de cadena en C para proteger funciones como printf() o strcmp()
            lora->received_data[data_len] = '\0';

            //printf("RX LORA: [%s] (%d bytes)\r\n", lora->received_data, data_len);
            return 1; // Paquete válido y decodificado exitosamente en RAM
        }
    }
    return 0; // No hay datos nuevos o eran ruido
}
