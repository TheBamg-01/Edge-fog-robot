/*
 * uartRingBuffer.c
 *
 *  Created on: Oct 11, 2025
 *      Author: HP
 *
 *  Implementación del gestor de colas asíncronas para UART.
 *  Utiliza deshabilitación de interrupciones (__disable_irq) para crear
 *  Secciones Críticas (Critical Sections), previniendo Condiciones de Carrera (Race Conditions)
 *  entre las tareas de FreeRTOS y los eventos de hardware.
 */

#include "uartRingBuffer.h"

// Directiva de compilación para seguridad de arquitectura.
// Advierte al desarrollador si la RAM asignada es estructuralmente insuficiente
// para soportar las ráfagas del protocolo LoRa, previniendo fallos en tiempo de ejecución.
#if UART_BUFFER_SIZE < 256
    #warning "UART_BUFFER_SIZE es muy pequeño para LoRa. Recomendado: 512 o 1024"
#endif

// --- Función interna para almacenar un carácter (Contexto ISR) ---
// Se ejecuta EXCLUSIVAMENTE dentro de la interrupción de hardware. No requiere
// deshabilitar IRQs porque ya estamos en el nivel más alto de prioridad.
static void store_char(unsigned char c, ring_buffer_t *buffer)
{
  // Cálculo del siguiente índice utilizando aritmética de anillo (módulo)
  int i = (unsigned int)(buffer->head + 1) % UART_BUFFER_SIZE;

  // Prevención de Overrun: Solo guardamos si el anillo no ha dado la vuelta completa
  // alcanzando a la cola (tail). Si choca, sacrificamos el nuevo byte para preservar
  // la integridad cronológica de los datos más antiguos.
  if(i != buffer->tail) {
    buffer->buffer[buffer->head] = c;
    buffer->head = i; // Avanzamos el puntero atómicamente
  }
  // Si el buffer está lleno (i == tail), el carácter se descarta silenciosamente
}

// Inicializa las estructuras lógicas y excita el silicio
void Ringbuf_Init(UART_RingBuffer_t *rb, UART_HandleTypeDef *huart)
{
  rb->huart = huart;
  rb->rx_buffer.head = 0;
  rb->rx_buffer.tail = 0;
  rb->tx_buffer.head = 0;
  rb->tx_buffer.tail = 0;

  // Habilitar interrupciones de hardware directamente en el periférico
  // UART_IT_ERR: Despierta al CPU en caso de ruido de línea (Framing/Noise error)
  // UART_IT_RXNE: Despierta al CPU cada vez que el registro de desplazamiento recibe 1 byte físico
  __HAL_UART_ENABLE_IT(rb->huart, UART_IT_ERR);
  __HAL_UART_ENABLE_IT(rb->huart, UART_IT_RXNE);
}

// Lee un carácter (Extracción FIFO)
int Uart_Read(UART_RingBuffer_t *rb)
{
  // Si la cabeza y la cola apuntan al mismo bloque, la memoria está vacía
  if(rb->rx_buffer.head == rb->rx_buffer.tail) {
    return -1; // Código universal de buffer vacío
  } else {
    // Extraer el byte y avanzar la cola destructivamente
    unsigned char c = rb->rx_buffer.buffer[rb->rx_buffer.tail];
    rb->rx_buffer.tail = (unsigned int)(rb->rx_buffer.tail + 1) % UART_BUFFER_SIZE;
    return c;
  }
}

// Escribe un carácter al buffer de transmisión
void Uart_Write(UART_RingBuffer_t *rb, int c)
{
	if (c >= 0) {
		int i = (rb->tx_buffer.head + 1) % UART_BUFFER_SIZE;

		// --- OPTIMIZACIÓN ANTI-BLOQUEO (Timeout Guard) ---
		// Si el buffer de salida está lleno porque el periférico es más lento (ej. 9600 baudios)
		// que el procesador, esperamos un máximo de 50ms antes de abortar.
		// Previene que una falla en el módulo LoRa cuelgue por completo el sistema operativo.
		uint32_t start_tick = HAL_GetTick();
		while (i == rb->tx_buffer.tail) {
			if ((HAL_GetTick() - start_tick) > 50) {
				// Opcional: Descomentar para debug
				// printf("WARN: TX buffer full\r\n");
				return; // Timeout: Salir para proteger el scheduler de FreeRTOS
			}
		}

		// --- SECCIÓN CRÍTICA (Protección de Memoria Compartida) ---
		// Apagamos momentáneamente todas las interrupciones del microcontrolador.
		// Evita que la ISR del UART se dispare a la mitad de la actualización de los punteros
		// y corrompa la estructura del anillo matemático.
		__disable_irq();
		rb->tx_buffer.buffer[rb->tx_buffer.head] = (uint8_t)c;
		rb->tx_buffer.head = i;
		__enable_irq();

        // Activamos la interrupción de transmisión vacía (TXE).
        // Esto le dice al hardware: "Hay datos nuevos, empieza a vaciar el buffer en background".
		__HAL_UART_ENABLE_IT(rb->huart, UART_IT_TXE);
	}
}

// Serializador de cadenas. Encola byte por byte hacia el anillo de transmisión
void Uart_SendString(UART_RingBuffer_t *rb, const char *s)
{
	while(*s) Uart_Write(rb, *s++);
}

// Comprueba la profundidad de llenado (Cantidad de datos pendientes)
int IsDataAvailable(UART_RingBuffer_t *rb)
{
  // Operación matemática Thread-safe
  // Si la ISR inserta un byte exactamente durante este cálculo, el resultado sería basura.
  __disable_irq();
  int available = (uint16_t)(UART_BUFFER_SIZE + rb->rx_buffer.head - rb->rx_buffer.tail) % UART_BUFFER_SIZE;
  __enable_irq();
  return available;
}

// Reseteo duro y seguro de la memoria
void Uart_Flush(UART_RingBuffer_t *rb)
{
	__disable_irq();
	rb->rx_buffer.head = 0;
	rb->rx_buffer.tail = 0;
    // Forzamos ceros en toda el área de memoria para eliminar rastros criptográficos o datos corruptos
	memset(rb->rx_buffer.buffer, '\0', UART_BUFFER_SIZE);
	__enable_irq();
}

// Parser asíncrono avanzado: Extrae tramas completas sin bloquear la CPU
int Ringbuf_Read_Line(UART_RingBuffer_t *rb, char *buffertocopyinto, int max_len)
{
    // 1. Capturar snapshot de 'head' de forma atómica.
    // Esto asegura que la búsqueda del salto de línea ('\n') no se vuelva loca
    // si el hardware sigue inyectando bytes nuevos mientras buscamos.
    __disable_irq();
    int current_head = rb->rx_buffer.head;
    __enable_irq();

    int i = rb->rx_buffer.tail;
    int line_found = 0;

    // 2. Escaneo rápido de validación léxica
    while (i != current_head) {
        if (rb->rx_buffer.buffer[i] == '\n') {
            line_found = 1; // Trama validada
            break;
        }
        i = (i + 1) % UART_BUFFER_SIZE;
    }

    if (!line_found) {
        // OPCIONAL: Mecanismo de autolimpieza.
        // Si el buffer está al 90% de capacidad y aún no hay delimitadores,
        // asumimos desalineación de baudios o ruido eléctrico continuo.
        int bytes_available = (UART_BUFFER_SIZE + current_head - rb->rx_buffer.tail) % UART_BUFFER_SIZE;
        if (bytes_available > (UART_BUFFER_SIZE - 64)) {
            // Buffer casi lleno sin \n encontrado -> probablemente basura
            // Descomentar siguiente línea para auto-limpieza agresiva:
            // Uart_Flush(rb);
        }
        return 0; // Indicador de "Siga esperando"
    }

    // 3. Extracción segura (Sanitización del Payload)
    int char_count = 0;
    while (rb->rx_buffer.tail != (i + 1) % UART_BUFFER_SIZE) {
        if (char_count < (max_len - 1)) {
            char c = Uart_Read(rb);
            if (c != '\r' && c != '\n') { // Filtramos los caracteres de control de carro (CR/LF)
                buffertocopyinto[char_count++] = c; // Transferimos solo la carga útil al buffer de destino
            }
        } else {
            Uart_Read(rb); // Estrategia de desgaste: Si el destino rebosó, descartamos el sobrante para vaciar la cola RX
        }
    }

    buffertocopyinto[char_count] = '\0'; // Terminador de cadena estricto en C
    return 1;
}

// ── NÚCLEO DE TIEMPO REAL: ISR (Interrupt Service Routine) ────────────────
// Esta función debe anclarse a los callbacks nativos de las interrupciones del microcontrolador.
// Evita el uso de las funciones HAL genéricas para reducir drásticamente
// la latencia (Overhead), operando directamente sobre los registros físicos (READ_REG/WRITE_REG).
void Uart_Isr(UART_RingBuffer_t *rb)
{
	uint32_t isrflags = READ_REG(rb->huart->Instance->SR);  // Lectura atómica del Status Register
	uint32_t cr1its   = READ_REG(rb->huart->Instance->CR1); // Lectura atómica del Control Register 1

    // EVENTO: Interrupción de Recepción (RXNE - Receive Data Register Not Empty)
    if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
        // Leer el registro SR seguido de DR limpia automáticamente la bandera de interrupción por hardware
		READ_REG(rb->huart->Instance->SR);
        unsigned char c = READ_REG(rb->huart->Instance->DR); // Extrae físicamente el byte del silicio
        store_char(c, &(rb->rx_buffer)); // Inyecta en el anillo circular
        return; // Retorno temprano para minimizar ciclos de reloj (Branch prediction friendly)
    }

    // EVENTO: Interrupción de Transmisión (TXE - Transmit Data Register Empty)
    if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
    {
    	if(rb->tx_buffer.head == rb->tx_buffer.tail) {
              // Si la cola de transmisión se vació por completo, APAGAMOS la interrupción TXE.
              // De lo contrario, el CPU se quedaría atrapado en un bucle infinito disparando esta ISR innecesariamente.
    	      __HAL_UART_DISABLE_IT(rb->huart, UART_IT_TXE);
    	} else {
              // Extraer el siguiente byte encolado e inyectarlo en la tubería del silicio (Shift Register)
    	      unsigned char c = rb->tx_buffer.buffer[rb->tx_buffer.tail];
    	      rb->tx_buffer.tail = (rb->tx_buffer.tail + 1) % UART_BUFFER_SIZE;

              // Limpieza de bandera y transmisión física
    	      READ_REG(rb->huart->Instance->SR);
    	      WRITE_REG(rb->huart->Instance->DR, c);
    	}
    	return;
    }
}
