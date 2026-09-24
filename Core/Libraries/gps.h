/*
 * gps.h
 *
 *  Created on: Oct 11, 2025
 *      Author: HP
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"

// ── ESTRUCTURA DE TELEMETRÍA ESPACIAL ──────────────────────────────────
// Estructura de datos para almacenar la información de posicionamiento global decodificada.
// Actúa como el vector de estado oficial del robot, transfiriendo los datos crudos del hardware
// hacia las tareas de navegación y evasión del RTOS de manera estructurada y unificada.
typedef struct {
    double latitude;           // Latitud en grados decimales (Doble precisión para exactitud submétrica)
    double longitude;          // Longitud en grados decimales
    float utc_time;            // Marca de tiempo universal coordinada (Útil para logs, sincronización y timestamps)
    char ns_indicator;         // Hemisferio Norte/Sur ('N' o 'S')
    char ew_indicator;         // Hemisferio Este/Oeste ('E' o 'W')
    uint8_t fix_quality;       // Estado de la triangulación: 0 = Inválido, 1 = GPS Fix, 2 = DGPS, etc.
    uint8_t satellites_tracked;// Número de satélites enlazados (Determina la confianza física del modelo geométrico HDOP)
    uint8_t data_is_valid;     // Bandera booleana de seguridad: 1 = Datos sanitizados y listos para uso, 0 = Pérdida de señal (Evita navegación a ciegas)
} GPS_Data_t;

// Prototipos de funciones públicas (API del módulo)
// Extrae, procesa y deposita de forma segura la trama NMEA del buffer UART hacia la estructura objetivo
uint8_t gps_process_buffer(char *buffer, GPS_Data_t *gps_data);

#endif /* INC_GPS_H_ */
