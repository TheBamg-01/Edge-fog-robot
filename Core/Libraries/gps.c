/*
 * gps.c
 *
 *  Created on: Oct 11, 2025
 *      Author: HP
 *  Updated: Enero 2026 - Robustez mejorada y validaciones
 *
 *  Módulo de abstracción de hardware para la decodificación de tramas NMEA.
 *  Este archivo actúa como la primera línea de defensa del sistema distribuido,
 *  sanitizando y validando la integridad de los datos espaciales crudos antes
 *  de que el controlador PD aplique fuerza a los motores.
 */

#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// ── CONFIGURACIÓN DE DEPURACIÓN ────────────────────────────────────────
#define GPS_DEBUG 0  // Cambiar a 1 para habilitar el volcado de alertas al monitor serie (Útil para diagnóstico en pruebas de campo)

// ===== RUTINAS DE VALIDACIÓN DE HARDWARE =====

// Función privada para validar el checksum de la trama NMEA
// Calcula el byte de redundancia lógico XOR del sistema para confirmar la integridad estructural de la cadena.
// Es crítico para descartar coordenadas fantasma inyectadas por ruido electromagnético en los cables TX/RX.
static uint8_t gps_validate_checksum(const char *nmea_sentence) {
    uint8_t checksum = 0;
    const char *p = nmea_sentence;

    // Ignorar el marcador de inicio de trama '$'
    if (*p == '$') {
        p++;
    }

    // Calcular el checksum lógico XOR acumulativo de todos los caracteres útiles (entre '$' y '*')
    while (*p && *p != '*') {
        checksum ^= *p++;
    }

    // Si la cadena termina abruptamente sin encontrar el delimitador '*', el paquete serial fue truncado
    if (*p != '*') {
#if GPS_DEBUG
        printf("GPS: Sin '*' de checksum\r\n");
#endif
        return 0; // Rechazo absoluto por falta de integridad
    }

    // Avanzar el puntero para leer el valor hexadecimal provisto por el módulo físico
    p++;

    // Convertir los dos caracteres ASCII hexadecimales del checksum recibido a un número entero
    uint8_t received_checksum = (uint8_t)strtol(p, NULL, 16);

    // Comparar el cálculo matemático del nodo edge contra la firma enviada por el satélite
    uint8_t valid = (checksum == received_checksum);

#if GPS_DEBUG
    if (!valid) {
        printf("GPS: Checksum inválido (calc=%02X, recv=%02X)\r\n", checksum, received_checksum);
    }
#endif

    return valid;
}

// Función de normalización espacial (Geodésica a Decimal)
// Transforma el formato crudo NMEA (DDMM.MMMM) a Grados Decimales puros (DD.DDDD)
// Requisito matemático indispensable para que la función de Haversine del XTE opere sin desviaciones.
static double nmea_to_decimal(double coordinate) {
    // Validar límites físicos de la geometría terrestre (Clamp de seguridad pasiva)
    // Latitud: máx 9000.0000 (90° 00.0000')
    // Longitud: máx 18000.0000 (180° 00.0000')

	if (coordinate < 0.0 || coordinate > 18000.0) {
		return 0.0; // Coordenada aberrante, anular cálculo
	}

    // Extraer la base de los grados enteros truncando los decimales (División sobre 100)
	int degrees = (int)(coordinate / 100);
    // Extraer la fracción de minutos de arco restando la base
	double minutes = coordinate - (double)(degrees * 100);

    // Validación geométrica estricta: Los minutos de arco no pueden superar o igualar a 60
	if (minutes < 0.0 || minutes >= 60.0) {
		return 0.0;
	}

    // Fórmula final de conversión a precisión doble
	return (double)degrees + (minutes / 60.0);
}


// ===== PARSER PRINCIPAL NMEA =====

// Función maestra (Parser). Escanea una línea completa extraída del buffer circular.
// Decodifica la métrica y bloquea o actualiza la memoria central dependiendo del estado de la señal.
uint8_t gps_process_buffer(char *buffer, GPS_Data_t *gps_data) {
    // ── PROTECCIÓN DE MEMORIA (SANDBOXING) ──
    // Crear una copia local aislada del buffer crudo.
    // strsep() modifica destructivamente la cadena original inyectando caracteres nulos ('\0').
    // Al trabajar con una copia, evitamos corromper el buffer original del UART si otra rutina depende de él.
    char local_buffer[128];
    strncpy(local_buffer, buffer, sizeof(local_buffer) - 1);
    local_buffer[sizeof(local_buffer) - 1] = '\0'; // Garantía estricta de terminación en C

    // Primer filtro de hardware: Descartar basura serial sin malgastar ciclos de CPU en el parser
    if (!gps_validate_checksum(local_buffer)) {
        return 0;
    }

    // Reconocimiento de constelación:
    // Aceptar tramas de posicionamiento americano ($GPGGA) o mallas combinadas multired GNSS ($GNGGA)
    if (strncmp(local_buffer, "$GPGGA", 6) == 0 ||
        strncmp(local_buffer, "$GNGGA", 6) == 0) {

        char *token;
        int token_count = 0;

        // Variables temporales en memoria Stack.
        // Regla arquitectónica: NUNCA se sobrescribe la estructura global por referencia (gps_data)
        // hasta no comprobar la legitimidad de todos los campos. Previene dejar el sistema en un estado inválido a la mitad.
        double temp_lat = 0.0;
        double temp_lon = 0.0;
        char temp_ns = 'N';
        char temp_ew = 'E';
        uint8_t temp_fix = 0;
        uint8_t temp_sats = 0;
        float temp_utc = 0.0f;

        char *p = local_buffer;

        // Motor de separación de tokens por delimitador ','
        // strsep() es superior a strtok() en este contexto porque strsep sí respeta campos vacíos consecutivos (,,)
        // manteniendo correctamente la alineación del formato estandarizado GPGGA.
        while ((token = strsep(&p, ",")) != NULL) {
            token_count++;
            switch (token_count) {
                case 2: // Tiempo UTC
                    if (token && strlen(token) > 0) {
                        temp_utc = atof(token);
                    }
                    break;

                case 3: // Latitud Cruda
                    if (token && strlen(token) > 0) {  // ✅ Validar ausencia de dato por pérdida de señal
                        temp_lat = nmea_to_decimal(atof(token));
                    }
                    break;

                case 4: // Indicador N/S (Norte/Sur)
                    if (token && strlen(token) > 0) {
                        temp_ns = token[0];
                    }
                    break;

                case 5: // Longitud Cruda
                    if (token && strlen(token) > 0) {  // ✅ Validar ausencia de dato
                        temp_lon = nmea_to_decimal(atof(token));
                    }
                    break;

                case 6: // Indicador E/W (Este/Oeste)
                    if (token && strlen(token) > 0) {
                        temp_ew = token[0];
                    }
                    break;

                case 7: // Calidad del Fix (Triangulación consolidada)
                    if (token && strlen(token) > 0) {
                        temp_fix = atoi(token);
                    }
                    break;

                case 8: // Satélites en línea de visión (Visibilidad geométrica)
                    if (token && strlen(token) > 0) {
                        temp_sats = atoi(token);
                    }
                    break;
            }
            //token = strtok(NULL, ","); // (Línea heredada removida por problemas técnicos con campos nulos)
        }

        // ── AUTORIZACIÓN DE ESCRITURA GLOBAL ──
        // Solo inyectar en el núcleo de navegación si la telemetría espacial es mecánicamente confiable.
        // temp_fix > 0 confirma el cálculo, y temp_sats >= 4 es el requerimiento matemático mínimo
        // para deducir una posición 3D real compensando el error del reloj atómico.
        if (temp_fix > 0 && temp_sats >= 4) {
            // Aplicar matriz de signos por cuadrantes hemisféricos
            gps_data->latitude = temp_lat;
            if (temp_ns == 'S') {
                gps_data->latitude *= -1.0f; // El hemisferio Sur decrece en negativo
            }

            gps_data->longitude = temp_lon;
            if (temp_ew == 'W') {
                gps_data->longitude *= -1.0f; // El hemisferio Oeste decrece en negativo
            }

            // Consolidar los metadatos y levantar bandera de validez
            gps_data->ns_indicator = temp_ns;
            gps_data->ew_indicator = temp_ew;
            gps_data->fix_quality = temp_fix;
            gps_data->satellites_tracked = temp_sats;
            gps_data->utc_time = temp_utc;
            gps_data->data_is_valid = 1; // Bandera de seguridad en ALTO: El nodo edge puede calcular trayectoria.

#if GPS_DEBUG
            printf("GPS OK: Lat=%.6f, Lon=%.6f, Sats=%d\r\n",
                   gps_data->latitude, gps_data->longitude, gps_data->satellites_tracked);
#endif

            return 1; // Éxito de transferencia
        } else {
            // Entorno denegado: El receptor está bloqueado por edificios, follaje espeso o en arranque frío
#if GPS_DEBUG
            printf("GPS: Sin fix (quality=%d, sats=%d)\r\n", temp_fix, temp_sats);
#endif
            gps_data->data_is_valid = 0; // Bandera de seguridad en BAJO: Forzar detención o inhibición de cálculo XTE
            return 0;
        }
    }

    // La trama descartada pertenecía a un protocolo no objetivo (ej. GPGSV, GPGLL)
    return 0;
}
