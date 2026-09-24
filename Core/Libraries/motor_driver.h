/*
 * motor_driver.h
 *
 * Created on: Nov 1, 2025
 * Author: HP
 *
 * Capa de abstracción de hardware (HAL) para el control cinemático diferencial.
 * Gestiona la interfaz de potencia (Puente H) aislando la lógica de navegación
 * de los registros de los temporizadores (PWM) y los pines de dirección.
 */

#ifndef LIBRARIES_MOTOR_DRIVER_H_
#define LIBRARIES_MOTOR_DRIVER_H_

#include "main.h"

// ── ESTRUCTURA DE COMANDO CINEMÁTICO ──────────────────────────────────
// Define el vector de tracción del chasis.
// La velocidad se abstrae de forma porcentual desde -100 (reversa máxima) a 100 (avance máximo),
// permitiendo que la capa de navegación no necesite conocer la resolución real del Timer PWM (0-999).
typedef struct {
    int8_t left_speed;    // Vector de velocidad de la oruga/rueda izquierda
    int8_t right_speed;   // Vector de velocidad de la oruga/rueda derecha

    // Bandera de control de inercia crítica:
    // 0 = Activa el suavizado cinemático (Ramping) para proteger la mecánica y la electrónica.
    // 1 = Salto instantáneo (Bypass). Obliga al puente H a inyectar la potencia de golpe.
    //     Uso exclusivo para paros de emergencia (STOP) o evasión reactiva por ultrasonido.
    uint8_t bypass_ramp;
} MotorCommand_t;

// Prototipos de la API del controlador de tracción
void Motor_Init(TIM_HandleTypeDef *htim);
void Motor_Process_Command(TIM_HandleTypeDef *htim, const MotorCommand_t *cmd);
void Motor_Update(TIM_HandleTypeDef *htim);

#endif /* LIBRARIES_MOTOR_DRIVER_H_ */
