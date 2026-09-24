/*
 * ICM20948.h
 *
 *  Created on: Dec 17, 2025
 *      Author: HP
 *
 *  Capa de abstracción de hardware (HAL) para la Unidad de Medición Inercial (IMU) de 9 grados de libertad.
 *  Este encabezado expone la topología de registros paginados (Bancos) del chip de TDK InvenSense,
 *  junto con las direcciones del bus I2C secundario interno que comunica al magnetómetro AK09916.
 */
#ifndef LIBRARIES_ICM20948_H_
#define LIBRARIES_ICM20948_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// ── DIRECCIONAMIENTO FÍSICO I2C ─────────────────────────────────────────
// Direcciones I2C (Desplazadas 1 bit a la izquierda para cumplir con la arquitectura del HAL de STM32)
// El scanner detectó la dirección base 0x68 -> 0x68 << 1 = 0xD0 (Bit de lectura/escritura liberado)
#define ICM20948_ADDR           (0x68 << 1)

// El Magnetómetro (AK09916) es un dado de silicio independiente encapsulado en el mismo chip.
// Vive en la dirección 0x0C y permanece oculto en el bus principal hasta que se activa el puente "Bypass".
#define AK09916_MAG_ADDR        (0x0C << 1)

// ── MAPA DE REGISTROS (BANCO 0 PRINCIPAL) ───────────────────────────────
#define REG_BANK_SEL            0x7F // Registro maestro para la paginación de memoria
#define REG_WHO_AM_I            0x00 // Identificador de silicio
#define REG_PWR_MGMT_1          0x06 // Control de energía y osciladores (PLL)
#define REG_INT_PIN_CFG         0x0F // Configuración del pin de interrupción y activación del Bypass I2C
#define REG_ACCEL_XOUT_H        0x2D // Puntero de inicio para lectura en ráfaga (Burst Read) del acelerómetro
#define REG_GYRO_XOUT_H         0x33 // Puntero de inicio para el giroscopio

// ── MAPA DE REGISTROS MAGNETÓMETRO (AK09916) ────────────────────────────
#define MAG_HXL                 0x11 // Puntero de inicio de datos magnéticos (X-axis Low Byte)
#define MAG_CNTL2               0x31 // Registro de control de operación (Tasas de muestreo y apagado)
#define MAG_ST1                 0x10 // Status 1: Bandera de "Dato Listo" (Data Ready)
#define MAG_ST2                 0x18 // Status 2: Validación de lectura y desbordamiento magnético

// ── MAPA DE REGISTROS (BANCO 2) ─────────────────────────────────────────
#define REG_GYRO_CONFIG_1 0x01       // Configuración del Filtro Pasa Bajas Digital (DLPF) del giroscopio

// ── FIRMAS DE VALIDACIÓN DE HARDWARE ────────────────────────────────────
// ID esperado para asegurar que estamos hablando con el chip correcto antes de iniciar la configuración
#define ICM20948_WHO_AM_I_VAL   0xEA
#define AK09916_WHO_AM_I_VAL    0x09 // ID del magnetómetro (opcional verificar)

// ── ESTRUCTURA VECTO-FÍSICA ─────────────────────────────────────────────
// Consolida las mediciones inerciales crudas transformadas a unidades de ingeniería reales.
// Se inyecta en el flujo del RTOS para que el filtro complementario y la navegación esférica operen.
typedef struct {
    float ax, ay, az;    // Vector de aceleración en gravedades (g)
    float gx, gy, gz;    // Velocidad angular en grados por segundo (dps)
    float mx, my, mz;    // Campo magnético en microteslas (uT)
} ICM_Data_t;

// Matrices de calibración magnética cargadas en RAM (Hard Iron / Soft Iron)
extern float mag_hard_iron_bias[3];
extern float mag_soft_iron_scale[3];

// ── API DEL MÓDULO (INTERFAZ PÚBLICA) ───────────────────────────────────
bool ICM_Init(I2C_HandleTypeDef *hi2c);
void ICM_CalibrarMagnetometro(I2C_HandleTypeDef *hi2c);
bool ICM_InitGyroDLPF(I2C_HandleTypeDef *hi2c);
float ICM_GetHeading_TiltCompensated(ICM_Data_t *imu_data);
uint8_t ICM_ReadAccelGyro(I2C_HandleTypeDef *hi2c, ICM_Data_t *data);
uint8_t ICM_ReadMag(I2C_HandleTypeDef *hi2c, ICM_Data_t *data);
void ICM_SelectBank(I2C_HandleTypeDef *hi2c, uint8_t bank);

#endif /* LIBRARIES_ICM20948_H_ */
