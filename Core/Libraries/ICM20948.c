/*
 * ICM20948.c
 *
 * Created on: Dec 17, 2025
 * Author: HP
 * Updated: Dec 29, 2025 (Fix Mag Read)
 *
 * Controlador de hardware a nivel de silicio para el IMU ICM20948.
 * Gestiona el puenteo I2C interno, la reconstrucción de bytes crudos (Endianness mixto),
 * el filtrado mecánico por hardware (DLPF) y la proyección trigonométrica del compás magnético
 * compensado por inclinación (Electronic Gimbaling).
 */

#include "ICM20948.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <float.h>
#include <stdio.h>

// ── CONSTANTES DE ESCALAMIENTO FÍSICO ───────────────────────────────────
// Transforma los ticks crudos del ADC interno de 16-bits a magnitudes físicas utilizables
// (Verificar configuración de rango completo en los registros de control si se cambia la init)
#define ACCEL_SCALE  16384.0f // Sensibilidad para escala de +/- 2g
#define GYRO_SCALE   131.0f   // Sensibilidad para escala de +/- 250 dps (grados por segundo)
#define MAG_SCALE    0.15f    // Resolución fija por datasheet del AK09916 (0.15 uT/LSB)

// ── MATRICES DE CALIBRACIÓN MAGNÉTICA ───────────────────────────────────
// Bias (Hard Iron): Desplaza el centro de la esfera magnética compensando los campos magnéticos
// estáticos generados por los motores y tornillos ferromagnéticos del propio chasis del robot.
float mag_hard_iron_bias[3] = { 4.43f, 0.75f, 46.28f };
// Scale (Soft Iron): Corrige la deformación elipsoidal causada por metales que distorsionan el campo terrestre cercano.
float mag_soft_iron_scale[3] = {0.9561f, 0.8565f, 1.2715f};

// Mecanismo de paginación de memoria. Dado que el ICM20948 excede el espacio de direcciones
// de un solo byte (0x00 a 0xFF), organiza sus registros en 4 "Bancos".
void ICM_SelectBank(I2C_HandleTypeDef *hi2c, uint8_t bank) {
    uint8_t data = (bank << 4);
    HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_BANK_SEL, 1, &data, 1, 10);
}

// Rutina de inicialización segura tolerante a fallos
bool ICM_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;

    // 1. Reset ICM20948 (Cold Boot del Silicio)
    // Limpia registros corruptos por ruidos de voltaje al encender el sistema
    ICM_SelectBank(hi2c, 0); // Asegurar Banco 0
    data = 0x80; // Bit de Reset de hardware
    if(HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_PWR_MGMT_1, 1, &data, 1, 100) != HAL_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(100)); // Esperar a que los osciladores internos se estabilicen

    // 2. Wake Up (Salir del estado Sleep por defecto y elegir la mejor fuente de reloj)
    data = 0x01; // Auto Select Clock (Utiliza el PLL del giroscopio si está disponible para mayor precisión térmica)
    if(HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_PWR_MGMT_1, 1, &data, 1, 100) != HAL_OK) return false;

    // 3. Verificación de Identidad (Handshake I2C)
    uint8_t whoami;
    HAL_I2C_Mem_Read(hi2c, ICM20948_ADDR, REG_WHO_AM_I, 1, &whoami, 1, 100);
    if (whoami != ICM20948_WHO_AM_I_VAL) return false; // Abortar si el bus responde con ruido o no es el chip correcto

    // 4. Bypass Enable (CRÍTICO para topología interna)
    // Desactiva el aislamiento maestro-esclavo interno del ICM y conecta físicamente
    // el bus I2C externo (STM32) directamente con el magnetómetro encapsulado (AK09916).
    data = 0x02; // Bit BYPASS_EN en el registro INT_PIN_CFG (0x0F)
    HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_INT_PIN_CFG, 1, &data, 1, 100);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* --- 5. Configuración del Magnetómetro (AK09916) --- */

    // Paso A: Resetear Mag a PowerDown Mode (0x00)
    // Importante: El magnetómetro necesita un estado de apagado transitorio para aceptar una nueva configuración
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, AK09916_MAG_ADDR, MAG_CNTL2, 1, &data, 1, 100);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Paso B: Activar Continuous Measurement Mode 2 (100Hz -> 0x08)
    // Sincroniza la frecuencia magnética con el ciclo de la tarea de sensores del RTOS
    data = 0x08;
    if(HAL_I2C_Mem_Write(hi2c, AK09916_MAG_ADDR, MAG_CNTL2, 1, &data, 1, 100) != HAL_OK) return false;

    return true; // Arranque exitoso
}

// Lectura unificada de Inercia (Burst Read)
// Lee todos los ejes de golpe para garantizar que las muestras de X, Y, Z pertenezcan exactamente al mismo instante de tiempo.
uint8_t ICM_ReadAccelGyro(I2C_HandleTypeDef *hi2c, ICM_Data_t *data) {
    uint8_t raw_data[12];

    // Asegurar Banco 0 antes de leer (Evita leer ruido si otra tarea cambió el banco)
    ICM_SelectBank(hi2c, 0);

    // Leer 12 bytes secuenciales desde el bus: 6 de Acelerómetro y 6 de Giroscopio
    if(HAL_I2C_Mem_Read(hi2c, ICM20948_ADDR, REG_ACCEL_XOUT_H, 1, raw_data, 12, 10) != HAL_OK) {
        return 1; // Falla de bus, la tarea de recuperación I2C atrapará esto
    }

    // Reconstrucción de palabra (El ICM20948 utiliza arquitectura Big Endian para el accel/gyro)
    // Desplaza el byte alto (H) 8 bits y hace un OR bit a bit con el byte bajo (L)
    int16_t ax_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    int16_t ay_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    int16_t az_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    int16_t gx_raw = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    int16_t gy_raw = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    int16_t gz_raw = (int16_t)((raw_data[10] << 8) | raw_data[11]);

    // Conversión a unidades físicas aplicando la sensibilidad del ADC
    data->ax = ax_raw / ACCEL_SCALE;
    data->ay = ay_raw / ACCEL_SCALE;
    data->az = az_raw / ACCEL_SCALE;

    data->gx = gx_raw / GYRO_SCALE;
    data->gy = gy_raw / GYRO_SCALE;
    data->gz = gz_raw / GYRO_SCALE;

    return 0; // Lectura íntegra
}

// Rutina geométrica de Calibración Hard/Soft Iron
// Crea una "jaula" de valores mínimos y máximos (min-max bounding box) en un espacio 3D,
// requiriendo que el operador gire el chasis del robot en todos los ejes posibles durante 60 segundos
// para mapear completamente el vector magnético local.
void ICM_CalibrarMagnetometro(I2C_HandleTypeDef *hi2c) {
    ICM_Data_t imu_data;
    float mag_min[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
    float mag_max[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

    printf("\r\n=== INICIANDO CALIBRACION DEL MAGNETOMETRO ===\r\n");
    printf("TIENES 60 SEGUNDOS. GIRA EL ROBOT EN TODAS LAS DIRECCIONES (COMO UN 8)\r\n");
    vTaskDelay(pdMS_TO_TICKS(3000)); // Espera preparatoria de 3 segundos

    uint32_t start_time = HAL_GetTick();
    uint32_t last_print = 0;

    // Bucle intensivo de 60 segundos bloqueando el RTOS
    while ((HAL_GetTick() - start_time) < 60000) {

        // Solo actualizamos la malla si la lectura I2C es exitosa
        if (ICM_ReadMag(hi2c, &imu_data) == 0) {

            // Extensión de la caja límite (Bounding Box) en X
            if (imu_data.mx < mag_min[0]) mag_min[0] = imu_data.mx;
            if (imu_data.mx > mag_max[0]) mag_max[0] = imu_data.mx;
            // Y
            if (imu_data.my < mag_min[1]) mag_min[1] = imu_data.my;
            if (imu_data.my > mag_max[1]) mag_max[1] = imu_data.my;
            // Z
            if (imu_data.mz < mag_min[2]) mag_min[2] = imu_data.mz;
            if (imu_data.mz > mag_max[2]) mag_max[2] = imu_data.mz;
        }

        // Impresión serial periódica (cada 500 ms) para retroalimentación del operador
        if ((HAL_GetTick() - last_print) > 500) {
            float bias[3], scale[3], delta[3];
            float avg_delta = 0.0f;

            for (int i = 0; i < 3; i++) {
                // El Bias (Hard Iron) es el punto central algebraico de la envolvente
                bias[i] = (mag_max[i] + mag_min[i]) / 2.0f;
                // El Delta es el radio de la elipse sobre cada eje
                delta[i] = (mag_max[i] - mag_min[i]) / 2.0f;
                avg_delta += delta[i];
            }
            avg_delta /= 3.0f; // Radio de la esfera perfecta ideal

            for (int i = 0; i < 3; i++) {
                // Factor de escala (Soft Iron) para forzar que los ejes deformados regresen a una esfera perfecta
                scale[i] = (delta[i] != 0.0f) ? (avg_delta / delta[i]) : 1.0f;
            }

            uint32_t tiempo_restante = (60000 - (HAL_GetTick() - start_time)) / 1000;
            printf("\n--- CALIBRANDO (%lu seg restantes) ---\r\n", tiempo_restante);
            printf("Hard Iron Bias [X, Y, Z]: %.2f, %.2f, %.2f\r\n", bias[0], bias[1], bias[2]);
            printf("Soft Iron Scale [X, Y, Z]: %.4f, %.4f, %.4f\r\n", scale[0], scale[1], scale[2]);

            last_print = HAL_GetTick();
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Muestreo a ~100Hz para capturar suficientes vectores sin saturar el bus
    }
    printf("\r\n✅ CALIBRACION FINALIZADA. COPIA ESTOS VALORES AL CODIGO.\r\n");
}


// Lectura del vector magnético con protección de estado (Máquina de estados interna del AK09916)
uint8_t ICM_ReadMag(I2C_HandleTypeDef *hi2c, ICM_Data_t *data) {
    uint8_t st1;

    // 1. Leer Status 1 (DRDY)
    // El bit 0 indica si el hardware finalizó la conversión analógico-digital magnética.
    if(HAL_I2C_Mem_Read(hi2c, AK09916_MAG_ADDR, MAG_ST1, 1, &st1, 1, 10) != HAL_OK) return 1;

    if (!(st1 & 0x01)) return 2; // Dato en procesamiento (Early Poll, no es un error de bus)

    // 2. Extracción de Datos Crudos
    // OBLIGATORIO por hardware: Leer desde HXL (0x11) secuencialmente hasta ST2 (0x18)
    // Si no se lee ST2, el magnetómetro retiene los datos viejos en el latch y deja de actualizar.
    uint8_t raw_mag[8];
    if(HAL_I2C_Mem_Read(hi2c, AK09916_MAG_ADDR, MAG_HXL, 1, raw_mag, 8, 10) != HAL_OK) return 1;

    // 3. Verificación de Integridad de Campo en ST2
    // Leemos el índice 7 del buffer donde reside ST2 (Validación de cierre de lectura)
    uint8_t st2 = raw_mag[7];
    if (st2 & 0x08) return 3; // Bit de Saturación. El sensor detectó un imán fuerte y desbordó su ADC.

    // 4. Reconstrucción de palabra
    // A diferencia del acelerómetro, el dado del AK09916 procesa en formato Little Endian (El LSB llega primero)
    int16_t mx_raw = (int16_t)((raw_mag[1] << 8) | raw_mag[0]);
    int16_t my_raw = (int16_t)((raw_mag[3] << 8) | raw_mag[2]);
    int16_t mz_raw = (int16_t)((raw_mag[5] << 8) | raw_mag[4]);

    data->mx = mx_raw * MAG_SCALE;
    data->my = my_raw * MAG_SCALE;
    data->mz = mz_raw * MAG_SCALE;

    return 0; // Transacción completa y limpia
}

// Inicialización del Filtro Pasa Bajas Digital (DLPF - Digital Low Pass Filter)
// Mecanismo de hardware inyectado directamente en el silicio para atenuar las vibraciones mecánicas
// originadas por la fricción de los motores y el terreno, antes de que el ruido contamine la lectura en el RTOS.
bool ICM_InitGyroDLPF(I2C_HandleTypeDef *hi2c) {
    uint8_t data;

    // 1. Desplazamiento al Banco 2 (Área de configuración del giroscopio)
    ICM_SelectBank(hi2c, 2);

    // 2. Configurar el ancho de banda del filtro a 11.6 Hz (Configuración 5 desplazada 3 bits hacia la izquierda)
    // y activar el flag de encendido del DLPF (Bit 0 en ALTO).
    // Esto rechaza eficazmente frecuencias espurias por encima de la dinámica natural de giro del chasis.
    data = (5 << 3) | 0x01;

    if(HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_GYRO_CONFIG_1, 1, &data, 1, 100) != HAL_OK) {
        ICM_SelectBank(hi2c, 0); // Rollback: Forzar regreso al banco operativo por seguridad arquitectónica en caso de falla
        return false;
    }

    // 3. Regresar siempre al Banco 0 para preservar la disponibilidad de lectura cíclica en las tareas
    ICM_SelectBank(hi2c, 0);
    return true;
}


// Rutina Geométrica Aislada: Compass Electrónico 3D (Tilt-Compensated Heading)
// Extrae un Acimut magnético resistente a los baches o inclinaciones del chasis en terrenos irregulares.
float ICM_GetHeading_TiltCompensated(ICM_Data_t *imu_data) {

    // 1. Aplicar los coeficientes de corrección vectorial de fábrica al entorno de montaje
    float mx_cal = (imu_data->mx - mag_hard_iron_bias[0]) * mag_soft_iron_scale[0];
    float my_cal = (imu_data->my - mag_hard_iron_bias[1]) * mag_soft_iron_scale[1];
    float mz_cal = (imu_data->mz - mag_hard_iron_bias[2]) * mag_soft_iron_scale[2];

    // --- CORRECCIÓN CRÍTICA DE EJES (Mapeo Tridimensional) ---
    // El dado de silicio del magnetómetro (AK09916) está físicamente rotado 90 grados respecto
    // al chip principal del Acelerómetro/Giroscopio dentro del encapsulado negro.
    // Aplicamos una matriz de rotación por software para unificar ambos marcos de referencia (Y=Frente, X=Der, Z=Arriba).
    float mx_align = my_cal;   // El eje Y magnético interno es el X físico del robot
    float my_align = mx_cal;   // El eje X magnético interno es el Y físico del robot
    float mz_align = -mz_cal;  // El eje Z está invertido estructuralmente

    // 2. Extraer y normalizar el vector de gravedad de la Tierra.
    // Esto se utiliza como nuestra referencia plomada vertical inercial absoluta.
    float acc_norm = sqrtf(imu_data->ax * imu_data->ax +
                           imu_data->ay * imu_data->ay +
                           imu_data->az * imu_data->az);

    float ax_n = 0.0f, ay_n = 0.0f, az_n = 1.0f; // Perfil inercial por defecto (Robot perfectamente plano)
    if (acc_norm > 0.001f) { // Prevención de división por cero (Singularidad matemática) en caída libre
        ax_n = imu_data->ax / acc_norm;
        ay_n = imu_data->ay / acc_norm;
        az_n = imu_data->az / acc_norm;
    }

    // 3. Cinemática inversa de Euler: Calcular Pitch (Cabeceo) y Roll (Alabeo).
    // atan2f aísla los polos y previene cuadrantes ambiguos, manteniendo solidez frente a ruidos NaN.
    float pitch = atan2f(-ay_n, sqrtf(ax_n * ax_n + az_n * az_n));
    float roll  = atan2f(ax_n, az_n);

    // 4. Transformación al plano horizontal (Electronic Gimbaling / Tilt Compensation)
    // Proyecta el vector magnético inclinado de vuelta a un disco plano horizontal perfecto 2D
    // cancelando la deformación inducida por la gravedad (ángulos Pitch/Roll).
    float MXh = mx_align * cosf(roll) + mz_align * sinf(roll);
    float MYh = mx_align * sinf(roll) * sinf(pitch) +
                my_align * cosf(pitch) -
                mz_align * sinf(pitch) * cosf(roll);

    // 5. Cálculo del Rumbo Final (Norte geodésico referencial = 0°)
    // Al intercambiar las coordenadas MYh y MXh en atan2f, ejecutamos una reflexión espacial que:
    // 1. Invierte matemáticamente el sentido trigonométrico (Anti-horario a Horario para comportarse como brújula real).
    // 2. Desplaza el punto cero intrínsecamente, ahorrando la necesidad de restar 90 grados por hardware.
    float heading = atan2f(MYh, MXh) * (180.0f / M_PI); // Conversión a grados escalares absolutos

    // heading -= 90.0f; (Comentado: Compensación embebida y solucionada elegantemente en la línea anterior)

    // Ajuste de anillo circular para mantener la coherencia geométrica estricta de 0-359.9°
    if (heading < 0.0f) {
        heading += 360.0f;
    }

    return heading; // Salida saneada y estabilizada lista para el filtro complementario y control PD
}
