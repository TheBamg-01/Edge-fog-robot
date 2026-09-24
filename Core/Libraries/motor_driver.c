/*
 * motor_driver.c
 *
 * Created on: Nov 1, 2025
 * Author: HP
 *
 * Implementación del controlador de potencia.
 * Incorpora un algoritmo de "Ramping" (Control de Aceleración/Desaceleración)
 * que previene picos de corriente inductiva inversos (Back-EMF) que podrían
 * reiniciar el microcontrolador (Brownout) o destrozar los engranajes de los motorreductores
 * durante cambios bruscos de sentido dictados por el controlador PD de navegación o joystick.
 */
#include "motor_driver.h"
#include <stdlib.h> // Necesario para la función abs() en el cálculo de ciclo de trabajo

// ── CONFIGURACIÓN DE INERCIA (RAMPING) ──────────────────────────────────
// Cuánto cambia porcentualmente la velocidad por cada ciclo de ejecución del RTOS.
// Un RAMP_STEP de 20 significa que al robot le tomará 5 ciclos (ej. 100ms si el tick es de 20ms)
// pasar de 0 a 100% de potencia, creando una curva de aceleración trapezoidal estable.
#define RAMP_STEP 20

// Estructura interna de estado de la planta física.
// Mantiene en memoria RAM la diferencia entre la velocidad que exige el software (target)
// y la velocidad física real que actualmente llevan las ruedas (current).
typedef struct {
    int16_t current_speed;
    int16_t target_speed;
} MotorState_t;

// Instancias aisladas para la tracción diferencial
static MotorState_t motor_left = {0, 0};
static MotorState_t motor_right = {0, 0};

// Función privada (Helper de bajo nivel) para excitar los pines del hardware
static void Motor_Write_Hardware(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef* IN_port_A, uint16_t IN_pin_A, GPIO_TypeDef* IN_port_B, uint16_t IN_pin_B, int16_t speed)
{
    // Lógica de estado para el circuito del Puente H (L298N/TB6612FNG)
    if (speed > 0) { // Vector Positivo -> Giro de Avance
        HAL_GPIO_WritePin(IN_port_A, IN_pin_A, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN_port_B, IN_pin_B, GPIO_PIN_RESET);
    } else if (speed < 0) { // Vector Negativo -> Inversión de polaridad (Reversa)
        HAL_GPIO_WritePin(IN_port_A, IN_pin_A, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN_port_B, IN_pin_B, GPIO_PIN_SET);
    } else {
        // Vector Nulo (speed = 0) -> Freno Activo (Short Brake)
        // Poner ambos pines lógicos en ALTO cortocircuita las bobinas del motor,
        // utilizando la propia fuerza contraelectromotriz para un frenado brusco
        // y manteniendo fricción estática para que el robot no ruede en pendientes.
    	HAL_GPIO_WritePin(IN_port_A, IN_pin_A, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(IN_port_B, IN_pin_B, GPIO_PIN_SET);
    }

    // Mapeo lineal de resolución (Escalamiento)
    // Transforma el porcentaje algorítmico (0-100) al ciclo de trabajo de los registros del Timer (0-999)
    uint32_t duty_cycle = abs(speed) * 10;

    // Clamp de seguridad estructural del hardware para no desbordar el registro ARR del PWM
    if (duty_cycle > 999) duty_cycle = 999;

    // Inyección de la señal cuadrada final al canal del microcontrolador
    __HAL_TIM_SET_COMPARE(htim, channel, duty_cycle);
}

// Inicialización de la base de tiempos del hardware
void Motor_Init(TIM_HandleTypeDef *htim)
{
    // Arranca físicamente la generación de señal de los Timers
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
}

// Despachador de comandos (Interfaz entre la lógica de alto nivel y la memoria del driver)
// Actualizamos el OBJETIVO (Target) inercial y evaluamos los privilegios de emergencia.
void Motor_Process_Command(TIM_HandleTypeDef *htim, const MotorCommand_t *cmd)
{
    // Limitación (Clamp) matemática de entrada por seguridad
    // Protege contra errores de desbordamiento si el controlador PD de navegación o
    // la lógica difusa arrojan valores aberrantes fuera de las capacidades mecánicas.
    int16_t l_targ = cmd->left_speed;
    int16_t r_targ = cmd->right_speed;

    if (l_targ > 100) l_targ = 100;
    if (l_targ < -100) l_targ = -100;
    if (r_targ > 100) r_targ = 100;
    if (r_targ < -100) r_targ = -100;

    // Almacenamos el deseo de la arquitectura cognitiva
    motor_left.target_speed = l_targ;
    motor_right.target_speed = r_targ;

    // --- BYPASS DE EMERGENCIA (OVERRIDE DE SEGURIDAD) ---
    if (cmd->bypass_ramp == 1) {
        // Al anular el delta e igualar la velocidad actual con la objetivo,
        // la función iterativa Motor_Update() no tendrá distancia matemática que recorrer (Rampa anulada).
        motor_left.current_speed = motor_left.target_speed;
        motor_right.current_speed = motor_right.target_speed;

        // Escribimos de forma inmediata e incondicional al silicio para evitar latencias.
        // Vital cuando los sonares detectan colisión inminente y exigen un bloqueo de ejes en el mismo milisegundo.
        Motor_Write_Hardware(htim, TIM_CHANNEL_1, I1_GPIO_Port, I1_Pin, I2_GPIO_Port, I2_Pin, motor_left.current_speed);
        Motor_Write_Hardware(htim, TIM_CHANNEL_2, D1_GPIO_Port, D1_Pin, D2_GPIO_Port, D2_Pin, motor_right.current_speed);
    }
}

// ── MÁQUINA DE ESTADOS CINEMÁTICA ───────────────────────────────────────
// ESTA función actúa como el integrador físico del sistema y debe llamarse
// periódicamente por el RTOS (ej. cada 20ms en el lazo de la tarea MotorControlTask).
void Motor_Update(TIM_HandleTypeDef *htim)
{
    // --- Motor Izquierdo (Seguimiento de perfil de rampa) ---
    if (motor_left.current_speed < motor_left.target_speed) {
        // Aceleración gradual
        motor_left.current_speed += RAMP_STEP;
        // Evitar el sobrepaso (Overshoot) mecánico bloqueando el valor exacto del objetivo
        if (motor_left.current_speed > motor_left.target_speed) motor_left.current_speed = motor_left.target_speed;
    }
    else if (motor_left.current_speed > motor_left.target_speed) {
        // Desaceleración gradual
        motor_left.current_speed -= RAMP_STEP;
        if (motor_left.current_speed < motor_left.target_speed) motor_left.current_speed = motor_left.target_speed;
    }

    // --- Motor Derecho (Seguimiento de perfil de rampa) ---
    if (motor_right.current_speed < motor_right.target_speed) {
        motor_right.current_speed += RAMP_STEP;
        if (motor_right.current_speed > motor_right.target_speed) motor_right.current_speed = motor_right.target_speed;
    }
    else if (motor_right.current_speed > motor_right.target_speed) {
        motor_right.current_speed -= RAMP_STEP;
        if (motor_right.current_speed < motor_right.target_speed) motor_right.current_speed = motor_right.target_speed;
    }

    // --- Actualización de Registros Periféricos ---
    // Tras calcular el nuevo escalón de la rampa trapezoidal, inyectamos la orden refrescada al Puente H.
    Motor_Write_Hardware(htim, TIM_CHANNEL_1, I1_GPIO_Port, I1_Pin, I2_GPIO_Port, I2_Pin, motor_left.current_speed);
    Motor_Write_Hardware(htim, TIM_CHANNEL_2, D1_GPIO_Port, D1_Pin, D2_GPIO_Port, D2_Pin, motor_right.current_speed);
}
