#include "hc_sr04.h"
// Ya NO necesitas cmsis_os.h si solo lo usabas para osDelay aquí.
// Desacoplamos arquitectónicamente esta librería del RTOS, ya que los tiempos físicos de
// los pulsos ultrasónicos requieren precisión a nivel de ciclos de reloj, no de milisegundos.

// Punteros globales estáticos de mapeo inverso.
// Dado que la interrupción (ISR) del HAL de STM32 es genérica por timer, estos punteros
// enrutan el evento de hardware (disparo de un canal) hacia su instancia de estructura correspondiente.
static HCSR04_Sensor_t* sensor_ch1 = NULL;
static HCSR04_Sensor_t* sensor_ch2 = NULL;
static HCSR04_Sensor_t* sensor_ch4 = NULL;

// ─── Delay preciso en microsegundos usando DWT ───────────────────────────────
// El despachador de FreeRTOS (osDelay) opera con una resolución típica de 1 ms.
// Para generar el pulso obligatorio de 10 µs del HC-SR04 sin ceder el procesador, explotamos
// el registro DWT (Data Watchpoint and Trace) interno del núcleo ARM Cortex-M, contando
// los ciclos de reloj del sistema para un bloqueo temporal ultra-preciso.
void HCSR04_Delay_us(uint32_t us)
{
    // Habilitar el DWT counter (solo necesario una vez, pero es idempotente)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t start = DWT->CYCCNT;
    // cycles_per_us = SystemCoreClock / 1,000,000
    // Para STM32F4 a 168MHz -> 168 ciclos por microsegundo
    uint32_t wait  = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < wait); // Polling bloqueante de alta velocidad
}

// ─── Init ────────────────────────────────────────────────────────────────────
void HCSR04_Init_Sensor(HCSR04_Sensor_t* sensor, GPIO_TypeDef* port,
                        uint16_t pin, TIM_HandleTypeDef* tim, uint32_t channel)
{
    // Poblado estructural del objeto
    sensor->TRIG_Port    = port;
    sensor->TRIG_Pin     = pin;
    sensor->TIM_Handle   = tim;
    sensor->TIM_Channel  = channel;
    sensor->distance_cm  = 0;
    sensor->flag_captured = 0;

    // Aterrizamos el pin de disparo y armamos el módulo del timer para que comience
    // a escuchar interrupciones (Input Capture) de manera continua en background
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    HAL_TIM_IC_Start_IT(tim, channel);

    // Enlace del mapeo inverso para la ISR del TIM1
    if (tim->Instance == TIM1) {
        if (channel == TIM_CHANNEL_1) sensor_ch1 = sensor;
        if (channel == TIM_CHANNEL_2) sensor_ch2 = sensor;
        if (channel == TIM_CHANNEL_4) sensor_ch4 = sensor;
    }
}

// ─── Trigger ─────────────────────────────────────────────────────────────────
// Rutina de excitación acústica
void HCSR04_Trigger(HCSR04_Sensor_t* sensor)
{
    // Genera la perturbación eléctrica de excitación
    HAL_GPIO_WritePin(sensor->TRIG_Port, sensor->TRIG_Pin, GPIO_PIN_SET);
    HCSR04_Delay_us(10);   // ✅ Exactamente 10µs dictados por el datasheet del transductor
    HAL_GPIO_WritePin(sensor->TRIG_Port, sensor->TRIG_Pin, GPIO_PIN_RESET);

    // Reconfigura el detector de hardware al vuelo:
    // Le indicamos al periférico que la próxima interrupción debe dispararse en el flanco
    // de SUBIDA (RISING). Esto marcará el instante exacto (t_ini) en el que la onda sale del robot.
    __HAL_TIM_SET_CAPTUREPOLARITY(sensor->TIM_Handle, sensor->TIM_Channel,
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_SetCounter(sensor->TIM_Handle, 0); // Reseteamos la base de tiempo para evitar desbordamientos prematuros
    sensor->flag_captured = 0; // Reiniciamos la máquina de estados
}

// ─── Read ────────────────────────────────────────────────────────────────────
// Getter seguro para extraer el último cálculo consolidado
uint16_t HCSR04_Read(HCSR04_Sensor_t* sensor)
{
    return sensor->distance_cm;
}

// ─── Lógica interna de medición ──────────────────────────────────────────────
// Máquina de estados ejecutada asíncronamente dentro de la Interrupción (ISR) por hardware.
// Al ejecutarse a nivel de silicio (fuera del RTOS), garantizamos que la latencia de conmutación
// de tareas no distorsione el cálculo temporal del viaje del sonido.
static void Process_Capture(HCSR04_Sensor_t* sensor)
{
    if (sensor == NULL) return; // Cláusula de seguridad pasiva por punteros vacíos

    if (sensor->flag_captured == 0)  // FASE 1: Flanco de SUBIDA detectado (El pulso sónico inicia su viaje en el aire)
    {
        // Guardamos la marca de tiempo inicial congelada por el hardware
        sensor->t_ini = HAL_TIM_ReadCapturedValue(sensor->TIM_Handle,
                                                   sensor->TIM_Channel);
        sensor->flag_captured = 1;

        // Conmutación on-the-fly: Le pedimos al periférico que ahora vigile el flanco de BAJADA (FALLING)
        // para atrapar el momento exacto en el que el eco rebota y regresa al micrófono.
        __HAL_TIM_SET_CAPTUREPOLARITY(sensor->TIM_Handle, sensor->TIM_Channel,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else if (sensor->flag_captured == 1)  // FASE 2: Flanco de BAJADA detectado (El eco rebotó y regresó)
    {
        sensor->t_end = HAL_TIM_ReadCapturedValue(sensor->TIM_Handle,
                                                   sensor->TIM_Channel);
        uint32_t time_diff;

        // Manejo matemático del desbordamiento (Overflow):
        // Si el contador del timer llegó a su límite (0xFFFF) mientras el sonido volaba
        // y volvió a empezar desde cero, calculamos la diferencia geométrica correcta.
        if (sensor->t_end > sensor->t_ini)
            time_diff = sensor->t_end - sensor->t_ini;
        else
            time_diff = (0xFFFF - sensor->t_ini) + sensor->t_end;

        // Ecuación cinemática de la distancia: Distancia = Velocidad * Tiempo
        // La velocidad del sonido en condiciones ideales es ~340 m/s o 0.034 cm/µs.
        // Se divide entre 2.0f porque el tiempo 'time_diff' abarca el viaje completo (ida y vuelta del sonido).
        sensor->distance_cm = (uint16_t)((time_diff * 0.034f) / 2.0f);

        sensor->flag_captured = 0; // Cerramos el ciclo de medición
        // Restauramos la polaridad por defecto para estar listos ante el próximo trigger
        __HAL_TIM_SET_CAPTUREPOLARITY(sensor->TIM_Handle, sensor->TIM_Channel,
                                      TIM_INPUTCHANNELPOLARITY_RISING);
    }
}

// ─── Callback global del Timer ───────────────────────────────────────────────
// Sobrescritura (override) de la rutina débil del HAL (Hardware Abstraction Layer).
// Atrapa el disparo asíncrono y enruta la señal hacia el objeto correspondiente
// sin necesidad de sondear repetitivamente (zero-polling) el estado de los pines.
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        if      (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) Process_Capture(sensor_ch1);
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) Process_Capture(sensor_ch2);
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) Process_Capture(sensor_ch4); // corregido
    }
}
