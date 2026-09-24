/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h> //Para convertir cadenas a números utillizando atof(), atoi() y strol, además de uitlizar la función abs() para obtener valor absoluto.
#include <stdio.h> //Para usar printf() para el puerto serial de depuración con la función _write redirecionada al UART2, además de formatear Payloads usando snprintf() y extracción de datos usando sscanf()
#include <math.h> //Para realizar los cálculos de navegación
#include <string.h>  //Para identificar comandos con strncmp() y strcmp() y para tokenización con strok() y strsep() además de strncpy() y memset() para seguridad de memoria
#include "uartRingBuffer.h" //Librería para implementar búferes circulares en memoria para el envío y recepción de bytes a traves de los puertos seriales UART.
#include "gps.h" //Librería  que procesa, limpia y defodifica las tramas de texto NMEA recibideas desde el receptor GPS.
#include "ICM20948.h" //Librería que provee la interfaz I2C para controlar el chip IMU ICM20948 de 9 fgrados de libertad, el cual aloja un acelerómetro, un giroscopio y un magnetómetro (AK09916)
#include "hc_sr04.h" //Librería que maneja la inicialización, disparo y lectura de múltiples sensores de distyqancia ultrasónicos HC-SR04 de manera eficiente
#include "rylr998.h" //Librería para interfaz de configuración y en lace de radio de largo alcance para la topología de red utilizando el módulo RYLR998
#include "motor_driver.h" //Libreria que traduce los comandos de navegación en señales eléctricas (PWM) para actuar sobre los puentes H que mueven los motores.

#include <float.h> // Para usar FLT_MAX (el valor flotante más alto posible) para la calibración del magnetómetro
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//Estructura para guardar las coordenadas geográficas individuales que conforman la ruta del robot
typedef struct {
    double lat;
    double lon;
} Waypoint_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Constantes LoRa
#define LORA_ROBOT_ADDRESS      1 //Identificador del robot dentro de la red Lora
#define LORA_BASE_ADDRESS       2 //Identificador de la base dentro de la red Lora

// Constantes de Navegación
#define RADIO_TIERRA        6371000.0f //Radio de la tierra en metros
#define TOLERANCIA_METROS   1.8f   // Radio para considerar que llegó al punto
#define MIN_SPEED           -75     // Velocidad mínima para vencer inercia
#define MAX_SPEED           85     // Velocidad crucero
#define TURN_SPEED          75     // Velocidad de giro

//Cálculos de conversión
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

//Offset magnéticos
#define MAG_OFFSET_X  0.0f
#define MAG_OFFSET_Y  0.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Definitions for sensorUpdateTas */
osThreadId_t sensorUpdateTasHandle;
const osThreadAttr_t sensorUpdateTas_attributes = {
  .name = "sensorUpdateTas",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorControlTas */
osThreadId_t motorControlTasHandle;
const osThreadAttr_t motorControlTas_attributes = {
  .name = "motorControlTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for loraCommTask */
osThreadId_t loraCommTaskHandle;
const osThreadAttr_t loraCommTask_attributes = {
  .name = "loraCommTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for i2cRecoveryTask */
osThreadId_t i2cRecoveryTaskHandle;
const osThreadAttr_t i2cRecoveryTask_attributes = {
  .name = "i2cRecoveryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for NavigationTask */
osThreadId_t NavigationTaskHandle;
const osThreadAttr_t NavigationTask_attributes = {
  .name = "NavigationTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorCommandQueue */
osMessageQueueId_t motorCommandQueueHandle;
const osMessageQueueAttr_t motorCommandQueue_attributes = {
  .name = "motorCommandQueue"
};
/* Definitions for waypointQueue */
osMessageQueueId_t waypointQueueHandle;
const osMessageQueueAttr_t waypointQueue_attributes = {
  .name = "waypointQueue"
};
/* Definitions for sensorDataMutex */
osMutexId_t sensorDataMutexHandle;
const osMutexAttr_t sensorDataMutex_attributes = {
  .name = "sensorDataMutex"
};
/* Definitions for printfMutex */
osMutexId_t printfMutexHandle;
const osMutexAttr_t printfMutex_attributes = {
  .name = "printfMutex"
};
/* Definitions for i2cRecoverySemaphore */
osSemaphoreId_t i2cRecoverySemaphoreHandle;
const osSemaphoreAttr_t i2cRecoverySemaphore_attributes = {
  .name = "i2cRecoverySemaphore"
};
/* USER CODE BEGIN PV */

//Instancias de sensores y datos
GPS_Data_t          my_gps_data; //Almacena la última información válida procesada desde el GPS
ICM_Data_t          my_imu; //Guarda las lecturas crudas y procesadas del IMU
RYLR998_t           lora_module; //Contiene el estado y las referencias del puerto serial necesarios para controlar el transceptor de radio RYLR998
UART_RingBuffer_t   gps_buffer_manager;  // Manejador del buffer del GPS
UART_RingBuffer_t   lora_buffer_manager; // Manejador del buffer del LoRa

// Variables de estado
volatile float      MAGNETIC_DECLINATION = 0.0f;//Diferencia angular (en grados) entre el norte magnético (magnetómero) y el norte geográfico (verdadero), en Ciudad Obregón aproximadamente es de 8.28f;
float               global_robot_heading = 0.0f; //Variable que guarda el heading del robot

// Filtro GPS
volatile uint8_t gps_filter = 0; //Selector de GPS predeterminado

// Variables globales para los 3 sonares
uint16_t global_dist_c = 0; //Sonar central
uint16_t global_dist_l = 0; //Sonar izquierdo
uint16_t global_dist_r = 0; //Sonar derecho

// Bandera de estado crítica del puerto I2C
volatile uint8_t is_recovering_i2c = 0; // Esta bandera indica si el IMC falló en conectarse al puerto I2C, es más común de lo que se esperaría al probar modelos de sensores de medición inercial

// Instancias de los sensores ultrasónicos
//Aquí creamos tres objetos (estructuras) independientes para gestionar cada uno de los sensores ultrasónicos, teniendo cada instancia su propia configuración de hardware (puerto y pin de disparo, canal temporizador para el eco) y la medición de distancia en cm
HCSR04_Sensor_t sonar_center; //Sonar centro
HCSR04_Sensor_t sonar_left; //Sonar izquierdo
HCSR04_Sensor_t sonar_right; //Sonar derecho

//Variables de navegación
volatile uint8_t autonomous_mode_active = 0; // Bandera para activar/desactivar el algoritmo de seguimiento de rutas (valor 1) o si debe quedarse en modo manual/espera (0)
volatile uint8_t evasion_enabled = 0;  //Bandera para activar (valor 1) o desactivar (valor 0) la lógica difusa de evasión de obstáculos, Desactivada por defecto al encender
volatile uint8_t global_is_evading = 0; //Bandera que comunica a las tareas de navegación y evasión, cuando se detecta un obstáculo y el robot comienza a evadir, le avisa a la tarea de navegación que ceda el control temporalmente y no inntente corregir el rumbo del robot durante la evasión
int waypoints_reached = 0; //Contador de puntos alcanzados durante la navegación

//Variables corrección de error XTE
//Almacenan las coordenadas exactas del punto de partida o del último waypoint visitado por el robot
volatile double prev_wp_lat = 0.0;
volatile double prev_wp_lon = 0.0;
volatile uint8_t prev_wp_valid = 0; //Para asegurar que el robot no intente calcular un punto de partida no válido

// Parámetros de navegación ajustables en tiempo real (vía LoRa)
volatile float NAV_KP     = 1.8f; //Ganancia Propocional del controlador PD de navegación
volatile float NAV_KD     = 0.04f; //Ganancia derivativa del controladro PD de navegación
volatile float NAV_KP_XTE = 0.65f; //Determina cuántos grados de corrección de rumbo se deben aplicar por cada metro que el robot se haya desviado de su línea recta ideal
volatile float XTE_MAX_DEG = 25.0f; //Es un límite de seguridad (clamp) estructural fijado en 25 grados que evita que el robot gire de forma excesiva o caótica si se desvía bruscamente de la ruta


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
void StartSensorUpdateTask(void *argument);
void StartMotorControlTask(void *argument);
void StartLoraCommTask(void *argument);
void StartI2cRecoveryTask(void *argument);
void StartNavigationTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Función de redireccionamiento de bajo nivel (override) de la biblioteca estándar de C
//Le enseña a la función estándar printf() cómo enviar los caracteres físicos a través del hardware del microcontrolador (En este caso a través del puero serial huart2)
int _write(int file, char *ptr, int len) {
    // Verificamos que el mutex exista antes de usarlo (Protección de arranque)
    if (printfMutexHandle != NULL) {
        if (osMutexAcquire(printfMutexHandle, 1000) == osOK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
            osMutexRelease(printfMutexHandle);
        }
    } else {
        // Fallback si el RTOS no ha arrancado aun
        HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
    }
    return len;
}


//Filtro antiruido
//Calcula la mediana estadística ordenando tres lecturas de coordenadas
// Mediana de 3 doubles(ordenamiento por comparación directa)
static double GPS_Median3(double a, double b, double c) {
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

//Función de navegación esférica
//Calcula la distancia ortodrómica real en metros entre dos puntos sobre la superficie de una esfera (La Tierra)
//Utiliza la fórmula del semiverseno (Haversine)
double Nav_GetDistance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return (double)RADIO_TIERRA * c;
}

//Función de orientación geográfica
//Calcula el acimut directo (Forward Azimuth), es decir el ángulo de la brúfula al que debe apuntar el robot desde su posición actual para mirar directamente hacia la coordenada objetivo
double Nav_GetBearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    double y = sin(dLon) * cos(lat2 * DEG_TO_RAD);
    double x = cos(lat1 * DEG_TO_RAD) * sin(lat2 * DEG_TO_RAD) -
               sin(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * cos(dLon);
    double brng = atan2(y, x) * RAD_TO_DEG;
    if (brng < 0) brng += 360.0;
    return brng;
}

// Función de control de desviación
// Calcula el Cross-Track Error (XTE) (Error de seguimiento de ruta) en metros.
// Compara el rumbo que el robot debería llevar (la línea entre el origen y el destino P12), contra el rumbo real entre el origen y la posiciónn actual del robot (P13). El signo del resultado dictamina hacia qué lado se desvió
// Positivo = robot a la derecha de la línea ideal, Negativo = a la izquierda.
double Nav_GetCrossTrackError(double lat_prev, double lon_prev,
                               double lat_curr, double lon_curr,
                               double lat_tgt,  double lon_tgt) {
    double d13 = Nav_GetDistance(lat_prev, lon_prev, lat_curr, lon_curr);
    if (d13 < 0.001) return 0.0; // Sin movimiento suficiente

    double brng13 = Nav_GetBearing(lat_prev, lon_prev, lat_curr, lon_curr) * DEG_TO_RAD;
    double brng12  = Nav_GetBearing(lat_prev, lon_prev, lat_tgt,  lon_tgt)  * DEG_TO_RAD;

    // XTE = asin(sin(d13/R) * sin(brng13 - brng12)) * R
    double xte = asin(sin(d13 / RADIO_TIERRA) * sin(brng13 - brng12)) * RADIO_TIERRA;
    return xte; // en metros
}


//FUNCIONES LÓGICA DIFUSA

// Función de pertenencia para el conjunto difuso "CERCA"
// Retorna un valor entre 0.0 (Lejos) y 1.0 (Muy Cerca)
static float Fuzzy_Grado(uint16_t dist_cm, float min, float max) {
    const float min_dist = min; // 100% Cerca (Grado = 1.0)
    const float max_dist = max; // 0% Cerca (Grado = 0.0)

    if (dist_cm <= min_dist) return 1.0f;
    if (dist_cm >= max_dist) return 0.0f;

    // Rampa lineal descendente
    return (max_dist - (float)dist_cm) / (max_dist - min_dist);
}


// Operador AND difuso (Mínimo)
static float Fuzzy_AND(float a, float b) {
    return (a < b) ? a : b;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of sensorDataMutex */
  sensorDataMutexHandle = osMutexNew(&sensorDataMutex_attributes);

  /* creation of printfMutex */
  printfMutexHandle = osMutexNew(&printfMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of i2cRecoverySemaphore */
  i2cRecoverySemaphoreHandle = osSemaphoreNew(1, 1, &i2cRecoverySemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of motorCommandQueue */
  motorCommandQueueHandle = osMessageQueueNew (16, sizeof(MotorCommand_t), &motorCommandQueue_attributes);

  /* creation of waypointQueue */
  waypointQueueHandle = osMessageQueueNew (35, sizeof(Waypoint_t), &waypointQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of sensorUpdateTas */
  sensorUpdateTasHandle = osThreadNew(StartSensorUpdateTask, NULL, &sensorUpdateTas_attributes);

  /* creation of motorControlTas */
  motorControlTasHandle = osThreadNew(StartMotorControlTask, NULL, &motorControlTas_attributes);

  /* creation of loraCommTask */
  loraCommTaskHandle = osThreadNew(StartLoraCommTask, NULL, &loraCommTask_attributes);

  /* creation of i2cRecoveryTask */
  i2cRecoveryTaskHandle = osThreadNew(StartI2cRecoveryTask, NULL, &i2cRecoveryTask_attributes);

  /* creation of NavigationTask */
  NavigationTaskHandle = osThreadNew(StartNavigationTask, NULL, &NavigationTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, buzzer_Pin|trigD_Pin|LedD_Pin|LedI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, trigC_Pin|trigI_Pin|I1_Pin|I2_Pin
                          |D1_Pin|D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : buzzer_Pin trigD_Pin LedD_Pin LedI_Pin */
  GPIO_InitStruct.Pin = buzzer_Pin|trigD_Pin|LedD_Pin|LedI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : trigC_Pin trigI_Pin I1_Pin I2_Pin
                           D1_Pin D2_Pin */
  GPIO_InitStruct.Pin = trigC_Pin|trigI_Pin|I1_Pin|I2_Pin
                          |D1_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensorUpdateTask */
/**
  * @brief  Function implementing the sensorUpdateTas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSensorUpdateTask */
/*
 *  TAREA DE SENSORES
 *  Esta tarea es el hilo de "percepción" prinsipal del sistema
 *  actuando como la principal fuente de recolección de datos
 *  en el nodo edge de la arquitect5ua distribuida, esta tarea
 *  adquiere, filtra y empaqueta de forma segura la información
 *  física del entorno antes de que otras tareas la utilicen
 */
void StartSensorUpdateTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	/*
	 * --- PREPARACIÓN DE PERIFERICOS ---
	 */

		osDelay(100);

		printf("\r\n--- INICIO DEL SISTEMA ---\r\n"); // Debug impresión de inicio del sistema

		Ringbuf_Init(&gps_buffer_manager, &huart1);
		HCSR04_Init_Sensor(&sonar_center, trigC_GPIO_Port, trigC_Pin, &htim1, TIM_CHANNEL_1);
		HCSR04_Init_Sensor(&sonar_left,   trigI_GPIO_Port, trigI_Pin, &htim1, TIM_CHANNEL_2);
		HCSR04_Init_Sensor(&sonar_right,  trigD_GPIO_Port, trigD_Pin, &htim1, TIM_CHANNEL_4);

		// --- INICIALIZACIONES DE SENSORES

		// Inicialización IMU, verificación de que el puerto I2C no esté fallando
		is_recovering_i2c = 1;
		if (!ICM_Init(&hi2c1)) {
			printf("ERROR INICIAL: ICM20948 fallo. Activando recuperacion.\r\n");
			osSemaphoreRelease(i2cRecoverySemaphoreHandle);
		} else {
			printf("SISTEMA OK: Sensores inicializados.\r\n");
			is_recovering_i2c = 0;
		}

		ICM_InitGyroDLPF(&hi2c1); // Inicialización del giroscopia para Filtrar vibración de motores

		// ***Descomentar la siguiente línea para calibrar el magneómetor, para calibrarlo hay que tener activada un monitor serie para visualizar los valores finales de la calibración, la calibración consiste en mover el robot durante 30 segunmdos en todas direcciones realizando movimientos en forma de 8s, al final el monitor serie mostrará los valores que hay que poner en la librería del ICM para el funcionamiento óptimo del heading
		//ICM_CalibrarMagnetometro(&hi2c1);

		// ... DEFINICIÓN DE CONSTANTES Y VARIABLES

		#define FILTER_ALPHA 0.98f //  Valor de Alpha del filtro complementario
		uint32_t last_imu_tick = 0; //

		//Variables GPS
		char gps_rx_line[128];
		GPS_Data_t local_gps_data = {0};

		// Variables Filtro Mediana-3 + EMA residual del GPS
		#define GPS_MEDIAN_SIZE     3 // Indicar el tamaño de la mediana, en este caso 3 ya que es Mediana-3
		#define GPS_EMA_ALPHA       0.85f   // Suavizado fino post-mediana (más reactivo

		// Buffers circulares para la mediana
		static double lat_buf[GPS_MEDIAN_SIZE] = {0.0};
		static double lon_buf[GPS_MEDIAN_SIZE] = {0.0};
		static uint8_t  median_idx   = 0;
		static uint8_t  median_count = 0;   // Cuántas muestras válidas hay en el buffer

		// EMA residual (se aplica sobre la salida de la mediana)
		static double filtered_lat = 0.0;
		static double filtered_lon = 0.0;
		static uint8_t gps_first_fix = 1;

		// Variables Navegación y Sonares
		uint32_t last_sonar_trigger_time = 0;
		uint8_t sonar_state = 0;
		uint32_t last_alive_msg = 0;
		float local_heading = 0.0f;
		static uint8_t filter_initialized = 0;
		uint16_t dist_c = 0, dist_l = 0, dist_r = 0;

		// --- RUTINA DE CALIBRACIÓN DEL GIROSCOPIO - Esta calibración es indispensable cada vez que se enciende el sistema - Para la calibración solo hay que dejar el robot inmovil por unos segundos al iniciar el sistema
		float gz_bias = 0.0f;
		printf("Calibrando giroscopio. Manten el robot inmovil...\r\n");
		osDelay(500);

		for (int i = 0; i < 100; i++) {
			ICM_ReadAccelGyro(&hi2c1, &my_imu);
			gz_bias += my_imu.gz;
			osDelay(10);
		}
		gz_bias /= 100.0f;
		printf("Bias de Gz calculado: %.4f dps\r\n", gz_bias);

  /* Infinite loop */
  for(;;)
  {
	        // ==========================================================
	  		// 1. LEER GPS
	  	  	// Leemos las líneas entrantes almacenadas en el búfer circular para el gps
	  		// ==========================================================

	  	  	//Selector del filtro para el gps a escoger po el usuario en la central
	  	  	switch(gps_filter){

	  	  		case 0:  //Filtro de media (3 valores)
					if (Ringbuf_Read_Line(&gps_buffer_manager, gps_rx_line, sizeof(gps_rx_line))) {
							  if (gps_process_buffer(gps_rx_line, &local_gps_data)) {

								  // 1. Insertar nueva muestra en buffer circular
								  lat_buf[median_idx] = local_gps_data.latitude;
								  lon_buf[median_idx] = local_gps_data.longitude;
								  median_idx = (median_idx + 1) % GPS_MEDIAN_SIZE;
								  if (median_count < GPS_MEDIAN_SIZE) median_count++;

								  // 2. Calcular promedio con las muestras disponibles
								  double lat_sum = 0.0, lon_sum = 0.0;
								  for (uint8_t i = 0; i < median_count; i++) {
									  lat_sum += lat_buf[i];
									  lon_sum += lon_buf[i];
								  }
								  filtered_lat = lat_sum / median_count;
								  filtered_lon = lon_sum / median_count;

								  // 3. Escribir posición filtrada de vuelta
								  local_gps_data.latitude  = filtered_lat;
								  local_gps_data.longitude = filtered_lon;
							  }
						  }
					break;
	  	  		case 1: //Filtro de mediana + EMA
	  	  			if (Ringbuf_Read_Line(&gps_buffer_manager, gps_rx_line, sizeof(gps_rx_line))) {
							  if (gps_process_buffer(gps_rx_line, &local_gps_data)) {

								  if (gps_first_fix) {
									  // Primer fix: inicializar buffer y EMA con la misma lectura
									  for (uint8_t i = 0; i < GPS_MEDIAN_SIZE; i++) {
										  lat_buf[i] = local_gps_data.latitude;
										  lon_buf[i] = local_gps_data.longitude;
									  }
									  filtered_lat  = local_gps_data.latitude;
									  filtered_lon  = local_gps_data.longitude;
									  median_count  = GPS_MEDIAN_SIZE; // Buffer completo desde el inicio
									  gps_first_fix = 0;

								  } else {
									  // 1. Insertar nueva muestra en el buffer circular
									  lat_buf[median_idx] = local_gps_data.latitude;
									  lon_buf[median_idx] = local_gps_data.longitude;
									  median_idx = (median_idx + 1) % GPS_MEDIAN_SIZE;
									  if (median_count < GPS_MEDIAN_SIZE) median_count++;

									  // 2. Calcular mediana (solo cuando el buffer tiene 3 muestras completas)
									  double median_lat, median_lon;
									  if (median_count == GPS_MEDIAN_SIZE) {
										  median_lat = GPS_Median3(lat_buf[0], lat_buf[1], lat_buf[2]);
										  median_lon = GPS_Median3(lon_buf[0], lon_buf[1], lon_buf[2]);
									  } else {
										  // Buffer aún llenándose: usar la muestra directa sin mediana
										  median_lat = local_gps_data.latitude;
										  median_lon = local_gps_data.longitude;
									  }

									  // 3. EMA residual sobre la salida de la mediana
									  filtered_lat = (GPS_EMA_ALPHA * median_lat) + ((1.0 - GPS_EMA_ALPHA) * filtered_lat);
									  filtered_lon = (GPS_EMA_ALPHA * median_lon) + ((1.0 - GPS_EMA_ALPHA) * filtered_lon);
								  }

								  // 4. Escribir posición filtrada de vuelta
								  local_gps_data.latitude  = filtered_lat;
								  local_gps_data.longitude = filtered_lon;
							  }
						  }
	  	  			break;
	  	  	case 2: // Filtro de Rechazo de Atípicos (Distancia Máxima)
	  	  	    if (Ringbuf_Read_Line(&gps_buffer_manager, gps_rx_line, sizeof(gps_rx_line))) {
	  	  	        if (gps_process_buffer(gps_rx_line, &local_gps_data)) {
	  	  	            static uint8_t outlier_first_fix = 1;
	  	  	            // Distancia máxima permitida entre lecturas (ej. 2.0 metros por lectura)
	  	  	            const double MAX_DIST_JUMP = 2.0;

	  	  	            if (outlier_first_fix) {
	  	  	                filtered_lat = local_gps_data.latitude;
	  	  	                filtered_lon = local_gps_data.longitude;
	  	  	                outlier_first_fix = 0;
	  	  	            } else {
	  	  	                // Usamos tu función Nav_GetDistance ya existente
	  	  	                double dist_jump = Nav_GetDistance(filtered_lat, filtered_lon,
	  	  	                                                   local_gps_data.latitude, local_gps_data.longitude);

	  	  	                if (dist_jump <= MAX_DIST_JUMP) {
	  	  	                    // Es un punto válido, actualizamos
	  	  	                    filtered_lat = local_gps_data.latitude;
	  	  	                    filtered_lon = local_gps_data.longitude;
	  	  	                } else {
	  	  	                    // Salto anormal (ruido), ignoramos la lectura y mantenemos la anterior
	  	  	                    // Opcional: Podrías contar cuántos rechazos seguidos hay para forzar un reset
	  	  	                }
	  	  	            }
	  	  	            local_gps_data.latitude  = filtered_lat;
	  	  	            local_gps_data.longitude = filtered_lon;
	  	  	        }
	  	  	    }
	  	  	    break;

	  	  case 3: // Filtro Predictivo Alfa-Beta
	  	      if (Ringbuf_Read_Line(&gps_buffer_manager, gps_rx_line, sizeof(gps_rx_line))) {
	  	          if (gps_process_buffer(gps_rx_line, &local_gps_data)) {
	  	              static uint8_t ab_first_fix = 1;

	  	              // Constantes empíricas (0 < alpha, beta < 1)
	  	              // Alpha alto = confía más en el GPS. Beta alto = confía más en la velocidad calculada
	  	              const double ALPHA = 0.6;
	  	              const double BETA  = 0.2;
	  	              const double DT    = 0.2; // Asumiendo que el GPS actualiza a 1Hz (1 segundo)

	  	              static double v_lat = 0.0, v_lon = 0.0;

	  	              if (ab_first_fix) {
	  	                  filtered_lat = local_gps_data.latitude;
	  	                  filtered_lon = local_gps_data.longitude;
	  	                  ab_first_fix = 0;
	  	              } else {
	  	                  // 1. Predicción (Ecuación de estado)
	  	                  double pred_lat = filtered_lat + (v_lat * DT);
	  	                  double pred_lon = filtered_lon + (v_lon * DT);

	  	                  // 2. Cálculo del residual (Error)
	  	                  double res_lat = local_gps_data.latitude - pred_lat;
	  	                  double res_lon = local_gps_data.longitude - pred_lon;

	  	                  // 3. Actualización de posición
	  	                  filtered_lat = pred_lat + (ALPHA * res_lat);
	  	                  filtered_lon = pred_lon + (ALPHA * res_lon);

	  	                  // 4. Actualización de velocidad
	  	                  v_lat = v_lat + ((BETA / DT) * res_lat);
	  	                  v_lon = v_lon + ((BETA / DT) * res_lon);
	  	              }
	  	              local_gps_data.latitude  = filtered_lat;
	  	              local_gps_data.longitude = filtered_lon;
	  	          }
	  	      }
	  	      break;

	  	case 4: // Filtro DEMA (Doble EMA con menor retraso)
	  	    if (Ringbuf_Read_Line(&gps_buffer_manager, gps_rx_line, sizeof(gps_rx_line))) {
	  	        if (gps_process_buffer(gps_rx_line, &local_gps_data)) {
	  	            static uint8_t dema_first_fix = 1;
	  	            const double DEMA_ALPHA = 0.6; // Ajustar entre 0.1 y 0.9

	  	            static double ema1_lat = 0.0, ema1_lon = 0.0;
	  	            static double ema2_lat = 0.0, ema2_lon = 0.0;

	  	            if (dema_first_fix) {
	  	                ema1_lat = ema2_lat = local_gps_data.latitude;
	  	                ema1_lon = ema2_lon = local_gps_data.longitude;
	  	                dema_first_fix = 0;
	  	            } else {
	  	                // Primer EMA
	  	                ema1_lat = (DEMA_ALPHA * local_gps_data.latitude) + ((1.0 - DEMA_ALPHA) * ema1_lat);
	  	                ema1_lon = (DEMA_ALPHA * local_gps_data.longitude) + ((1.0 - DEMA_ALPHA) * ema1_lon);

	  	                // Segundo EMA (basado en el primero)
	  	                ema2_lat = (DEMA_ALPHA * ema1_lat) + ((1.0 - DEMA_ALPHA) * ema2_lat);
	  	                ema2_lon = (DEMA_ALPHA * ema1_lon) + ((1.0 - DEMA_ALPHA) * ema2_lon);

	  	                // Cálculo de DEMA
	  	                filtered_lat = (2.0 * ema1_lat) - ema2_lat;
	  	                filtered_lon = (2.0 * ema1_lon) - ema2_lon;
	  	            }
	  	            local_gps_data.latitude  = filtered_lat;
	  	            local_gps_data.longitude = filtered_lon;
	  	        }
	  	    }
	  	    break;

	  	case 5: // Sin filtro (Datos crudos / Raw)
			if (Ringbuf_Read_Line(&gps_buffer_manager, gps_rx_line, sizeof(gps_rx_line))) {
				if (gps_process_buffer(gps_rx_line, &local_gps_data)) {

					// Los valores crudos ya están almacenados en local_gps_data por la función gps_process_buffer().
					// No hacemos ninguna operación matemática sobre ellos.
					filtered_lat = local_gps_data.latitude;
					filtered_lon = local_gps_data.longitude;

				}
			}
			break;
	  	case 6: // Filtro combinado: Rechazo de outliers + Mediana-3 + EMA
	  	    if (Ringbuf_Read_Line(&gps_buffer_manager, gps_rx_line, sizeof(gps_rx_line))) {
	  	        if (gps_process_buffer(gps_rx_line, &local_gps_data)) {

	  	            static uint8_t  c6_first_fix        = 1;
	  	            static uint8_t  c6_reject_streak    = 0;     // Rechazos consecutivos
	  	            static double   c6_last_valid_lat   = 0.0;
	  	            static double   c6_last_valid_lon   = 0.0;

	  	            // Buffers circulares para la mediana (comparte tamaño con GPS_MEDIAN_SIZE)
	  	            static double   c6_lat_buf[GPS_MEDIAN_SIZE] = {0.0};
	  	            static double   c6_lon_buf[GPS_MEDIAN_SIZE] = {0.0};
	  	            static uint8_t  c6_median_idx   = 0;
	  	            static uint8_t  c6_median_count = 0;

	  	            // EMA residual post-mediana
	  	            static double   c6_ema_lat = 0.0;
	  	            static double   c6_ema_lon = 0.0;

	  	            // Umbrales
	  	            const double MAX_JUMP_M     = 2.0;   // metros — salto máximo aceptable entre muestras
	  	            const uint8_t MAX_STREAK    = 5;     // rechazos consecutivos antes de forzar aceptación
	  	            const double  C6_EMA_ALPHA  = 0.85;  // mismo alpha que filtro 1

	  	            // ── ETAPA 1: Primer fix ──────────────────────────────────────────
	  	            if (c6_first_fix) {
	  	                c6_last_valid_lat = local_gps_data.latitude;
	  	                c6_last_valid_lon = local_gps_data.longitude;

	  	                // Inicializar buffer de mediana completo con el primer valor
	  	                for (uint8_t i = 0; i < GPS_MEDIAN_SIZE; i++) {
	  	                    c6_lat_buf[i] = local_gps_data.latitude;
	  	                    c6_lon_buf[i] = local_gps_data.longitude;
	  	                }
	  	                c6_median_count = GPS_MEDIAN_SIZE;
	  	                c6_ema_lat      = local_gps_data.latitude;
	  	                c6_ema_lon      = local_gps_data.longitude;
	  	                c6_first_fix    = 0;

	  	                filtered_lat = local_gps_data.latitude;
	  	                filtered_lon = local_gps_data.longitude;
	  	                local_gps_data.latitude  = filtered_lat;
	  	                local_gps_data.longitude = filtered_lon;
	  	                break;
	  	            }

	  	            // ── ETAPA 2: Rechazo de outliers ────────────────────────────────
	  	            double dist_jump = Nav_GetDistance(c6_last_valid_lat, c6_last_valid_lon,
	  	                                               local_gps_data.latitude, local_gps_data.longitude);

	  	            uint8_t sample_accepted = 0;

	  	            if (dist_jump <= MAX_JUMP_M) {
	  	                // Salto normal: aceptar y resetear racha
	  	                sample_accepted  = 1;
	  	                c6_reject_streak = 0;
	  	            } else {
	  	                c6_reject_streak++;
	  	                if (c6_reject_streak >= MAX_STREAK) {
	  	                    // Demasiados rechazos seguidos: el robot realmente se movió
	  	                    // (o el GPS convergió a una posición nueva). Forzar aceptación
	  	                    // y resetear todo el historial para evitar lag acumulado.
	  	                    sample_accepted  = 1;
	  	                    c6_reject_streak = 0;

	  	                    // Reiniciar buffer de mediana con el nuevo valor aceptado
	  	                    for (uint8_t i = 0; i < GPS_MEDIAN_SIZE; i++) {
	  	                        c6_lat_buf[i] = local_gps_data.latitude;
	  	                        c6_lon_buf[i] = local_gps_data.longitude;
	  	                    }
	  	                    c6_median_count = GPS_MEDIAN_SIZE;
	  	                    c6_ema_lat      = local_gps_data.latitude;
	  	                    c6_ema_lon      = local_gps_data.longitude;
	  	                }
	  	                // Si no llegamos a MAX_STREAK: descartar muestra silenciosamente.
	  	                // filtered_lat/lon mantienen el último valor suavizado.
	  	            }

	  	            // ── ETAPA 3: Mediana-3 + EMA (solo con muestras aceptadas) ──────
	  	            if (sample_accepted) {
	  	                c6_last_valid_lat = local_gps_data.latitude;
	  	                c6_last_valid_lon = local_gps_data.longitude;

	  	                // Insertar en buffer circular
	  	                c6_lat_buf[c6_median_idx] = local_gps_data.latitude;
	  	                c6_lon_buf[c6_median_idx] = local_gps_data.longitude;
	  	                c6_median_idx = (c6_median_idx + 1) % GPS_MEDIAN_SIZE;
	  	                if (c6_median_count < GPS_MEDIAN_SIZE) c6_median_count++;

	  	                // Calcular mediana (solo cuando el buffer está completo)
	  	                double median_lat, median_lon;
	  	                if (c6_median_count == GPS_MEDIAN_SIZE) {
	  	                    median_lat = GPS_Median3(c6_lat_buf[0], c6_lat_buf[1], c6_lat_buf[2]);
	  	                    median_lon = GPS_Median3(c6_lon_buf[0], c6_lon_buf[1], c6_lon_buf[2]);
	  	                } else {
	  	                    // Buffer aún llenándose: usar muestra directa
	  	                    median_lat = local_gps_data.latitude;
	  	                    median_lon = local_gps_data.longitude;
	  	                }

	  	                // EMA residual sobre la salida de la mediana
	  	                c6_ema_lat = (C6_EMA_ALPHA * median_lat) + ((1.0 - C6_EMA_ALPHA) * c6_ema_lat);
	  	                c6_ema_lon = (C6_EMA_ALPHA * median_lon) + ((1.0 - C6_EMA_ALPHA) * c6_ema_lon);

	  	                filtered_lat = c6_ema_lat;
	  	                filtered_lon = c6_ema_lon;
	  	            }

	  	            local_gps_data.latitude  = filtered_lat;
	  	            local_gps_data.longitude = filtered_lon;
	  	        }
	  	    }
	  	    break;


	  	  	}


	  		// ==========================================================
	  		// 2. LEER SONARES (Cada 40ms)
	  		// ==========================================================

	  	  	//Ejecutamos una lectura secuencial de los sonares cada 40 ms
	  	  	//Utilizamos una máquina de estados para disparar secuencialmente solo un sensor a la vez, esto para prevenir que el eco de rebote de del sensor izquierdo sea leído erróneamente por el sensor derecho y viceversa
	  		if (HAL_GetTick() - last_sonar_trigger_time > 40) {
	  			last_sonar_trigger_time = HAL_GetTick();
	  			switch(sonar_state) {
	  				case 0:
	  					dist_r = HCSR04_Read(&sonar_right);
	  					HCSR04_Trigger(&sonar_center);
	  					sonar_state = 1;
	  					break;
	  				case 1:
	  					dist_c = HCSR04_Read(&sonar_center);
	  					HCSR04_Trigger(&sonar_left);
	  					sonar_state = 2;
	  					break;
	  				case 2:
	  					dist_l = HCSR04_Read(&sonar_left);
	  					HCSR04_Trigger(&sonar_right);
	  					sonar_state = 0;
	  					break;

	  			}
	  		}

	  		// ==========================================================
	  		// 3. LEER IMU Y FILTRAR (Depende del estado del I2C)
	  		// ==========================================================

	  		// Realizamos la lectura y fusión se sensores IMU
	  		// Se cuenta con tolerancia a fallos
	  		// Si detecta que bus I2C colapsó (modo 2), congela la actualización de datos para no colgar el sistema operativo y lanza alertas, permitiendo que el robot siga operativo únicamente en el gps y los sonraes
	  		if (is_recovering_i2c == 2) {
	  			// MODO FALLO CRÍTICO
	  			if (HAL_GetTick() - last_alive_msg > 2000) {
	  				last_alive_msg = HAL_GetTick();
	  				printf("ALERTA: Robot operando SIN IMU. LoRa/GPS/Sonares OK.\r\n");
	  			}
	  		}
	  		else if (is_recovering_i2c == 1) {
	  			// MODO RECUPERACIÓN (Pausado esperando al semáforo)
	  			filter_initialized = 0;
	  		}
	  		else {
	  			// MODO NORMAL (IMU OK)

	  			// Calculamos el tiempo real transcurrido entre ciclos (dt_real) para realizar una integración precisa de la velocidad angular del giroscopio en grados
	  			uint32_t now_tick = HAL_GetTick();
				float dt_real = (float)(now_tick - last_imu_tick) / 1000.0f;
				// Guard: si el valor es absurdo (primer ciclo, o colgón largo), usar 10ms como dt
				if (last_imu_tick == 0 || dt_real <= 0.0f || dt_real > 0.1f) {
					dt_real = 0.01f;
				}
				last_imu_tick = now_tick;

	  			// Obtenemos las leacturas
	  			uint8_t accel_status = ICM_ReadAccelGyro(&hi2c1, &my_imu);
	  			uint8_t mag_status = ICM_ReadMag(&hi2c1, &my_imu);

	  			if (accel_status == 1 || mag_status == 1) {
	  				printf("ERROR: Fallo I2C...\r\n");
	  				osSemaphoreRelease(i2cRecoverySemaphoreHandle);
	  			}
	  			else {
	  				// Aplicar calibración Hard Iron
	  				// 1. Obtener el heading compensado por inclinación
	  				 float mag_heading = ICM_GetHeading_TiltCompensated(&my_imu);

	  				if (!filter_initialized) {
	  					local_heading = mag_heading;
	  					filter_initialized = 1;
	  				}
	  				else {
	  					//FILTRO COMPLEMENTARIO con variable ajustable FILTER_ALPHA, de 0.98 en este caso con excelentes resultados
	  					// A corto plazo, el filtro reacciona ágilmente a los giros evasivos del robot usando el giroscopio, pero a largo plazo, arrastra suavemente el resultado hacia el norte magnético entregado por el magnetómetro (compensado por inclinación), logrando un rumbo estabnle  y sin deriva

	  					// Corregimos Giroscopio con Bias
	  					float gz_real = -my_imu.gz - gz_bias;
	  					float gyro_pred = local_heading + (gz_real * dt_real);

	  					if (gyro_pred >= 360.0f) gyro_pred -= 360.0f;
	  					else if (gyro_pred < 0.0f) gyro_pred += 360.0f;

	  					// Aplicamos el Filtro Complementario
	  					float error = mag_heading - gyro_pred;
	  					if (error > 180.0f) error -= 360.0f;
	  					else if (error < -180.0f) error += 360.0f;

	  					local_heading = gyro_pred + (error * (1.0f - FILTER_ALPHA));

	  					if (local_heading >= 360.0f) local_heading -= 360.0f;
	  					else if (local_heading < 0.0f) local_heading += 360.0f;
	  				}
	  			}
	  		}

	  		// ==========================================================
	  		// 4. SINCRONIZACIÓN GLOBAL MUTEX
	  		// ==========================================================

	  		// Aplicamos protección de memoria compartida adquieriendo el candado del sistema operativo sensorDataMutexHandle. Esta es la barrera de seguridad de FreeRTOS
	  		// Copiamos de golpe todas las variables locales filtradas (gps, sonar, IMU) a las estructuras de variables globales del sistema. Esto garantiza que si la tarea de control de motores intenta leer la disntacia frontal por ejemplo justo en este milisegundo, nunca obtenga un byte a medio escribir (condición de carrera)
	  		// Al finalizar suelta el mutex y ejecuta el osDelay(10), lo cual ponea a la tarea a dormir durante 10 milisegundos, devolviendo el control del procesador al despachador de FreeRTOS para que atienda al resto del sistema.
	  		if (osMutexAcquire(sensorDataMutexHandle, 10) == osOK) {
	  			my_gps_data = local_gps_data;
	  			global_dist_c = dist_c;
	  			global_dist_l = dist_l;
	  			global_dist_r = dist_r;

	  			// Si el IMU está en fallo, local_heading conserva su último valor seguro
	  			global_robot_heading = local_heading;

	  			osMutexRelease(sensorDataMutexHandle);
	  		}

	  		// (Opcional) Descomentar las siguientes dos líneas para debug de los sensoresen monitor serie
	  		//printf("Sonares L:%d C:%d R:%d \r\n", dist_l, dist_c, dist_r);
	  		//printf("Heading: %.2f | Sonares L:%d C:%d R:%d \r\n", local_heading, dist_l, dist_c, dist_r);

	  		osDelay(10); // DT = 10ms
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotorControlTask */
/**
* @brief Function implementing the motorControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorControlTask */
/*
 *  TAREA DE CONTROL DE MOTORES Y EVASIÓN
 *  Esta tarea actúa como el hilo "actuador" del sistema, recibiendo las órdenes
 *  de navegación general y traduciéndolas en movimiento físico mediante señales PWM.
 *  Además, integra la capa de seguridad reactiva de bajo nivel (lógica difusa)
 *  para la evasión de obstáculos en tiempo real, tomando el control de la tracción
 *  cuando el entorno físico representa un riesgo inminente de colisión.
 */
void StartMotorControlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorControlTask */
    // Inicialización del puente H (L298N o similar) y los canales PWM del temporizador
    Motor_Init(&htim2);

    // Instancias de comandos de motor
    // Estructuras para almacenar las velocidades objetivo (izquierda, derecha) y configuración de rampa
    MotorCommand_t normal_command  = {0, 0, 0}; // Comando proveniente de la navegación o control manual
    MotorCommand_t evasion_command = {0, 0, 1}; // Comando generado localmente por la lógica difusa (con bypass de rampa = 1 para respuesta inmediata)

    // Variables de estado local para la máquina de evasión
    uint8_t is_evading      = 0; // Bandera local que indica si el robot está ejecutando una maniobra evasiva actualmente
    //int8_t  evasion_turn_dir = 0;  // +1=derecha, -1=izquierda, 0=sin decidir

    // ── Umbrales (cm) ───────────────────────────────────────────────────────
    // Los 3 sensores apuntan al FRENTE (centro, esquina-izq, esquina-der).
    // Se necesita detectar con anticipación suficiente para girar antes de
    // chocar. Un robot a 70 cm/s necesita ~0.8 s = ~56 cm para detenerse
    // girando. Umbral amplio + respuesta agresiva desde lejos.
    #define DANGER_ZONE_C   70    // Frontal central: activar evasión
    #define DANGER_ZONE_S   60    // Frontales laterales: activar evasión
    #define SAFE_ZONE       75    // Desactivar evasión (10 cm sobre el umbral mayor)

    // Zona crítica: distancia a la que se fuerza retroceso inmediato
    #define CRITICAL_ZONE_C 25
    #define CRITICAL_ZONE_S 6
	#define DEAD_ZONE       6

    // ── Parámetros cinemáticos ───────────────────────────────────────────────
    #define INERTIA_MIN     20    // Velocidad mínima para mantener inercia cinética
    #define EVADE_OUTER     80    // Velocidad rueda exterior en giro cerrado
    #define EVADE_INNER     10    // Velocidad rueda interior en giro cerrado (giro en sitio)
    #define BASE_FWD        75    // Velocidad base de avance durante esquive lateral
    #define REVERSE_SPEED   -40
    #define REVERSE_STEER   -40

    // Umbral de velocidad para considerar que el robot "está en movimiento".
    // Si ambas ruedas tienen velocidad objetivo menor a esto, no se activa evasión.
    #define MOVING_THRESHOLD 10

    // Temporizadores y banderas para detección de atascos (Robot acorralado)
    uint32_t evasion_start_tick = 0; // Almacena el momento exacto (en ms) en el que inició la evasión actual
    //uint8_t evasion_timed_out = 0;   // Flag: timeout ocurrió, permitir re-armado
    uint8_t is_trapped = 0; // Bandera que se activa si el robot pasa demasiado tiempo evadiendo sin encontrar salida
    #define EVASION_TIMEOUT_MS  9500  // Límite de tiempo (9.5 segundos) de evasión continua antes de declararse atrapado
	#define EVASION_REVER_MS 1500     // Duración de la maniobra de evasión en reversa
    uint8_t robot_en_movimiento = 0;  // Bandera general del estado cinemático actual
    uint8_t reverse_evation = 0;      // Bandera para indicar si el robot está ejecutando reversa por proximidad crítica
    uint32_t reverse_tick = 0;        // Marca de tiempo para controlar la duración de la maniobra de reversa

  /* Infinite loop */
  for(;;)
  {
    // 1. Recibir comando normal sin bloquear
	  uint8_t new_command_received = 0; // Bandera local para saber si llegó una nueva orden de la cola
      // Consultamos la cola de mensajes del sistema operativo (RTOS). Si hay un mensaje, se copia en normal_command
	  if (osMessageQueueGet(motorCommandQueueHandle, &normal_command, NULL, 0) == osOK) {
		  normal_command.bypass_ramp = 0; // En operación normal, respetamos la rampa de aceleración para evitar picos de corriente
		  new_command_received = 1;
	  }

	  // 2. Leer sensores de forma segura
	      uint16_t c = 999, l = 999, r = 999; // Valores por defecto (camino libre) en caso de que los sensores fallen
          // Adquirimos el candado para leer las variables globales compartidas sin que otra tarea las modifique a la mitad
	      if (osMutexAcquire(sensorDataMutexHandle, 10) == osOK) {
	          c = (global_dist_c > 0) ? global_dist_c : 999;

	          // CORRECCIÓN DE HARDWARE: Intercambiamos L y R por software
	          // para compensar el cruce físico de los sensores
	          l = (global_dist_r > 0) ? global_dist_r : 999;
	          r = (global_dist_l > 0) ? global_dist_l : 999;

	          osMutexRelease(sensorDataMutexHandle);

	          // Filtro EMA (Filtro de Media Móvil Exponencial): suaviza picos de ruido del HC-SR04
              // Evita respuestas bruscas de los motores ante lecturas atípicas momentáneas del ultrasonido
			  static float fc = 100.0f, fl = 100.0f, fr = 100.0f;
			  fc = 0.7f * fc + 0.3f * (float)c;
			  fl = 0.7f * fl + 0.3f * (float)l;
			  fr = 0.7f * fr + 0.3f * (float)r;

			  c = (uint16_t)fc;
			  l = (uint16_t)fl;
			  r = (uint16_t)fr;
	      }

    // 2.5 Determinar si el robot está en movimiento.
        // Se considera en movimiento si el comando actual tiene velocidad
        // significativa en al menos una rueda. Esto cubre:
        //   - Navegación autónoma activa (waypoints enviando comandos)
        //   - Comandos manuales F/B/L/R
        // No se activa con: CMD:STOP, fin de misión, robot recién encendido.
    // Solo actualizar robot_en_movimiento si hubo comando nuevo, si no, conservar estado
    if (new_command_received) {
        // Calculamos el valor absoluto para considerar movimiento tanto hacia adelante como en reversa
        robot_en_movimiento = (abs(normal_command.left_speed)  > MOVING_THRESHOLD ||
                               abs(normal_command.right_speed) > MOVING_THRESHOLD);

        // Rescate Manual. Si NO estamos en navegación autónoma y se manda un
		// comando de movimiento, rompemos el estado "atrapado" para que obedezca.
        // Esto permite a un operador humano destrabar al robot remotamente
		if (!autonomous_mode_active && robot_en_movimiento) {
			is_trapped = 0;
		}
    }

    // 3. MÁQUINA DE ESTADOS DE EVASIÓN
        // Condiciones para activar:
            //   a) No estamos ya evadiendo
            //   b) El usuario habilitó la evasión (evasion_enabled)
            //   c) El robot tiene un comando de movimiento activo
            //   d) Algún sensor detecta un obstáculo
    if (!is_evading && !is_trapped && evasion_enabled && robot_en_movimiento &&
                (c < DANGER_ZONE_C || l < DANGER_ZONE_S || r < DANGER_ZONE_S))
            {
                is_evading = 1; // Cambia el estado a evasión activa
                evasion_start_tick = HAL_GetTick(); // Registra el tiempo de inicio para el timeout
                HAL_GPIO_WritePin(GPIOC, buzzer_Pin, GPIO_PIN_SET); // Alerta sonora física indicando evasión
            }
            else if (is_evading) {
                if (c > SAFE_ZONE && l > SAFE_ZONE && r > SAFE_ZONE) {
                    // Camino libre antes del timeout -> Salir de evasión
                    is_evading = 0;
                    HAL_GPIO_WritePin(GPIOC, buzzer_Pin, GPIO_PIN_RESET);
                }
                else if (HAL_GetTick() - evasion_start_tick > EVASION_TIMEOUT_MS) {
                    // TIMEOUT: Robot acorralado. Ha pasado demasiado tiempo intentando evadir sin lograrlo
                    is_evading = 0;
                    is_trapped = 1; // Transición al estado bloqueado
                    HAL_GPIO_WritePin(GPIOC, buzzer_Pin, GPIO_PIN_RESET);
                }
            }
            else if (is_trapped) {
                // Si la pared desaparece (alguien la quita), se libera solo
                // Recuperación pasiva del entorno
                if (c > SAFE_ZONE && l > SAFE_ZONE && r > SAFE_ZONE) {
                    is_trapped = 0;
                }
            }

            // Cancelar evasión/atrapado si el robot recibe orden explícita de STOP (S)
            // Esto prioriza la orden de seguridad humana por encima de la autonomía de bajo nivel
            if ((is_evading || is_trapped) && !robot_en_movimiento) {
                is_evading = 0;
                is_trapped = 0;
                HAL_GPIO_WritePin(GPIOC, buzzer_Pin, GPIO_PIN_RESET);
            }

            // SOLUCIÓN CLAVE: Sincronizar el estado de trampa con la navegación.
            // Ahora el GPS no mandará paros de emergencia mientras estés acorralado.
            // Se notifica a la variable global compartida para que la tarea de navegación congele su corrección XTE
            global_is_evading = (is_evading || is_trapped);

            // 4. EJECUCIÓN DEL MOVIMIENTO
            if (is_trapped) {
                // MODO SEGURO: Motores apagados, esperando que quiten el obstáculo
                // o llegue un comando manual.
                evasion_command.left_speed = 0;
                evasion_command.right_speed = 0;
                evasion_command.bypass_ramp = 1; // Freno inmediato
                Motor_Process_Command(&htim2, &evasion_command);
            }
            else if (is_evading) {
                evasion_command.bypass_ramp = 1; // Salto de rampa para asegurar reactividad inmediata ante obstáculos

                // Variables para almacenar los grados de pertenencia de cada sensor en los conjuntos "Cerca" y "Lejos"
                float u_C_cerca = 0;
				float u_L_cerca = 0;
				float u_R_cerca = 0;
				float u_L_lejos = 0;
				float u_R_lejos = 0;


				// 4.1 Fuzzificación (Obtener grados de pertenencia)
                    // Traducimos las distancias crudas (en cm) a un valor continuo entre 0.0 y 1.0
					u_C_cerca = Fuzzy_Grado(c, CRITICAL_ZONE_S, SAFE_ZONE);
					u_L_cerca = Fuzzy_Grado(l, CRITICAL_ZONE_S, SAFE_ZONE);
					u_R_cerca = Fuzzy_Grado(r, CRITICAL_ZONE_S, SAFE_ZONE);

                    // El complemento matemático (1.0 - Grado de Cerca) nos da el grado de "Lejos"
					u_L_lejos = 1.0f - u_L_cerca;
					u_R_lejos = 1.0f - u_R_cerca;

					// 4.2 Evaluación de Reglas (Pesos 'w' mediante operador AND)
					// Regla 1: Frente bloqueado, Izquierda libre -> Giro Fuerte Izquierda
					float w1 = Fuzzy_AND(u_C_cerca, u_L_lejos);
					float cons1_L = EVADE_INNER; // Frena o reduce drásticamente la rueda interna
					float cons1_R = EVADE_OUTER; // Acelera la rueda externa para pivotar

					// Regla 2: Frente bloqueado, Derecha libre -> Giro Fuerte Derecha
					float w2 = Fuzzy_AND(u_C_cerca, u_R_lejos);
					float cons2_L = EVADE_OUTER;
					float cons2_R = EVADE_INNER;

					// Regla 3: Obstáculo a la Izquierda -> Desvío Suave Derecha
					float w3 = u_L_cerca;
					float cons3_L = BASE_FWD;
					float cons3_R = INERTIA_MIN;

					// Regla 4: Obstáculo a la Derecha -> Desvío Suave Izquierda
					float w4 = u_R_cerca;
					float cons4_L = INERTIA_MIN;
					float cons4_R = BASE_FWD;

					// Regla 5: Atrapado (Los 3 sensores cerca) -> Giro en base a diferencia (Sugeno 1er Orden)
					float w5 = Fuzzy_AND(u_C_cerca, Fuzzy_AND(u_L_cerca, u_R_cerca));
					// Calculamos la diferencia de distancias crudas para saber qué lado está más libre
					float dif_dist = (float)l - (float)r;
					// Calculamos la proporción de giro basándonos en REVERSE_STEER
					float steer = (dif_dist / 6.0f) * REVERSE_STEER;
					// Consecuentes dinámicos:
					// Si l > r (dif_dist positivo), la izquierda está más libre. steer es positivo.
					// La rueda derecha (cons5_R) se vuelve más rápida en reversa, empujando la cola hacia la izquierda.
					float cons5_L = REVERSE_SPEED + steer;
					float cons5_R = REVERSE_SPEED - steer;


					// 4.3 Defuzzificación (Promedio Ponderado Sugeno)
                    // Calculamos la salida neta del controlador unificando todas las reglas activadas
					float suma_pesos = w1 + w2 + w3 + w4 + w5;

					if (suma_pesos > 0.0f) {
                        // Multiplicamos la salida de cada regla por su peso de activación, y promediamos
						float vel_L_difusa = (w1*cons1_L + w2*cons2_L + w3*cons3_L + w4*cons4_L + w5*cons5_L) / suma_pesos;
						float vel_R_difusa = (w1*cons1_R + w2*cons2_R + w3*cons3_R + w4*cons4_R + w5*cons5_R) / suma_pesos;


					// Evasión en reversa en dado caso que el robot esté muy cerca de un obstáculo (Punto ciego / Riesgo crítico)
                    // Fuerza un retroceso ciego durante EVASION_REVER_MS milisegundos para despegarse del objeto físico
					if (c <= CRITICAL_ZONE_S || l <= DEAD_ZONE  || r <= DEAD_ZONE  || (HAL_GetTick() - reverse_tick < EVASION_REVER_MS)) {

                        // Decide hacia dónde enfilar la cola evaluando qué lado tiene más espacio
						if(l>r){
							vel_L_difusa = -BASE_FWD;
							vel_R_difusa = -INERTIA_MIN;
						} else{
							vel_R_difusa = -BASE_FWD;
							vel_L_difusa = -INERTIA_MIN;
						}

						reverse_evation = 1;
					} else {reverse_evation = 0; reverse_tick = 0; }

                    // Captura el tiempo inicial en el que se gatilló la maniobra de reversa
					if(reverse_evation && reverse_tick == 0){
							reverse_tick = HAL_GetTick();
					}

				// Limitamos las velocidades (Filtro Clamp de seguridad)
                // Previene que los cálculos difusos envíen velocidades físicamente imposibles o peligrosas a los PWM
				 if (vel_L_difusa > EVADE_OUTER)  vel_L_difusa = EVADE_OUTER;
				 if (vel_L_difusa < -EVADE_OUTER) vel_L_difusa = -EVADE_OUTER;
				 if (vel_R_difusa > EVADE_OUTER)  vel_R_difusa = EVADE_OUTER;
				 if (vel_R_difusa < -EVADE_OUTER) vel_R_difusa = -EVADE_OUTER;

                // Asignamos las velocidades finales calculadas al comando del motor local
				evasion_command.left_speed  = (int16_t)vel_L_difusa;
				evasion_command.right_speed = (int16_t)vel_R_difusa;



			} else {
				// Caso por defecto (falla de seguridad si no hay reglas activas pero is_evading es true)
                // Se ordena un avance precautorio base para evitar que el robot se apague inesperadamente
				evasion_command.left_speed  = BASE_FWD;
				evasion_command.right_speed = BASE_FWD;
			}

            // Enviamos el comando procesado por la lógica de evasión al driver L298N
	        Motor_Process_Command(&htim2, &evasion_command);
	    } else {
            // Si el camino está libre, simplemente ejecutamos la orden normal que nos mandó la tarea de navegación o el control manual
	        Motor_Process_Command(&htim2, &normal_command);
	    }

    // 5. Actualizar ramping
    // Ejecuta la lógica matemática que incrementa o decrementa suavemente los PWM (Aceleración/Desaceleración)
    Motor_Update(&htim2);

    osDelay(20); // Retorna el control al RTOS durante 20ms, fijando la frecuencia de actualización de motores a ~50Hz
  }
  /* USER CODE END StartMotorControlTask */
}

/* USER CODE BEGIN Header_StartLoraCommTask */
/**
* @brief Function implementing the loraCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLoraCommTask */
/*
 *  TAREA DE COMUNICACIÓN LORA
 *  Esta tarea representa el enlace inalámbrico del nodo edge con la arquitectura fog/cloud.
 *  Se encarga de escuchar, decodificar y validar los comandos recibidos desde la
 *  estación central, gestionar el guardado de coordenadas GPS (waypoints) enviadas en bloque
 *  y retornar paquetes de telemetría con el estado actual del robot.
 */
void StartLoraCommTask(void *argument)
{
  /* USER CODE BEGIN StartLoraCommTask */

    // Retardo inicial para dar tiempo a que el transceptor RYLR998 encienda y se estabilice
	osDelay(500);
    // Inicialización del buffer circular del UART dedicado al LoRa
	Ringbuf_Init(&lora_buffer_manager, &huart6);
    // Configuración del driver Lora (puerto serial, dirección de la red y reintentos)
	rylr998_init(&lora_module, &huart6, &lora_buffer_manager, LORA_ROBOT_ADDRESS, 5);

	printf("LORA: Protocolo de Carga Segura Listo.\r\n");

	// Inicializamos TODA la estructura con 0 desde el principio por seguridad para evitar comportamientos erráticos
	MotorCommand_t cmd = {0, 0, 0};
	char lora_payload[128]; // Buffer genérico para armar los mensajes de respuesta

	// CACHÉ ANTI-DUPLICADOS
    // Sistema de seguridad robusto para ignorar ráfagas de comandos repetidos generados por la red LoRa o rebotes
	 char last_wpt_payload[256] = "";
	 int last_batch_count = 0;

	// Variables temporales para telemetría
    // Protegemos la integridad de los datos extrayendo copias rápidas desde el Mutex
	float temp_geo_heading = 0.0f;
	GPS_Data_t temp_gps_data = {0};
	//uint16_t temp_distance = 0;


  /* Infinite loop */
  for(;;)
  {

	  // Polling cada 100ms
	        // Esto da tiempo al módulo LoRa para procesar comandos AT

            // Verificamos si la interrupción UART llenó un frame completo de datos validado
	        if (rylr998_receive_data(&lora_module))
	        {
	            // Copiar received_data a buffer local INMEDIATAMENTE
	            // Esto evita que se corrompa si llega otro mensaje por interrupción mientras lo estamos procesando
	            char cmd_buffer[256];
	            strncpy(cmd_buffer, lora_module.received_data, sizeof(cmd_buffer));
	            cmd_buffer[255] = '\0'; // Aseguramos el terminador nulo del string

	            // --- COMANDO 1: REINICIAR RUTA ---
                // Verifica si la cadena coincide. Este comando formatea todo el plan de vuelo
				if (strncmp(cmd_buffer, "CMD:RESET", 9) == 0) {
					osMessageQueueReset(waypointQueueHandle); // Vacía la cola de coordenadas de FreeRTOS
					autonomous_mode_active = 0; // Desactiva el modo autónomo por seguridad

					// Limpiar la caché al iniciar una ruta nueva
					last_wpt_payload[0] = '\0';
					last_batch_count = 0;
					waypoints_reached = 0;

					prev_wp_valid = 0; // Reseteamos la línea base del XTE para que no asuma una trayectoria irreal

                    // Enviamos un acuse de recibo (ACK) al servidor Fog/Cloud
					rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "MSG:RESET_OK");
					//printf("LORA: Cola limpia. Esperando puntos...\r\n");
				}

				// --- COMANDO 2: RECIBIR PUNTOS ---
                // Procesa un lote de coordenadas enviadas de golpe en formato WPT:lat,lon;lat,lon
				else if (strncmp(cmd_buffer, "WPT:", 4) == 0) {

					// ANTI-DUPLICADOS: Comparamos si es exactamente el mismo string de antes
                    // Previene que se apilen coordenadas idénticas si la central reenvía el paquete por falta de ACK
					if (strcmp(cmd_buffer, last_wpt_payload) == 0) {
						//printf("LORA: ⚠️ Paquete duplicado detectado. Ignorando guardado, re-enviando ACK...\r\n");

						// Solo reenviamos el ACK para destrabar al ESP32 que funciona como Gateway
						char ack_msg[40];
						uint32_t espacios_libres = osMessageQueueGetSpace(waypointQueueHandle);
						snprintf(ack_msg, sizeof(ack_msg), "MSG:WPT_OK_%d_FREE_%lu", last_batch_count, espacios_libres);
						osDelay(150); // Dar tiempo al módulo de la central para salir del modo TX
						rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, ack_msg);
					}
					else {
						// Es un paquete genuinamente nuevo. Lo copiamos al caché de seguridad.
						strncpy(last_wpt_payload, cmd_buffer, sizeof(last_wpt_payload));

						char *ptr = cmd_buffer + 4; // Trabajar en copia local, saltándonos el encabezado "WPT:"
						char *token_pair = strtok(ptr, ";"); // Tokenizamos utilizando el punto y coma como delimitador
						int batch_count = 0;

                        // Iteramos sobre todos los pares (lat,lon) que encontremos en el string
						while (token_pair != NULL) {
							double lat, lon;
                            // Parseamos numéricamente
							if (sscanf(token_pair, "%lf,%lf", &lat, &lon) == 2) {
                                // Validación geográfica dura para rechazar basura serial
								if (lat >= -90.0f && lat <= 90.0f &&
									lon >= -180.0f && lon <= 180.0f &&
									lat != 0.0f && lon != 0.0f) {

									Waypoint_t new_pt;
									new_pt.lat = lat;
									new_pt.lon = lon;

                                    // Intentamos encolar el waypoint en la memoria del RTOS
									if(osMessageQueuePut(waypointQueueHandle, &new_pt, 0, 0) == osOK){
										batch_count++;
									} else {
										//printf("ERROR: Cola llena!\r\n");
									}
								}
							}
							token_pair = strtok(NULL, ";");
						}

						// Guardamos cuántos puntos tuvo para si nos piden reenviar el ACK
						last_batch_count = batch_count;

						//printf("LORA: Recibido lote de %d puntos.\r\n", batch_count);

						// Enviar confirmación normal informando los puntos recibidos y la memoria RAM restante en la cola
						char ack_msg[40];
						uint32_t espacios_libres = osMessageQueueGetSpace(waypointQueueHandle);
						snprintf(ack_msg, sizeof(ack_msg), "MSG:WPT_OK_%d_FREE_%lu", batch_count, espacios_libres);
						osDelay(150); // Dar tiempo al módulo de la central para salir del modo TX
						rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, ack_msg);

                        // Confirmación acústica del hardware para el desarrollador en pruebas de campo
						if (batch_count > 0) {
							HAL_GPIO_WritePin(GPIOC, buzzer_Pin, GPIO_PIN_SET);
							osDelay(50);
							HAL_GPIO_WritePin(GPIOC, buzzer_Pin, GPIO_PIN_RESET);
						}
					}
				}

	            // --- COMANDO 3: VERIFICAR INTEGRIDAD ---
                // Reporta a la estación central cuántas coordenadas residen actualmente en memoria
	            else if (strncmp(cmd_buffer, "CMD:CHECK", 9) == 0) {
	                uint32_t count = osMessageQueueGetCount(waypointQueueHandle);

	                snprintf(lora_payload, sizeof(lora_payload), "MSG:COUNT=%lu", count);
	                rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, lora_payload);
	                //printf("LORA: Reportando %lu puntos en memoria.\r\n", count);
	            }

	            // --- COMANDO 4: INICIAR NAVEGACIÓN ---
                // Transición oficial del robot a estado autónomo si cumple las condiciones
	            else if (strncmp(cmd_buffer, "CMD:START", 9) == 0) {
	                uint32_t count = osMessageQueueGetCount(waypointQueueHandle);
	                if (count > 0) {
	                    autonomous_mode_active = 1; // Habilita la ejecución matemática en la tarea de navegación
	                    rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "MSG:NAV_STARTED");
	                    //printf("LORA: INICIANDO NAVEGACION AUTONOMA.\r\n");
	                } else {
	                    rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "ERR:NO_POINTS");
	                }
	            }

	            // --- COMANDO 5: PARAR ---
                // Freno total de emergencia remoto
	            else if (strncmp(cmd_buffer, "CMD:STOP", 8) == 0) {
	                autonomous_mode_active = 0;
	                osMessageQueueReset(waypointQueueHandle); // Purga la ruta

	                cmd.left_speed = 0;
	                cmd.right_speed = 0;
	                cmd.bypass_ramp = 0; // ACTUALIZADO: Rampa suave para detenerse cinemáticamente estable
	                osMessageQueuePut(motorCommandQueueHandle, &cmd, 0, 0);

                    // Apaga los LEDs indicadores de navegación
	                HAL_GPIO_WritePin(GPIOC, LedI_Pin, GPIO_PIN_RESET);
	                HAL_GPIO_WritePin(GPIOC, LedD_Pin, GPIO_PIN_RESET);

	                rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "MSG:STOPPED");
	            }

				// --- COMANDO 6b: EVASIÓN LOCAL ON/OFF ---
                // Permite apagar remotamente la lógica difusa (Útil durante depuración o para navegar pasillos apretados a la fuerza)
				else if (strncmp(cmd_buffer, "CMD:EVASION_ON", 14) == 0) {
					evasion_enabled = 1;
					rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "MSG:EVASION_ON");
				}

				else if (strncmp(cmd_buffer, "CMD:EVASION_OFF", 15) == 0) {
					evasion_enabled = 0;
					rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "MSG:EVASION_OFF");
				}

				// --- COMANDO: CAMBIAR FILTRO GPS ---
                // Tuning dinámico sin necesidad de recompilar el nodo edge
				else if (strncmp(cmd_buffer, "CMD:SET_GPSF:", 13) == 0) {
				    uint8_t filtro = (uint8_t)atoi(cmd_buffer + 13);
				    if (filtro <= 6) { // Seguridad estructural de límites del índice
				        gps_filter = filtro;
				        char ack[30];
				        snprintf(ack, sizeof(ack), "ACK:GPSF_%d", filtro);
				        rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, ack);
				    }
				}

				// --- COMANDO: AJUSTAR GANANCIAS PD ---
                // Configuración remota en caliente para el controlador Proporcional-Derivativo del seguimiento de ruta
				else if (strncmp(cmd_buffer, "CMD:SET_KP:", 11) == 0) {
				    float val = atoff(cmd_buffer + 11);
				    if (val > 0.0f && val <= 10.0f) {
				        NAV_KP = val;
				        rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "ACK:KP");
				    }
				}
				else if (strncmp(cmd_buffer, "CMD:SET_KD:", 11) == 0) {
				    float val = atoff(cmd_buffer + 11);
				    if (val >= 0.0f && val <= 2.0f) {
				        NAV_KD = val;
				        rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "ACK:KD");
				    }
				}

				// --- COMANDO: AJUSTAR PARÁMETROS XTE ---
                // Tuning dinámico del cálculo del Cross-Track Error
				else if (strncmp(cmd_buffer, "CMD:SET_XTEKP:", 14) == 0) {
				    float val = atoff(cmd_buffer + 14);
				    if (val >= 0.0f && val <= 5.0f) {
				        NAV_KP_XTE = val;
				        rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "ACK:XTEKP");
				    }
				}
				else if (strncmp(cmd_buffer, "CMD:SET_XTEMAX:", 15) == 0) {
				    float val = atoff(cmd_buffer + 15);
				    if (val >= 5.0f && val <= 90.0f) { // Prevención de giros caóticos o inestabilidades limitando a grados lógicos
				        XTE_MAX_DEG = val;
				        rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "ACK:XTEMAX");
				    }
				}

				// --- COMANDO: AJUSTAR DECLINACIÓN MAGNÉTICA ---
                // Ajuste crítico para el entorno local (p. ej. en Ciudad Obregón ronda los 8.28 grados)
				else if (strncmp(cmd_buffer, "CMD:SET_DECL:", 13) == 0) {
				    float val = atoff(cmd_buffer + 13);
				    if (!isnanf(val) && !isinff(val) && val >= -90.0f && val <= 90.0f) {
				        MAGNETIC_DECLINATION = val;
				        rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "ACK:DECL");
				    }
				}

	            // --- COMANDO 6: TELEMETRÍA (POLL) ---
                // Responde a solicitudes del nodo Fog para actualizar el dashboard web / aplicación
	            else if (strncmp(cmd_buffer, "POLL", 4) == 0) {
                    // Accedemos a los datos protegidos por el mutex (variables fusionadas de los sensores)
	                if (osMutexAcquire(sensorDataMutexHandle, 50) == osOK) {
	                     temp_gps_data = my_gps_data;
	                     temp_geo_heading = global_robot_heading + MAGNETIC_DECLINATION;
	                    // temp_distance = global_dist_c; // usa el sensor central
	                     osMutexRelease(sensorDataMutexHandle);
	                }
                    // Ajuste de anillo circular para el rumbo (0-360 grados)
	                if (temp_geo_heading >= 360.0f) temp_geo_heading -= 360.0f;

                    // Empaqueta los datos en un formato ligero y lo transmite
	                snprintf(lora_payload, sizeof(lora_payload), "LAT:%.7f,LON:%.7f,HDG:%d",
	                         temp_gps_data.latitude, temp_gps_data.longitude, (int)temp_geo_heading);
	                rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, lora_payload);
	            }

	            // --- COMANDOS MANUALES ---
                // Permite el control tipo "RC" (Radio Control) si la navegación autónoma está apagada
	            else if ((strlen(cmd_buffer) > 0) && (autonomous_mode_active == 0))
	            {
	                int valid = 1;

	                switch(cmd_buffer[0]) {
	                    case 'F': // FORWARD (Adelante)
	                        cmd.left_speed = 80; cmd.right_speed = 80;
	                        break;
	                    case 'L': // LEFT (Pivotear a la Izquierda)
	                        cmd.left_speed = -70; cmd.right_speed = 70;
	                        break;
	                    case 'R': // RIGHT (Pivotear a la Derecha)
	                        cmd.left_speed = 70; cmd.right_speed = -70;
	                        break;
	                    case 'B': // BACKWARD (Reversa)
	                        cmd.left_speed = -80; cmd.right_speed = -80;
	                        break;
	                    case 'S': // STOP (Detener)
	                        cmd.left_speed = 0; cmd.right_speed = 0;
	                        break;
	                    default:
	                        valid = 0; // Descartamos comandos de basura serial
	                        break;
	                }

	                if (valid) {
	              	   cmd.bypass_ramp = 0; // ACTUALIZADO: Movimientos manuales suaves aplicando rampas de aceleración
	                     osMessageQueuePut(motorCommandQueueHandle, &cmd, 0, 0); // Despacha la orden a los motores
	                     // ACK para que el ESP32 Gateway libere la cola y sepa que el robot reaccionó
	                      rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "ACK:CMD");
	                }
	            }
	        }

	        osDelay(100); // Polling amigable cediendo la CPU para que corran las demás tareas de FreeRTOS
  }
  /* USER CODE END StartLoraCommTask */
}

/* USER CODE BEGIN Header_StartI2cRecoveryTask */
/**
* @brief Function implementing the i2cRecoveryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartI2cRecoveryTask */
/*
 *  TAREA DE RECUPERACIÓN I2C
 *  Hilo de tolerancia a fallos dedicado exclusivamente a monitorear y reparar
 *  la comunicación con el sensor IMU (ICM20948). Dado que el bus I2C es propenso a
 *  bloquearse por ruido eléctrico o problemas físicos en las conexiones, esta tarea
 *  reinicia los periféricos a bajo nivel para restablecer el "heading" del robot
 *  sin tener que reiniciar por completo el procesador o detener la ejecución del RTOS.
 */
void StartI2cRecoveryTask(void *argument)
{
  /* USER CODE BEGIN StartI2cRecoveryTask */
	uint8_t retry_count = 0; // Contador de intentos consecutivos fallidos de inicializar el IMU
  /* Infinite loop */
  for(;;)
  {
      // La tarea se suspende de forma indefinida (osWaitForever) sin consumir CPU.
      // Solo despierta cuando la tarea de sensores detecta el cuelgue y libera este semáforo
	  if (osSemaphoreAcquire(i2cRecoverySemaphoreHandle, osWaitForever) == osOK)
	        {
                  // Elevamos bandera global de recuperación, avisando al resto del firmware que la IMU no es confiable temporalmente
	              is_recovering_i2c = 1;

	              printf("RECUPERANDO I2C...\r\n");

                  // Reseteo forzado a nivel de hardware del periférico de comunicación
	              HAL_I2C_DeInit(&hi2c1);
	              osDelay(10); // Tiempo para estabilización electrónica de las líneas SDA y SCL
	              HAL_I2C_Init(&hi2c1);
	              osDelay(20);

                  // Intentamos arrancar nuevamente la configuración interna de los registros del ICM20948
	              if (!ICM_Init(&hi2c1)) {
	            	  retry_count++;
					   if (retry_count >= 5) {  // 5 intentos antes de rendirse por completo
						   printf("FALLO CRITICO I2C: Desactivando IMU.\r\n");
                           // Estado 2: El IMU se declara oficialmente muerto. El robot seguirá operando degradado utilizando solo GPS y sonares
						   is_recovering_i2c = 2;
						   retry_count = 0;  // Reset para próxima vez que se solicite manualmente
					   } else {
						   printf("Reintentando en 1s...\r\n");
						   osDelay(1000);
                           // Auto-inyección del semáforo para gatillar el siguiente ciclo de la iteración de recuperación
						   osSemaphoreRelease(i2cRecoverySemaphoreHandle);
					   }
	              } else {
	                  printf("EXITO: I2C Recuperado.\r\n");
	                  is_recovering_i2c = 0; // ESTADO VIVO: Regresa el control y la confianza a la tarea de sensores
	                  retry_count = 0;  // Reset contador de fallas consecutivas
	              }
	        }
  }
  /* USER CODE END StartI2cRecoveryTask */
}
/* USER CODE BEGIN Header_StartNavigationTask */
/**
* @brief Function implementing the NavigationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNavigationTask */
/*
 *  TAREA DE NAVEGACIÓN AUTÓNOMA
 *  Este hilo es el "cerebro planificador" del nodo edge. Se encarga de procesar
 *  la cola de coordenadas geográficas (waypoints) enviadas desde la nube/fog,
 *  calculando continuamente la distancia y el rumbo (bearing) necesario para
 *  alcanzar el objetivo. Implementa un controlador Proporcional-Derivativo (PD)
 *  con corrección de error de trayectoria cruzada (Cross-Track Error o XTE) y
 *  gestiona la transición segura del control cinemático tras una maniobra de evasión.
 */
void StartNavigationTask(void *argument)
{
  /* USER CODE BEGIN StartNavigationTask */

	Waypoint_t current_target; // Estructura que almacena latitud y longitud del objetivo actual
	MotorCommand_t nav_cmd;    // Estructura del comando de velocidad que se enviará a la tarea de motores
	uint8_t has_target = 0;    // Bandera de estado: 1 si el robot está persiguiendo un punto, 0 si necesita extraer uno nuevo de la cola

	// ===== LÓGICA DE CONTROL PD =====
	// Ganancias iniciales (Normalmente sobreescritas dinámicamente vía LoRa desde el dashboard)
	//#define NAV_KP          1.8f    // Ganancia Proporcional (Fuerza de giro)
	//#define NAV_KD          0.04f   // Ganancia Derivativa (Amortiguación)
	//#define NAV_DT          0.05f   // Delta de tiempo de la tarea (osDelay(50) = 0.05s)
	//#define NAV_DEAD_ZONE   8.0f    // ±5° → avance recto
	//#define NAV_MIN_INNER   MIN_SPEED
    #define CORRECTION_MAX  180.0f      // Máxima desviación angular considerada para el escalado matemático
	#define TARGET_RANGE    3.0f        // Rango (en grados) para entrar a la zona muerta de corrección (avance recto)
    #define HISTER_ZONE     6.0f        // Rango (en grados) para salir de la zona muerta (evita oscilaciones por ruido)

	// ── CLAMPING (LIMITACIÓN DE SEGURIDAD) ──────
	// Definimos el PWM mínimo para vencer la fricción estática de los motorreductores
    // Evita que el algoritmo asigne un PWM tan bajo que el motor zumbe pero no logre mover físicamente la rueda
	#define MIN_MOVING_PWM 10

	static float last_error_heading = 0.0f; // Memoria de estado anterior para calcular el término derivativo (Tasa de cambio del error)

	// --- Variables para el Enfriamiento (Cooldown) post-evasión ---
    // Mecanismo de seguridad robótica para evitar caer en "Mínimos Locales".
    // Previene que el robot, inmediatamente después de evadir un obstáculo, intente girar bruscamente
    // hacia su objetivo GPS y termine chocando de nuevo con el mismo obstáculo.
	uint32_t cooldown_end_tick = 0;
	uint8_t was_evading = 0; // Detector de flanco de bajada para el estado de evasión
	#define EVASION_COOLDOWN_MS 1500  // 1.5 segundos de avance ciego (recto) obligatorio tras soltar la evasión


	//#define NAV_KP_XTE  0.78f   // Ganancia del XTE → grados de corrección por metro de error
	//#define XTE_MAX_DEG 25.0f  // Límite máximo de corrección por XTE (no girar más de 25°)


  /* Infinite loop */
  for(;;)
  {
	  // Solo corremos la matemática computacionalmente costosa si el modo autónomo está activo
	  if (!autonomous_mode_active) {
		  has_target = 0;
		  osDelay(500); // Hibernación ligera si el robot está en modo manual o en espera
		  continue;
	  }

	  // 1. Gestión de Puntos (Buffer FIFO del RTOS)
	  if (!has_target) {
		  // Intentar sacar el siguiente punto geográfico de la cola de mensajes
		  if (osMessageQueueGet(waypointQueueHandle, &current_target, NULL, 0) == osOK) {

			  has_target = 1; // Punto cargado exitosamente en memoria RAM de trabajo

		  } else {
			  // Cola vacía -> Fin de ruta (Misión cumplida)
			  autonomous_mode_active = 0;
			  nav_cmd.left_speed = 0; nav_cmd.right_speed = 0;
			  nav_cmd.bypass_ramp = 0; // Aseguramos rampa normal para un frenado suave por inercia
			  osMessageQueuePut(motorCommandQueueHandle, &nav_cmd, 0, 0); // Despachar orden de freno
			  HAL_GPIO_WritePin(GPIOC, LedD_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, LedI_Pin, GPIO_PIN_RESET);

			  // Avisar a la Base (Capa Fog/Node-RED) que la misión terminó con éxito vía telemetría LoRa
			  rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, "MSG:STOPPED");

			  // Aviso de buzzer acústico físico confirmando en campo que se finalizó la ruta
			  for(char i=0; i<4; i++){
				 HAL_GPIO_TogglePin(GPIOC, buzzer_Pin);
				 osDelay(250);
			  }
			  HAL_GPIO_WritePin(GPIOC, buzzer_Pin, GPIO_PIN_RESET);
			  continue;
		  }
	  }

	  // 2. Obtener Datos Actuales (Thread Safe)
	  GPS_Data_t current_pos;
	  float current_heading;

      // Adquisición crítica del Mutex. Garantizamos leer una "foto" íntegra y sincronizada de los sensores
      // sin riesgo de que la tarea sensora actualice la latitud pero no la longitud a mitad de nuestra lectura
	  if (osMutexAcquire(sensorDataMutexHandle, 20) == osOK) {
		  current_pos = my_gps_data;
		  current_heading = global_robot_heading + MAGNETIC_DECLINATION; // Compensación magnética local
		  osMutexRelease(sensorDataMutexHandle);
          // Normalización circular de la brújula
		  if (current_heading >= 360.0f) current_heading -= 360.0f;

		  // Validar datos GPS ****************** por seguridad
          // Si el sensor perdió fix (por nubes, árboles) o envía ceros absolutos, el robot no debe moverse a ciegas
		  if (!current_pos.data_is_valid ||
		      fabs(current_pos.latitude)  < 1e-9 ||
		      fabs(current_pos.longitude) < 1e-9 ||
		      current_pos.satellites_tracked < 4) {

			// Detener el chasis estructuralmente si los datos espaciales son basura
			  if (!global_is_evading) {   // Solo detener si NO estamos evadiendo activamente (La evasión difusa tiene prioridad de vida o muerte)
				  nav_cmd.left_speed = 0;
				  nav_cmd.right_speed = 0;
				  osMessageQueuePut(motorCommandQueueHandle, &nav_cmd, 0, 0);
			  }

			osDelay(200); // Esperar a que recupere satélites
			continue;
			} //***************************


	  } else {
		  osDelay(10); continue; // Reintentar en el próximo tick si el mutex estaba ocupado (Prevención de cuelgues)
	  }


	  // Si es el primer punto de la ruta, la línea base ideal
	  // empieza en nuestra posición actual real validada.
	  if (!prev_wp_valid &&
	      current_pos.data_is_valid &&
	      fabs(current_pos.latitude)  > 1e-9 &&
	      fabs(current_pos.longitude) > 1e-9 &&
	      current_pos.satellites_tracked >= 4) {
	      prev_wp_lat   = current_pos.latitude; // Anclamos el origen de la recta
	      prev_wp_lon   = current_pos.longitude;
	      prev_wp_valid = 1;
	  }

	  // 3. Cálculos de Navegación Esférica
      // Resoluciones matemáticas sobre la forma geoidal de la tierra
	  double dist = Nav_GetDistance(current_pos.latitude, current_pos.longitude,
	                                 current_target.lat, current_target.lon); // Distancia ortodrómica en metros
	  double target_heading = Nav_GetBearing(current_pos.latitude, current_pos.longitude,
	                                          current_target.lat, current_target.lon); // Acimut directo objetivo

	  // --- Corrección por XTE (Cross-Track Error) ---
      // El XTE compensa la deriva lateral causada por terrenos irregulares o motores desbalanceados,
      // obligando al robot a volver a la línea recta imaginaria entre el punto A y el punto B,
      // en lugar de hacer una curva parabólica infinita hacia el objetivo.

	  // Aplicar corrección XTE siempre y cuando NO se esté evadiendo (ni en su periodo de enfriamiento)
	  if (!(HAL_GetTick() < cooldown_end_tick || global_is_evading)){
		  if (prev_wp_valid) {
			  double xte = Nav_GetCrossTrackError(prev_wp_lat, prev_wp_lon,
												   current_pos.latitude, current_pos.longitude,
												   current_target.lat, current_target.lon);

			  // Convertir el error XTE (metros) a una corrección de rumbo compensatoria (grados):
              // Si el error es positivo (robot derivó a la derecha de la línea), corrige restando grados (giro a la izquierda)
			  float xte_correction = (float)(-xte * NAV_KP_XTE);

			  // Limitar (clamp) la corrección XTE paramétricamente para no sobregirar de manera inestable
			  if (xte_correction >  XTE_MAX_DEG) xte_correction =  XTE_MAX_DEG;
			  if (xte_correction < -XTE_MAX_DEG) xte_correction = -XTE_MAX_DEG;

			  // Sumar al bearing objetivo geográfico la inyección correctiva del XTE
			  target_heading += xte_correction;
              // Normalizar los 360 grados de la circunferencia
			  if (target_heading >= 360.0) target_heading -= 360.0;
			  if (target_heading <    0.0) target_heading += 360.0;
		  }
	  }

      // Cálculo del Error Principal del sistema de control
	  float error_heading = (float)(target_heading - current_heading);
	  	  // Normalizar el error al camino más corto (-180 a 180 grados).
          // Ej: Si estoy en 350° y quiero ir a 10°, el error no es 340°, es 20° girando a la derecha.
	  	  if (error_heading > 180) error_heading -= 360;
	  	  if (error_heading < -180) error_heading += 360;

	  // --- Lógica de Enfriamiento (Evitar Mínimo Local post-evasión) ---
		// Detectamos el momento exacto en que la tarea de motores canceló la bandera de evasión (flanco de bajada lógico)
		if (was_evading == 1 && global_is_evading == 0) {
            // Seteamos el timer temporal hacia el futuro
			cooldown_end_tick = HAL_GetTick() + EVASION_COOLDOWN_MS;
		}
		was_evading = global_is_evading; // Actualizamos la memoria del flanco

		// Si estamos en periodo de enfriamiento, o si el robot está evadiendo AHORA mismo a bajo nivel,
		// suprimimos la necesidad algorítmica de girar hacia el objetivo GPS.
		if (HAL_GetTick() < cooldown_end_tick || global_is_evading) {
			// Engañamos al controlador PD diciendo que el error es siempre 0.
			// Esto forzará al algoritmo a enviar comandos de trazado de una línea perfectamente recta
			// basándose en su rumbo actual inercial, permitiéndole alejarse físicamente del obstáculo
            // antes de intentar perfilarse de nuevo al punto GPS.
			error_heading = 0.0f;
			last_error_heading = 0.0f; // Resetear también la integral/derivada para que no acumule tensión fantasma
		}
		// -----------------------------------------------------------

	  // 4. Lógica de Control Cinemático (Máquina de Estados de 3 Niveles)

      // Verificamos si ya entramos en el radio de aceptación (hitbox) del waypoint objetivo
	  if (dist < TOLERANCIA_METROS) {

		  // Guardar posición actual validada como punto de partida (origen de la recta) del siguiente segmento
		  prev_wp_lat   = current_pos.latitude;
		  prev_wp_lon   = current_pos.longitude;
		  prev_wp_valid = 1;

		  // Llegamos al punto
		  has_target = 0; // Levantar bandera para forzar a extraer el siguiente waypoint de la cola FIFO en la próxima iteración


		  // Confirmar a la arquitectura superior (Base) que se completó un hito
		  waypoints_reached++;

		  char wp_msg[40];
		  snprintf(wp_msg, sizeof(wp_msg), "MSG:WPT_REACHED_%d", waypoints_reached);
		  rylr998_send_data(&lora_module, LORA_BASE_ADDRESS, wp_msg);

		  // Aviso físico (Buzzer) de que el chasis alcanzó exitosamente un nodo de la malla de navegación
		  HAL_GPIO_WritePin(GPIOC, buzzer_Pin, GPIO_PIN_SET);
		  osDelay(300);
		  HAL_GPIO_WritePin(GPIOC, buzzer_Pin, GPIO_PIN_RESET);
		  continue;
	  }

	        // Asegurarnos de que los comandos de navegación usen la rampa suave del puente H
            // para no estresar los engranajes ni generar picos de corriente inversos
	        nav_cmd.bypass_ramp = 0;

	        // Variable estática de estado para el ciclo de histéresis de la zona muerta
	        static uint8_t in_dead_zone = 0;

	        // Histéresis de Rumbo (Trigger de Schmitt por software)
            // Previene que el robot "vibre" corrigiendo micrométricamente cuando ya está casi alineado al objetivo
	        if (!in_dead_zone && fabsf(error_heading) < TARGET_RANGE)  in_dead_zone = 1; // Entrar a zona recta (ej. error menor a 3°)
	        if (in_dead_zone  && fabsf(error_heading) > HISTER_ZONE) in_dead_zone = 0; // Salir solo si se desvía considerablemente (ej. mayor a 6°)

	        if (in_dead_zone) {
	            // Avance 100% recto, el error magnético es despreciable
	            nav_cmd.left_speed  = MAX_SPEED;
	            nav_cmd.right_speed = MAX_SPEED;
	            last_error_heading  = error_heading; // Sincronizamos la memoria derivativa
	        } else {
	        	// --- Controlador PD (Proporcional-Derivativo) ---

                // Acción Proporcional: Aplica fuerza de giro dependiendo de qué tan lejos esté del rumbo correcto
				float P_term = error_heading * NAV_KP;

                // Acción Derivativa: Calcula la velocidad a la que se está corrigiendo el error para frenar suavemente
                // el giro y evitar pasarse del objetivo (Overshoot pendular)
				float d_error = error_heading - last_error_heading;
                // Normalización de la derivada cruzando el meridiano magnético
				if (d_error >  180.0f) d_error -= 360.0f;
				if (d_error < -180.0f) d_error += 360.0f;
				float D_term = d_error * NAV_KD;
				last_error_heading = error_heading; // Guardar error actual para el próximo ciclo de reloj

                // Ecuación general del controlador
				float correction = P_term + D_term;
				/*if (correction >  CORRECTION_MAX) correction =  CORRECTION_MAX;
				if (correction < -CORRECTION_MAX) correction = -CORRECTION_MAX; */

				// ── DIFERENCIAL SIMÉTRICO DE TRACCIÓN (Ajuste Cinemático) ──────────────────
				// Normaliza la señal de control a un rango de [-1.0, +1.0]
				float norm = correction / CORRECTION_MAX;

				// Usamos MAX_SPEED como vector base de avance. Al ir recto (norm=0), ambas llantas van a tope.
			    // Al girar, una mantiene su velocidad y la interior frena proporcionalmente.
				// Esto garantiza que el robot NUNCA pierda momento de avance longitudinal mientras gira,
                // evitando que se atasque en terrenos difíciles como tierra o pasto.
				float abs_norm = fabsf(norm);
				int16_t outer = MAX_SPEED; // Rueda exterior siempre empuja al frente
				int16_t inner = (int16_t)(MAX_SPEED * (1.0f - 2.0f * abs_norm)); // Rueda interior reduce su PWM para pivotar
				if (inner < 0) inner = 0; // Evitamos giro sobre su propio eje en navegación normal, solo pivote suave

				int16_t left_spd  = (int16_t)(MAX_SPEED * (1.0f + norm));
				int16_t right_spd = (int16_t)(MAX_SPEED * (1.0f - norm));

				/*if (correction > 0) {  left_spd = outer;  right_spd = inner; }
				else                 {  left_spd = inner;  right_spd = outer; }*/

                // Asignación de potencias cruzadas dependiendo del signo del error (Izquierda o Derecha)
				if (correction > 0) {
                    left_spd = inner;
                    right_spd = outer;
                } else {
                    left_spd = outer;
                    right_spd = inner;
                }


				// Clamp superior (Seguridad Hardware)
                // Nunca mandar al PWM un valor mayor a la máxima velocidad del chasis
			    if (left_spd  > MAX_SPEED) left_spd  = MAX_SPEED;
			    if (right_spd > MAX_SPEED) right_spd = MAX_SPEED;

				// Clamp inferior (Seguridad Cinética)
                // Si la rueda no debe detenerse por completo (0 inercial),
				// nos aseguramos de que el puente H reciba al menos el torque mínimo para no trabar el motor por fricción estática
				if (left_spd  < MIN_MOVING_PWM) left_spd  = MIN_MOVING_PWM;
				if (right_spd < MIN_MOVING_PWM) right_spd = MIN_MOVING_PWM;

                // Asignar el vector resultante al comando estructural
				nav_cmd.left_speed  = left_spd;
				nav_cmd.right_speed = right_spd;

				// LEDs Indicadores visuales en hardware de estado de corrección
				if (correction > 0) {
					HAL_GPIO_WritePin(GPIOC, LedD_Pin, GPIO_PIN_SET); // Corrigiendo hacia Derecha
					HAL_GPIO_WritePin(GPIOC, LedI_Pin, GPIO_PIN_RESET);
				} else {
					HAL_GPIO_WritePin(GPIOC, LedI_Pin, GPIO_PIN_SET); // Corrigiendo hacia Izquierda
					HAL_GPIO_WritePin(GPIOC, LedD_Pin, GPIO_PIN_RESET);
				}
	        }


	  // 5. Enviar al Controlador de Motores
      // Encolamos (Thread Safe) el comando hacia la tarea que realmente interactúa con los Timers PWM del microcontrolador
	  osMessageQueuePut(motorCommandQueueHandle, &nav_cmd, 0, 0);

	  osDelay(50); // Frecuencia de ciclo de navegación: 50ms (20 Hz), balance ideal entre precisión GPS y carga de CPU
  }
  /* USER CODE END StartNavigationTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
