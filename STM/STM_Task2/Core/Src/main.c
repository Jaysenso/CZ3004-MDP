/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "oled.h"
#include "ICM20948.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//////////////// PID Controller ////////////////
typedef struct _pidParams
{
	float Kp;
	float Ki;
	float Kd;
	float e;
	float eSum;
}PID;


//////////////// COMMAND PROCESSING ////////////////
typedef struct _command
{
	uint8_t index;
	uint16_t val;
}Command;

typedef struct _cmdConfig
{
	uint16_t leftDuty;
	uint16_t rightDuty;
	float servoTurnVal;
	float targetAngle;
	uint8_t direction;
}CmdConfig;

typedef struct _commandQueue
{
	uint8_t head;
	uint8_t tail;
	uint8_t size;
	Command buffer[12];
}CommandQueue;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONFIG_FL00 7
#define CONFIG_FR00 8
#define CONFIG_BL00 9
#define CONFIG_BR00 10

#define CONFIG_FL20 11
#define CONFIG_FR20 12
#define CONFIG_BL20 13
#define CONFIG_BR20 14

#define CONFIG_FL30 15
#define CONFIG_FR30 16
#define CONFIG_BL30 17
#define CONFIG_BR30 18

#define CONFIG_FC21 19
#define CONFIG_FC22 20

#define ADC_REF 3.3
#define ADC_STEPS 4096
#define BOT_LENGTH 25

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
enum TASK_TYPE{
	TASK_MOVE,
	TASK_MOVE_BACKWARD,
	TASK_FL,
	TASK_FR,
	TASK_BL,
	TASK_BR,
	TASK_ADC,
	TASK_MOVE_OBS,
	TASK_FASTESTPATH,
	TASK_FASTESTCAR,
	TASK_BUZZER,
	TASK_NONE
};

enum TASK_TYPE curTask = TASK_NONE, prevTask = TASK_NONE;

enum MOVE_MODE {
	SLOW,
	FAST
};

enum MOVE_MODE moveMode = FAST;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for OLED */
osThreadId_t OLEDHandle;
const osThreadAttr_t OLED_attributes = {
  .name = "OLED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UltrasonicTask */
osThreadId_t UltrasonicTaskHandle;
const osThreadAttr_t UltrasonicTask_attributes = {
  .name = "UltrasonicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LEncoderTask */
osThreadId_t LEncoderTaskHandle;
const osThreadAttr_t LEncoderTask_attributes = {
  .name = "LEncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for REncoderTask */
osThreadId_t REncoderTaskHandle;
const osThreadAttr_t REncoderTask_attributes = {
  .name = "REncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IRTask */
osThreadId_t IRTaskHandle;
const osThreadAttr_t IRTask_attributes = {
  .name = "IRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CmdProcessorTas */
osThreadId_t CmdProcessorTasHandle;
const osThreadAttr_t CmdProcessorTas_attributes = {
  .name = "CmdProcessorTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MoveDistTask */
osThreadId_t MoveDistTaskHandle;
const osThreadAttr_t MoveDistTask_attributes = {
  .name = "MoveDistTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MoveFLTask */
osThreadId_t MoveFLTaskHandle;
const osThreadAttr_t MoveFLTask_attributes = {
  .name = "MoveFLTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MoveFRTask */
osThreadId_t MoveFRTaskHandle;
const osThreadAttr_t MoveFRTask_attributes = {
  .name = "MoveFRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MoveBLTask */
osThreadId_t MoveBLTaskHandle;
const osThreadAttr_t MoveBLTask_attributes = {
  .name = "MoveBLTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MoveBRTask */
osThreadId_t MoveBRTaskHandle;
const osThreadAttr_t MoveBRTask_attributes = {
  .name = "MoveBRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MoveDistObsIR */
osThreadId_t MoveDistObsIRHandle;
const osThreadAttr_t MoveDistObsIR_attributes = {
  .name = "MoveDistObsIR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MoveDistObsUSTa */
osThreadId_t MoveDistObsUSTaHandle;
const osThreadAttr_t MoveDistObsUSTa_attributes = {
  .name = "MoveDistObsUSTa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FastestCarTask */
osThreadId_t FastestCarTaskHandle;
const osThreadAttr_t FastestCarTask_attributes = {
  .name = "FastestCarTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */
//////////////// USER BUTTON ////////////////
int btnClicked = 0;
int targetD = 5;
uint8_t tempDir = 1 ;
int8_t step = 0;
uint8_t turnMode = 2;
uint32_t last_int = 0;

//////////////// OLED ////////////////
uint8_t showTask[20];

//////////////// UART ////////////////
uint8_t RX_BUFFER_SIZE = 5;
uint8_t aRxBuffer[10];

//////////////// Ultrasonic ////////////////
//uint8_t UltraDistBuffer[20];
uint8_t first_capture = 0;
uint8_t ultra_distance = 0;
uint32_t ultra_tc1 = 0;
uint32_t ultra_tc2 = 0;
uint32_t ultra_diff = 0;
uint32_t startDist = 0;
uint32_t distTravelled = 0;
float obj1_dist;
float obj2_dist;
float return_dist;


//////////////// Infrared Sensor /////////////////
uint16_t obsTick_IR = 0;
float obsDist_IR = 0;
uint16_t dataPoint = 0;
uint32_t IR_data_raw_acc = 0;

volatile uint16_t adcResultsDMA[40];
volatile uint16_t ir_sensor_a_sum;
volatile uint16_t ir_sensor_b_sum;
const int adcChannelCount = sizeof(adcResultsDMA) / sizeof(adcResultsDMA[0]);
int IR_distances[2];
//////////////// Motor Encoder ////////////////
int l_encoder_diff;
int r_encoder_diff;
uint16_t l_encoder_dir;
uint16_t r_encoder_dir;
uint16_t record_ticks =0 ;
uint32_t start_tick = 0;
uint32_t encoder_count1 = 0;
uint32_t encoder_count2 = 0;
uint32_t encoder_count_positive_delta_x = 0;
uint32_t encoder_count_positive_delta_y = 0;
uint32_t encoder_count_negative_delta_x = 0;
uint32_t encoder_count_negative_delta_y = 0;
uint32_t total_count_y = 0;
uint32_t total_count_x = 0;
uint32_t obs2xdist = 0;
float target_obs2xdist = 0;

//////////////// GYROSCOPE ////////////////
uint8_t gZData[2];
int16_t gyroZ;

//////////////// PID Controller ////////////////
PID pidTurn, pidSlow, pidFast;

float targetAngle = 0;
float curAngle = 0;
float speedScale = 1;
uint16_t LMotorPWM, RMotorPWM;
uint32_t last_curTask_tick = 0;

float targetDist = 0;
uint16_t curDistTick = 0;
uint16_t targetDistTick = 0;
uint16_t dist_dL = 0;
uint16_t lastDistTick_L = 0;

int correction = 0;
int dir = 1;

//////////////// COMMAND PROCESSING ////////////////
CmdConfig cfgs[21] = {
//	{leftPWM, rightPWM, servoPWM, targetAngle, direction}
//	{0,0,SMOTOR_CENTER,0, DIR_FORWARD}, // 0. STOP
//	{1200, 1200, SMOTOR_CENTER, 0, DIR_FORWARD}, // 1. FW00
//	{1200, 1200, SMOTOR_CENTER, 0, DIR_BACKWARD}, // 2. BW00
//
//	{800, 1200, 50, 0, DIR_FORWARD}, // 3. FL--
//	{1200, 800, 115, 0, DIR_FORWARD}, // 4. FR--
//	{800, 1200, 50, 0, DIR_BACKWARD}, // 5. BL--
//	{1200, 800, 115, 0, DIR_BACKWARD}, // 6. BR--
//
////	{600, 1800, 75, 89, DIR_FORWARD}, // 7. FL00
////	{1800, 400, 255 ,-87, DIR_FORWARD}, // 8. FR00
////	{500, 1700, 75, -88, DIR_BACKWARD}, // 9. BL00
////	{1800, 500, 255, 89, DIR_BACKWARD}, // 10. BR00, was {1800, 500, 245, 89, DIR_BACKWARD}
//
//	// {600, 1800, 80, 89, DIR_FORWARD}, // 7. FL00
//	  {600, 1800, 80, 86.75, DIR_FORWARD}, // 7. FL00
//	// {1800, 400, 245 ,-87, DIR_FORWARD}, // 8. FR00
//	  {1800, 400, 245 ,-89, DIR_FORWARD}, // 8. FR00
//	{400, 1800, 80, -88, DIR_BACKWARD}, // 9. BL00
//	// {1800, 500, 245, 89, DIR_BACKWARD}, // 10. BR00,
//	  {1700, 500, 245, 88, DIR_BACKWARD}, // 10. BR00,
//
//	{800, 1800, 75, 89, DIR_FORWARD}, // 11. FL20
//	{1800, 900, 255 ,-87, DIR_FORWARD}, // 12. FR20
//	{700, 1800, 75, -89, DIR_BACKWARD}, // 13. BL20
//	{1800, 700, 255, 89, DIR_BACKWARD}, // 14. BR20,
//
//	{1500, 1500, 53, 87.5, DIR_FORWARD}, // 15. FL30
//	{1500, 1500, 108, -86.5, DIR_FORWARD}, // 16. FR30
//	{1500, 1500, 51, -87.5, DIR_BACKWARD}, // 17. BL30
//	{1500, 1100, 115, 88, DIR_BACKWARD}, // 18. BR30

	{0,0,SMOTOR_CENTER,0, DIR_FORWARD}, // STOP
	{3500, 3500, SMOTOR_CENTER, 0, DIR_FORWARD}, // FW00
	{3500, 3500, SMOTOR_CENTER, -1, DIR_BACKWARD}, // BW00

	{800, 1200, 85, 0, DIR_FORWARD}, // FL--
	{1200, 800, 240, 0, DIR_FORWARD}, // FR--
	{800, 1200, 85, 0, DIR_BACKWARD}, // BL--
	{1200, 800, 240, 0, DIR_BACKWARD}, // BR--

	{1400, 3500, 85, 83, DIR_FORWARD}, // FL00
	{3500, 1400, 240 ,-84.5, DIR_FORWARD}, // FR00
	{500, 1700, 85, -88, DIR_BACKWARD}, // BL00
	{1800, 500, 240, 89, DIR_BACKWARD}, // BR00,

	{900, 1800, 85, 88, DIR_FORWARD}, // FL20
	{1800, 900, 240 ,-88, DIR_FORWARD}, // FR20
	{700, 1800, 85, -87, DIR_BACKWARD}, // BL20
	{1800, 700, 240, 87, DIR_BACKWARD}, // BR20,

	{1500, 1500, 85, 87, DIR_FORWARD}, // FL30
	{1500, 1500, 240, -88, DIR_FORWARD}, // FR30
	{1500, 1500, 85, -86, DIR_BACKWARD}, // BL30
	{1500, 1500, 240, 86, DIR_BACKWARD}, // BR30

	{900, 1800, 85, 45, DIR_FORWARD}, // FC 1st obstacle left
	{1800, 900, 240 ,-45, DIR_FORWARD}, // FC 1st obstacle rigt


};

Command curCmd;
CommandQueue cmdq;

uint8_t CMD_BUFFER_SIZE = 12;
uint8_t manual_mode = 0;
uint8_t msg[16];
uint8_t cmdBuffer[4];

uint8_t taskStep = 0;
uint8_t task_test = 2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void motors(void *argument);
void show(void *argument);
void ultrasonic(void *argument);
void lencoder(void *argument);
void rencoder(void *argument);
void infrared(void *argument);
void processCmd(void *argument);
void moveDist(void *argument);
void moveFL(void *argument);
void moveFR(void *argument);
void moveBL(void *argument);
void moveBR(void *argument);
void moveDistObsIR(void *argument);
void moveDistObsUS(void *argument);
void fastestCar(void *argument);
/* USER CODE BEGIN PFP */
void PIDinit(PID *pid, const float Kp, const float Ki, const float Kd);
void PIDReset(PID *pid);
void MoveStraight(const uint8_t speedMode);
void MoveDist(float *targetDist, const uint8_t dir, const uint8_t speedMode);
void Turn(float *targetAngle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

  HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer, RX_BUFFER_SIZE);
  ICM20948_init(&hi2c1,0,GYRO_FULL_SCALE_2000DPS);

  // initialise command queue
    curCmd.index = 100;
    curCmd.val = 0;

    cmdq.head = 0;
    cmdq.tail = 0;
    cmdq.size = CMD_BUFFER_SIZE;
    for (int i = 0; i < CMD_BUFFER_SIZE; i++) {
  	  Command cmd;
  	  cmd.index = 100;
  	  cmd.val = 0;
  	  cmdq.buffer[i] = cmd;
    }

  PIDinit(&pidTurn, 2.5, 0.0,0.8);
  PIDinit(&pidSlow, 2.5, 0.0,0);
  PIDinit(&pidFast, 1.5, 0.0,0);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); //Start PWM timer for Rear Motor Left
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); //Start PWM timer for Rear Motor Right
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //Start PWM timer for Servo Motor

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  __RESET_SMOTOR_ANGLE(&htim1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motors, NULL, &MotorTask_attributes);

  /* creation of OLED */
  OLEDHandle = osThreadNew(show, NULL, &OLED_attributes);

  /* creation of UltrasonicTask */
  UltrasonicTaskHandle = osThreadNew(ultrasonic, NULL, &UltrasonicTask_attributes);

  /* creation of LEncoderTask */
  LEncoderTaskHandle = osThreadNew(lencoder, NULL, &LEncoderTask_attributes);

  /* creation of REncoderTask */
  REncoderTaskHandle = osThreadNew(rencoder, NULL, &REncoderTask_attributes);

  /* creation of IRTask */
  IRTaskHandle = osThreadNew(infrared, NULL, &IRTask_attributes);

  /* creation of CmdProcessorTas */
  CmdProcessorTasHandle = osThreadNew(processCmd, NULL, &CmdProcessorTas_attributes);

  /* creation of MoveDistTask */
  MoveDistTaskHandle = osThreadNew(moveDist, NULL, &MoveDistTask_attributes);

  /* creation of MoveFLTask */
  MoveFLTaskHandle = osThreadNew(moveFL, NULL, &MoveFLTask_attributes);

  /* creation of MoveFRTask */
  MoveFRTaskHandle = osThreadNew(moveFR, NULL, &MoveFRTask_attributes);

  /* creation of MoveBLTask */
  MoveBLTaskHandle = osThreadNew(moveBL, NULL, &MoveBLTask_attributes);

  /* creation of MoveBRTask */
  MoveBRTaskHandle = osThreadNew(moveBR, NULL, &MoveBRTask_attributes);

  /* creation of MoveDistObsIR */
  MoveDistObsIRHandle = osThreadNew(moveDistObsIR, NULL, &MoveDistObsIR_attributes);

  /* creation of MoveDistObsUSTa */
  MoveDistObsUSTaHandle = osThreadNew(moveDistObsUS, NULL, &MoveDistObsUSTa_attributes);

  /* creation of FastestCarTask */
  FastestCarTaskHandle = osThreadNew(fastestCar, NULL, &FastestCarTask_attributes);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|LMotor_ACW_CIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LMotor_CW_AIN2_Pin|LMotor_ACW_AIN1_Pin|RMotor_CW_BIN1_Pin|RMotor_ACW_BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LMotor_CW_CIN2_GPIO_Port, LMotor_CW_CIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RMotor_CW_DIN1_Pin|RMotor_ACW_DIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ULTRA_TRIG_GPIO_Port, ULTRA_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin LMotor_ACW_CIN1_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|LMotor_ACW_CIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LMotor_CW_AIN2_Pin LMotor_ACW_AIN1_Pin */
  GPIO_InitStruct.Pin = LMotor_CW_AIN2_Pin|LMotor_ACW_AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RMotor_CW_BIN1_Pin RMotor_ACW_BIN2_Pin */
  GPIO_InitStruct.Pin = RMotor_CW_BIN1_Pin|RMotor_ACW_BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LMotor_CW_CIN2_Pin */
  GPIO_InitStruct.Pin = LMotor_CW_CIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LMotor_CW_CIN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMotor_CW_DIN1_Pin RMotor_ACW_DIN2_Pin */
  GPIO_InitStruct.Pin = RMotor_CW_DIN1_Pin|RMotor_ACW_DIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULTRA_TRIG_Pin */
  GPIO_InitStruct.Pin = ULTRA_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ULTRA_TRIG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//////////////// INTERRUPT CALLBACK FUNCTIONS ////////////////
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	if (btnClicked) return;
	if (GPIO_Pin == USER_BTN_Pin) {
//		uint8_t tmp[20];
//		sprintf(tmp, "btn: %5d\0", 0);
//		OLED_ShowString(10, 10, tmp);
//		OLED_Refresh_Gram();
//		osDelay(1000);
//		HAL_UART_RxCpltCallback(&huart3);
//		targetDist = 120;
//		MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
//		uint8_t clicked[20];
		uint32_t int_time = HAL_GetTick();
		if(HAL_GetTick() - last_int > 250)
		{
			btnClicked++;
			if(task_test ==2) taskStep++;
			last_int = int_time;
		}
//		osDelay(2500);
//		btnClicked = 1;
//		d=1;

//		manualMode = 1;
//		moveMode = FAST;
//		__ADD_COMMAND(cQueue, 1, 90);
//		__READ_COMMAND(cQueue, curCmd, rxMsg);

//		__ADD_CMD(cmdq, 7 + step, 0);
//		__READ_CMD(cmdq, curCmd, msg);
//
//		step = (step + 1) % 4;
//		step = (step + 1) % 7;
//		__ADD_COMMAND(cQueue, 17, turnMode + 1);
//		__READ_COMMAND(cQueue, curCmd, rxMsg);
//		turnMode = (turnMode + 1) % 4;
//		__ADD_COMMAND(cQueue, 1, 100);
//		__READ_COMMAND(cQueue, curCmd, rxMsg);
	}

}

void task_btn_fn(){
		if(btnClicked == 1){
				aRxBuffer[0] = 'F';
				aRxBuffer[1] = 'C';
				aRxBuffer[2] = '0';
				aRxBuffer[3] = '1';
				btnClicked = 0;
		}

		else if(btnClicked == 2){
				aRxBuffer[0] = 'F';
				aRxBuffer[1] = 'C';
				aRxBuffer[2] = '2';
				aRxBuffer[3] = '1';
				btnClicked = 0;
		}
	//
		else if(btnClicked == 3){
				aRxBuffer[0] = 'F';
				aRxBuffer[1] = 'C';
				aRxBuffer[2] = '2';
				aRxBuffer[3] = '3';
				btnClicked = 0;
		}
	//
		else if(btnClicked == 4){
				aRxBuffer[0] = 'F';
				aRxBuffer[1] = 'C';
				aRxBuffer[2] = '2';
				aRxBuffer[3] = '4';
				btnClicked = 0;
		}

		else if(btnClicked == 5){
					aRxBuffer[0] = 'F';
					aRxBuffer[1] = 'C';
					aRxBuffer[2] = '2';
					aRxBuffer[3] = '5';
					btnClicked = 0;
		}

		else if(btnClicked == 6){
					aRxBuffer[0] = 'F';
					aRxBuffer[1] = 'C';
					aRxBuffer[2] = '2';
					aRxBuffer[3] = '7';
					btnClicked = 0;
		}

		else if(btnClicked == 7){
					aRxBuffer[0] = 'F';
					aRxBuffer[1] = 'C';
					aRxBuffer[2] = '2';
					aRxBuffer[3] = '9';
					btnClicked = 0;
		}

		else if(btnClicked == 8){
					aRxBuffer[0] = 'F';
					aRxBuffer[1] = 'C';
					aRxBuffer[2] = '4';
					aRxBuffer[3] = '0';
					btnClicked = 0;
		}

		else if(btnClicked == 9){
					aRxBuffer[0] = 'F';
					aRxBuffer[1] = 'C';
					aRxBuffer[2] = '4';
					aRxBuffer[3] = '1';
					btnClicked = 0;
		}
		else if(btnClicked == 10){
					aRxBuffer[0] = 'F';
					aRxBuffer[1] = 'C';
					aRxBuffer[2] = '4';
					aRxBuffer[3] = '2';
					btnClicked = 0;
		}
}

void task_step_fn(){
	btnClicked = taskStep;
	if(btnClicked == 1){
			aRxBuffer[0] = 'F';
			aRxBuffer[1] = 'C';
			aRxBuffer[2] = '0';
			aRxBuffer[3] = '1';
			btnClicked = 0;
	}

	else if(btnClicked == 2){
			aRxBuffer[0] = 'F';
			aRxBuffer[1] = 'C';
			aRxBuffer[2] = '2';
			aRxBuffer[3] = '1';
			btnClicked = 0;
	}
//
	else if(btnClicked == 3){
			aRxBuffer[0] = 'F';
			aRxBuffer[1] = 'C';
			aRxBuffer[2] = '2';
			aRxBuffer[3] = '2';
			btnClicked = 0;
	}
//
//	else if(btnClicked == 4){
//			aRxBuffer[0] = 'F';
//			aRxBuffer[1] = 'C';
//			aRxBuffer[2] = '2';
//			aRxBuffer[3] = '4';
//			btnClicked = 0;
//	}

//	else if(btnClicked == 5){
//				aRxBuffer[0] = 'F';
//				aRxBuffer[1] = 'C';
//				aRxBuffer[2] = '2';
//				aRxBuffer[3] = '5';
//				btnClicked = 0;
//	}

//	else if(btnClicked == 6){
//				aRxBuffer[0] = 'F';
//				aRxBuffer[1] = 'C';
//				aRxBuffer[2] = '2';
//				aRxBuffer[3] = '7';
//				btnClicked = 0;
//	}

//	else if(btnClicked == 7){
//				aRxBuffer[0] = 'F';
//				aRxBuffer[1] = 'C';
//				aRxBuffer[2] = '2';
//				aRxBuffer[3] = '9';
//				btnClicked = 0;
//	}

//	else if(btnClicked == 8){
//				aRxBuffer[0] = 'F';
//				aRxBuffer[1] = 'C';
//				aRxBuffer[2] = '4';
//				aRxBuffer[3] = '0';
//				btnClicked = 0;
//	}

//	else if(btnClicked == 9){
//				aRxBuffer[0] = 'F';
//				aRxBuffer[1] = 'C';
//				aRxBuffer[2] = '4';
//				aRxBuffer[3] = '1';
//				btnClicked = 0;
//	}
//	else if(btnClicked == 10){
//				aRxBuffer[0] = 'F';
//				aRxBuffer[1] = 'C';
//				aRxBuffer[2] = '4';
//				aRxBuffer[3] = '2';
//				btnClicked = 0;
//	}
	else if(btnClicked == 4){
			aRxBuffer[0] = 'F';
			aRxBuffer[1] = 'C';
			aRxBuffer[2] = '4';
			aRxBuffer[3] = '3';
			btnClicked = 0;
		}
//	else if(btnClicked == 12){
//			aRxBuffer[0] = 'F';
//			aRxBuffer[1] = 'C';
//			aRxBuffer[2] = '4';
//			aRxBuffer[3] = '5';
//			btnClicked = 0;
//		}
	else if(btnClicked == 5){
			aRxBuffer[0] = 'F';
			aRxBuffer[1] = 'C';
			aRxBuffer[2] = '4';
			aRxBuffer[3] = '6';
			btnClicked = 0;
		}
	else if(btnClicked == 6){
				aRxBuffer[0] = 'F';
				aRxBuffer[1] = 'C';
				aRxBuffer[2] = '4';
				aRxBuffer[3] = '7';
				btnClicked = 0;
			}
	else if(btnClicked == 7){
				aRxBuffer[0] = 'F';
				aRxBuffer[1] = 'C';
				aRxBuffer[2] = '4';
				aRxBuffer[3] = '8';
				btnClicked = 0;
			}
	else if(btnClicked == 8){
				aRxBuffer[0] = 'F';
				aRxBuffer[1] = 'C';
				aRxBuffer[2] = '4';
				aRxBuffer[3] = '9';
				btnClicked = 0;
			}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//This function is for Interrupt for UART Rx
	UNUSED(huart); //Prevent unused argument(s) compilation warning
	//Put received commands into a command queue (TODO)
//	HAL_UART_Transmit(&huart, (uint8_t *)aRxBuffer, 10, 0xFFFF);

	if (task_test == 1) task_btn_fn();
	else if (task_test == 2) task_step_fn();

//	else
//	{
//				aRxBuffer[0] = 'F';
//				aRxBuffer[1] = 'W';
//				aRxBuffer[2] = '0';
//				aRxBuffer[3] = '0';
//				btnClicked = 0;
//	}

//	cmdBuffer[0] = aRxBuffer[0];
//	cmdBuffer[1] = aRxBuffer[1];
//	cmdBuffer[2] = aRxBuffer[2];
//	cmdBuffer[3] = aRxBuffer[3];

	int val;

	val = (aRxBuffer[2] - 48) * 10 + (aRxBuffer[3] - 48);
	if (aRxBuffer[1] >= '0' && aRxBuffer[1] <= '9') val += (aRxBuffer[1] - 48) * 100;

	manual_mode = 0;

	if (aRxBuffer[0] == 'S' && aRxBuffer[1] == 'T') { // only STOP can preempt any greedy task
//		__ADD_COMMAND(cQueue, 0, 0); // stop
		__TASK_END(&htim8, prevTask, curTask);
		  curAngle = 0;
		  gyroZ = 0; // reset angle for PID
		PIDReset(&pidTurn);
		PIDReset(&pidSlow);
		PIDReset(&pidFast);
		curDistTick = 0;
		if (__IS_EMPTY(cmdq)) {
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, msg);
		}
		else {
			__READ_CMD(cmdq, curCmd, msg);
		}
	}
	else if (aRxBuffer[0] == 'F' && (aRxBuffer[1] == 'W' || aRxBuffer[1] == 'S')) { //FW or FS
		manual_mode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		moveMode = aRxBuffer[1] == 'S' ? SLOW : FAST;
		__ADD_CMD(cmdq, 1, val);
	}
	else if (aRxBuffer[0] == 'B' && (aRxBuffer[1] == 'W' || aRxBuffer[1] == 'S')) { //BW or BS
		manual_mode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		moveMode = aRxBuffer[1] == 'S' ? SLOW : FAST;
		__ADD_CMD(cmdq, 2, val);
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'L') { // FL
		manual_mode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		__ADD_CMD(cmdq, 3 + (manual_mode ? 0 : 4), val);
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'R') { // FR
		manual_mode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		__ADD_CMD(cmdq, 4 + (manual_mode ? 0 : 4), val);
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'L') { // BL
		manual_mode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		__ADD_CMD(cmdq, 5 + (manual_mode ? 0 : 4), val);
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'R') { // BR
		manual_mode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		__ADD_CMD(cmdq, 6 + (manual_mode ? 0 : 4), val);
	}
	else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'L') __ADD_CMD(cmdq, 11, val); // TL turn left max
	else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'R') __ADD_CMD(cmdq, 12, val); // TR turn right max
	else if (aRxBuffer[0] == 'I' && aRxBuffer[1] == 'R') __ADD_CMD(cmdq, 13, val); // test IR sensor
	else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'T') __ADD_CMD(cmdq, 14, val); // DT move until specified distance from obstacle
	else if (aRxBuffer[0] == 'Z' && aRxBuffer[1] == 'Z') __ADD_CMD(cmdq, 15, val); // ZZ buzzer
	else if (aRxBuffer[0] == 'W' && aRxBuffer[1] == 'X') __ADD_CMD(cmdq, 16, val); // WN fastest path
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'C') __ADD_CMD(cmdq, 17, val); // FC fastest car
	else if (aRxBuffer[0] == 'A') __ADD_CMD(cmdq, 88, val); // anti-clockwise rotation with variable
	else if (aRxBuffer[0] == 'C') __ADD_CMD(cmdq, 89, val); // clockwise rotation with variable

	if (!__IS_EMPTY(cmdq)) {
		__READ_CMD(cmdq, curCmd, msg);
	}

	// clear aRx buffer
	  __HAL_UART_FLUSH_DRREGISTER(&huart3);
	  HAL_UART_Receive_IT(&huart3, aRxBuffer, RX_BUFFER_SIZE);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//This function is for Interrupt generated by timer input capture
	if(htim==&htim4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) //if the interrupt is from timer4ch4 (ULTRA_ECHO)
	{
		if(first_capture == 0)
		{
			ultra_tc1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); //read first captured time
			first_capture = 1; //Set first capture True; First captured is recorded
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING); //Change capture polarity to falling
		}
		else if (first_capture == 1) //for second time capture; first time has already been recorded
		{
			ultra_tc2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			__HAL_TIM_SET_COUNTER(htim, 0); //reset counter

			if(ultra_tc2 > ultra_tc1)
			{
				ultra_diff = ultra_tc2 - ultra_tc1;
			}
			else
			{
				ultra_diff = (65535 - ultra_tc1) + ultra_tc2;
			}

			ultra_distance = ultra_diff * 0.034/2;
			//ultra_index = (ultra_index + 1) % NUM_SAMPLES;
			first_capture = 0; //Reset after computation

			//Set polarity to rising
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
		}
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	ir_sensor_a_sum = adcResultsDMA[0] + adcResultsDMA[2] + adcResultsDMA[4] + adcResultsDMA[6] + adcResultsDMA[8]+ adcResultsDMA[10] + adcResultsDMA[12] + adcResultsDMA[14] + adcResultsDMA[16] + adcResultsDMA[18];
	ir_sensor_b_sum = adcResultsDMA[1] + adcResultsDMA[3] + adcResultsDMA[5] + adcResultsDMA[7] + adcResultsDMA[9]+ adcResultsDMA[11] + adcResultsDMA[13] + adcResultsDMA[15] + adcResultsDMA[17] + adcResultsDMA[19];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	ir_sensor_a_sum = adcResultsDMA[20] + adcResultsDMA[22] + adcResultsDMA[24] + adcResultsDMA[26] + adcResultsDMA[28]+ adcResultsDMA[30] + adcResultsDMA[32] + adcResultsDMA[34] + adcResultsDMA[36] + adcResultsDMA[38];
	ir_sensor_b_sum = adcResultsDMA[21] + adcResultsDMA[23] + adcResultsDMA[25] + adcResultsDMA[27] + adcResultsDMA[29]+ adcResultsDMA[31] + adcResultsDMA[33] + adcResultsDMA[35] + adcResultsDMA[37] + adcResultsDMA[39];

}
//////////////// LED3 HELPER FUNCTIONS ////////////////
void BlinkLED3()
{
	while(1)
	{
		if(btnClicked) break;
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		osDelay(1000);
	}
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	btnClicked = 0;
}

//////////////// ULTRASONIC HELPER FUNCTIONS ////////////////
void delay_us(uint16_t us)
{ //delay in microseconds for ultrasonic sensor
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while (__HAL_TIM_GET_COUNTER(&htim6) < us);
}

//////////////// INFRARED HELPER FUNCTIONS ////////////////
uint16_t ADC_To_Dist(uint16_t raw)
{
	float volt = raw * ADC_REF / ADC_STEPS;
	return (uint16_t)(29.988 * pow(volt, -1.173));
}

//////////////// PID CONTROLLER HELPER FUNCTIONS ////////////////
void PIDinit(PID *pid, const float Kp, const float Ki, const float Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->e = 0;
	pid->eSum = 0;
}

void PIDReset(PID *pid)
{
	pid->e = 0;
	pid->eSum = 0;
}

//////////////// MOTOR DRIVER FUNCTIONS ////////////////
void MoveStraight(const uint8_t speed_mode)
{
	__READ_GYRO_Z(&hi2c1, gZData, gyroZ); //poll to update global
	dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? -1 : 1; //Using RMotor to determine robot direction
	curAngle += ((gyroZ >= -4 && gyroZ <= 8) ? 0 : gyroZ);

	if(speed_mode == SPEED_MODE_TURN) __PID_TURN(pidTurn, curAngle, correction, dir, LMotorPWM, RMotorPWM);
	else if(speed_mode == SPEED_MODE_SLOW) __PID_SLOW(pidSlow, curAngle, correction, dir, LMotorPWM, RMotorPWM);
	else if(speed_mode == SPEED_MODE_FAST) __PID_FAST(pidFast, curAngle, correction, dir, LMotorPWM, RMotorPWM);

	__SET_MOTOR_PWM(&htim8, LMotorPWM, RMotorPWM);
}

void MoveStraightSpeedScale(const uint8_t speed_mode, float *speedScale)
{
	__READ_GYRO_Z(&hi2c1, gZData, gyroZ); //poll to update global
	dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? -1 : 1; //Using RMotor to determine robot direction
	curAngle += ((gyroZ >= -4 && gyroZ <= 8) ? 0 : gyroZ);

	if(speed_mode == SPEED_MODE_SLOW) __PID_SLOW(pidSlow, curAngle, correction, dir, LMotorPWM, RMotorPWM);
	else if(speed_mode == SPEED_MODE_FAST) __PID_FAST(pidFast, curAngle, correction, dir, LMotorPWM, RMotorPWM);

	__SET_MOTOR_PWM(&htim8, LMotorPWM * (*speedScale), RMotorPWM  * (*speedScale));
}

void MoveDist(float *targetDist, const uint8_t dir, const uint8_t speedMode)
{
	curAngle = 0;
	gyroZ = 0;
	PIDReset(&pidTurn);
	PIDReset(&pidSlow);
	PIDReset(&pidFast);
	curDistTick = 0;

	__GET_TARGETTICK(*targetDist, targetDistTick);
	last_curTask_tick = HAL_GetTick();
	__SET_MOTOR_DIR(dir);
	__SET_ENCODER_LAST_TICK(&htim2, lastDistTick_L);
	while(1)
	{
		__GET_ENCODER_TICK_DELTA(&htim2, lastDistTick_L, dist_dL);
		curDistTick += dist_dL;

		if(curDistTick >= targetDistTick) break;

		if(HAL_GetTick() - last_curTask_tick >= 10) {
			if(speedMode == SPEED_MODE_TURN)
				MoveStraight(SPEED_MODE_TURN);
			else
			{
				speedScale = abs(curDistTick - targetDistTick) / 990; //slow down at last 15cm (990 ticks)
				if(speedMode == SPEED_MODE_SLOW) speedScale = speedScale > 1 ? 1 : (speedScale < 0.75 ? 0.75 : speedScale);
				else if (speedMode == SPEED_MODE_FAST)speedScale = speedScale > 1 ? 1 : (speedScale < 0.4 ? 0.4 : speedScale);
				MoveStraightSpeedScale(speedMode, &speedScale);
			}
		last_curTask_tick = HAL_GetTick();
		}
	}
	__SET_MOTOR_PWM(&htim8, 0, 0);
}

void Turn(float *targetAngle)
{
	curAngle = 0;
	gyroZ = 0;

	last_curTask_tick = HAL_GetTick();

	while(1)
	{
//		printf(HAL_GetTick() - last_curTask_tick);
		if(HAL_GetTick() - last_curTask_tick >= 10)
		{
			__READ_GYRO_Z(&hi2c1, gZData, gyroZ);
			curAngle += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;

			if(abs(curAngle - *targetAngle) < 0.01) break;

			last_curTask_tick = HAL_GetTick();
		}
	}

	__SET_MOTOR_PWM(&htim8, 0, 0);
	__RESET_SMOTOR_ANGLE(&htim1);
}
uint8_t Obs_Detected_IR(uint8_t sensor)
{
  int dist = IR_distances[sensor];
//  float sum = 0;
//  for(int i = 0; i < 50; i++)
//  {
//    sum += IR_distances[sensor];
//    osDelay(1);
//  }
//  dist = (int) (sum / 50);

  if(dist >= 90 || (dist > 10 && dist < 16)) return 1;
  else return 0;
}

uint8_t IR_Hit(uint8_t prev, uint8_t cur)
{
  return prev == 0 && cur == 1 ? 1 : 0;
}

uint8_t IR_Miss(uint8_t prev, uint8_t cur)
{
  return prev == 1 && cur == 0 ? 1 : 0;
}

void RobotMoveUntilIRHit() {
	obsDist_IR = 1000;
	curAngle = 0; gyroZ = 0;
	  last_curTask_tick = HAL_GetTick();
	  do {
		  __ADC_Read_Dist(&hadc1, dataPoint, IR_data_raw_acc, obsDist_IR, obsTick_IR);
		  if (abs(obsDist_IR) < 30) break;
		  if (HAL_GetTick() - last_curTask_tick >= 10) {
			  MoveStraight(SPEED_MODE_2);
			  last_curTask_tick = HAL_GetTick();
		  }
	  } while (1);
	  __SET_MOTOR_PWM(&htim8, 0, 0);
}

void MoveDistObstacleIR(float *targetDist,uint8_t sensor)
{
	dataPoint = 0; IR_data_raw_acc = 0; obsDist_IR = 1000;
	last_curTask_tick = HAL_GetTick();
	// declare previous and current
	uint8_t previous;
	uint8_t current;
	previous = Obs_Detected_IR(sensor);
	// assign previous = obstacle detector function;
//	__PEND_CURCMD(curCmd);

	do {
		//get current = obstacle detector function;
		current = Obs_Detected_IR(sensor);
		//if IR miss break;

		if(IR_Miss(previous,current) || IR_distances[sensor] > 70) break;

		current = previous;
		//update previous;
		if (*targetDist > 0 && abs(*targetDist - ultra_distance) < 0.1) break;



		__SET_MOTOR_DIR(obsDist_IR >= *targetDist);
	  if (HAL_GetTick() - last_curTask_tick >=10) {
		  speedScale = 0.5;
		  //speedScale = abs(obsDist_IR - *targetDist) / 15; // slow down at 15cm
		  //speedScale = speedScale > 1 ? 1 : (speedScale < 0.3 ? 0.3 : speedScale);
		  MoveStraightSpeedScale(SPEED_MODE_FAST, &speedScale);

		  last_curTask_tick = HAL_GetTick();
	  }
	  osDelay(10);
	} while (1);

//  __ON_TASK_END(&htim8, prevTask, curTask);
	__SET_MOTOR_PWM(&htim8, 0, 0);
	HAL_ADC_Stop(&hadc1);
}


float USavg(){
	  float sum = 0;
	    for (int i = 0; i < 10; i++) {
	        sum += ultra_distance;
	        osDelay(5);
	    }
	    return (sum / 10);
}
void MoveDistObstacleUS(float *targetDist, const uint8_t speedMode)
{
	curAngle = 0; gyroZ = 0;
	PIDReset(&pidTurn);
	PIDReset(&pidSlow);
	PIDReset(&pidFast);
//	obsDist_US = 1000;
//	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	last_curTask_tick = HAL_GetTick();
	while(1)
	{
		if (abs(*targetDist - ultra_distance) < 0.2) break;
		__SET_MOTOR_DIR(ultra_distance >= *targetDist);

		if (HAL_GetTick() - last_curTask_tick >=20) {
			//		  speedScale = 1;
			if (speedMode == SPEED_MODE_SLOW)
			{
				speedScale = abs(ultra_distance - *targetDist) / 15; // slow down at 15cm
				speedScale = speedScale > 1 ? 1 : (speedScale < 0.75 ? 0.75 : speedScale);
				MoveStraightSpeedScale(SPEED_MODE_SLOW, &speedScale);
			}
			else
			{
				speedScale = abs(ultra_distance - *targetDist) / 15; // slow down at 15cm
				speedScale = speedScale > 1 ? 1 : (speedScale < 0.4 ? 0.4 : speedScale);
				MoveStraightSpeedScale(SPEED_MODE_FAST, &speedScale);
			}
			last_curTask_tick = HAL_GetTick();
		}
		osDelay(10);
	}
	__SET_MOTOR_PWM(&htim8, 0, 0);
//	distTravelled = abs(startDist - ultra_dist);
	//TODO: send distTravelled
}

void RobotMoveUntilIRHitR() {
    last_curTask_tick = HAL_GetTick();
    do {
        // Assume IR_distances[] is updated elsewhere, like in an ADC interrupt
        if (abs(IR_distances[1]) < 40) break; // Check left IR sensor
        if (HAL_GetTick() - last_curTask_tick >= 10) {
        	speedScale = 0.5;
        	MoveStraightSpeedScale(SPEED_MODE_FAST, &speedScale);
            last_curTask_tick = HAL_GetTick();
        }
    } while (1);
    __SET_MOTOR_PWM(&htim8, 0, 0);
}

void RobotMoveUntilIRHitL() {
    last_curTask_tick = HAL_GetTick();
    do {
        // Assume IR_distances[] is updated elsewhere, like in an ADC interrupt
        if (abs(IR_distances[0]) < 40) break; // Check left IR sensor
        if (HAL_GetTick() - last_curTask_tick >= 10) {
        	speedScale = 0.5;
        	MoveStraightSpeedScale(SPEED_MODE_FAST, &speedScale);
            last_curTask_tick = HAL_GetTick();
        }
    } while (1);
    __SET_MOTOR_PWM(&htim8, 0, 0);
}

void RobotMoveUntilIROvershootR() {
//	obsDist_IR = 0;
//	curAngle = 0; gyroZ = 0;
	  last_curTask_tick = HAL_GetTick();
	  do {
//		  __ADC_Read_Dist(&hadc1, dataPoint, IR_data_raw_acc, obsDist_IR, obsTick_IR);
//		  if (obsDist_IR > 50) break;
		  if(abs(IR_distances[1]) > 40) break;
		  if (HAL_GetTick() - last_curTask_tick >= 10) {
			  speedScale = 0.5;
			  MoveStraightSpeedScale(SPEED_MODE_FAST, &speedScale);
			  last_curTask_tick = HAL_GetTick();
		  }
	  } while (1);
	  __SET_MOTOR_PWM(&htim8, 0, 0);
}

void RobotMoveUntilIROvershootL() {
//	obsDist_IR = 0;
//	curAngle = 0; gyroZ = 0;
	  last_curTask_tick = HAL_GetTick();
	  do {
//		  __ADC_Read_Dist(&hadc1, dataPoint, IR_data_raw_acc, obsDist_IR, obsTick_IR);
//		  if (obsDist_IR > 50) break;
		  if(abs(IR_distances[0]) > 40) break;
		  if (HAL_GetTick() - last_curTask_tick >= 10) {
			  speedScale = 0.5;
			  MoveStraightSpeedScale(SPEED_MODE_FAST, &speedScale);
			  last_curTask_tick = HAL_GetTick();
		  }
	  } while (1);
	  __SET_MOTOR_PWM(&htim8, 0, 0);
}

void FastestCar_Turn_L180(uint8_t turnSize) {
	__SET_MOTOR_DIR(1);
	switch (turnSize) {
	case 1:
		targetAngle = 172;
		__SET_SMOTOR_ANGLE(&htim1, 83);
		__SET_MOTOR_PWM(&htim8, 2000, 3500);
		break;
	case 2:

	case 3:
	case 4:
	default:
//		targetAngle = -176;
		targetAngle = 170;
		__SET_SMOTOR_ANGLE(&htim1, 83);
//		__SET_MOTOR_PWM(&htim8, 2700, 2500);
		__SET_MOTOR_PWM(&htim8, 3500, 3240);
		break;
	}
	Turn(&targetAngle);
}


void FastestCar_Turn_R180(uint8_t turnSize) {
	__SET_MOTOR_DIR(1);
	switch (turnSize) {
	case 1:
		targetAngle = -172;
		__SET_SMOTOR_ANGLE(&htim1, 200);
		__SET_MOTOR_PWM(&htim8, 3500, 2000);
		break;
	case 2:

	case 3:
	case 4:
	default:
//		targetAngle = -176;
		targetAngle = -170;
		__SET_SMOTOR_ANGLE(&htim1, 240);
//		__SET_MOTOR_PWM(&htim8, 2700, 2500);
		__SET_MOTOR_PWM(&htim8, 3500, 3240);
		break;
	}
	Turn(&targetAngle);
}

float test_obslen = 0;
float ObsLenUS(uint8_t turnDir)
{
	//Use pythagoras to estimate obstacle length
	float a, b, c;
	float prev_c;
	a = (float) ultra_distance;
	c = a;

	curAngle = 0;
	gyroZ = 0;

	last_curTask_tick = HAL_GetTick();

	targetAngle = turnDir ? 90 : -90;
	__SET_SMOTOR_ANGLE(&htim1, turnDir ? 80 : 245);
	__SET_MOTOR_PWM(&htim8, turnDir ? 0 : 2000, turnDir ? 2000 : 0);

	while(1)
	{
		if(HAL_GetTick() - last_curTask_tick >= 10)
		{
			c = ultra_distance;

			__READ_GYRO_Z(&hi2c1, gZData, gyroZ);
			curAngle += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;

			if(abs(c - prev_c) < 10) prev_c = c;
			if(abs(curAngle - targetAngle) < 0.01) break;

			last_curTask_tick = HAL_GetTick();
		}
	}

	c = prev_c;
	b = sqrt(pow(c, 2) - pow(a, 2));

	__SET_MOTOR_PWM(&htim8, 0, 0);
	__RESET_SMOTOR_ANGLE(&htim1);

	return b*2;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	sprintf(showTask, "Task: ", 0);
  /* Infinite loop */
//  HAL_UART_RxCpltCallback(&huart3);
//  uint8_t buf[20];
  for(;;)
  {
	  /*
	//For Checklist A.4 & A.5
	if(btnClicked)
	{
		osDelay(3000);
		if(btnClicked == 1) sprintf(showTask, "Task: FW100", 0);
		else if(btnClicked == 2) sprintf(showTask, "Task: BW100", 0);
		else if(btnClicked == 3) sprintf(showTask, "Task: FL00", 0);
		else if(btnClicked == 4) sprintf(showTask, "Task: FR00", 0);
		else if(btnClicked == 5) sprintf(showTask, "Task: BL00", 0);
		else if(btnClicked == 6) sprintf(showTask, "Task: BR00", 0);
		else if(btnClicked == 7) sprintf(showTask, "Task: IR00", 0);
		else if(btnClicked == 8) sprintf(showTask, "Task: DT40", 0);
		else sprintf(showTask, "Task: ", 0);
		HAL_UART_RxCpltCallback(&huart3);
	} */
//	sprintf(buf, "%5d", ultra_distance);
//    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
//    if(ch < 'Z')
//    	ch++;
//    else
//    	ch = 'A';
//    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motors */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motors */
void motors(void *argument)
{
  /* USER CODE BEGIN motors */
//  HAL_UART_RxCpltCallback(&huart3);
//  uint16_t pwmVal = 0;

//  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); //Start PWM timer for Rear Motor Left
//  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); //Start PWM timer for Rear Motor Right
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //Start PWM timer for Servo Motor
  /* Infinite loop */
  for(;;)
  {
//	//Rear Motor clockwise
//	//Min speed 380 (rough guide)
//    while(pwmVal < 4000)
//    {
//    	HAL_GPIO_WritePin(GPIOA, LMotor_CW_AIN2_Pin, GPIO_PIN_SET);
//    	HAL_GPIO_WritePin(GPIOA, LMotor_ACW_AIN1_Pin, GPIO_PIN_RESET);
//    	HAL_GPIO_WritePin(GPIOA, RMotor_ACW_BIN2_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA, RMotor_CW_BIN1_Pin, GPIO_PIN_RESET);
//    	pwmVal++;
//    	//Modify the comparison value for the duty cycle
//    	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmVal);
//    	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmVal);
//    	osDelay(10);
//    }
//    //Rear Motor anticlockwise
//    while(pwmVal > 0)
//    {
//    	HAL_GPIO_WritePin(GPIOA, LMotor_ACW_AIN1_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA, LMotor_CW_AIN2_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOA, RMotor_CW_BIN1_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA, RMotor_ACW_BIN2_Pin, GPIO_PIN_RESET);
//    	pwmVal--;
//    	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmVal);
//    	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmVal);
//    	osDelay(10);
//    }
//
//    //Servo Motor (80, 142, 252)
//    htim1.Instance->CCR4 = 142; //center
//    osDelay(5000);
//	htim1.Instance->CCR4 = 252; //most right
//	osDelay(5000);
//	htim1.Instance->CCR4 = 142; //center
//	osDelay(5000);
//	htim1.Instance->CCR4 = 80; //most left
//	osDelay(5000);
////	Motor_Forward(2000);
//    osDelay(1000);
  }
  /* USER CODE END motors */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the OLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
	uint8_t oled_buf[20];
  /* Infinite loop */
  for(;;)
  {
    //sprintf(oled_buf, "Clicked: %d\0", btnClicked);
    //OLED_ShowString(10, 0, oled_buf);
	sprintf(oled_buf, "CMD: %5s\0", aRxBuffer);
	OLED_ShowString(0, 0, oled_buf);
    sprintf(oled_buf, "USDist: %5d\0", ultra_distance);
    OLED_ShowString(0, 10, oled_buf);
    //sprintf(oled_buf, "Ticks %5d\0", record_ticks);

    // sprintf(oled_buf, "county: %5d\0", total_count_y);
    sprintf(oled_buf, "IRDistL: %5d\0", IR_distances[0]);
	OLED_ShowString(0, 20, oled_buf);
	// sprintf(oled_buf, "countx: %5d\0", total_count_x);
	sprintf(oled_buf, "IRDistR: %5d\0", IR_distances[1]);
	OLED_ShowString(0, 30, oled_buf);
	// sprintf(oled_buf, "ObsLen: %5d\0", test_obslen);
	sprintf(oled_buf, "count_Y: %5d\0", total_count_y);
	OLED_ShowString(0, 40, oled_buf);
	sprintf(oled_buf, "count_X: %5d\0", total_count_x);
	OLED_ShowString(0, 50, oled_buf);
	//sprintf(oled_buf, "ObsLen: %5d\0", test_obslen);
	//OLED_ShowString(0, 50, oled_buf);
    OLED_Refresh_Gram();

    //To test transmission of data every time OLED refreshes
//    HAL_UART_Transmit(&huart3, oled_buf, 11, 0xFFFF);

    osDelay(100);
  }
  /* USER CODE END show */
}

/* USER CODE BEGIN Header_ultrasonic */
/**
* @brief Function implementing the UltrasonicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ultrasonic */
void ultrasonic(void *argument)
{
  /* USER CODE BEGIN ultrasonic */
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1); //Enable interrupt for echo
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4); //Start timer interrupt for echo
  HAL_TIM_Base_Start(&htim6); //Start timer for delay_us
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_WritePin(ULTRA_TRIG_GPIO_Port, ULTRA_TRIG_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(ULTRA_TRIG_GPIO_Port, ULTRA_TRIG_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,GPIO_PIN_SET);
    delay_us(10); //wait for 10us (use tim6 to generate this delay?)
    HAL_GPIO_WritePin(ULTRA_TRIG_GPIO_Port, ULTRA_TRIG_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,GPIO_PIN_RESET);
    HAL_Delay(50);
//    osDelay(5000);
  }
  /* USER CODE END ultrasonic */
}

/* USER CODE BEGIN Header_lencoder */
/**
* @brief Function implementing the LEncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lencoder */
void lencoder(void *argument)
{
  /* USER CODE BEGIN lencoder */
//	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //Speed for LMotor
//
//	int lcnt1, lcnt2, ldiff;
//	uint32_t ltick;
//	uint16_t ldir;
//
//	lcnt1 = __HAL_TIM_GET_COUNTER(&htim2);
//	ltick = HAL_GetTick();
  /* Infinite loop */
	for(;;)
	{
		/////////Left Motor Encoder////////
//		if(HAL_GetTick() - ltick > 1000L)
//		{
//			lcnt2 = __HAL_TIM_GET_COUNTER(&htim2);
//			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
//			{
//				if(lcnt2<lcnt1)
//					ldiff = lcnt1 - lcnt2;
//				else
//					ldiff = (65535 - lcnt2) + lcnt1;
//			}
//			else
//			{
//				if(lcnt2 > lcnt1)
//					ldiff = lcnt2 - lcnt1;
//				else
//					ldiff = (65535 - lcnt1) + lcnt2;
//			}
//			//use global and use oled task to show
//			l_encoder_diff = ldiff;
//			ldir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
//			l_encoder_dir = ldir;
//
//			//Restart process
//			lcnt1 = __HAL_TIM_GET_COUNTER(&htim2);
//			ltick = HAL_GetTick();
//	}
    osDelay(1); //Don't delay, it'll cause encoder to be inaccurate
  }
  /* USER CODE END lencoder */
}

/* USER CODE BEGIN Header_rencoder */
/**
* @brief Function implementing the REncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rencoder */
void rencoder(void *argument)
{
  /* USER CODE BEGIN rencoder */
//	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //Speed for RMotor
//	uint32_t rtick;
//	int rcnt1, rcnt2, rdiff;
//	uint16_t rdir;
//
//	rcnt1 = __HAL_TIM_GET_COUNTER(&htim3);
//	rtick = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
	////////Right Motor Encoder////////
//	if(HAL_GetTick() - rtick > 1000L)
//		{
//			rcnt2 = __HAL_TIM_GET_COUNTER(&htim3);
//			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
//			{
//				if(rcnt2<rcnt1)
//					rdiff = rcnt1 - rcnt2;
//				else
//					rdiff = (65535 - rcnt2) + rcnt1;
//			}
//			else
//			{
//				if(rcnt2 > rcnt1)
//					rdiff = rcnt2 - rcnt1;
//				else
//					rdiff = (65535 - rcnt1) + rcnt2;
//			}
//			//use global and use oled task to show
//			r_encoder_diff = rdiff;
//			rdir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
//			r_encoder_dir = rdir;
//
//			//Update tick
//			rcnt1 = __HAL_TIM_GET_COUNTER(&htim3);
//			rtick = HAL_GetTick();
//		}
	//osDelay(1); //No delay because we won't get the right tick
  }
    osDelay(1); //Don't delay, it'll cause encoder to be inaccurate
  /* USER CODE END rencoder */
}

/* USER CODE BEGIN Header_infrared */
/**
* @brief Function implementing the IRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_infrared */
void infrared(void *argument)
{
  /* USER CODE BEGIN infrared */
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResultsDMA, adcChannelCount);
	IR_distances[0] = ADC_To_Dist((uint16_t)(ir_sensor_a_sum / 10));
	IR_distances[1] = ADC_To_Dist((uint16_t)(ir_sensor_b_sum / 10));
    osDelay(500);
  }
  /* USER CODE END infrared */
}

/* USER CODE BEGIN Header_processCmd */
/**
* @brief Function implementing the CmdProcessorTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_processCmd */
void processCmd(void *argument)
{
  /* USER CODE BEGIN processCmd */
//	uint8_t tmp[20];
  /* Infinite loop */
  for(;;)
  {
	  switch(curCmd.index)
	  {
	  //	  	 case 0: // STOP handled in UART IRQ directly
	  //	  	  	  break;
	  	  	 case 1: //FW
	  	  	 case 2: //BW
	  	  		curTask = curCmd.index == 1 ? TASK_MOVE : TASK_MOVE_BACKWARD;
	  	  		__PEND_CURCMD(curCmd);
	  	  		 break;
	  	  	case 3: //FL manual
	  		case 4: //FR manual
	  		case 5: //BL manual
	  		case 6: //BR manual
	  			__SET_CMD_CONFIG(cfgs[curCmd.index], &htim8, &htim1, targetAngle);
	  			if (__IS_EMPTY(cmdq)) {
	  				__CLEAR_CURCMD(curCmd);
	  				__ACK_TASK_DONE(&huart3, msg);
	  			} else __READ_CMD(cmdq, curCmd, msg);
	  			__PEND_CURCMD(curCmd);
	  			 break;
	  	  	 case 7: // FL
//	  			sprintf(tmp, "curCmdi: %5d\0", curCmd.index);
//	  			OLED_ShowString(10, 30, tmp);
//	  			OLED_Refresh_Gram();

	  	  		 curTask = TASK_FL;
	  	  		__PEND_CURCMD(curCmd);
	  	  		 break;
	  	  	 case 8: // FR
	  	  		curTask = TASK_FR;
	  	  		__PEND_CURCMD(curCmd);
	  	  		break;
	  	  	 case 9: // BL
	  	  		curTask = TASK_BL;
	  	  		__PEND_CURCMD(curCmd);
	  	  		break;
	  	  	 case 10: //BR
	  	  		curTask = TASK_BR;
	  	  		__PEND_CURCMD(curCmd);
	  	  		break;
	  	  	 case 11: // TL
	  	  	 case 12: // TR
//	  	  		 __SET_SERVO_TURN_MAX(&htim1, curCmd.index - 11 ? 1 : 0);
//	  	  		__CLEAR_CURCMD(curCmd);
//	  			__ACK_TASK_DONE(&huart3, rxMsg);
	  	  		 break;
	  	  	 case 13: // debug IR sensor
	  	  		 curTask = TASK_ADC;
	  	  		 break;
	  	  	 case 14: // DT move until specified distance from obstacle
	  	  		  curTask = TASK_MOVE_OBS;
	  	  		  __PEND_CURCMD(curCmd);
	  	  		 break;
	  	  	 case 15:
//	  	  		 curTask = TASK_BUZZER;
//	  	  		__PEND_CURCMD(curCmd);
	  	  		break;
	  	  	 case 16:
//	  	  		 curTask = TASK_FASTESTPATH;
//	  	  		__PEND_CURCMD(curCmd);
	  	  		 break;
	  	  	 case 17:
	  	  		 curTask = TASK_FASTESTCAR;
	  	  		__PEND_CURCMD(curCmd);
	  	  		 break;
	  	  	 case 88: // Axxx, rotate left by xxx degree
	  	  	 case 89: // Cxxx, rotate right by xxx degree
//	  	  		 __SET_SERVO_TURN_MAX(&htim1, curCmd.index - 88);
//	  	  		 __SET_MOTOR_DIR(DIR_FORWARD);
//	  	  		 if (curCmd.index == 88) {
//	  	  			 targetAngle = curCmd.val;
//	  	  			 __SET_MOTOR_PWM(&htim8, 800, 1200);
//	  	  		 } else {
//	  	  			targetAngle = -curCmd.val;
//	  	  		__SET_MOTOR_PWM(&htim8, 1200, 800);
//	  	  		 }
//	  	  		__PEND_CURCMD(curCmd);
//	  	  		 RobotTurn(&targetAngle);
	  	  		 break;
	  	  	 case 99:
	  	  		 break;
	  	  	 case 100:
	  	  		 break;
	  	  	 default:
	  	  //		 curCmd.index = 99;
	  	  		 break;
	  	  	 }
    osDelay(100);
  }
  /* USER CODE END processCmd */
}

/* USER CODE BEGIN Header_moveDist */
/**
* @brief Function implementing the MoveDistTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveDist */
void moveDist(void *argument)
{
  /* USER CODE BEGIN moveDist */
//	uint8_t tmp[20];
  /* Infinite loop */
  for(;;)
  {
	  if(curTask != TASK_MOVE && curTask != TASK_MOVE_BACKWARD) osDelay(1000);
	  else
	  {
//		  uint32_t startTimer = HAL_GetTick(); //for getting task time elapsed
		  if(manual_mode)
		  {
			  curAngle = 0;
			  gyroZ = 0;
			  PIDReset(&pidTurn);
			  PIDReset(&pidSlow);
			  PIDReset(&pidFast);

			  __SET_MOTOR_DIR(curTask == TASK_MOVE ? DIR_FORWARD : DIR_BACKWARD);

			  __TASK_END(&htim8, prevTask, curTask);
			  btnClicked = 0;
			  __CLEAR_CURCMD(curCmd);
			  __ACK_TASK_DONE(&huart3, msg);

			  last_curTask_tick = HAL_GetTick();
			  while(1)
			  {
				  if(!manual_mode) break;

				  if(HAL_GetTick() - last_curTask_tick >= 10)
				  {
					  MoveStraight(SPEED_MODE_TURN);
					  last_curTask_tick = HAL_GetTick();
				  }
			  }
		  }
		  else
		  {
			  targetDist = (float) curCmd.val;
//			  targetDist = (float) 4;
			  if(targetDist <= 15) moveMode = SLOW; //if target distance is lesser than 15, force move mode to SLOW

			  if(moveMode == SLOW)
				  MoveDist(&targetDist, curTask == TASK_MOVE ? DIR_FORWARD : DIR_BACKWARD, SPEED_MODE_SLOW);
			  else
				  MoveDist(&targetDist, curTask == TASK_MOVE ? DIR_FORWARD : DIR_BACKWARD, SPEED_MODE_FAST);

			  __TASK_END(&htim8, prevTask, curTask);
//			  uint32_t timeElapse = HAL_GetTick() - startTimer;
//			  sprintf(tmp, "timeMoved: %5.2fs\0", timeElapse * 0.001);
//			  OLED_ShowString(10, 10, tmp);
//			  OLED_Refresh_Gram();
			  btnClicked = 0;

			  if(__IS_EMPTY(cmdq))
			  {
				  __CLEAR_CURCMD(curCmd);
				  __ACK_TASK_DONE(&huart3, msg);
			  }
			  else
				  __READ_CMD(cmdq, curCmd, msg);
		  }
	}
//    osDelay(1);
  }
  /* USER CODE END moveDist */
}

/* USER CODE BEGIN Header_moveFL */
/**
* @brief Function implementing the MoveFLTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveFL */
void moveFL(void *argument)
{
  /* USER CODE BEGIN moveFL */
	uint8_t tmp[20];
  /* Infinite loop */
  for(;;)
  {
	  if(curTask != TASK_FL) osDelay(1000);
	  else
	  {
		  switch(curCmd.val){
		  	  case 30: //FL30 (4x2)
		  		  __SET_CMD_CONFIG(cfgs[CONFIG_FL30], &htim8, &htim1, targetAngle);
		  		  Turn(&targetAngle);
		  		  osDelay(10);
		  		  targetDist = 4;
		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  break;

		  	  case 20: //FL20 (3x1)
		  		  targetDist = (float) 4;
		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  __SET_CMD_CONFIG(cfgs[CONFIG_FL20], &htim8, &htim1, targetAngle);
		  		  Turn(&targetAngle);
		  		  osDelay(10);
		  		  targetDist = (float) 7;
		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  break;

		  	  default:
//		  		  uint32_t startTimer = HAL_GetTick();
//		  		  targetDist = 12;
//		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
//		  		  osDelay(10);

//		  		  targetDist = 5; //


		  		  __SET_CMD_CONFIG(cfgs[CONFIG_FL00], &htim8, &htim1, targetAngle);
		  		  Turn(&targetAngle);
		  		  osDelay(10);

		  		  targetDist = 12.5; // 10.25;
		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
		  		  osDelay(10);

//		  		  targetDist = 7;
//		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
//		  		  osDelay(10);
//		  		  uint32_t timeElapse = HAL_GetTick() - startTimer;
//		  		  sprintf(tmp, "timeTurn: %5.2fs\0", timeElapse * 0.001);
//		  		  OLED_ShowString(10, 10, tmp);
//		  		  OLED_Refresh_Gram();
		  		  break;
		}
		btnClicked = 0;
		prevTask = curTask;
		curTask = TASK_NONE;

		if(__IS_EMPTY(cmdq))
		{
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, msg);
		}
		else
			__READ_CMD(cmdq, curCmd, msg);
	}
//    osDelay(1);
  }
  /* USER CODE END moveFL */
}

/* USER CODE BEGIN Header_moveFR */
/**
* @brief Function implementing the MoveFRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveFR */
void moveFR(void *argument)
{
  /* USER CODE BEGIN moveFR */
  /* Infinite loop */
  for(;;)
  {
	  if(curTask != TASK_FR) osDelay(1000);
	  else
	  {
		  switch(curCmd.val){
		  	  case 30: //FR30 (4x2)
		  		  __SET_CMD_CONFIG(cfgs[CONFIG_FR30], &htim8, &htim1, targetAngle);
		  		  Turn(&targetAngle);
		  		  osDelay(10);
		  		  targetDist = 4;
		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  break;

		  	  case 20: //FR20 (3x1)
		  		  targetDist = 4;
		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  __SET_CMD_CONFIG(cfgs[CONFIG_FR20], &htim8, &htim1, targetAngle);
		  		  Turn(&targetAngle);
		  		  osDelay(10);
		  		  targetDist = 8;
		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  break;

		  	  default: //FR00 (indoor 3x1)
//		  		  targetDist = 7;
//		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
//		  		  osDelay(10);
//		  		  targetDist = 2.3;
//		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
//		  		  osDelay(10);

		  		  targetDist = 10; // 2.5;
		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
		  		  osDelay(10);


		  		  __SET_CMD_CONFIG(cfgs[CONFIG_FR00], &htim8, &htim1, targetAngle);
		  		  Turn(&targetAngle);

		  		  targetDist = 4;  // 38;
		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
		  		  osDelay(10);


//		  		  osDelay(10);
//		  		  targetDist = 0;
//		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
//		  		  osDelay(10);
//
//		  		  targetDist = 6;
//		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
//		  		  osDelay(10);
		  		  break;
		}
		btnClicked = 0;
		prevTask = curTask;
		curTask = TASK_NONE;

		if(__IS_EMPTY(cmdq))
		{
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, msg);
		}
		else
			__READ_CMD(cmdq, curCmd, msg);

	  }
//    osDelay(1);
  }
  /* USER CODE END moveFR */
}

/* USER CODE BEGIN Header_moveBL */
/**
* @brief Function implementing the MoveBLTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveBL */
void moveBL(void *argument)
{
  /* USER CODE BEGIN moveBL */
  /* Infinite loop */
  for(;;)
  {
	  if(curTask != TASK_BL) osDelay(1000);
	  else
	  {
		  switch(curCmd.val){
		  	  case 30: //FL30 (4x2)
		  		  __SET_CMD_CONFIG(cfgs[CONFIG_BL30], &htim8, &htim1, targetAngle);
		  		  Turn(&targetAngle);
		  		  osDelay(10);
		  		  targetDist = 4.5;
		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  break;

		  	  case 20: //BL20 (outdoor 3x1)
		  		  targetDist = 6;
		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  __SET_CMD_CONFIG(cfgs[CONFIG_BL20], &htim8, &htim1, targetAngle);
		  		  Turn(&targetAngle);
		  		  osDelay(10);
		  		  targetDist = 2;
		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  break;

		  	  default: //BL00 (indoor 3x1)
		  		  targetDist = 5;
		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
//		  		  targetDist = 4; // 0.85;
//		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
//		  		  osDelay(10);

		  		  __SET_CMD_CONFIG(cfgs[CONFIG_BL00], &htim8, &htim1, targetAngle);
		  		  Turn(&targetAngle);
		  		  osDelay(10);

//		  		  targetDist = 7;
//		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
//		  		  osDelay(10);

//		  		  targetDist = 2.5; // 12.3;
//		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
//		  		  osDelay(10);
		  		  break;
		}
		btnClicked = 0;
		prevTask = curTask;
		curTask = TASK_NONE;

		if(__IS_EMPTY(cmdq))
		{
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, msg);
		}
		else
			__READ_CMD(cmdq, curCmd, msg);

	  }
//    osDelay(1);
  }
  /* USER CODE END moveBL */
}

/* USER CODE BEGIN Header_moveBR */
/**
* @brief Function implementing the MoveBRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveBR */
void moveBR(void *argument)
{
  /* USER CODE BEGIN moveBR */
  /* Infinite loop */
  for(;;)
  {
	  if(curTask != TASK_BR) osDelay(1000);
	  else
	  {
		  switch(curCmd.val){
		  	  case 30: //FL30 (4x2)
		  		  __SET_CMD_CONFIG(cfgs[CONFIG_BR30], &htim8, &htim1, targetAngle);
		  		  Turn(&targetAngle);
		  		  osDelay(10);
		  		  targetDist = 5;
		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  break;

		  	  case 20: //BR20 (outdoor 3x1)
		  		  targetDist = 7;
		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  __SET_CMD_CONFIG(cfgs[CONFIG_BR20], &htim8, &htim1, targetAngle);
		  		  Turn(&targetAngle);
		  		  osDelay(10);
		  		  targetDist = 3;
		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  break;

		  	  default: //BR00 (indoor 3x1)
		  		  targetDist = 13; //1.75;
		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
		  		  osDelay(10);

//		  		  targetDist = 0;
//		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
//		  		  osDelay(10);

		  		  __SET_CMD_CONFIG(cfgs[CONFIG_BR00], &htim8, &htim1, targetAngle);
		  		  Turn(&targetAngle);
		  		  osDelay(10);

//		  		  targetDist = 2.3;
//		  		  MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_TURN);
//		  		  osDelay(10);

		  		  targetDist = 3; // 11.8;
		  		  MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_TURN);
		  		  osDelay(10);
		  		  break;
		}
		btnClicked = 0;
		prevTask = curTask;
		curTask = TASK_NONE;

		if(__IS_EMPTY(cmdq))
		{
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, msg);
		}
		else
			__READ_CMD(cmdq, curCmd, msg);

	  }
//    osDelay(1);
  }
  /* USER CODE END moveBR */
}

/* USER CODE BEGIN Header_moveDistObsIR */
/**
* @brief Function implementing the MoveDistObsIR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveDistObsIR */
void moveDistObsIR(void *argument)
{
  /* USER CODE BEGIN moveDistObsIR */
  uint16_t dataPoint = 0; uint32_t IR_data_raw_acc = 0;
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_ADC) osDelay(1000);
	  else
	  {
		  //			dataPoint = 0; IR_data_raw_acc = 0; obsDist_IR = 1000;
		  //			last_curTask_tick = HAL_GetTick();
		  __PEND_CURCMD(curCmd);
		  targetDist = 40;
		  //MoveDistObstacleIR(&targetDist,0);
		  //			do {
		  //				__ADC_Read_Dist(&hadc1, dataPoint, IR_data_raw_acc, obsDist_IR, obsTick_IR);
		  //			  osDelay(5);
		  //			} while (1);
		  //
		  //		  __ON_TASK_END(&htim8, prevTask, curTask);
		  //		  HAL_ADC_Stop(&hadc1);
	//		  btnClicked = 0;
		  prevTask = curTask;
		  curTask = TASK_NONE;
		  if (__IS_EMPTY(cmdq)) {
			  __CLEAR_CURCMD(curCmd);
			  __ACK_TASK_DONE(&huart3, msg);

		  } else __READ_CMD(cmdq, curCmd, msg);
	  }

//    osDelay(1);
  }
  /* USER CODE END moveDistObsIR */
}

/* USER CODE BEGIN Header_moveDistObsUS */
/**
* @brief Function implementing the MoveDistObsUSTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveDistObsUS */
void moveDistObsUS(void *argument)
{
  /* USER CODE BEGIN moveDistObsUS */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_MOVE_OBS) osDelay(1000);
	  else {
		  targetDist = (float) curCmd.val;
		  MoveDistObstacleUS(&targetDist, SPEED_MODE_FAST);

		  __TASK_END(&htim8, prevTask, curTask);

		if (__IS_EMPTY(cmdq)) {
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, msg);
		} else __READ_CMD(cmdq, curCmd, msg);
	  }
    osDelay(1);
  }
  /* USER CODE END moveDistObsUS */
}

/* USER CODE BEGIN Header_fastestCar */
/**
* @brief Function implementing the FastestCarTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fastestCar */
void fastestCar(void *argument)
{
  /* USER CODE BEGIN fastestCar */

  /* Infinite loop */
  for(;;)
  {
	if(curTask != TASK_FASTESTCAR) osDelay(1000);
	else
	{
		//Calculate distance of obstacles?
		//uint8_t turnDir = (uint8_t) curCmd.val;
		//test_obslen = ObsLenUS(turnDir);
		__SET_ENCODER_LAST_TICK(&htim8,encoder_count1);
		//1. Move forward until XXcm away from obstacle. (This distance must allow car to turn without hitting the obstacle)

		switch(curCmd.val){
			case 1: // Move to obs 1
				obj1_dist = USavg();
				// return_dist = return_dist + obj1_dist;
				targetDist = 30;
				MoveDistObstacleUS(&targetDist,SPEED_MODE_FAST);
				osDelay(5);
				// straight till next obstacle
				break;
			case 2: // Clear obs 1 LEFT
				__SET_CMD_CONFIG(cfgs[CONFIG_FL00],&htim8,&htim1,targetAngle);
				Turn(&targetAngle);
				osDelay(5);
				//turn left forward
				//__SET_CMD_CONFIG(cfgs[CONFIG_FR00],&htim8,&htim1,targetAngle);
				//Turn(&targetAngle);
				//osDelay(5);
				//turn right forward
				//__SET_CMD_CONFIG(cfgs[CONFIG_FR00],&htim8,&htim1,targetAngle);
				//Turn(&targetAngle);
				//osDelay(5);
				//turn right forward
				FastestCar_Turn_R180(0);
				osDelay(10);
				__SET_CMD_CONFIG(cfgs[CONFIG_FL00],&htim8,&htim1,targetAngle);
				Turn(&targetAngle);
				osDelay(5);
				//turn left forward
				break;
			case 3: // Clear obs 1 RIGHT
				__SET_CMD_CONFIG(cfgs[CONFIG_FR00],&htim8,&htim1,targetAngle);
				Turn(&targetAngle);
				osDelay(5);
				//turn right forward
				//__SET_CMD_CONFIG(cfgs[CONFIG_FL00],&htim8,&htim1,targetAngle);
				//Turn(&targetAngle);
				//osDelay(5);
				//turn left forward
				//__SET_CMD_CONFIG(cfgs[CONFIG_FL00],&htim8,&htim1,targetAngle);
				//Turn(&targetAngle);
				//osDelay(5);
				//turn left forward
				FastestCar_Turn_L180(0);
				osDelay(10);
				__SET_CMD_CONFIG(cfgs[CONFIG_FR00],&htim8,&htim1,targetAngle);
				Turn(&targetAngle);
				osDelay(5);
				//turn right forward
				break;
			case 4: // Move to obs 2
				obj2_dist = USavg();
				return_dist = 80 + obj2_dist;
				targetDist = 20;
				MoveDistObstacleUS(&targetDist,SPEED_MODE_FAST);
				osDelay(5);
				// straight till next obstacle
				break;
			case 30: // Clear obs 2 LEFT
				__SET_CMD_CONFIG(cfgs[CONFIG_FL00],&htim8,&htim1,targetAngle);
				Turn(&targetAngle);
				osDelay(1000);
				//turn left forward

				// RIGHT IR OVERSHOOT
				/*
				targetDist = 50;
				if (IR_distances[1] < 50){
				MoveDistObstacleIR(&targetDist,1); //forward till IR do not see obstacle
				__SET_ENCODER_LAST_TICK(&htim8,encoder_count2);
				__GET_ENCODER_DELTA(encoder_count1,encoder_count2,encoder_count_negative_delta_x,&htim8);
				osDelay(10);
				}
				*/

				// RIGHT IR OVERSHOOT NEW CODE
				targetDist = 50;
				if (IR_distances[1] < 50){
				RobotMoveUntilIROvershootR();
				osDelay(10);
				}
				
				FastestCar_Turn_R180(0);
				osDelay(10);
				// turn right 180

				// RIGHT IR HIT
				/*
				if (IR_distances[1] > 50){
				RobotMoveUntilIRHit(); //forward till IR see obstacle
				__SET_ENCODER_LAST_TICK(&htim8,encoder_count2);
				__GET_ENCODER_DELTA(encoder_count1,encoder_count2,encoder_count_negative_delta_x,&htim8);
				osDelay(10);
				}
				*/

				// RIGHT IR HIT NEW CODE
				RobotMoveUntilIRHitR();
				osDelay(10);
				break;
			case 31:
				// RIGHT IR OVERSHOOT
				// MoveDistObstacleIR(&targetDist,1); //forward till IR do not see obstacle
				// osDelay(10);

				// RIGHT IR OVERSHOOT NEW CODE
				RobotMoveUntilIROvershootR();
				osDelay(10);

				__SET_CMD_CONFIG(cfgs[CONFIG_FR00],&htim8,&htim1,targetAngle);
				Turn(&targetAngle);
				osDelay(10);
				// turn right
				total_count_y = (int)return_dist;
				break;
			case 32: // Clear obs 2 RIGHT
				__SET_CMD_CONFIG(cfgs[CONFIG_FR00],&htim8,&htim1,targetAngle);
				Turn(&targetAngle);
				osDelay(1000);
				//turn right forward

				// LEFT IR OVERSHOOT
				/*
				targetDist = 50;
				if (IR_distances[0] < 50){
				MoveDistObstacleIR(&targetDist,0); //forward till IR do not see obstacle
				__SET_ENCODER_LAST_TICK(&htim8,encoder_count2);
				__GET_ENCODER_DELTA(encoder_count1,encoder_count2,encoder_count_negative_delta_x,&htim8);
				osDelay(10);
				}
				*/

				// LEFT IR OVERSHOOT NEW CODE
				targetDist = 50;
				if (IR_distances[0] < 50){
				RobotMoveUntilIROvershootL();
				osDelay(10);
				}

				FastestCar_Turn_L180(0);
				osDelay(10);
				// turn left 180

				// LEFT IR HIT
				/*
				if (IR_distances[0] > 50){
				RobotMoveUntilIRHit(); //forward till IR see obstacle
				__SET_ENCODER_LAST_TICK(&htim8,encoder_count2);
				__GET_ENCODER_DELTA(encoder_count1,encoder_count2,encoder_count_negative_delta_x,&htim8);
				osDelay(10);
				}
				*/

				// LEFT IR HIT NEW CODE
				RobotMoveUntilIRHitL();
				osDelay(10);
				break;
			case 33:
				// LEFT IR OVERSHOOT
				// MoveDistObstacleIR(&targetDist,0); //forward till IR do not see obstacle
				// osDelay(10);

				// LEFT IR OVERSHOOT NEW CODE
				RobotMoveUntilIROvershootL();
				osDelay(10);
				
				__SET_CMD_CONFIG(cfgs[CONFIG_FL00],&htim8,&htim1,targetAngle);
				Turn(&targetAngle);
				osDelay(10);
				//turn left forward
				total_count_y = (int)return_dist;
				// clear obs 2 back
				break;
			case 38:
				targetDist = return_dist;
				MoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_FAST);
				osDelay(10);
				break;
			case 40: // Return to carpark LEFT
				__SET_CMD_CONFIG(cfgs[CONFIG_FR00],&htim8,&htim1,targetAngle);
				Turn(&targetAngle);
				osDelay(1000);
				// turn right

				// RIGHT IR HIT
				/*
				RobotMoveUntilIRHit(); //forward till IR see obstacle
				__SET_ENCODER_LAST_TICK(&htim8,encoder_count2);
				__GET_ENCODER_DELTA(encoder_count1,encoder_count2,encoder_count_negative_delta_x,&htim8);
				osDelay(10);
				*/

				// RIGHT IR HIT NEW CODE
				RobotMoveUntilIRHitR();
				osDelay(1000);

				targetDist = 15;
				MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_FAST);
				osDelay(10);

				__SET_CMD_CONFIG(cfgs[CONFIG_FL00],&htim8,&htim1,targetAngle);
				Turn(&targetAngle);
				osDelay(10);
				// turn left
				break;
			case 41: // Return to carpark RIGHT
				__SET_CMD_CONFIG(cfgs[CONFIG_FL00],&htim8,&htim1,targetAngle);
				Turn(&targetAngle);
				osDelay(1000);
				// turn left

				// LEFT IR HIT
				/*
				RobotMoveUntilIRHit(); //forward till IR see obstacle
				__SET_ENCODER_LAST_TICK(&htim8,encoder_count2);
				__GET_ENCODER_DELTA(encoder_count1,encoder_count2,encoder_count_negative_delta_x,&htim8);
				osDelay(10);
				*/

				// LEFT IR HIT NEW CODE
				RobotMoveUntilIRHitL();
				osDelay(10);

				targetDist = 15;
				MoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_FAST);
				osDelay(10);

				__SET_CMD_CONFIG(cfgs[CONFIG_FR00],&htim8,&htim1,targetAngle);
				Turn(&targetAngle);
				osDelay(10);
				// turn right
				break;
			case 60: // L180
				FastestCar_Turn_L180(0);
				osDelay(10);
				// turn left 180
				break;
			case 61: // R180
				FastestCar_Turn_R180(0);
				osDelay(10);
				// turn right 180
				break;
			}


	//		FastestCar_Turn_R180(0);
			//5. Turn R/L 180 (Turning size to be determined) [Can use US/IR overshoot]
			//6. Turn L/R 90 to align with second obstacle (10cmx60cm) (How to make alignment more accurate?)
			//7. DTXX to move back/forward to XXcm away from the obstacle
			//8. Read arrow
			//9. Wait for L/R instruction
			//10. Turn L/R 180 (Turning size to be determined)
			//11. DTXX to base until XXcm away from size of base
			//12. Turn R/L 90
			//13. Turn L/R 90
			//14. Move into base DTXX

			total_count_y += encoder_count_positive_delta_y;
			total_count_y -= encoder_count_negative_delta_y;
			total_count_x += encoder_count_positive_delta_x;
			total_count_x -= encoder_count_negative_delta_x;
			encoder_count1 = (uint32_t) __HAL_TIM_GET_COUNTER(&htim2);
			encoder_count2 = (uint32_t) __HAL_TIM_GET_COUNTER(&htim3);
			encoder_count_positive_delta_x = 0;
			encoder_count_positive_delta_y = 0;
			encoder_count_negative_delta_x = 0;
			encoder_count_negative_delta_y = 0;
			osDelay(50);

			  __TASK_END(&htim8, prevTask, curTask);
				btnClicked = 0;
				prevTask = curTask;
				curTask = TASK_NONE;
			if (__IS_EMPTY(cmdq)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, msg);
			} else __READ_CMD(cmdq, curCmd, msg);

	}
    osDelay(1);
  }
  /* USER CODE END fastestCar */
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

#ifdef  USE_FULL_ASSERT
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
