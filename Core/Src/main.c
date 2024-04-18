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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
//State
uint8_t state = 0;

//Time Const
uint64_t currentTime;
uint64_t currentTimeLED;

//ADC Poten Read
uint16_t ADCBuffer[50] = {0};
//uint16_t ADCBuffer2[10] = {0};
int ADC_Average[2] = {0};
int ADC_SumAPot[2] = {0};
//int ADC_Average2 = 0;
//int ADC_SumAPot2 = 0;
int turn;
int32_t diffPosition;

//Positions Motor
float Degrees_Position = 0;
float DutyCycle;

float Degrees_Position2 = 0;
float DutyCycle2;

uint32_t A;
uint16_t B;

float P;
float I;
float D;

float P2;
float I2;
float D2;


//PID Const
arm_pid_instance_f32 PID = {0};
arm_pid_instance_f32 PID2 = {0};

float setposition = 90;
float Vfeedback = 0;

float setposition2 = 90;
float setposition20 = 90;
float Vfeedback2 = 0;

uint32_t QEIReadRaw;
int16_t RPMspeed;

//Waijung Recieve
uint8_t Rx[5];
int PWMDrive; //From MathLab

//Waijung Transmit
uint8_t header = 0x45; // Header byte
uint8_t parityBit = 0; // Parity bit initialized to 0
uint8_t dataBytes[4]; // Array to hold the bytes of uint16_t and parity bit
uint16_t dataSend = 4096;

typedef struct
{
	// for record New / Old value to calculate dx / dt
	uint32_t Position[2];
	uint64_t TimeStamp[2];
	float QEIPostion_1turn;
	float QEIAngularVelocity;
}

QEI_StructureTypeDef;
QEI_StructureTypeDef QEIdata = {0};
uint64_t _micros = 0;

enum
{
	NEW,OLD
};

typedef struct
{
	// for record New / Old value to calculate dx / dt
	uint32_t Position[2];
	uint64_t TimeStamp[2];
	float PotenPostion_1turn;
	float PotenAngularVelocity;
}

Poten_StructureTypeDef;
Poten_StructureTypeDef Potendata = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void ADC_Averaged();
void ADC_Averaged2();
void MotorControl();
void MotorControl2();
void MotorControl3();
void QEIEncoderPosVel_Update();
void PotenEncoderPosVel_Update();
void UARTInterruptConfig();
uint64_t micros();
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
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3); //DMA Out Event 1000 Hz
  HAL_TIM_Base_Start(&htim4); //QEI Read
  HAL_TIM_Base_Start(&htim5); //Microsencond Timer
  HAL_TIM_Base_Start(&htim8); //PWM Generation

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); //L298N
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); //DRV8833 AI1
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); //DRV8833 AI2
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, ADCBuffer, 50);
//  HAL_ADC_Start_DMA(&hadc3, ADCBuffer2, 10);
//  HAL_ADC_Start_IT(&hadc2);

  // PID Parameter for L298N Motor
//  PID.Kp = 0.06;
//  PID.Ki = 0.00005;
//  PID.Kd = 0.03;
	PID.Kp = 0.2;
	PID.Ki = 0.0005;
	PID.Kd = 0.3;
  arm_pid_init_f32(&PID, 0);

  // PID Parameter for DRV8833 Motor
//  PID2.Kp = 0.18;
//  PID2.Ki = 0.00000;;
//  PID2.Kd = 0.3;
//  PID2.Kp = 0.1;
//  PID2.Ki = 0.00000;;
//  PID2.Kd = 0.05;

  PID2.Kp = 0.2;
  PID2.Ki = 0.0000001;
  PID2.Kd = 0.3;
  arm_pid_init_f32(&PID2, 0);

  UARTInterruptConfig();
  _micros = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
//	  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, fabs(A));
	  if (state == 0) //L298N Driver Control
	  {
		  static uint32_t timestamp = 0;
		  if(timestamp < HAL_GetTick())
		  {
			  timestamp = HAL_GetTick() + 1; //1000 Hz
//			  PotenEncoderPosVel_Update();
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
			  MotorControl(); //L298N
//			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
//			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, fabs(A));

		  }
	  }

	  else if (state == 1) //DRV8833 Driver Control
	  {
		  static uint32_t timestamp = 0;
		  if(timestamp < HAL_GetTick())
		  {
			  timestamp = HAL_GetTick() + 1;//1000 Hz

			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
			  MotorControl2(); //DRV8833
		  }
	  }

	  else if (state == 2) //PWM Waijung Control L298N
	  {
		  static uint32_t timestamp2 = 0; //Waijung Time Control
		  static uint32_t timestampLED = 0; //LED Time Control
		  static uint32_t timestampMOR = 0; //Motor Time Control

		  currentTime = micros(); //Time Counter for Waijung
		  currentTimeLED = micros(); //Time Counter for LED

		  //Map Rx Data to PWM
		  PWMDrive = (int16_t)(Rx[2]<< 8)+Rx[1];

		  if(currentTime > timestamp2)
		  {
			  timestamp2 = currentTime + 5000; //us 200 Hz

			  if(timestamp2 > 4294967296) timestamp2 = 0; //Counter Reset Overflow

			  dataSend = fabs(Degrees_Position);

			  dataBytes[0] = header; // Header byte
			  dataBytes[1] = (uint8_t)(dataSend & 0xFF); // Lower byte
			  dataBytes[2] = (uint8_t)((dataSend >> 8) & 0xFF); // Upper byte
			  dataBytes[3] = 0x0A;

			  HAL_UART_Transmit(&hlpuart1, dataBytes, sizeof(dataBytes), 10);
		  }

		  if(timestampMOR < HAL_GetTick()) //Motor Control from Waijung PWM
		  {
			  timestampMOR = HAL_GetTick() + 1; //1000Hz
			  MotorControl3();
		  }
		  if(currentTimeLED > timestampLED) //LED Constance Frequency 0.5 s
		  {
			  timestampLED = currentTime + 250000; //us
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		  }

	  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 84999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3071;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 169;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  htim8.Init.Prescaler = 16;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 4999;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA11 PA12 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ADC_Averaged()
{
	for (int i = 0; i < 25; i++)
	{
		ADC_SumAPot[0] += ADCBuffer[2*i];
		ADC_SumAPot[1] += ADCBuffer[1+(2*i)];
	}

	for (int i = 0; i < 2; i++)
	{
		ADC_Average[i] = ADC_SumAPot[i] / 25;
		ADC_SumAPot[i] = 0;
	}

	Degrees_Position = (ADC_Average[0] * 360.00) / 4095.00;
	setposition = (ADC_Average[1] * 360.00) / 4095.00;
	setposition2 = (ADC_Average[1] * 360.00) / 4095.00;
}

//void ADC_Averaged2()
//{
//	for (int i = 0; i < 10; i++)
//	{
//		ADC_SumAPot2 += ADCBuffer2[i];
//	}
//
//	ADC_Average2 = ADC_SumAPot2 / 10;
//	ADC_SumAPot2 = 0;
//	setposition = (ADC_Average2 * 360.00) / 4095.00;
//	setposition2 = (ADC_Average2 * 360.00) / 4095.00;
//}

void MotorControl()
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);

	ADC_Averaged();

	Vfeedback = arm_pid_f32(&PID, setposition - Degrees_Position);

	if (Vfeedback >= 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
		DutyCycle = ((Vfeedback * 83999.00) / 20.00) + 100;
		if (DutyCycle > 84999) DutyCycle = 84999;
		else if (DutyCycle < 17000) DutyCycle = 0;
		else if (DutyCycle < 18000) DutyCycle = 18000;
//		DutyCycle = ((Vfeedback * 4899.00) / 7.00) + 100;
//
//		if (DutyCycle > 4999) DutyCycle = 4999;
//		else if (DutyCycle < 1800) DutyCycle = 0;
////		else if (DutyCycle < 1700) DutyCycle = 1800;
//		else if (DutyCycle < 2300) DutyCycle = 2600;

//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, fabs(DutyCycle));
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, fabs(DutyCycle));
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
		DutyCycle = ((Vfeedback * 83999.00) / 20.00) - 100;
		if (DutyCycle < -84999) DutyCycle = -84999;
		else if (DutyCycle > -17000) DutyCycle = 0;
		else if (DutyCycle > -18000) DutyCycle = -18000;
//		DutyCycle = ((Vfeedback * 4899.00) / 7.00) - 100;
//
//		if (DutyCycle < -4999) DutyCycle = -4999;
//		else if (DutyCycle > -1800) DutyCycle = 0;
////		else if (DutyCycle > -1700) DutyCycle = -1800;
//		else if (DutyCycle > -2300) DutyCycle = -2600;

//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, fabs(DutyCycle));
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, fabs(DutyCycle));
	}
}

void MotorControl2()
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

	ADC_Averaged();

	QEIReadRaw = __HAL_TIM_GET_COUNTER(&htim4);
	Degrees_Position2 = (QEIReadRaw * 360.00) / 3072.00;

	Vfeedback2 = arm_pid_f32(&PID2, setposition2 - Degrees_Position2);

	if (Vfeedback2 > 1500) Vfeedback2 = 1500;
	else if (Vfeedback2 < -1500) Vfeedback2 = -1500;


	if (Vfeedback2 >= 0)
	{
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
		DutyCycle2 = ((Vfeedback2 * 4799.00) / 20.00) + 200;
		if (DutyCycle2 > 4999) DutyCycle2 = 4999;

		if (fabs(setposition2 - Degrees_Position2) <= 1.5) DutyCycle2 = 0;

		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, fabs(DutyCycle2));
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);

		DutyCycle2 = ((Vfeedback2 * 4799.00) / 20.00) - 200;
		if (DutyCycle2 < -4999) DutyCycle2 = -4999;

		if (fabs(setposition2 - Degrees_Position2) <= 1.5) DutyCycle2 = 0;

		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, fabs(DutyCycle2));
	}
}

void MotorControl3()
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);

	ADC_Averaged();

	if (PWMDrive >= 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
//		DutyCycle = ((PWMDrive * 84999.00) / 2500.00) + 100;
//		if (DutyCycle > 84999) DutyCycle = 84999;
//		else if (DutyCycle < 17000) DutyCycle = 0;
//		else if (DutyCycle < 18000) DutyCycle = 18000;
		DutyCycle = ((PWMDrive * 83999.00) / 20.00) + 100;
		if (DutyCycle > 84999) DutyCycle = 84999;
		else if (DutyCycle < 17000) DutyCycle = 0;
		else if (DutyCycle < 18000) DutyCycle = 18000;
		//
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, fabs(DutyCycle));
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
		DutyCycle = ((PWMDrive * 83999.00) / 20.00) - 100;
		if (DutyCycle < -84999) DutyCycle = -84999;
		else if (DutyCycle > -17000) DutyCycle = 0;
		else if (DutyCycle > -18000) DutyCycle = -18000;
//		DutyCycle = ((PWMDrive * 84999.00) / 2500.00) - 100;
//		if (DutyCycle < -84999) DutyCycle = -84999;
//		else if (DutyCycle > -17000) DutyCycle = 0;
//		else if (DutyCycle > -18000) DutyCycle = -18000;
//		DutyCycle = ((PWMDrive * 4899.00) / 2500.00) - 100;
//		if (PWMDrive < -4999) PWMDrive = -4999;
//		else if (PWMDrive > -1250) PWMDrive = 0;
//		else if (PWMDrive > -1800) PWMDrive = -1900;
//		if (DutyCycle < -4999) DutyCycle = -4999;
//		else if (DutyCycle > -1800) DutyCycle = 0;
//		else if (DutyCycle > -2300) DutyCycle = -2300;
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, fabs(DutyCycle));
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, fabs(DutyCycle));
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim5)
	{
		_micros += UINT32_MAX;
	}
}

uint64_t micros()
{
	return __HAL_TIM_GET_COUNTER(&htim5) + _micros;
}
void PotenEncoderPosVel_Update()
{
	Potendata.Position[NEW] = Degrees_Position;
	Potendata.PotenPostion_1turn = Potendata.Position[NEW] % 360;
	diffPosition = Potendata.Position[NEW] - Potendata.Position[OLD];
	if(diffPosition > 180)
	turn++;
	if(diffPosition < -180)
	turn--;

	Potendata.Position[OLD] = Potendata.Position[NEW];

}
void QEIEncoderPosVel_Update()
{
	//collect data
	QEIdata.TimeStamp[NEW]= micros();
	QEIdata.Position[NEW] = __HAL_TIM_GET_COUNTER(&htim4);

	//Postion 1 turn calculation
	QEIdata.QEIPostion_1turn = QEIdata.Position[NEW] % 3072;

	//calculate dx
	int32_t diffPosition = QEIdata.Position[NEW] - QEIdata.Position[OLD];

	//Handle Warp around
	if(diffPosition > 28672)
		diffPosition -= 57344;
	if(diffPosition < -28672)
		diffPosition += 57344;

	//calculate dt
	float diffTime = (QEIdata.TimeStamp[NEW]-QEIdata.TimeStamp[OLD]) * 0.000001;

	//calculate anglar velocity
	QEIdata.QEIAngularVelocity = diffPosition / diffTime;
	RPMspeed = (QEIdata.QEIAngularVelocity * 180.00) / (2.00*PI);

	//store value for next loop
	QEIdata.Position[OLD] = QEIdata.Position[NEW];
	QEIdata.TimeStamp[OLD]= QEIdata.TimeStamp[NEW];
}

void UARTInterruptConfig()
{
	HAL_UART_Receive_IT(&hlpuart1, Rx,4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart == &hlpuart1)
	{
		Rx[4] = '\0';
		HAL_UART_Receive_IT(&hlpuart1, Rx, 4);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
		switch (state)
		{
		case 0:
			state = 1;
			break;
		case 1:
			state = 2;
			break;
		case 2:
			state = 0;
			break;
		}
	}
}
/* USER CODE END 4 */

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
