/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR0 0
#define MOTOR1 1
#define MOTOR2 2
#define CANID_MOTORS 0x200
#define CANID_MOTOR0FB 0x201
#define CANID_MOTOR1FB 0x202
#define CANID_MOTOR2FB 0x203
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;
static int16_t RobotVel[3] = {}; //X,Y,Om
static int16_t rcvRobotVel[3] = {};
static int16_t actMotorVel[3] = {};
//static int16_t prvMotorVel[3] = {};
static int16_t trgMotorVel[3] = {0,0,0};
static int32_t error[3]={};
static int32_t prverror[3]={};
static int32_t Output[3] = {};
static int32_t integral[3];
uint8_t Flag = 0;
uint8_t FlagCount = 0;
uint16_t frq = 1000; //hz
int16_t spd=-3000;
float Period = 0.001;
float Kp[3] = {9,9,9};
float Ki[3] = {0.001,0.001,0.001};
float Kd[3] = {0.03,0.03,0.03};
uint8_t gain = 128;
int8_t acc = 6;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void CAN_Motordrive(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
	return len;
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
  MX_FDCAN1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = CANID_MOTORS;
	sFilterConfig.FilterID2 = 0x7F0;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
	  Error_Handler();
	}

	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
	TxHeader.Identifier = CANID_MOTORS;

	//printf("CAN initialized\n");

  /* USER CODE END FDCAN1_Init 2 */

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
  htim3.Init.Prescaler = 799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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
  huart2.Init.BaudRate = 115200*4;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  setbuf(stdout, NULL);

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	//printf("Get message\n");
	if (hfdcan == &hfdcan1) {
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
			Error_Handler();
		}
		if(RxHeader.Identifier==0x20F){
			printf("Get mes\r\n");
			for(int i=0; i<3; i++){
				rcvRobotVel[i] = (RxData[i]-127	)*gain;

			}
			printf("X: %9d, Y: %9d, Om: %9d\r\n", rcvRobotVel[0],  rcvRobotVel[1],  rcvRobotVel[2]);
		}
		else{
			if(Flag==0)Flag=1;

			actMotorVel[RxHeader.Identifier - 0x201] = RxData[2]<<8 | RxData[3];

			//printf("Motor%x:%d\r\n", RxHeader.Identifier- 0x201, actMotorVel[RxHeader.Identifier- 0x201]); //index is motorID-201 (0x201 - 0x201 =0)
			//Motor%x:RxHeader.Identifier- 0x201,
		}
	}
}

void CAN_Motordrive(void)
{

	uint8_t i;
	uint8_t TxData[8];

	for(i=0; i<3; i++){
		if(Output[i]<-16384)Output[i]=-16384;
		else if(Output[i]>16384)Output[i]=16384;
		TxData[i*2]=Output[i]>>8;//‰∏ä‰Ωç„Éì?????????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?
		TxData[i*2+1]=Output[i]&0x00FF;
	}



	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {
		/* Transmission request Error */
		Error_Handler();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){



	if(htim==&htim3){
		uint8_t i;

		for(i=0; i<3; i++){
			if(RobotVel[i]<rcvRobotVel[i])RobotVel[i]+=acc;
			else if(RobotVel[i]>rcvRobotVel[i])RobotVel[i]-=acc;
		}

		trgMotorVel[0] = (RobotVel[2] + 2*RobotVel[0])/3;
		trgMotorVel[1] = (RobotVel[2] - RobotVel[0] +sqrt(3)*RobotVel[1])/3;
		trgMotorVel[2] = (RobotVel[2] - RobotVel[0] -sqrt(3)*RobotVel[1])/3;


		for(i=0; i<3; i++){

			//trgMotorVel[i]=500;
			error[i] = actMotorVel[i] - trgMotorVel[i];
			integral[i] += (error[i] + prverror[i])/(2*frq);
			integral[i] *= integral[i]*Flag;
			if(integral[i]>10)integral[i]= 10;
			else if(integral[i]<-10) integral[i]=-10;
			Output[i]=/*trgMotorVel[i]*/ - (error[i]*Kp[i] + (error[i] - prverror[i])*Kd[i]*frq+integral[i]*Ki[i]);
			prverror[i]=error[i];
			//Flag=0;
			//Output[i]=trgMotorVel[i];
			//printf("Motor%d:target%d\r\n", i, trgMotorVel[i]);
			//printf("Robot%d:%d\r\n",i ,RobotVel[i]);


		}


		/*if(Flag==0)FlagCount++;
		if(FlagCount>10){
			for(i=0; i<3; i++){
				integral[i]=0;
				//Count =0;
				actMotorVel[i] = 0;
				error[i] = 0;
				trgMotorVel[i] = 0;

			}
		}*/
		//trgMotorVel[0] = 1000; trgMotorVel[1] = 1000; trgMotorVel[2] = 1000;

		CAN_Motordrive();
	}


	//printf("Output: %d\r\n", Output[1]);
	//printf("Error: %d\r\n", error[1]);


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
