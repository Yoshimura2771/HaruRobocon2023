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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CANID_FSTCYL 0x114
#define DELAYMS 500
#define CANID_CYL_BLARM_1 0x116
#define CANID_CYL_BLARM_2 0x117
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
GPIO_TypeDef *GPIOs[8] = {CYL1A_GPIO_Port, CYL1B_GPIO_Port, CYL2A_GPIO_Port, CYL2B_GPIO_Port, CYL3A_GPIO_Port, CYL3B_GPIO_Port, CYL4A_GPIO_Port, CYL4B_GPIO_Port};
uint16_t GPIOPins[8] = {CYL1A_Pin, CYL1B_Pin, CYL2A_Pin, CYL2B_Pin, CYL3A_Pin, CYL3B_Pin, CYL4A_Pin, CYL4B_Pin};
uint8_t CylFlag[4] = {};
uint8_t TxData[1];
uint8_t Flag = 0;
FDCAN_TxHeaderTypeDef TxHeader;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
	return len;
}

void CylN(uint8_t CylID){
	printf("N\r\n");
	HAL_GPIO_WritePin(GPIOs[CylID*2], GPIOPins[CylID*2], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOs[CylID*2+1], GPIOPins[CylID*2+1], GPIO_PIN_RESET);
}

void CylPush(uint8_t CylID){
	printf("Push\r\n");
	HAL_GPIO_WritePin(GPIOs[CylID*2], GPIOPins[CylID*2], GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOs[CylID*2+1], GPIOPins[CylID*2+1], GPIO_PIN_RESET);
}

void CylPull(uint8_t CylID){
	printf("Move B\r\n");
	HAL_GPIO_WritePin(GPIOs[CylID*2], GPIOPins[CylID*2], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOs[CylID*2+1], GPIOPins[CylID*2+1], GPIO_PIN_SET);
}

void CylThrow(uint8_t CylID){
	printf("Throw\r\n");
	CylPush(CylID);
	HAL_Delay(DELAYMS);
	CylPull(CylID);
	HAL_Delay(DELAYMS);
	CylN(CylID);
	CylFlag[CylID]=0;
}

void CylRThrow(uint8_t CylID){
	printf("Throw\r\n");
	CylPull(CylID);
	HAL_Delay(DELAYMS);
	CylPush(CylID);
	HAL_Delay(DELAYMS);
	CylN(CylID);
	CylFlag[CylID]=0;
}

void CylStop(uint8_t CylID){
	HAL_GPIO_WritePin(GPIOs[CylID*2], GPIOPins[CylID*2], GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOs[CylID*2+1], GPIOPins[CylID*2+1], GPIO_PIN_SET);
}

void CylDrive2(uint8_t CylID){
	printf("set\r\n");
	Flag = 1;
	CylPush(CylID);
	HAL_Delay(DELAYMS);
	CylPush(CylID+1);
	//HAL_Delay(DELAYMS/5);
	//CylN(CylID+1);
	HAL_Delay(DELAYMS);
	CylPull(CylID+1);
	HAL_Delay(DELAYMS);
	CylPull(CylID);
	HAL_Delay(DELAYMS);
	CylFlag[CylID] =0;



}

void CylDrive1(uint8_t CylID){
	printf("exception2\r\n");
	if(Flag == 1){
		CylPush(0);
		HAL_Delay(DELAYMS);
		CylPush(1);
		HAL_Delay(DELAYMS);
		Flag=0;
	}

	CylPull(CylID);
	CylFlag[CylID] = 2;
}

void CylDrive3(uint8_t CylID){
	printf("exception2\r\n");
	/*CylPush(CylID);
	HAL_Delay(200);
	TxHeader.Identifier == CANID_CYL_BLARM_2;
	TxData[0] = 2;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {
		Error_Handler();
	}*/
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for(uint8_t i=0; i<4; i++){

		  switch(CylFlag[i]){

		  		case 0:
		  			CylN(i);
		  			break;

		  		case 1:
		  			CylPush(i);
		  			break;

		  		case 2:
		  			CylPull(i);
		  			break;

		  		case 3:
		  			CylThrow(i);
		  			break;

		  		case 4:
		  			CylRThrow(i);
		  			break;
		  		case 5:
		  			break;
		  		case 6:
		  			CylDrive1(i);
		  			break;
		  		case 7:
		  			CylDrive2(i);
		  			break;
		  		case 8:
		  			CylDrive3(i);
		  			break;
		  		}

	  }

	  HAL_Delay(15);
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
  	sFilterConfig.FilterID1 = CANID_FSTCYL;
  	sFilterConfig.FilterID2 = 0x7FC;

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

  /* USER CODE END FDCAN1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, CYL1A_Pin|CYL1B_Pin|CYL2A_Pin|CYL2B_Pin
                          |CYL3A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CYL3B_Pin|CYL4A_Pin|CYL4B_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CYL1A_Pin CYL1B_Pin CYL2A_Pin CYL2B_Pin
                           CYL3A_Pin */
  GPIO_InitStruct.Pin = CYL1A_Pin|CYL1B_Pin|CYL2A_Pin|CYL2B_Pin
                          |CYL3A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CYL3B_Pin CYL4A_Pin CYL4B_Pin LD2_Pin */
  GPIO_InitStruct.Pin = CYL3B_Pin|CYL4A_Pin|CYL4B_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[1];
	uint8_t CylID;

	if (hfdcan == &hfdcan1) {
		printf("Get message\r\n");
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
			Error_Handler();
		}
		CylID = RxHeader.Identifier - CANID_FSTCYL;
		CylFlag[CylID] = RxData[0];

		//HAL_GPIO_WritePin(GPIOs[CylID*2], GPIOPins[CylID*2], RxData[0]);

		//printf("Motor%x:%d\r\n", RxHeader.Identifier- 0x201, actMotorVel[MOTOR1]); //index is motorID-201 (0x201 - 0x201 =0)
		//Motor%x:RxHeader.Identifier- 0x201,
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
