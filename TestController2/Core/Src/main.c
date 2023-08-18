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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CANID_CYL_THROW 0x110


#define CANID_CYL_SET 0x114
#define CANID_CYL_TABLE 0x115
#define CANID_CYL_BLARM_1 0x116
#define CANID_CYL_BLARM_2 0x117


#define CANID_CYL_PICK 0x118
#define CANID_CYL_RELEASE 0x119
#define CANID_LED 0x11A
#define CANID_BAL_SPL_1 0x11B
#define CANID_BAL_SPL_2 0x11C
#define CANID_BAL_DSC_1 0x11D
#define CANID_BAL_DSC_2 0x11E



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
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


/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_15){
		printf("a\r\n");
	}
	else if(GPIO_Pin == GPIO_PIN_10){
		printf("b\r\n");
	}
	else if(GPIO_Pin == GPIO_PIN_9){
		printf("c\r\n");
	}
	else if(GPIO_Pin == GPIO_PIN_7){
		printf("d\r\n");
	}
	else if(GPIO_Pin == GPIO_PIN_6){
		printf("e\r\n");
	}
	else if(GPIO_Pin == GPIO_PIN_5){
		printf("f\r\n");
	}
	else if(GPIO_Pin == GPIO_PIN_4){
		printf("g\r\n");
	}
	else if(GPIO_Pin == GPIO_PIN_1){
		printf("h\r\n");
	}
	else if(GPIO_Pin == GPIO_PIN_0){
		printf("i\r\n");
	}
	else if(GPIO_Pin == GPIO_PIN_3){
		printf("j\r\n");
	}
}*/
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
  uint8_t TxData[1];
  uint8_t _TxData = 0;

  printf("Initialized\r\n");
  uint8_t Flag1 = 1;
  uint8_t Flag2 = 1;
  uint8_t FlagL = 0;
  uint8_t FlagR = 1;
  GPIO_PinState _inputs[11]={};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  GPIO_PinState inputs[11]={
			  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0),
			  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1),
			  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4),
			  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5),
			  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6),
			  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7),
			  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8),
			  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9),
			  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10),
			  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0),
			  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)

	  };

	  for(uint8_t i=0; i<11; i++){
		  if(inputs[i] != _inputs[i]){
			  switch(i){
			  case 0:
				  if(inputs[i]==GPIO_PIN_RESET){
					  TxHeader.Identifier = CANID_CYL_SET;
					  TxData[0] = 7;
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }
				  //printf("Set\r\n");

				  break;


			  case 1:
				  if(inputs[i]==GPIO_PIN_RESET){
					  TxHeader.Identifier = CANID_BAL_SPL_1;
					  TxData[0] = 7;
					  //printf("BAL1 supply\r\n");
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }
				  else if(inputs[i] == GPIO_PIN_SET) {
					  TxHeader.Identifier = CANID_BAL_SPL_1;
					  TxData[0] = 6;
					  //printf("BAL1 Stop\r\n");
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }

				  break;


			  case 2:
				  if(inputs[i]==GPIO_PIN_RESET){
					  TxHeader.Identifier = CANID_BAL_SPL_2;
					  TxData[0] = 7;
					  //printf("BAL2 Supply\r\n");
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }
				  else if(inputs[i] == GPIO_PIN_SET) {
					  TxHeader.Identifier = CANID_BAL_SPL_2;
					  //printf("BAL2 Stop\r\n");
					  TxData[0] = 6;
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }
				  break;


			  case 3:
				  if(inputs[i]==GPIO_PIN_RESET){
					  TxHeader.Identifier = CANID_CYL_PICK;
					  TxData[0] = 3;

					  //printf("Pick\r\n");
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }
				  break;


			  case 4:
				  if(inputs[i]==GPIO_PIN_RESET){
					  TxHeader.Identifier = CANID_LED;

					  if(FlagL==0)FlagL=1;
					  else if(FlagL==1)FlagL=0;

					  TxData[0] = FlagL;
					  //printf("LED\r\n");
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }
				  break;


			  case 5:
				  if(inputs[i]==GPIO_PIN_RESET){
					  TxHeader.Identifier = CANID_CYL_RELEASE;

					  if(FlagR==1)FlagR=2;
					  else if(FlagR==2)FlagR=1;

					  TxData[0] = FlagR;
					  //printf("Release%d\r\n", FlagR);
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }
				  break;


			  case 6:
				  if(inputs[i]==GPIO_PIN_RESET){
					  TxHeader.Identifier = CANID_BAL_DSC_2;
					  TxData[0] = 7;

					  //printf("BAL2 Discharge\r\n");
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }
				  else if(inputs[i] == GPIO_PIN_SET) {
					  TxHeader.Identifier = CANID_BAL_DSC_2;
					  TxData[0] = 6;

					  //printf("BAL2 Stop\r\n");
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }
				  break;


			  case 7:
				  if(inputs[i]==GPIO_PIN_RESET){
					  TxHeader.Identifier = CANID_CYL_THROW;
					  TxData[0] = 3;
					  //printf("Throw\r\n");
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }
				  break;


			  case 8:
				  if(inputs[i]==GPIO_PIN_RESET){
					  TxHeader.Identifier = CANID_CYL_BLARM_2;
					  if(Flag2 == 1)Flag2=6;
					  else Flag2=1;

					  TxData[0] = Flag2;

					  //printf("Arm2-%d\r\n",Flag2);
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }

				  }

				  break;


			  case 9:
				  if(inputs[i]==GPIO_PIN_RESET){
					  TxHeader.Identifier = CANID_BAL_DSC_1;
					  TxData[0] = 7;

					  //printf("BAL1 Discharge\r\n");
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }
				  else if(inputs[i] == GPIO_PIN_SET) {
					  //printf("BAL1 Stop\r\n");
					  TxHeader.Identifier = CANID_BAL_DSC_1;
					  TxData[0] = 6;
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }
				  }
				  break;


			  case 10:

				  if(inputs[i]==GPIO_PIN_RESET){
					  TxHeader.Identifier = CANID_CYL_BLARM_1;
					  if(Flag1 == 1)Flag1=6;
					  else Flag1=1;
					  //printf("arm1-%d\r\n",Flag1);

					  TxData[0] = Flag1;
					  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

							Error_Handler();
					  }

				  }
				  break;
			  }
		  }
		  _inputs[i] = inputs[i];
	  }
	  /*
	   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==GPIO_PIN_RESET){
		  TxData[0] = 1;
		  printf("Move A\r\n");
	  }
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==GPIO_PIN_RESET){
		  TxData[0] = 2;
		  printf("Move B\r\n");
	  }
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)==GPIO_PIN_RESET){
		  TxData[0] = 3;
		  printf("Throw\r\n");
	  }
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)==GPIO_PIN_RESET){
		  TxData[0] = 3;
		  printf("C\r\n");
	  }
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)==GPIO_PIN_RESET){
		  printf("D\r\n");
	  }
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==GPIO_PIN_RESET){
		  printf("E\r\n");
	  }
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)==GPIO_PIN_RESET){
	  		  printf("F\r\n");
	  	  }
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)==GPIO_PIN_RESET){
	  		  printf("G\r\n");
	  	  }
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==GPIO_PIN_RESET){
	  		  printf("H\r\n");
	  	  }
	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)==GPIO_PIN_RESET){
	  		  printf("I\r\n");
	  	  }
	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_RESET){
	  		  printf("J\r\n");
	  	  }*/


	  /*if(_TxData != TxData[0]){
		  printf("Send");
		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {

	  			Error_Handler();
	  		}
	  }*/

	  HAL_Delay(100);

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

  TxHeader.IdType = FDCAN_STANDARD_ID;
  	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
  	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  	TxHeader.MessageMarker = 0;
  	TxHeader.Identifier = 0x210;
  	FDCAN_FilterTypeDef sFilterConfig;
  		sFilterConfig.IdType = FDCAN_STANDARD_ID;
  		sFilterConfig.FilterIndex = 0;
  		sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  		sFilterConfig.FilterID1 = 0x000;
  		sFilterConfig.FilterID2 = 0x000;

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

  /*Configure GPIO pins : PA0 PA1 PA4 PA5
                           PA6 PA7 PA8 PA9
                           PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	uint16_t PBattVol, CBattVol, AirPress;
	uint8_t RxData[8];
	if(hfdcan==&hfdcan1){
		//printf("get message!!\r\n");
		uint8_t RxData[8]={};
		FDCAN_RxHeaderTypeDef RxHeader;
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
			Error_Handler();
		}
		if(RxHeader.Identifier == 0x700){
			PBattVol = RxData[0]<<8 | RxData[1];
			CBattVol = RxData[2]<<8 | RxData[3];
			AirPress = RxData[4]<<8 | RxData[5];
			printf("Power Battery Vol: %d\r\n", PBattVol);
			printf("Controller Battery Vol: %d\r\n", CBattVol);
			printf("Air Pressure: %d\r\n", AirPress);

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
