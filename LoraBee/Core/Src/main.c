/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora_sx1276.h"
#include <strings.h>
#include <stdio.h>
#include "retarget.h"
#include "lib_mrf24j.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const char DEVICE_ID[] = "A";
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
	uint8_t point = 0;

	const uint8_t STATE = 1;	// 0 transmitter, 1 is receiver
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	RetargetInit(&huart1);
	//initialize LoRa module
	lora_sx1276 lora;

	uint8_t res = lora_init(&lora, &hspi1, LORA_NSS_GPIO_Port, LORA_NSS_Pin, LORA_BASE_FREQUENCY_EU);
	uint8_t ver = lora_version(&lora);
	HAL_Delay(100);
	printf("%d: Starting Lora up...", point++);
	if (res != LORA_OK) {
		// Initialization failed
		printf("failed\n");
	} else {
		printf("\nLora version: %d\n", ver);
	}

	// Initialize MRF module
	mrf_reset();
	mrf_deselect();

	printf("%d: MRF starting up...", point++);
	mrf_init();
	mrf_set_ignorebytes(2);
	mrf_pan_write(0xFFFF);


	if (STATE){
		mrf_promiscuous(1);
		mrf_address16_write(0xFFFF);
		printf("Set as receiver.\n");
	}
	else{
		mrf_address16_write(0x0001);
		printf("Set as transmitter.\n");
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	HAL_Delay(50);

	if (STATE == 1){
		receive_lora(lora);
		receive_mrf();
	}
	else{
		transmit_lora(lora);
		transmit_mrf();
	}


	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	_delay_ms(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MRF_NSS_Pin|LED_Pin|MRF_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LORA_NSS_Pin|LORA_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : MRF_INT_Pin */
  GPIO_InitStruct.Pin = MRF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MRF_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MRF_NSS_Pin LED_Pin */
  GPIO_InitStruct.Pin = MRF_NSS_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_RESET_Pin */
  GPIO_InitStruct.Pin = LORA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MRF_RESET_Pin */
  GPIO_InitStruct.Pin = MRF_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MRF_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_INT_Pin */
  GPIO_InitStruct.Pin = LORA_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LORA_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void mrf_reset(void) {
    HAL_GPIO_WritePin(MRF_RESET_GPIO_Port, MRF_RESET_Pin, 0);
    HAL_Delay(10);
    HAL_GPIO_WritePin(MRF_RESET_GPIO_Port, MRF_RESET_Pin, 1);
    HAL_Delay(20);
}

void handle_rx(mrf_rx_info_t *rxinfo, uint8_t *rx_buffer) {
    printf("MRF RSSI=%d\n", rxinfo->rssi);
}

void handle_tx(mrf_tx_info_t *txinfo) {
    if (txinfo->tx_ok) {
        printf("MRF message sent!\n");
    } else {
        printf("MRF transmission failed after %d retries\n", txinfo->retries);
    }
}

void mrf_select(void){
	HAL_GPIO_WritePin(MRF_NSS_GPIO_Port, MRF_NSS_Pin, 0);
	_delay_ms(100);
}

void mrf_deselect(void){
	HAL_GPIO_WritePin(MRF_NSS_GPIO_Port, MRF_NSS_Pin, 1);
}

uint8_t spi_tx(uint8_t cData){
	uint8_t buff;
	HAL_SPI_TransmitReceive(&hspi1, &cData, &buff, sizeof(buff), 100);
	return buff;
}

void _delay_ms(int t){
  HAL_Delay(t);
}

void transmit_mrf(){
	printf("Sending MRF...");
	mrf_send16(0xFFFF, 8,&DEVICE_ID);
	HAL_Delay(500);
	mrf_check_flags(&handle_rx, &handle_tx);
	if (HAL_GPIO_ReadPin(MRF_INT_GPIO_Port, MRF_INT_Pin) == 0){
		mrf_interrupt_handler();
	}
}

void transmit_lora(lora_sx1276 lora){
	printf("Sending LORA...");

	uint8_t res = lora_send_packet(&lora, (uint8_t *)DEVICE_ID, 1);
	HAL_Delay(300);
	if (res != LORA_OK) {
		// Send failed
		printf("Send failed\n");
	}
	else {
		printf("Packet sent!\n");
	}
}

void receive_mrf(){
	printf("MRF receiving....");
	mrf_check_flags(&handle_rx, &handle_tx);
	if (HAL_GPIO_ReadPin(MRF_INT_GPIO_Port, MRF_INT_Pin) == 0){
		printf("\n");
		mrf_interrupt_handler();
	} else {
		printf(" Nothing to receive!\n");
	}

}

void receive_lora(lora_sx1276 lora){
	printf("Lora receiving....");
	uint8_t buffer[32];

	lora_mode_receive_continious(&lora);

	uint8_t res;
	uint8_t len = lora_receive_packet_blocking(&lora, buffer, sizeof(buffer), 3000, &res);
	uint8_t rssi = lora_packet_rssi(&lora);
	if (res != LORA_OK) {
		printf(" No message received!\n");
	} else {
		buffer[len] = 0;  // null terminate string to print it
		printf("success!\nData: %s\n", buffer);
		printf("LORA RSSI = %d\n", rssi);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
