/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib_mrf24j.h"
#include "lora_sx1276.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MRF_INT_Pin GPIO_PIN_1
#define MRF_INT_GPIO_Port GPIOF
#define MRF_NSS_Pin GPIO_PIN_0
#define MRF_NSS_GPIO_Port GPIOB
#define LORA_NSS_Pin GPIO_PIN_8
#define LORA_NSS_GPIO_Port GPIOA
#define LORA_RESET_Pin GPIO_PIN_11
#define LORA_RESET_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB
#define MRF_RESET_Pin GPIO_PIN_4
#define MRF_RESET_GPIO_Port GPIOB
#define LORA_INT_Pin GPIO_PIN_5
#define LORA_INT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void handle_rx(mrf_rx_info_t *rxinfo, uint8_t *rx_buffer);
void handle_tx(mrf_tx_info_t *txinfo);
void mrf_reset(void);
void transmit_mrf(void);
void transmit_lora(lora_sx1276);
void receive_mrf(void);
void receive_lora(lora_sx1276);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
