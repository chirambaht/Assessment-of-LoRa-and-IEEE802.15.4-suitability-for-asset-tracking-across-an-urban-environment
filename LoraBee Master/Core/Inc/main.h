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
#define    CLEAR           0x01
#define    CURSOR_HOME     0x02
#define    DISPLAY_ON      0x0C
#define    DISPLAY_OFF     0x08
#define    LINE_TWO        0xC0
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW0_Pin GPIO_PIN_0
#define SW0_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_1
#define SW1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_2
#define SW2_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_3
#define SW3_GPIO_Port GPIOA
#define MRF_NSS_Pin GPIO_PIN_0
#define MRF_NSS_GPIO_Port GPIOB
#define MRF_RESET_Pin GPIO_PIN_1
#define MRF_RESET_GPIO_Port GPIOB
#define MRF_INT_Pin GPIO_PIN_2
#define MRF_INT_GPIO_Port GPIOB
#define LORA_NSS_Pin GPIO_PIN_10
#define LORA_NSS_GPIO_Port GPIOB
#define LORA_RESET_Pin GPIO_PIN_11
#define LORA_RESET_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void handle_rx(mrf_rx_info_t *rxinfo, uint8_t *rx_buffer);
void handle_tx(mrf_tx_info_t *txinfo);
void mrf_reset(void);
void transmit_mrf(void);
void transmit_lora(lora_sx1276);
void receive_mrf(void);
void receive_lora(lora_sx1276);
void UlToStr(char *s, unsigned long bin, unsigned char n);
void print_display(void);
void print_joint(void);
void print_distance(void);
uint8_t get_lora_distance(uint8_t);
uint8_t get_mrf_distance(uint8_t);
void print_count(void);
void print_live(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
