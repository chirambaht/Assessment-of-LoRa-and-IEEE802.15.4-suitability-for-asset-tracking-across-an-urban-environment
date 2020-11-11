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
#include <string.h>
#include <stdio.h>
#include "retarget.h"
#include "experiments.h"
#include "lib_mrf24j.h"

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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t count_lora = 0;
uint8_t count_mrf = 0;
char val[4]; // Char array for numbers
uint8_t last_mrf_rssi = 0; // Last MRF packet RSSI
uint8_t last_lora_rssi = 0; // Last Lora packet RSSI
uint8_t last_lora_TA = 0;
uint8_t last_lora_TB = 0;
uint8_t last_lora_TC = 0;
uint8_t last_mrf_TA = 0;
uint8_t last_mrf_TB = 0;
uint8_t last_mrf_TC = 0;
uint8_t total_lora = 0;
uint8_t total_mrf = 0;
uint8_t live_vals[4] = {0};
char DEVICE_ID[] = "A";
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */
    uint8_t PROGRAM = 0;
    // Modes:
    /*
     * Tower: This collects 1 packet from the environment and displays is RSSI value
     * 			0: Print		1: Collect MRF		2: Collect LORA		3: Check functionality
     * Joint: This collects all available packets in the network. It collects until enough have been obtained and displays them or till the limit is reached
     * 			0: Print		1: Collect LORA		2: Collect MRF
     * Count: Counts the number of packets received and auto updates the counter
     * Live: Receive packet RSSI's as they come in
     * 			0: Pause		1: Continue			2: LORA				3: MRF
     * Transmit: Mimics the transmitter modules by sending packets with their ID + D
     * 			0: A			1: B				2: C				3: D
     *
     * */
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
    MX_SPI1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    RetargetInit( & huart2);
    // Init LCD
    init_LCD();

    // Initialise LORA
    lora_sx1276 lora;
    uint8_t res = lora_init( & lora, & hspi1, LORA_NSS_GPIO_Port, LORA_NSS_Pin, LORA_BASE_FREQUENCY_EU);
    uint8_t ver = lora_version( & lora);
    _delay_ms(100);
    printf("Starting Lora up...");
    if (res != LORA_OK) {
        // Initialization failed
        printf("failed\n");
        lcd_putstring("LORA failed!");
        lcd_command(LINE_TWO);
        lcd_putstring("Please reset");

        while (1) {
            if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                lcd_command(CLEAR);
                lcd_putstring("Bypass LORA");
                break;
            }
            _delay_ms(1000);
        }

    } else {
        printf("\nLora version: %d\n", ver);
        lcd_putstring("LORA up!");
    }

    // Initialize MRF module
    mrf_reset();
    mrf_deselect();
    printf("MRF starting up...");
    uint8_t g = mrf_read_short(MRF_RXMCR);
    if (g != 0x00) {
        // Initialization failed
        UlToStr(val, g, 3);
        lcd_putstring(val);
        lcd_command(LINE_TWO);
        printf("failed\n");
        lcd_putstring("MRF failed!");
        lcd_command(LINE_TWO);
        lcd_putstring("Please reset");
        while (1) {
            if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                lcd_command(CLEAR);
                lcd_putstring("Bypass MRF");
                break;
            }
            _delay_ms(1000);
        }
    } else {
        mrf_write_short(MRF_SOFTRST, 0x7); // from manual
        while ((mrf_read_short(MRF_SOFTRST) & 0x7) != 0) {
            ; // wait for soft reset to finish
        }
        mrf_init();
        mrf_set_ignorebytes(2);
        mrf_pan_write(0xFFFF);
        lcd_putstring("MRF up!");
    }

    mrf_promiscuous(1);
    mrf_address16_write(0xFFFF);

    _delay_ms(1000);
    lcd_command(CLEAR);
    lcd_putstring("Setup complete");

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        switch (PROGRAM) {
        case 0: // Default with access to separate receiveing stations.
            if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == 0) {
                    _delay_ms(DEBOUNCE);
                    lcd_command(CLEAR);
                    lcd_putstring("Going into tower mode.");
                    _delay_ms(1000);
                    PROGRAM = 1;
                    continue;
                }
                print_display();
            }

            if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                lcd_command(CLEAR); // Clear LCD
                lcd_putstring("Obtaining MRF");
                uint8_t t = 0;
                volatile uint8_t counter = 0;

                uint8_t tmp = last_mrf_rssi;
                lcd_command(LINE_TWO);
                while (last_mrf_rssi == tmp) {
                    receive_mrf();
                    if (counter >= MIN_WAIT) {
                        t = 1;
                        break;
                    }
                    counter += 1; // counter will get to 100ms * ( MIN_WAIT = 100) = 10s

                    if (counter % 100 == 0) {
                        lcd_putchar('.');
                    }
                }

                if (t) {
                    last_mrf_rssi = 0;
                    lcd_command(CLEAR);
                    lcd_putstring("MRF failed");
                } else {
                    print_display();
                }
            }

            if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                lcd_command(CLEAR); // Clear LCD
                lcd_putstring("Obtaining LORA");
                uint8_t t = 0;
                volatile uint8_t counter = 0;

                uint8_t tmp = last_lora_rssi;
                lcd_command(LINE_TWO);
                while (last_lora_rssi == tmp) {
                    receive_lora(lora);
                    if (counter >= MIN_WAIT) {
                        t = 1;
                        break;
                    }
                    counter += 50;

                    lcd_putstring("...");
                }

                if (t) {
                    last_lora_rssi = 0;
                    lcd_command(CLEAR);
                    lcd_putstring("LORA failed");
                } else {
                    print_display();
                }
            }

            if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                lcd_command(CLEAR); // Clear LCD
                lcd_putstring("LORA --> ");
                uint8_t t = 0;
                volatile uint8_t counter = 0;

                uint8_t tmp = last_lora_rssi;
                while (last_lora_rssi == tmp) {
                    receive_lora(lora);
                    if (counter >= MIN_WAIT) {
                        t = 1;
                        break;
                    }
                    counter += 50;
                }

                if (t) {
                    last_lora_rssi = 0;
                    lcd_putstring("FAIL");
                } else {
                    lcd_putstring("GOOD");
                }

                lcd_command(LINE_TWO);
                lcd_putstring("MRF --> ");
                t = 0;
                counter = 0;

                tmp = last_mrf_rssi;

                while (last_mrf_rssi == tmp) {
                    receive_mrf();
                    if (counter >= MIN_WAIT) {
                        t = 1;
                        break;
                    }
                    counter += 1; // counter will get to 100ms * ( MIN_WAIT = 100) = 10s
                }

                if (t) {
                    last_mrf_rssi = 0;
                    lcd_putstring("FAIL");
                } else {
                    lcd_putstring("GOOD");
                }
            }

            break;
        case 1:
            if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == 0) {
                    _delay_ms(DEBOUNCE);
                    lcd_command(CLEAR);
                    lcd_putstring("Going into count mode.");
                    _delay_ms(1000);
                    PROGRAM = 2;
                    continue;
                }
                print_joint();
            }

            if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                lcd_command(CLEAR);
                lcd_putstring("MRF Test - ");
                uint8_t g = mrf_read_short(0x12);

                if (g == 0x39) {
                    lcd_command(LINE_TWO);
                    lcd_putstring("Working");
                } else {
                    UlToStr(val, g, 3);
                    lcd_putstring(val);
                    lcd_command(LINE_TWO);
                    lcd_putstring("Fault in SPI lines");
                }
                _delay_ms(2000);
                print_joint();
            }

            if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                lora_mode_receive_continious( & lora);
                // Update LORA
                printf("Lora receiving....");
                lcd_command(CLEAR);
                lcd_putstring("Fetching LORA");
                lcd_command(LINE_TWO);
                uint8_t buffer[32];
                uint8_t loops = 0;
                uint8_t A = 0, B = 0, C = 0;
                uint8_t res;
                do {
                    lora_receive_packet_blocking( & lora, buffer, sizeof(buffer), 1000, & res);
                    uint8_t rssi = lora_packet_rssi( & lora);
                    if (res != LORA_OK) {
                        printf(" No message received!\n");
                        lcd_putstring(".");
                    } else {
                        buffer[1] = '\0';
                        if ((uint8_t) buffer[0] == (uint8_t)
                            'A') {
                            A = 1;
                            printf("Got from tower A");
                            lcd_putstring("A");
                            last_lora_TA = rssi;
                        } else if ((uint8_t) buffer[0] == (uint8_t)
                            'B') {
                            B = 1;
                            printf("Got from tower B");
                            lcd_putstring("B");
                            last_lora_TB = rssi;
                        } else if ((uint8_t) buffer[0] == (uint8_t)
                            'C') {
                            C = 1;
                            printf("Got from tower C");
                            lcd_putstring("C");
                            last_lora_TC = rssi;
                        } else {
                            buffer[1] = '\0';
                            lcd_putstring("e");
                            lcd_putstring(buffer);
                        }
                    }
                    loops++;
                    if ((A + B + C) == 3) {
                        break;
                    }
                } while (loops < 15);
                print_joint();
            }
            lora_mode_sleep( & lora);

            if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0) {

            }

            break;
        case 2:
            if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == 0) {
                    _delay_ms(DEBOUNCE);
                    lcd_command(CLEAR);
                    lcd_putstring("Live mode");
                    _delay_ms(1000);
                    PROGRAM = 3;
                    mrf_address16_write(0x0001);
                    mrf_promiscuous(0);
                    printf("Set as transmitter.\n");
                    continue;
                }
                print_count();
            }
            uint8_t buffer[32];
            uint8_t res;
            lora_mode_receive_continious( & lora);
            lora_receive_packet_blocking( & lora, buffer, sizeof(buffer), 5000, & res);
            if (res == LORA_OK) {
                count_lora++;
                print_count();
            }
            mrf_check_flags( & handle_rx, & handle_tx);
            if (HAL_GPIO_ReadPin(MRF_INT_GPIO_Port, MRF_INT_Pin) == 0) {
                mrf_interrupt_handler();
                count_mrf++;
            }
            break;
        case 3:
            if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == 0) {
                    _delay_ms(DEBOUNCE);
                    lcd_command(CLEAR);
                    lcd_putstring("Transmit mode");
                    _delay_ms(MAX_WAIT);
                    PROGRAM = 4;
                    mrf_reset();
                    mrf_init();
					mrf_set_ignorebytes(2);
					mrf_pan_write(0xFFFF);
					mrf_address16_write(0x0001);
                    continue;
                }
                print_live();
            }
            uint8_t p = count_lora;
            receive_lora(lora);

            if (p != count_lora) {
                live_vals[3] = live_vals[2];
                live_vals[2] = live_vals[1];
                live_vals[1] = live_vals[0];
                live_vals[0] = last_lora_rssi;
            }

            print_live();
            _delay_ms(500);
            break;
        case 4:
            if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0) {
                    _delay_ms(DEBOUNCE);
                    lcd_command(CLEAR);
                    lcd_putstring("Going to joint");
                    _delay_ms(1000);
                    PROGRAM = 0;
                    mrf_reset();
					mrf_init();
					mrf_set_ignorebytes(2);
					mrf_pan_write(0xFFFF);
					mrf_promiscuous(1);
					mrf_address16_write(0xFFFF);
                    continue;
                }

                if (DEVICE_ID[0] != "A") {
                    DEVICE_ID[0] = "A";
                    lcd_command(CLEAR);
                    lcd_putstring("Mimicking A");
                    _delay_ms(1000);
                }
            }
            if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                if (DEVICE_ID[0] != "B") {
                    DEVICE_ID[0] = "B";
                    lcd_command(CLEAR);
                    lcd_putstring("Mimicking B");
                    _delay_ms(1000);
                }
            }
            if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                if (DEVICE_ID[0] != "C") {
                    DEVICE_ID[0] = "C";
                    lcd_command(CLEAR);
                    lcd_putstring("Mimicking C");
                    _delay_ms(1000);
                }
            }
            if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0) {
                _delay_ms(DEBOUNCE);
                if (DEVICE_ID[0] != "D") {
                    DEVICE_ID[0] = "D";
                    lcd_command(CLEAR);
                    lcd_putstring("Mimicking D");
                    _delay_ms(1000);
                }
            }

            _delay_ms(1000);
            lcd_command(CLEAR);
            transmit_mrf();
            transmit_lora(lora);

            break;
        default:
        	PROGRAM = 0;
        	lcd_command(CLEAR);
			lcd_putstring("Going to joint");
            break;
        }
        _delay_ms(10);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {
        0
    };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {
        0
    };

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig( & RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig( & RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init( & hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 38400;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_HalfDuplex_Init( & huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {
        0
    };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, MRF_NSS_Pin | MRF_RESET_Pin | LORA_NSS_Pin | LORA_RESET_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

    /*Configure GPIO pins : PC14 PC15 */
    GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, & GPIO_InitStruct);

    /*Configure GPIO pins : SW0_Pin SW1_Pin SW2_Pin SW3_Pin */
    GPIO_InitStruct.Pin = SW0_Pin | SW1_Pin | SW2_Pin | SW3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, & GPIO_InitStruct);

    /*Configure GPIO pins : MRF_NSS_Pin LORA_NSS_Pin */
    GPIO_InitStruct.Pin = MRF_NSS_Pin | LORA_NSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, & GPIO_InitStruct);

    /*Configure GPIO pins : MRF_RESET_Pin LORA_RESET_Pin PB8 PB9 */
    GPIO_InitStruct.Pin = MRF_RESET_Pin | LORA_RESET_Pin | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, & GPIO_InitStruct);

    /*Configure GPIO pin : MRF_INT_Pin */
    GPIO_InitStruct.Pin = MRF_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MRF_INT_GPIO_Port, & GPIO_InitStruct);

    /*Configure GPIO pins : PA12 PA15 */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, & GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void print_display() {
    lcd_command(0x01);
    lcd_putstring(" LORA  ||  MRF  ");
    lcd_command(0xC0); // Next line
    UlToStr(val, count_lora, 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, last_lora_rssi, 3);
    lcd_putstring(val);
    lcd_putstring("||");
    UlToStr(val, count_mrf, 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, last_mrf_rssi, 3);
    lcd_putstring(val);
}

void print_joint() {
    lcd_command(CLEAR);
    lcd_putstring("LORA ");
    UlToStr(val, last_lora_TA, 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, last_lora_TB, 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, last_lora_TC, 3);
    lcd_putstring(val);
    lcd_command(0xC0); // Next line
    lcd_putstring(" MRF ");
    UlToStr(val, last_mrf_TA, 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, last_mrf_TB, 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, last_mrf_TC, 3);
    lcd_putstring(val);
}

void print_count() {
    lcd_command(CLEAR);
    lcd_putstring("LORA: ");
    UlToStr(val, count_lora, 4);
    lcd_putstring(val);
    lcd_command(LINE_TWO);
    lcd_putstring("MRF:  ");
    UlToStr(val, count_mrf, 4);
    lcd_putstring(val);
}

void print_distance(void) {
    lcd_command(CLEAR);
    lcd_putstring("LORA ");
    UlToStr(val, get_lora_distance(last_lora_TA), 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, get_lora_distance(last_lora_TB), 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, get_lora_distance(last_lora_TC), 3);
    lcd_putstring(val);
    lcd_command(0xC0); // Next line
    lcd_putstring(" MRF ");
    UlToStr(val, get_mrf_distance(last_mrf_TA), 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, get_mrf_distance(last_mrf_TB), 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, get_mrf_distance(last_mrf_TC), 3);
    lcd_putstring(val);
}

void print_live() {
    lcd_command(LINE_TWO);
    UlToStr(val, get_mrf_distance(live_vals[3]), 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, get_mrf_distance(live_vals[2]), 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, get_mrf_distance(live_vals[1]), 3);
    lcd_putstring(val);
    lcd_putstring(" ");
    UlToStr(val, get_mrf_distance(last_lora_rssi), 3);
    lcd_putstring(val);
    lcd_putstring(" ");
}

uint8_t get_lora_distance(uint8_t rssi) {
    return rssi / 8;
}

uint8_t get_mrf_distance(uint8_t rssi) {
    return rssi / 4;
}

void mrf_reset(void) {
    HAL_GPIO_WritePin(MRF_RESET_GPIO_Port, MRF_RESET_Pin, 0);
    _delay_ms(10);
    HAL_GPIO_WritePin(MRF_RESET_GPIO_Port, MRF_RESET_Pin, 1);
    _delay_ms(20);
}

void handle_rx(mrf_rx_info_t * rxinfo, uint8_t * rx_buffer) {
    printf("MRF RSSI=%d\n", rxinfo -> rssi);
    last_mrf_rssi = rxinfo -> rssi;
    count_mrf++;
}

void handle_tx(mrf_tx_info_t * txinfo) {
    lcd_command(LINE_TWO);
    lcd_putstring("MRF - ");
    if (txinfo -> tx_ok) {
        lcd_putstring("Sent ");
        lcd_putstring(DEVICE_ID[0]);
        _delay_ms(MED_WAIT);
        printf("MRF message sent!\n");
    } else {
        lcd_putstring("Fail!");
        _delay_ms(MED_WAIT);
        printf("MRF transmission failed after %d retries\n", txinfo -> retries);
    }
    lcd_putstring("end");
}

void mrf_select(void) {
    HAL_GPIO_WritePin(MRF_NSS_GPIO_Port, MRF_NSS_Pin, 0);
    _delay_ms(100);
}

void mrf_deselect(void) {
    HAL_GPIO_WritePin(MRF_NSS_GPIO_Port, MRF_NSS_Pin, 1);
}

uint8_t spi_tx(uint8_t cData) {
    uint8_t buff;
    HAL_SPI_TransmitReceive( & hspi1, & cData, & buff, sizeof(buff), 100);
    return buff;
}

void _delay_ms(int t) {
    HAL_Delay(t);
}

void transmit_mrf() {
    lcd_command(LINE_TWO);
    lcd_putstring("MRF - ");
    printf("Sending MRF...");
    mrf_send16(0xFFFF, 8, &DEVICE_ID);
    HAL_Delay(500);
    mrf_check_flags( & handle_rx, & handle_tx);
    if (HAL_GPIO_ReadPin(MRF_INT_GPIO_Port, MRF_INT_Pin) == 0) {
    	lcd_putstring("-i-");
        mrf_interrupt_handler();
        mrf_check_flags( & handle_rx, & handle_tx);
    }
}

void transmit_lora(lora_sx1276 lora) {
    printf("Sending LORA...");

    uint8_t res = lora_send_packet( & lora, (uint8_t * ) DEVICE_ID, 1);
    HAL_Delay(300);

    lcd_command(CURSOR_HOME);
    lcd_putstring("LORA - ");

    if (res != LORA_OK) {
        // Send failed
        lcd_putstring("Fail!");
        printf("Send failed\n");
    } else {
        lcd_putstring("Sent ");
        lcd_putstring(DEVICE_ID[0]);
        printf("Packet sent!\n");
    }
}

void receive_mrf() {
    printf("MRF receiving....");
    mrf_check_flags( & handle_rx, & handle_tx);
    if (HAL_GPIO_ReadPin(MRF_INT_GPIO_Port, MRF_INT_Pin) == 0) {
        printf("\n");
        mrf_interrupt_handler();
    } else {
        printf(" Nothing to receive!\n");
    }

}

void receive_lora(lora_sx1276 lora) {
    printf("Lora receiving....");
    uint8_t buffer[32];

    lora_mode_receive_continious( & lora);

    uint8_t res;
    uint8_t len = lora_receive_packet_blocking( & lora, buffer, sizeof(buffer), 1500, & res);
    uint8_t rssi = lora_packet_rssi( & lora);
    if (res != LORA_OK) {
        printf(" No message received!\n");
    } else {
        buffer[len] = 0; // null terminate string to print it
        printf("success!\nData: %s\n", buffer);
        printf("LORA RSSI = %d\n", rssi);
        last_lora_rssi = rssi;
        count_lora++;
    }
}

void UlToStr(char * s, unsigned long bin, unsigned char n) {
    s += n;
    * s = '\0';

    while (n--) {
        *--s = (bin % 10) + '0';
        bin /= 10;
    }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t * file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
