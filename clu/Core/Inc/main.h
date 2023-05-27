/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define ADC_IN10_Board_Temp_Pin GPIO_PIN_0
#define ADC_IN10_Board_Temp_GPIO_Port GPIOC
#define DIN1_Pin GPIO_PIN_1
#define DIN1_GPIO_Port GPIOC
#define ADC_IN12_Board_VCC_Pin GPIO_PIN_2
#define ADC_IN12_Board_VCC_GPIO_Port GPIOC
#define DIN2_Pin GPIO_PIN_3
#define DIN2_GPIO_Port GPIOC
#define DIN2_EXTI_IRQn EXTI3_IRQn
#define WHEELENCODER_A_Pin GPIO_PIN_0
#define WHEELENCODER_A_GPIO_Port GPIOA
#define WHEELENCODER_B_Pin GPIO_PIN_1
#define WHEELENCODER_B_GPIO_Port GPIOA
#define AN4_Pin GPIO_PIN_2
#define AN4_GPIO_Port GPIOA
#define AN3_Pin GPIO_PIN_3
#define AN3_GPIO_Port GPIOA
#define AN5_Pin GPIO_PIN_4
#define AN5_GPIO_Port GPIOA
#define AN6_Pin GPIO_PIN_5
#define AN6_GPIO_Port GPIOA
#define AN7_Pin GPIO_PIN_6
#define AN7_GPIO_Port GPIOA
#define AN8_Pin GPIO_PIN_7
#define AN8_GPIO_Port GPIOA
#define AN2_Pin GPIO_PIN_4
#define AN2_GPIO_Port GPIOC
#define AN1_Pin GPIO_PIN_5
#define AN1_GPIO_Port GPIOC
#define SDC_Pin GPIO_PIN_1
#define SDC_GPIO_Port GPIOB
#define TMP_LED_OUTPUT_Pin GPIO_PIN_10
#define TMP_LED_OUTPUT_GPIO_Port GPIOE
#define CS_IMU2_Pin GPIO_PIN_10
#define CS_IMU2_GPIO_Port GPIOB
#define CS_IMU3_Pin GPIO_PIN_11
#define CS_IMU3_GPIO_Port GPIOB
#define CS_IMU1_Pin GPIO_PIN_12
#define CS_IMU1_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOD
#define LED_ORANGE_Pin GPIO_PIN_13
#define LED_ORANGE_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOD
#define SDC_IN_STATUS_Pin GPIO_PIN_3
#define SDC_IN_STATUS_GPIO_Port GPIOD
#define SDC_OUT_STATUS_Pin GPIO_PIN_4
#define SDC_OUT_STATUS_GPIO_Port GPIOD
#define DBG_UART_TX_Pin GPIO_PIN_5
#define DBG_UART_TX_GPIO_Port GPIOD
#define DBG_UART_RX_Pin GPIO_PIN_6
#define DBG_UART_RX_GPIO_Port GPIOD
#define DIN3_Pin GPIO_PIN_4
#define DIN3_GPIO_Port GPIOB
#define DIN4_Pin GPIO_PIN_7
#define DIN4_GPIO_Port GPIOB
#define I2C_SCL_IMU_Pin GPIO_PIN_8
#define I2C_SCL_IMU_GPIO_Port GPIOB
#define I2C_SDA_IMU_Pin GPIO_PIN_9
#define I2C_SDA_IMU_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
