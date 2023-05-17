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
#define LP9_sensing_Pin GPIO_PIN_1
#define LP9_sensing_GPIO_Port GPIOC
#define ADC_IN12_Board_VCC_Pin GPIO_PIN_2
#define ADC_IN12_Board_VCC_GPIO_Port GPIOC
#define LP10_sensing_Pin GPIO_PIN_3
#define LP10_sensing_GPIO_Port GPIOC
#define LP6_sensing_Pin GPIO_PIN_0
#define LP6_sensing_GPIO_Port GPIOA
#define LP5_sensing_Pin GPIO_PIN_1
#define LP5_sensing_GPIO_Port GPIOA
#define HP2_sensing_Pin GPIO_PIN_2
#define HP2_sensing_GPIO_Port GPIOA
#define LP8_sensing_Pin GPIO_PIN_3
#define LP8_sensing_GPIO_Port GPIOA
#define LP7_sensing_Pin GPIO_PIN_4
#define LP7_sensing_GPIO_Port GPIOA
#define LP2_sensing_Pin GPIO_PIN_5
#define LP2_sensing_GPIO_Port GPIOA
#define LP1_sensing_Pin GPIO_PIN_6
#define LP1_sensing_GPIO_Port GPIOA
#define HP3_sensing_Pin GPIO_PIN_7
#define HP3_sensing_GPIO_Port GPIOA
#define HP1_sensing_Pin GPIO_PIN_4
#define HP1_sensing_GPIO_Port GPIOC
#define HP4_sensing_Pin GPIO_PIN_5
#define HP4_sensing_GPIO_Port GPIOC
#define LP4_sensing_Pin GPIO_PIN_0
#define LP4_sensing_GPIO_Port GPIOB
#define LP3_sensing_Pin GPIO_PIN_1
#define LP3_sensing_GPIO_Port GPIOB
#define LP2_control_Pin GPIO_PIN_10
#define LP2_control_GPIO_Port GPIOB
#define D3_control_Pin GPIO_PIN_11
#define D3_control_GPIO_Port GPIOB
#define LP7_control_Pin GPIO_PIN_12
#define LP7_control_GPIO_Port GPIOB
#define LP1_control_Pin GPIO_PIN_15
#define LP1_control_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOD
#define LP8_control_Pin GPIO_PIN_6
#define LP8_control_GPIO_Port GPIOC
#define HP2_control_Pin GPIO_PIN_7
#define HP2_control_GPIO_Port GPIOC
#define D4_control_Pin GPIO_PIN_8
#define D4_control_GPIO_Port GPIOC
#define HP1_control_Pin GPIO_PIN_9
#define HP1_control_GPIO_Port GPIOC
#define HP3_control_Pin GPIO_PIN_8
#define HP3_control_GPIO_Port GPIOA
#define SDC_control_Pin GPIO_PIN_9
#define SDC_control_GPIO_Port GPIOA
#define HP4_control_Pin GPIO_PIN_10
#define HP4_control_GPIO_Port GPIOA
#define LP4_control_Pin GPIO_PIN_12
#define LP4_control_GPIO_Port GPIOA
#define LP3_control_Pin GPIO_PIN_15
#define LP3_control_GPIO_Port GPIOA
#define LP5_control_Pin GPIO_PIN_11
#define LP5_control_GPIO_Port GPIOC
#define LP6_control_Pin GPIO_PIN_12
#define LP6_control_GPIO_Port GPIOC
#define SDC_IN_STATUS_Pin GPIO_PIN_3
#define SDC_IN_STATUS_GPIO_Port GPIOD
#define SDC_OUT_STATUS_Pin GPIO_PIN_4
#define SDC_OUT_STATUS_GPIO_Port GPIOD
#define DBG_UART_TX_Pin GPIO_PIN_5
#define DBG_UART_TX_GPIO_Port GPIOD
#define DBG_UART_RX_Pin GPIO_PIN_6
#define DBG_UART_RX_GPIO_Port GPIOD
#define D1_control_Pin GPIO_PIN_4
#define D1_control_GPIO_Port GPIOB
#define LP9_control_Pin GPIO_PIN_7
#define LP9_control_GPIO_Port GPIOB
#define D2_control_Pin GPIO_PIN_8
#define D2_control_GPIO_Port GPIOB
#define LP10_control_Pin GPIO_PIN_9
#define LP10_control_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
