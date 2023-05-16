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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <Application.hpp>
#include <cz_interface.hpp>
#include <Emergency.hpp>
#include <handles.hpp>
#include <Heartbeat.hpp>
#include "main.h"

#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "rng.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "iwdg.h"
#include "led.h"
#include "FreeRTOS.h"

#include "TaskManager.hpp"
#include "log.h"
#include "AdcDma.hpp"
#include "BuildDateTime.hpp"

#include <AdcDma.hpp>
#include "BuildDateTime.hpp"

#include "dbc_parser.hpp"
using namespace can;

#include "PDU.hpp"
#include "ProjectXX.hpp"


void SystemClock_Config(void);
//TODO: callbacks in a separate file?
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	LED_RGB_Write(100, 0, 0);
}
void wdTask(void* params){
	while(1){
		HAL_IWDG_Refresh(&hiwdg);
		osDelay(pdMS_TO_TICKS(490));
	}
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

//Necessary to map the vector table correctly
static void VectorBase_Config(void)
{
	/* The constant array with vectors of the vector table is declared externally in the
	 * c-startup code.
	 */
	extern const unsigned long g_pfnVectors[];

	/* Remap the vector table to where the vector table is located for this program. */
	SCB->VTOR = (unsigned long)&g_pfnVectors[0];
}


/**
 * @brief  The application entry point.
 * @retval int
 */
int main() {
	//disable interrupts during init
	taskENTER_CRITICAL();

	VectorBase_Config();
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_CRC_Init();
	MX_RNG_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM8_Init();
	MX_TIM10_Init();
	MX_TIM11_Init();
	MX_TIM12_Init();
#ifdef RELEASE
	MX_IWDG_Init();
#endif
	LED_RGB_Init();



	//create the Applications that will run
	Application canreceiveApp;
	Application cansendApp;
	Application emergencyApp;
	Application hbsendApp;
	Application hbconsumeApp;
	Application runtimeStats;
	Application sendOdEntriesApp;
#ifdef RELEASE
	Application watchdog;
#endif

	/* Init scheduler */
	osKernelInitialize();
#ifdef DEBUG
	LED_RGB_Write(100, 0, 100);
#endif


	//creating the Taks
	canreceiveApp.setPriority(osPriorityNormal1);
	canReceiveTaskHandle = canreceiveApp.create("receiveTask",
			cz_interface::getInstance()->cz_receive, NULL);
	canSendTaskHandle = cansendApp.create("canSend", cz_interface::getInstance()->cz_send, NULL );

	emergencyTaskHandle = emergencyApp.create("emergency",
			Emergency::getEmergencyInstance()->waitForEmergency, NULL);

	emergencyApp.setPriority(osPriorityAboveNormal);

	hbsendApp.setPriority(osPriorityNormal);
	heartbeatProduceHandle = hbsendApp.create("heartbeat",
			Heartbeat::getHeartbeatInstance()->sendHeartbeat, NULL);
	heartbeatConsumeTaskHandle = hbconsumeApp.create("hearbeatConsume",
			Heartbeat::getHeartbeatInstance()->consumeHeartbeat, NULL);
	runtimeStats.setPriority(osPriorityBelowNormal7);
	runtimeStatsHandle = (TaskHandle_t) runtimeStats.create("statsUpdate", stats::updateSensorStats, NULL);
	sendOdEntriesApp.setPriority(osPriorityBelowNormal7);
	sendOdEntriesTaskHandle = sendOdEntriesApp.create("sendOdEntries", sendOdEntriesTask, NULL);
#ifdef RELEASE
	watchdog.setPriority(osPriorityHigh1);
	watchdog.create("watchdog", wdTask, NULL);
#endif

	Application pduApp;
	pduApp.setStackSize(2 * 256 * 4);	// Needed, otherwise hardfault occurs sometimes!!
	pduApp.create("pduApp", pduAppFunction, NULL);

	Application projectXXApp;
	projectXXApp.setStackSize(2 * 256 * 4);
	projectXXApp.create("projectXXApp", projectXXFunction, NULL);

	//create stats the first time
	stats::estimateCPUusage();
	stats::estimateFreeMemory();
	//enable the interrupts again
	taskEXIT_CRITICAL();
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	while (1) {

	}
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
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

// ADC Conversion complete callback
// If your shield uses the ADC, define adc2 and uncomment the line below.
// See TaskManager.cpp on how to use the AdcDma class
extern AdcDma<4> adc1;
extern AdcDma<14> adc2;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	adc1.ADC_ConvCpltCallback(hadc);
	adc2.ADC_ConvCpltCallback(hadc);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	uint8_t RxData[8];
	CAN_RxHeaderTypeDef RxHeader;

	// Initialize StdId and ExtId with zero, because HAL_CAN_GetRxMessage() only sets one of them
	RxHeader.StdId = 0;
	RxHeader.ExtId = 0;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	RxMessage m;
	m.rxHeader = RxHeader;
	for(int i = 0; i<8;i++)
		m.rxBuf[i] = RxData[i];
	//if the function hangs/does not return it might because the interrupts have invalid priorities.
	//They have to be greater or equal to the max interrupt priority (default: 5) set in the FreeRTOS config Parameters
	//Interrupt priority is set in HAL_CAN_MspInit in can.c
	if(osMessageQueuePut(czReceiveQueue, &m, 0, 0) != osOK){

	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	uint8_t RxData[8];
	CAN_RxHeaderTypeDef RxHeader;

	// Initialize StdId and ExtId with zero, because HAL_CAN_GetRxMessage() only sets one of them
	RxHeader.StdId = 0;
	RxHeader.ExtId = 0;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	RxMessage m;
	m.rxHeader = RxHeader;
	for(int i = 0; i<8;i++)
		m.rxBuf[i] = RxData[i];
	//if the function hangs/does not return it might because the interrupts have invalid priorities.
	//They have to be greater or equal to the max interrupt priority (default: 5) set in the FreeRTOS config Parameters
	//Interrupt priority is set in HAL_CAN_MspInit in can.c
	if(osMessageQueuePut(czReceiveQueue, &m, 0, 0) != osOK){

	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_ResetError(hcan);
	printDebugISR("CAN Errors got reseted!\n");
}


/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
}

unsigned long getRunTimeCounterValue(){
	return uwTick;
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	printDebug("Error Handler called! (HAL error)\n");
	while (1) {
	}
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
	/* User can add his own implementation to report the file name and line number,*/
	printDebug("Wrong parameters value: file %s on line %lu\r\n", file, line);
	while(1);
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
