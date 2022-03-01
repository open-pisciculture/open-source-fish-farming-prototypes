/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//SPI_HandleTypeDef hspi1;
//TIM_HandleTypeDef htim3;

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
//#include "STM32L476RG.h"
//#include "STM32L476RG_gpio_driver.h"
//#include "STM32L476RG_i2c_driver.h"

#include "hal.h"
#include "arduino_lmic.h"
#include "lmic.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	uint16_t mcu_temperature;
	u2_t temperature;
	u2_t dissolvedOxygen;
	u2_t pH;
}AtlasSensorData;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern const lmic_pinmap lmic_pins;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//void HAL_IncTick(void);
uint32_t getCurrentMicro(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SENSORS_PWR_Pin GPIO_PIN_2
#define SENSORS_PWR_GPIO_Port GPIOC
#define USER_LED_Pin GPIO_PIN_5
#define USER_LED_GPIO_Port GPIOA
#define DIO0_Pin GPIO_PIN_4
#define DIO0_GPIO_Port GPIOC
#define DIO0_EXTI_IRQn EXTI4_IRQn
#define SPI2_NSS_Pin GPIO_PIN_1
#define SPI2_NSS_GPIO_Port GPIOB
#define RFM95_RST_Pin GPIO_PIN_2
#define RFM95_RST_GPIO_Port GPIOB
#define DIO1_Pin GPIO_PIN_10
#define DIO1_GPIO_Port GPIOA
#define DIO1_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DIO2_Pin GPIO_PIN_3
#define DIO2_GPIO_Port GPIOB
#define DIO2_EXTI_IRQn EXTI3_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
