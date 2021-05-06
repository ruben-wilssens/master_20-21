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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "ADF7242.h"
#include "ssh1106.h"
#include "OLED.h"
#include "CBUF.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CLASS_D_SHDN_Pin GPIO_PIN_0
#define CLASS_D_SHDN_GPIO_Port GPIOC
#define MIC_SHDN_Pin GPIO_PIN_1
#define MIC_SHDN_GPIO_Port GPIOC
#define DPOT_CS_Pin GPIO_PIN_2
#define DPOT_CS_GPIO_Port GPIOC
#define DPOT_MOSI_Pin GPIO_PIN_3
#define DPOT_MOSI_GPIO_Port GPIOC
#define PA_LNA_HGM_Pin GPIO_PIN_1
#define PA_LNA_HGM_GPIO_Port GPIOA
#define ADF7242_CS_Pin GPIO_PIN_2
#define ADF7242_CS_GPIO_Port GPIOA
#define ADF7242_SCK_Pin GPIO_PIN_5
#define ADF7242_SCK_GPIO_Port GPIOA
#define ADF7242_MISO_Pin GPIO_PIN_6
#define ADF7242_MISO_GPIO_Port GPIOA
#define ADF7242_MOSI_Pin GPIO_PIN_7
#define ADF7242_MOSI_GPIO_Port GPIOA
#define ADF7242_IRQ1_Pin GPIO_PIN_4
#define ADF7242_IRQ1_GPIO_Port GPIOC
#define ADF7242_IRQ1_EXTI_IRQn EXTI4_IRQn
#define ADF7242_GP3_Pin GPIO_PIN_5
#define ADF7242_GP3_GPIO_Port GPIOC
#define ADF7242_GP1_Pin GPIO_PIN_0
#define ADF7242_GP1_GPIO_Port GPIOB
#define ADF7242_IRQ2_Pin GPIO_PIN_1
#define ADF7242_IRQ2_GPIO_Port GPIOB
#define ADF7242_IRQ2_EXTI_IRQn EXTI1_IRQn
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define DPOT_SCK_Pin GPIO_PIN_10
#define DPOT_SCK_GPIO_Port GPIOB
#define ADF7242_GP0_Pin GPIO_PIN_11
#define ADF7242_GP0_GPIO_Port GPIOB
#define BTN_UP_Pin GPIO_PIN_12
#define BTN_UP_GPIO_Port GPIOB
#define BTN_UP_EXTI_IRQn EXTI15_10_IRQn
#define BTN_RIGHT_Pin GPIO_PIN_13
#define BTN_RIGHT_GPIO_Port GPIOB
#define BTN_RIGHT_EXTI_IRQn EXTI15_10_IRQn
#define BTN_LEFT_Pin GPIO_PIN_14
#define BTN_LEFT_GPIO_Port GPIOB
#define BTN_LEFT_EXTI_IRQn EXTI15_10_IRQn
#define BTN_DOWN_Pin GPIO_PIN_15
#define BTN_DOWN_GPIO_Port GPIOB
#define BTN_DOWN_EXTI_IRQn EXTI15_10_IRQn
#define BTN_TALK_Pin GPIO_PIN_6
#define BTN_TALK_GPIO_Port GPIOC
#define BTN_TALK_EXTI_IRQn EXTI9_5_IRQn
#define BTN_PWR_Pin GPIO_PIN_7
#define BTN_PWR_GPIO_Port GPIOC
#define BTN_PWR_EXTI_IRQn EXTI9_5_IRQn
#define LED_RGB_BLUE_Pin GPIO_PIN_8
#define LED_RGB_BLUE_GPIO_Port GPIOC
#define LED_RGB_GREEN_Pin GPIO_PIN_9
#define LED_RGB_GREEN_GPIO_Port GPIOC
#define LED_RGB_RED_Pin GPIO_PIN_8
#define LED_RGB_RED_GPIO_Port GPIOA
#define LBO_Pin GPIO_PIN_10
#define LBO_GPIO_Port GPIOA
#define LBO_EXTI_IRQn EXTI15_10_IRQn
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
