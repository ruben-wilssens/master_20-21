/* USER CODE BEGIN Header */
/**
  *************************************************************************
  * @file       : main.h
  * @brief      : Header for main.c file.
  *               This file contains the common defines of the application.
  *************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  *************************************************************************
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_TALK_Pin GPIO_PIN_1
#define BUTTON_TALK_GPIO_Port GPIOA
#define BUTTON_TALK_EXTI_IRQn EXTI1_IRQn
#define BUTTON_UP_Pin GPIO_PIN_3
#define BUTTON_UP_GPIO_Port GPIOA
#define BUTTON_UP_EXTI_IRQn EXTI3_IRQn
#define BUTTON_OK_Pin GPIO_PIN_4
#define BUTTON_OK_GPIO_Port GPIOA
#define BUTTON_OK_EXTI_IRQn EXTI4_IRQn
#define BUTTON_DOWN_Pin GPIO_PIN_6
#define BUTTON_DOWN_GPIO_Port GPIOA
#define BUTTON_DOWN_EXTI_IRQn EXTI9_5_IRQn
#define POT_MOSI_Pin GPIO_PIN_7
#define POT_MOSI_GPIO_Port GPIOA
#define DMIC_CK_Pin GPIO_PIN_0
#define DMIC_CK_GPIO_Port GPIOB
#define DMIC_WS_Pin GPIO_PIN_1
#define DMIC_WS_GPIO_Port GPIOB
#define BUTTON_PWR_Pin GPIO_PIN_2
#define BUTTON_PWR_GPIO_Port GPIOB
#define BUTTON_PWR_EXTI_IRQn EXTI2_IRQn
#define ADF7242_SCK_Pin GPIO_PIN_10
#define ADF7242_SCK_GPIO_Port GPIOB
#define ADF7242_CS_Pin GPIO_PIN_12
#define ADF7242_CS_GPIO_Port GPIOB
#define SWITCH_E_Pin GPIO_PIN_13
#define SWITCH_E_GPIO_Port GPIOB
#define ADF7242_MISO_Pin GPIO_PIN_14
#define ADF7242_MISO_GPIO_Port GPIOB
#define ADF7242_MOSI_Pin GPIO_PIN_15
#define ADF7242_MOSI_GPIO_Port GPIOB
#define ADF7242_IRQ1_Pin GPIO_PIN_8
#define ADF7242_IRQ1_GPIO_Port GPIOA
#define ADF7242_IRQ1_EXTI_IRQn EXTI9_5_IRQn
#define ADF7242_IRQ2_Pin GPIO_PIN_9
#define ADF7242_IRQ2_GPIO_Port GPIOA
#define ADF7242_IRQ2_EXTI_IRQn EXTI9_5_IRQn
#define DMIC_SD_Pin GPIO_PIN_10
#define DMIC_SD_GPIO_Port GPIOA
#define POT_CLK_Pin GPIO_PIN_3
#define POT_CLK_GPIO_Port GPIOB
#define POT_CS_Pin GPIO_PIN_4
#define POT_CS_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB
#define A_MIC_POWER_Pin GPIO_PIN_8
#define A_MIC_POWER_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
