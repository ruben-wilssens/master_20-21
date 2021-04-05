/*
 * OLED.h
 *
 *  Created on: Mar 15, 2020
 *      Author: jelle
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#ifdef __cplusplus
extern C {
#endif

/**
 * The OLED uses I2C for communication
 *
 * Default pinout
 *
OLED	  |STM32F4    |DESCRIPTION

SDA       |PB7        |I2C Serial Data
SCL       |PB6        |I2C Serial Clock
 */

/* Includes ------------------------------------------------------------------*/
#include "ssd1306.h"
#include "fonts.h"
#include "stm32f4xx_hal.h"
#include "main.h"

/* Defines -------------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
void OLED_init(void);
void OLED_clear_screen(void);
void OLED_update(void);
void OLED_shutdown(void);

void OLED_print_text(char[], uint8_t, uint8_t);
void OLED_print_variable(char[], uint32_t, uint8_t, uint8_t);
void OLED_print_hexadecimal(char[], uint32_t, uint8_t, uint8_t);

void OLED_print_date_and_time(void);
void OLED_print_talk(void);
void OLED_print_stoptalk(void);
void OLED_print_volume(uint8_t);

#ifdef __cplusplus
}
#endif

#endif /* INC_OLED_H_ */
