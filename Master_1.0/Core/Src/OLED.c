/*
 *  OLED.c
 *
 *  Created on: April 2021
 *  Author: Ruben Wilssens & Victor Van der Elst
 *
 */

/* Includes -------------------------------------------------------------------*/
#include "OLED.h"
#include "String.h"
#include <stdio.h>
#include "ssh1106_fonts.h"

/* Defines -------------------------------------------------------------------*/
// Function to print binary values of a byte
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

/* Variables -------------------------------------------------------------------*/
char s;
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern uint8_t LBO;
extern uint8_t menu;
extern uint8_t settings_volume;
extern uint8_t HGM;
extern uint8_t settings_encryption;
extern uint8_t settings_threshold;
extern uint8_t settings_USB_print;
extern char settings_mode;

/* Functions -------------------------------------------------------------------*/
void OLED_print_credits(void){
	ssh1106_SetCursor(40, 15);
	s = ssh1106_WriteString("Encrypted", Font_6x8, White);
	ssh1106_SetCursor(25, 25);
	s = ssh1106_WriteString("walkie talkie", Font_6x8, White);
	ssh1106_SetCursor(64, 35);
	s = ssh1106_WriteString("by", Font_6x8, White);
	ssh1106_SetCursor(10, 45);
	s = ssh1106_WriteString("Victor Van der Elst", Font_6x8, White);
	ssh1106_SetCursor(25, 55);
	s = ssh1106_WriteString("Ruben Wilssens", Font_6x8, White);
}

void OLED_print_status(char status){
	ssh1106_SetCursor(3, 0);
	s = ssh1106_WriteString("12:00", Font_6x8, White); //change with actual time
	ssh1106_SetCursor(63, 0);
	s = ssh1106_WriteChar(status, Font_6x8, White);
	if(LBO){
		ssh1106_SetCursor(92, 0);
		s = ssh1106_WriteString("BAT_Ok", Font_6x8, White);
	}
	else{
		ssh1106_SetCursor(86, 0);
		s = ssh1106_WriteString("BAT_Low", Font_6x8, White);
	}
}

void OLED(void){
	OLED_clear_screen();
	char stringValue[10];
	switch(menu){
		// Encryption ON OFF
		case 0:
			ssh1106_SetCursor(3, 10);
			s = ssh1106_WriteString("Menu 0", Font_6x8, White);
			if(settings_encryption){
				ssh1106_SetCursor(25, 30);
				s = ssh1106_WriteString("Encryption ON", Font_6x8, White);
			}
			else{
				ssh1106_SetCursor(25, 30);
				s = ssh1106_WriteString("Encryption OFF", Font_6x8, White);
			}
			break;
		// HGM LGM
		case 1:
			ssh1106_SetCursor(3, 10);
			s = ssh1106_WriteString("Menu 1", Font_6x8, White);
			if(HGM){
				ssh1106_SetCursor(25, 30);
				s = ssh1106_WriteString("HGM Enabled", Font_6x8, White);
			}
			else{
				ssh1106_SetCursor(25, 30);
				s = ssh1106_WriteString("HGM Disabled", Font_6x8, White);
			}
			break;
		// VOLUME
		case 2:
			ssh1106_SetCursor(3, 10);
			s = ssh1106_WriteString("Menu 2", Font_6x8, White);
			ssh1106_SetCursor(30, 30);
			s = ssh1106_WriteString("Volume: ", Font_6x8, White);
			sprintf(stringValue, "%u", settings_volume); // Copy value in string
			ssh1106_SetCursor(30+(strlen("Volume: ")*6),30);
			s = ssh1106_WriteString(stringValue, Font_6x8, White);
			break;
		// Threshold
		case 3:
			ssh1106_SetCursor(3, 10);
			s = ssh1106_WriteString("Menu 3", Font_6x8, White);
			ssh1106_SetCursor(25, 30);
			s = ssh1106_WriteString("Threshold: ", Font_6x8, White);
			sprintf(stringValue, "%u", settings_threshold); // Copy value in string
			ssh1106_SetCursor(25+(strlen("Threshold: ")*6),30);
			s = ssh1106_WriteString(stringValue, Font_6x8, White);
			break;
		// Credits
		case 4:
			ssh1106_SetCursor(3, 10);
			s = ssh1106_WriteString("Menu 4", Font_6x8, White);
			OLED_print_credits();
			break;
		// USB
		case 5:
			ssh1106_SetCursor(3, 10);
			s = ssh1106_WriteString("Menu 5", Font_6x8, White);
			if(settings_USB_print){
				ssh1106_SetCursor(25, 30);
				s = ssh1106_WriteString("Print RSS", Font_6x8, White);
			}
			else{
				ssh1106_SetCursor(15, 30);
				s = ssh1106_WriteString("Print 32-bit keys", Font_6x8, White);
			}
			break;
	}
	OLED_print_status(settings_mode);
	ssh1106_UpdateScreen();
}

void OLED_init(void){
	ssh1106_Init();
}

void OLED_clear_screen(void){
	ssh1106_Fill(Black);
}

void OLED_update(void){
	ssh1106_UpdateScreen();
}

void OLED_shutdown(void){
	ssh1106_SetDisplayOn(0);
}

/* Debug functions -------------------------------------------------------------------*/
void OLED_print_text(char* str, uint8_t pos_x, uint8_t pos_y){
	ssh1106_SetCursor(pos_x,pos_y);
	s = ssh1106_WriteString(str, Font_6x8, White);
}

void OLED_print_variable(char * str, uint32_t value, uint8_t pos_x, uint8_t pos_y){
	char stringValue[10];
	sprintf(stringValue, "%u", value); // Copy value in string
	OLED_print_text(str, pos_x, pos_y);
	ssh1106_SetCursor(pos_x+(strlen(str)*6),pos_y);
	s = ssh1106_WriteString(stringValue, Font_6x8, White);
}

void OLED_print_hexadecimal(char * str, uint32_t value, uint8_t pos_x, uint8_t pos_y){
	char stringValue[10];
	sprintf(stringValue, "%02x", value); // Copy value in string
	OLED_print_text(str, pos_x, pos_y);
	ssh1106_SetCursor(pos_x+(strlen(str)*6),pos_y);
	s = ssh1106_WriteString(stringValue, Font_6x8, White);
}

void OLED_print_binary(char * str, uint8_t value, uint8_t pos_x, uint8_t pos_y){
	char stringValue[10];
	sprintf(stringValue, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(value)); // Copy value in string
	OLED_print_text(str, pos_x, pos_y);
	ssh1106_SetCursor(pos_x+(strlen(str)*6),pos_y);
	s = ssh1106_WriteString(stringValue, Font_6x8, White);
}



