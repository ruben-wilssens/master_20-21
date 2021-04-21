/*
 * OLED.c
 *
 *  Created on: April 2021
 *  Author: Ruben Wilssens & Victor Van der Elst
 */
#include "OLED.h"
#include "String.h"
#include "ssh1106_fonts.h"

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

/* Defines */
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
/* Functions -------------------------------------------------------------------*/
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

void OLED_print_text(char* str, uint8_t pos_x, uint8_t pos_y){
	ssh1106_SetCursor(pos_x,pos_y);
	s = ssh1106_WriteString(str, Font_6x8, White);
}

void OLED_print_variable(char * str, uint32_t value, uint8_t pos_x, uint8_t pos_y){
	char stringValue[10];
	// Copy value in string
	sprintf(stringValue, "%u", value);
	OLED_print_text(str, pos_x, pos_y);
	ssh1106_SetCursor(pos_x+(strlen(str)*6),pos_y);
	s = ssh1106_WriteString(stringValue, Font_6x8, White);
}

void OLED_print_hexadecimal(char * str, uint32_t value, uint8_t pos_x, uint8_t pos_y){
	char stringValue[10];
	// Copy value in string
	sprintf(stringValue, "%02x", value);
	OLED_print_text(str, pos_x, pos_y);
	ssh1106_SetCursor(pos_x+(strlen(str)*6),pos_y);
	s = ssh1106_WriteString(stringValue, Font_6x8, White);
}

void OLED_print_binary(char * str, uint8_t value, uint8_t pos_x, uint8_t pos_y){
	char stringValue[10];
	// Copy value in string
	sprintf(stringValue, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(value));
	OLED_print_text(str, pos_x, pos_y);
	ssh1106_SetCursor(pos_x+(strlen(str)*6),pos_y);
	s = ssh1106_WriteString(stringValue, Font_6x8, White);
}

void OLED_print_credits(void){
	OLED_clear_screen();
	//print state: (TIME     TX/RX      BAT_status)
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
	ssh1106_Line(0,0,128,0,Black);
	ssh1106_Line(0,1,128,1,Black);
	ssh1106_Line(0,2,128,2,Black);
	ssh1106_Line(0,3,128,3,Black);
	ssh1106_Line(0,4,128,4,Black);
	ssh1106_SetCursor(3, 0);
	s = ssh1106_WriteString("12:00", Font_6x8, White); //change with actual time
	ssh1106_SetCursor(63, 0);
	s = ssh1106_WriteChar(status, Font_6x8, White);
	ssh1106_SetCursor(92, 0);
	s = ssh1106_WriteString("BAT_ok", Font_6x8, White); //change with variable
}
