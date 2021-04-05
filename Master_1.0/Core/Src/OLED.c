/*
 * OLED.c
 *
 *  Created on: Mar 15, 2020
 *      Author: jelle
 */
#include "OLED.h"

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

/* Variables -------------------------------------------------------------------*/

/* Functions -------------------------------------------------------------------*/
void OLED_init(void){
	SSD1306_Init();
}

void OLED_clear_screen(void){
	SSD1306_Fill(SSD1306_COLOR_BLACK);
}

void OLED_update(void){
	SSD1306_UpdateScreen();
}

void OLED_shutdown(void){
	SSD1306_OFF();
}

void OLED_print_text(char command[], uint8_t x, uint8_t y){
	SSD1306_GotoXY(x,y);
	SSD1306_Puts(command, &Font_7x10, SSD1306_COLOR_WHITE);
}

void OLED_print_variable(char command[], uint32_t value, uint8_t x, uint8_t y){
	char stringValue[10];
	sprintf(stringValue, "%d", value);
	SSD1306_GotoXY(x,y);
	SSD1306_Puts(command, &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_GotoXY(x+(strlen(command)*7),y);
	SSD1306_Puts(stringValue, &Font_7x10, SSD1306_COLOR_WHITE);
}

void OLED_print_date_and_time(void){
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	uint8_t hours = sTime.Hours;
	uint8_t minutes = sTime.Minutes;
	uint8_t seconds = sTime.Seconds;

	char stringHours[10];
	char stringMinutes[10];
	char stringSeconds[10];

	sprintf(stringHours, "%d", hours);
	sprintf(stringMinutes, "%d", minutes);
	sprintf(stringSeconds, "%d", seconds);

	SSD1306_GotoXY(92,0);

	if (hours < 10){
		SSD1306_Puts("0", &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(99,0);
		SSD1306_Puts(stringHours, &Font_7x10, SSD1306_COLOR_WHITE);
	}
	else
		SSD1306_Puts(stringHours, &Font_7x10, SSD1306_COLOR_WHITE);

	SSD1306_GotoXY(106,0);
	SSD1306_Puts(":", &Font_7x10, SSD1306_COLOR_WHITE);

	SSD1306_GotoXY(113,0);

	if (minutes < 10)	{
		SSD1306_Puts("0", &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(120,0);
		SSD1306_Puts(stringMinutes, &Font_7x10, SSD1306_COLOR_WHITE);
	}
	else
		SSD1306_Puts(stringMinutes, &Font_7x10, SSD1306_COLOR_WHITE);

}

void OLED_print_hexadecimal(char command[], uint32_t value, uint8_t x, uint8_t y){
	char stringValue[10];
	sprintf(stringValue, "%02X", value);
	SSD1306_GotoXY(x,y);
	SSD1306_Puts(command, &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_GotoXY(x+(strlen(command)*7),y);
	SSD1306_Puts(stringValue, &Font_7x10, SSD1306_COLOR_WHITE);
}

void OLED_print_talk(void){
	SSD1306_GotoXY(49,53);
	SSD1306_Puts("TALK", &Font_7x10, SSD1306_COLOR_BLACK);
}

void OLED_print_stoptalk(void){
	SSD1306_GotoXY(49,53);
	SSD1306_Puts("STOP", &Font_7x10, SSD1306_COLOR_BLACK);
}

void OLED_print_volume(uint8_t value){
//	uint8_t value = volume / 10;
	char stringValue[10];
	sprintf(stringValue, "%d", value);
	SSD1306_GotoXY(27,0);
	SSD1306_Puts("Thres:", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_GotoXY(27+(7*7),0);
	SSD1306_Puts(stringValue, &Font_7x10, SSD1306_COLOR_WHITE);
}
