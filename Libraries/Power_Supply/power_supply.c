#include "power_supply.h"
#include "delay.h"
#include "lcd_tft.h"
#include "tm_stm32f4_ili9341.h"

#include <stdlib.h>

extern uint8_t EnableDisplayLCD;
extern uint8_t BargraphValue;

void PowerSupply_Init(void)
{
	if(EnableDisplayLCD == 1) { LCD_ClearRow1(); TM_ILI9341_Puts(93, 105, "power supply", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }
	//konf power supply
	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('i'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('e'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; LCD_ClearAllRow(); }
}
