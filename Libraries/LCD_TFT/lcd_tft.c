#include "tm_stm32f4_ili9341.h"
#include "tm_stm32f4_fonts.h"

#include "allDefines.h"
#include "lcd_tft.h"
#include "lighting.h"
#include "gpio_config.h"
#include "tim_config.h"
#include "delay.h"

extern uint8_t EnableDisplayLCD;
extern uint8_t BargraphValue;

void DisplayStartPage(void)
{					//PWM dla podswietlenie zak³uca transmisje SPI !!!!!!!!!!!!!!!!!!
	//int a = 0;
/*
	TM_ILI9341_Fill(ILI9341_COLOR_BLACK);	//FIll lcd with color
	TM_ILI9341_Puts(56, 30, "P O R S C H E", &TM_Font_16x26, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	// (x y) Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_Puts(40, 90, "P A N A M E R A", &TM_Font_16x26, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_Puts(94, 150, "Mobile Robot", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_Puts(166, 230, "Autor: Mateusz Mydlarz", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font
*/
	TM_ILI9341_Fill(ILI9341_COLOR_BLACK);	//FIll lcd with color
	TM_ILI9341_Puts(24, 0, "Praca inzynierska", &TM_Font_16x26, ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_DrawLine(0, 29, 319, 29, ILI9341_COLOR_WHITE);
	TM_ILI9341_Puts(127, 33, "Temat:", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_Puts(22, 51, "SONAROWY SYSTEM NAWIGACJI", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	// (x y) Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_Puts(50, 69, "DLA ROBOTA MOBILNEGO", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_Puts(28, 87, "ZE STEROWANIEM ACKERMANA", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_DrawLine(0, 111, 319, 111, ILI9341_COLOR_WHITE);
	TM_ILI9341_Puts(110, 115, "Promotor:", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_Puts(22, 133, "dr hab. inz. Maciej Patan", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_DrawLine(0, 155, 319, 155, ILI9341_COLOR_WHITE);
	TM_ILI9341_Puts(127, 159, "Autor:", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_Puts(77, 177, "Mateusz Mydlarz", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_DrawLine(0, 199, 319, 199, ILI9341_COLOR_WHITE);
	TM_ILI9341_Puts(110, 203, "Kierunek:", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_Puts(44, 221, "Automatyka i Robotyka", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	//Put string with white foreground color and black background with 16x26px font

	LedLCD_SET;		//Wl podswietlenia
	//for (a = 0; a < 400; ++a)
	//{
	//	TIM_SetCompare1(LedLCD_TIM, a);
	//	delay_ms(1);
	//}

	//delay_ms(10000);

	//TIM_SetCompare1(LedLCD_TIM, 0);
}

void DisplayLoadSystemPage(void)
{
	TM_ILI9341_Fill(ILI9341_COLOR_BLACK);	//FIll lcd with color
	TM_ILI9341_Puts(48, 20, "Loading system", &TM_Font_16x26, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);	// (x y) Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_DrawLine(0, 65, 319, 65, ILI9341_COLOR_WHITE);
	TM_ILI9341_DrawLine(0, 66, 319, 66, ILI9341_COLOR_WHITE);
	TM_ILI9341_Puts(99, 76, "Initialize:", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	TM_ILI9341_DrawRectangle( 5, 179, 313, 219, ILI9341_COLOR_WHITE);
	TM_ILI9341_DrawRectangle( 4, 178, 314, 220, ILI9341_COLOR_WHITE);

	TM_ILI9341_Puts(143, 230, "Author", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);//Put string with white foreground color and black background with 16x26px font
	TM_ILI9341_Puts(182, 230, ":inz", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	TM_ILI9341_DrawPixel(206, 230, ILI9341_COLOR_WHITE);
	TM_ILI9341_DrawPixel(212, 237, ILI9341_COLOR_WHITE);
	TM_ILI9341_Puts(215, 230, "Mateusz Mydlarz", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

	//delay_ms(100+random()%1000);		//Czekanie o wylosowana wartosc z przedzialu od 100 do 1500

	LedLCD_SET;		//Wl podswietlenia

	delay_ms(2000);
	LCD_ClearRow1();
	TM_ILI9341_Puts(127, 105, "System", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	LCD_DrawBargraph(BargraphValue); ++BargraphValue;

	delay_ms(200);
}

void DisplayMainPage(void)
{
	TM_ILI9341_Fill(ILI9341_COLOR_BLACK);	//FIll lcd with color
	TM_ILI9341_Puts(0,1, "Mobile Robot", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	//Battery symbol
	TM_ILI9341_DrawRectangle(292, 0, 320, 10, ILI9341_COLOR_WHITE);				//Draw blue rectangle
	TM_ILI9341_DrawFilledRectangle(288, 2, 291, 8, ILI9341_COLOR_WHITE);		//Draw black filled rectangle
		TM_ILI9341_DrawFilledRectangle(294, 2, 297, 8, ILI9341_COLOR_BROWN);	//Draw black filled rectangle
		TM_ILI9341_DrawFilledRectangle(299, 2, 302, 8, ILI9341_COLOR_BROWN);	//Draw black filled rectangle
		TM_ILI9341_DrawFilledRectangle(304, 2, 307, 8, ILI9341_COLOR_BROWN);	//Draw black filled rectangle
		TM_ILI9341_DrawFilledRectangle(309, 2, 312, 8, ILI9341_COLOR_BROWN);	//Draw black filled rectangle
		TM_ILI9341_DrawFilledRectangle(314, 2, 317, 8, ILI9341_COLOR_BROWN);	//Draw black filled rectangle

	TM_ILI9341_Puts(246,1, "100", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	TM_ILI9341_Puts(267,1, " % ", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

	TM_ILI9341_DrawLine(0, 12, 320, 12, ILI9341_COLOR_WHITE);	//Draw line with custom color white
}

void LCD_DrawBargraph(int8_t a)
{
	TM_ILI9341_DrawFilledRectangle((10 + (a * 9)), 184, (10 + 9 + (a * 9)), 214, ILI9341_COLOR_BROWN);
	delay_ms(200);
}

void LCD_WriteTheTypeOfPeriphery(char a)
{
	switch(a)
	{
	case 'g':
		TM_ILI9341_Puts(126, 145, "(GPIO)", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		break;
	case 'e':
		TM_ILI9341_Puts(126, 145, "(EXTI)", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		break;
	case 't':
		TM_ILI9341_Puts(121, 145, " (TIM) ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		break;
	case 'u':
		TM_ILI9341_Puts(126, 145, "(UART)", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		break;
	case 'i':
		TM_ILI9341_Puts(121, 145, " (I2C) ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		break;
	case 's':
		TM_ILI9341_Puts(121, 145, " (SPI) ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		break;
	case 'a':
		TM_ILI9341_Puts(121, 145, " (ADC) ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		break;
	case 'd':
		TM_ILI9341_Puts(121, 145, " (DMA) ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		break;
	case '0':
		TM_ILI9341_Puts(121, 145, "       ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		break;
	}
}

void LCD_ClearRow1(void)
{
	TM_ILI9341_Puts(50, 105, "                    ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
}

void LCD_ClearRow2(void)
{
	TM_ILI9341_Puts(50, 124, "                    ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
}

void LCD_ClearAllRow(void)
{
	TM_ILI9341_Puts(50, 145, "                    ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	TM_ILI9341_Puts(50, 124, "                    ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	TM_ILI9341_Puts(50, 105, "                    ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
}

void LCD_Init(void)
{
	LedLCD_GPIO_Init();
	//LedLCD_TIM_Init();	//Tim jak uzywane bedzie PWM (trzeba tam odkomentowac w TIM i zmienic w GPIO)

	TM_ILI9341_Init();			//Initialize ILI9341
		TM_ILI9341_Rotate(TM_ILI9341_Orientation_Landscape_1);	//Rotate LCD for 90 degrees

	if(EnableDisplayLCD == 1)
	{
		//DisplayStartPage();			//Start page
		DisplayLoadSystemPage();
	}
}
