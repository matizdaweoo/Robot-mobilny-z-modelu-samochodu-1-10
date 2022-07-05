#include "stm32f4xx_dma.h"

#include "allDefines.h"
#include "light_sensor.h"
#include "gpio_config.h"
#include "dma_config.h"
#include "adc_config.h"
#include "lighting.h"
#include "delay.h"
#include "lcd_tft.h"
#include "tm_stm32f4_ili9341.h"

#include <stdlib.h>

int CheckLightingFlag = 0;

extern volatile uint16_t ADC1_Values[6];
extern uint8_t EnableDisplayLCD;
extern uint8_t BargraphValue;

float ReadTempCPU(void)										//
{
	const float V25 = 760;
	const float avgSlope = 2500;
	float temp;

	//temp = ADC1_Values[1];
	temp = (ADC1_Values[3] + ADC1_Values[4] + ADC1_Values[5]) / 3;	//Obliczenie sredniej z trzech probek Temp[3, 4, 5]
	temp = (temp * 3300.0) / 4096.0;										//Przeliczenie wart z ADC na napiêcie
	return temp = ((temp - V25) / avgSlope) + 25.0;					//Przeliczenie na temp. Wzor znajduje sie w DataSheet
}

void LightSensor_ChoseLight(void)							//
{
	//int i = 0;	Light[1, 2, 3]
	int IloscPomiarowADC = 10;
	int AverangeValue = 0;
	int AverangeValue1 = 0;

	for(int i = 0; i < IloscPomiarowADC; i++)
	{
		ADC_SoftwareStartConv(ADC1);						//Wlacza programowy start konwersji dla wybranego kanalu ADC
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));		//Czeka az pomiar zostanie wykonany (flaga EOC(End Of Conversion) == 1 jesli pomiar gotowy)
		AverangeValue = ADC_GetConversionValue(ADC1);		//Returns the last ADCx conversion result data for regular channel.

		AverangeValue1 = AverangeValue1 + AverangeValue;	//Sumowanie 10 probek
	}
	AverangeValue = AverangeValue1 / IloscPomiarowADC;		//Srednia z zebranych probek

	if(AverangeValue > 800)									//Okreslam czy jest dzien czy noc
	{
		DaytimeRunningLamp_SET();
		FrontPositionMainLamps_RESET();
		RearLight_RESET;
	}
	else
	{
		DaytimeRunningLamp_RESET();
		FrontPositionLamps();
		RearLight_SET;
	}
}

void DMA2_Stream0_IRQHandler(void)
{	//Sprawdzam przerwania od poszczegolnego kanalu
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))			//Channel 0
	{
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
	}
}

void LightSensor_Init(void)									//
{
	if(EnableDisplayLCD == 1) { LCD_ClearRow1(); TM_ILI9341_Puts(93, 105, "light sensor", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }

	LightSensor_GPIO_Init();
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	LightSensor_ADC_Init();
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('a'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	LightSensor_DMA_Init();
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('d'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; LCD_ClearAllRow(); }


}
