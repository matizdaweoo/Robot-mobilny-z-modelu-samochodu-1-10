#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"

#include "allDefines.h"
#include "distance_measurement.h"
#include "gpio_config.h"
#include "dma_config.h"
#include "adc_config.h"
#include "bt_hc_05.h"
#include "delay.h"
#include "lcd_tft.h"
#include "tm_stm32f4_ili9341.h"

#include <stdio.h>												//sprintf
#include <stdlib.h>
#include <math.h>

//Zmienne dla czujnikow SHARP
	uint8_t SharpFrontDataReady					= 0;
	volatile float SharpFrontVoltage[5] 		= {0.0, 0.0, 0.0, 0.0, 0.0};
	volatile uint8_t SharpFrontDistFromObst[5] 	= {  0,   0,   0,   0,   0};
	uint8_t SharpReartDataReady 				= 0;
	volatile float SharpRearVoltage[5] 			= {0.0, 0.0, 0.0, 0.0, 0.0};
	volatile uint8_t SharpRearDistFromObst[5] 	= {  0,   0,   0,   0,   0};
	extern volatile uint16_t ADC_SharpFrontValues[ (SharpFront_NrSensors * SharpFront_NrSamples) ];
	extern volatile uint16_t ADC_SharpRearValues[ (SharpRear_NrSensors * SharpRear_NrSamples) ];

//Zmienne dla czujnikow ultradzwiekowych
	uint8_t USFrontDistFromObst = 0;
	uint8_t USRearDistFromObst = 0;

	float DistanceFromObstacle = -1.0;									//Zmienna przechowujaca odleglosc od przeszkody
	int DistanceMeasurement = 0;
	int DistanceMeasurementFlag = 0;								//Zmienna zezwalajaca na pomiar czasu trwania sygnalu z czujnika
	int FrontRear = 0;												//(0 - zaden czujnik, 1 - przedni, 2 - tylni)
	int TimeOutDistanceMeasurementFlag = 0;
	//int IloscProbekPomiaruDystansu = 18;
	//float FrontDistance[IloscProbekPomiaruDystansu] = {};

//Zmienne ogolne
	extern uint8_t EnableDisplayLCD;
	extern uint8_t BargraphValue;
//---------------------------------------------------------------------------------------------------------//

void BT_SendDistanceFromObstacle(void)						//Funkcja wysylajaca zmierzony dystans przez BT
{
	char dstanceForFrontSensors[16];											//Lokalna tablica znakow 16 elementowa
	char dstanceForRearSensors[16];
	char distBuff[2];

	if(BT_State_ReadInputDataBit == 1)							//Jesli polaczony z BT
	{
		dstanceForFrontSensors[0] = 'R'; dstanceForFrontSensors[1] = 'S'; dstanceForFrontSensors[2] = 'F';	dstanceForFrontSensors[15] = '\n'; //Naglowek na danych odleglosci z czujnikow przednich
		dstanceForRearSensors[0]  = 'R'; dstanceForRearSensors[1]  = 'S'; dstanceForRearSensors[2]  = 'R';	dstanceForRearSensors[15]  = '\n'; //Naglowek na danych odleglosci z czujnikow tylnich

		for(uint8_t i = 0; i < 5; i++)	//Ukladanie danych w paczke dla czujnikow SHARP
		{
		//SHARP Front
			if(SharpFrontDistFromObst[i] < 10)
			{
				dstanceForFrontSensors[3 + (i * 2)] = '0';
				dstanceForFrontSensors[4 + (i * 2)] = '0';
			}
			else
			{
				sprintf(distBuff, "%d", SharpFrontDistFromObst[i]);
				dstanceForFrontSensors[3 + (i * 2)] = distBuff[0];
				dstanceForFrontSensors[4 + (i * 2)] = distBuff[1];
			}
		//SHARP Rear
			if(SharpRearDistFromObst[i] < 10)
			{
				dstanceForRearSensors[3 + (i * 2)] = '0';
				dstanceForRearSensors[4 + (i * 2)] = '0';
			}
			else
			{
				sprintf(distBuff, "%d", SharpRearDistFromObst[i]);
				dstanceForRearSensors[3 + (i * 2)] = distBuff[0];
				dstanceForRearSensors[4 + (i * 2)] = distBuff[1];
			}
		}

		//Ultrasonic Front
			if(USFrontDistFromObst < 10) { dstanceForFrontSensors[13] = '0'; dstanceForFrontSensors[14] = '0'; }
			else { sprintf(distBuff, "%d", USFrontDistFromObst); dstanceForFrontSensors[13] = distBuff[0]; dstanceForFrontSensors[14] = distBuff[1]; }
		//Ultrasonic Rear
			if(USRearDistFromObst < 10) { dstanceForRearSensors[13] = '0'; dstanceForRearSensors[14] = '0'; }
			else { sprintf(distBuff, "%d", USRearDistFromObst); dstanceForRearSensors[13] = distBuff[0]; dstanceForRearSensors[14] = distBuff[1]; }

		BT_SendString(dstanceForFrontSensors, 16);	//koment bo tablice sa postawione w pamieci jedna za druga (najpierw Rear potem Front) i to leci jako ciag w pamieci do wyslania???
		BT_SendString(dstanceForRearSensors,  16);
	}
}

uint8_t SharpFront_ReadData(void)
{
	//uint8_t distanceOK = 0;
	float value = 0.0;

	for(uint8_t i = 0; i < SharpFront_NrSensors; i++)
	{
		value = SharpFrontVoltage[i];

		if((value >= 31.5) && (value <= 7.0))
		{
			value = 0;
		}
		else if((value < 31.5) && (value > 10.0))
		{
			value = round(544 * pow(value, -1.25));
		}
		else if((value <= 10) && (value > 7.65))
		{
			value = round(1.62 * pow(value, 2) - 37 * value + 238);
		}
		else if((value <= 7.65) && (value > 7.0))
		{
			value = round(20.74 * pow(value, 2) - 346 * value + 1484);
		}

		SharpFrontDistFromObst[i] = value;
	}
	return 1;
}

void DMA2_Stream1_IRQHandler(void)	//Sharp Front
{
	if(DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF2))
	{
		float voltageValue;

		for(uint8_t i = 0; i < SharpFront_NrSensors; i++)
		{
			voltageValue = (float)(ADC_SharpFrontValues[i] + ADC_SharpFrontValues[i + 5] + ADC_SharpFrontValues[i + 10] + ADC_SharpFrontValues[i + 15] + ADC_SharpFrontValues[i + 20]) / 5;
			SharpFrontVoltage[4 - i] = voltageValue / 4096 * 33;
		}
		SharpFrontDataReady = 1;

		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF2);
	}
}

void DMA2_Stream2_IRQHandler(void)	//Sharp Rear
{
	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF1))
	{
		for(uint8_t i = 0; i < SharpRear_NrSensors; i++)
		{
			SharpRearVoltage[i] = (float)(ADC_SharpRearValues[i] + ADC_SharpRearValues[i + 5] + ADC_SharpRearValues[i + 10] + ADC_SharpRearValues[i + 15] + ADC_SharpRearValues[i + 20]) / 5;
		}
		SharpReartDataReady = 1;

		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF1);
	}
}

void US_ReadDistanceFromObstacle(int a)							//Funkcja wyzwalajaca pomiar wybranego czujnika
{
	TimeOutDistanceMeasurementFlag = 0;
	FrontRear = a;
	if (FrontRear == 1) 										//Jesli wybralem przedni czujnik
	{
		HC_SR04_FRONT_TRIGGER_RESET;							//Dla pewnosci ustawiam stan niski
		delay_us(2);											//Czekam
		HC_SR04_FRONT_TRIGGER_SET;								//Wyzwalam pomiar odleglosci
		delay_us(10);											//Czekam
		HC_SR04_FRONT_TRIGGER_RESET;							//Resetuje pin i koncze wyzwolenie
	}

	if(FrontRear == 2)											//Jesli wybralem tylni czujnik
	{
		HC_SR04_REAR_TRIGGER_RESET;								//Dla pewnosci ustawiam stan niski
		delay_us(2);											//Czekam
		HC_SR04_REAR_TRIGGER_SET;								//Wyzwalam pomiar odleglosci
		delay_us(10);											//Czekam
		HC_SR04_REAR_TRIGGER_RESET;								//Resetuje pin i koncze wyzwolenie
	}

	DistanceMeasurement = 0;								//Zerowanie odleglosci
	DistanceFromObstacle = 0.0;
	DistanceMeasurementFlag = 1;							//Zezwolenie na pomiar czasu trwania sygnalu z czujnika

	while(DistanceMeasurementFlag == 1) { }					//Czekam az wykona sie pomiar odleglosci
}

void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6, TIM_IT_Update))
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

		TIM6 -> SR &= ~TIM_SR_UIF;						//Zerowanie flagi dla przerwania TIM6

		TimeOutDistanceMeasurementFlag = 1;				//Wystapienie timeout dla pomiaru odleglosci (dystans > 2m)

		TIM6 -> CR1 &= ~TIM_CR1_CEN;					//Counter disable
		TIM6 -> CNT = 0;								//Zeruje timer

		DistanceFromObstacle = -1.0;					//
		DistanceMeasurementFlag = 0;					//Blokowanie pomiaru czasu trwania sygnalu z czujnika
	}
}

void SharpSensor_Init(void)
{
		if(EnableDisplayLCD == 1) { LCD_ClearRow1(); TM_ILI9341_Puts(87, 105, "SHARP sensors", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
													 TM_ILI9341_Puts(131, 124, "front", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('d'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('a'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; LCD_WriteTheTypeOfPeriphery(0); }

		if(EnableDisplayLCD == 1) { TM_ILI9341_Puts(126, 124, " rear ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('d'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('a'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; LCD_ClearAllRow(); }

	SharpFront_GPIO_Init();
	SharpFront_DMA_Init();
	SharpFront_ADC_Init();

	SharpRear_GPIO_Init();
	SharpRear_DMA_Init();
	SharpRear_ADC_Init();
}

void UltrasonicSensors_Init(void)
{
	if(EnableDisplayLCD == 1) { LCD_ClearRow1(); TM_ILI9341_Puts(60, 105, "ultrasonic sensors", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }

	UltrasonicSensors_GPIO_Init();
	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }

	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('t'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; LCD_ClearAllRow(); }
	//Ultra_HC_SR04_Init();
	//Ultrasonic_Radar_Init();
	//Ultrasonic_Timer_Init();

	//while(1) {}
}
