#include "stm32f4xx_adc.h"

#include "allDefines.h"
#include "adc_config.h"

#include "tm_stm32f4_ili9341.h"

volatile uint16_t ADC_SharpFrontValues[ (SharpFront_NrSensors * SharpFront_NrSamples) ];
volatile uint16_t ADC_SharpRearValues[ (SharpRear_NrSensors * SharpRear_NrSamples) ];
volatile uint16_t ADC1_Values[6];

extern uint8_t  EnableDisplayLCD;

void ADCCommon_Init(void)
{
	ADC_CommonInitTypeDef ADC_CommonInitStruct;

	ADC_DeInit();																		//Przywracanie wszystkich przetwornikow ADC do stanu poczatkowego

	//ADC_CommonInitStruct.ADC_DMAAccessMode		= ADC_DMAAccessMode_1;			//tez
	ADC_CommonInitStruct.ADC_Mode				= ADC_Mode_Independent;					//select continuous conversion mode
	ADC_CommonInitStruct.ADC_Prescaler			= ADC_Prescaler_Div6;					//All ADC use clock from APB2 (84MHz), max f for ADC i 14MHz! [84MHz / 6 = 14MHz is OK]
	//ADC_CommonInitStruct.ADC_TwoSamplingDelay	= ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStruct);

	/*if(EnableDisplayLCD == 1)
	{
		TM_ILI9341_Puts(50, 124, "                    ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		TM_ILI9341_Puts(143, 124, "ADC", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		TM_ILI9341_DrawFilledRectangle(199, 185, 219, 215, ILI9341_COLOR_BROWN);
		delay_ms(100+random()%1400);		//Czekanie o wylosowana wartosc z przedzialu od 100 do 1500
	}*/
}

void SharpFront_ADC_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure;			//ADC3 configuration

	ADC_InitStructure.ADC_DataAlign				= ADC_DataAlign_Right;				//
	ADC_InitStructure.ADC_Resolution			= ADC_Resolution_12b;				//
	ADC_InitStructure.ADC_ContinuousConvMode	= ENABLE;							//
	ADC_InitStructure.ADC_ScanConvMode			= ENABLE;							//
	ADC_InitStructure.ADC_NbrOfConversion		= 5;								//
	//ADC_InitStructure.ADC_ExternalTrigConv		= ADC_ExternalTrigConv_T5_CC1;		//
	ADC_InitStructure.ADC_ExternalTrigConvEdge	= ADC_ExternalTrigConvEdge_None;
	ADC_Init(ADC3, &ADC_InitStructure);												//
//Konfiguracja kanalow ADC3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_28Cycles);		//configure each channel
	ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 2, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_0 , 5, ADC_SampleTime_28Cycles);

	ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
	ADC_DMACmd(ADC3, ENABLE);														//enable DMA for ADC3
	ADC_Cmd(ADC3, ENABLE);															//Enable ADC3

	/*ADC_ResetCalibration(ADC3);														//Enable ADC3 reset calibration register
		while(ADC_GetResetCalibrationStatus(ADC3));									//Check the end of ADC3 reset calibration register
	ADC_StartCalibration(ADC3);														//Start ADC3 calibration
		while(ADC_GetCalibrationStatus(ADC3));*/										//Check the end of ADC3 calibration

	ADC_SoftwareStartConv(ADC3);
}

void SharpRear_ADC_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure;			//ADC2 configuration

	ADC_InitStructure.ADC_DataAlign				= ADC_DataAlign_Right;				//right 12-bit data alignment in ADC data register
	ADC_InitStructure.ADC_Resolution			= ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode	= ENABLE;							//we will convert one time
	ADC_InitStructure.ADC_ScanConvMode			= ENABLE;							//We will convert multiple channels
	ADC_InitStructure.ADC_NbrOfConversion		= 5;								//8 channels conversion
	//ADC_InitStructure.ADC_ExternalTrigConv		= ADC_ExternalTrigConv_T5_CC1;		//select no external triggering
	ADC_InitStructure.ADC_ExternalTrigConvEdge	= ADC_ExternalTrigConvEdge_None;
	ADC_Init(ADC2, &ADC_InitStructure);												//load structure values to control and status registers
//Konfiguracja kanalow ADC2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_28Cycles);		//configure each channel
	ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 2, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 3, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 4, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 5, ADC_SampleTime_28Cycles);

	ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
	ADC_DMACmd(ADC2, ENABLE);														//enable DMA for ADC3
	ADC_Cmd(ADC2, ENABLE);															//Enable ADC3

	/*ADC_ResetCalibration(ADC2);														//Enable ADC3 reset calibration register
		while(ADC_GetResetCalibrationStatus(ADC2));									//Check the end of ADC3 reset calibration register
	ADC_StartCalibration(ADC2);														//Start ADC3 calibration
		while(ADC_GetCalibrationStatus(ADC2));	*/									//Check the end of ADC3 calibration

	ADC_SoftwareStartConv(ADC2);
}

void LightSensor_ADC_Init(void)
{
	ADC_InitTypeDef ADC_InitStruct;				//ADC1 configuration

	ADC_InitStruct.ADC_DataAlign 				= ADC_DataAlign_Right;				//Dane wyrownane do prawej (12-bit z ADC zapisywane do 16-bit i trzeba to wyrownac)
	ADC_InitStruct.ADC_Resolution				= ADC_Resolution_12b; 				//Rozdzielczosc przetwornika (12-bit, 0 - 4096)
	ADC_InitStruct.ADC_ContinuousConvMode 		= ENABLE;							//Tryb konwersji ciaglej, czyli ADC mierzy zdefiniowane kanaly w kolko
	ADC_InitStruct.ADC_ScanConvMode 			= ENABLE;							//Aktwaja pomiarow z wybranych kanalow jeden za drugim
	ADC_InitStruct.ADC_NbrOfConversion 			= 2;								//Ilosc konwersji = ilosc zdefiniowanych kanalow
	//ADC_InitStruct.ADC_ExternalTrigConv 		= ADC_ExternalTrigConv_T5_CC1;		//Wybor przerwania aktywujace konwersje (ja startuje programowo wiec tego nie potrzebuje)
	ADC_InitStruct.ADC_ExternalTrigConvEdge 	= ADC_ExternalTrigConvEdge_None;	//Rozpoczynam konwersje programowo wiec zadne przerwanie wyzwalajace mnie nie interere
	ADC_Init(ADC1, &ADC_InitStruct);												//Przekazuje strukture do funkcji i inicjalizuje ADC1
//Konfiguracja kanalow ADC1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9,		   1, ADC_SampleTime_144Cycles); //Kanal czujnika oswietlenia, temperatura CPU
	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 2, ADC_SampleTime_144Cycles);//ADC_RegularChannelConfig(Przetwornik ADC(1, 2, 3), nr kanalu, pozycja w kolejce pomiarow, czas konwersji w cyklach zagara);

	ADC_TempSensorVrefintCmd(ENABLE);												//W³¹czam czujnik temperatury
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);								//Aktywuje zadanie z ADC do DMA po ostatnim transferze danych (czyli jak zapelni byfor w pamieci to zglasza)
	ADC_DMACmd(ADC1, ENABLE);														//Uruchomienie kanalu DMA dla ADC1
	ADC_Cmd(ADC1, ENABLE);															//Uruchomienie przetwornika ADC1

	/*ADC_ResetCalibration(ADC1);
		while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
		while (ADC_GetCalibrationStatus(ADC1)); */

	ADC_SoftwareStartConv(ADC1);
}


