#include "stm32f4xx_dma.h"

#include "allDefines.h"
#include "dma_config.h"

extern volatile uint16_t ADC_SharpFrontValues[ (SharpFront_NrSensors * SharpFront_NrSamples) ];
extern volatile uint16_t ADC_SharpRearValues[ (SharpRear_NrSensors * SharpRear_NrSamples) ];
extern volatile uint16_t ADC1_Values[6];

void SharpFront_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;

	DMA_InitStructure.DMA_Channel 				= DMA_Channel_2;

	DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t)&ADC3 -> DR;
	DMA_InitStructure.DMA_PeripheralInc			= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize	= DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralBurst		= DMA_PeripheralBurst_Single;

	DMA_InitStructure.DMA_Memory0BaseAddr		= (uint32_t)ADC_SharpFrontValues;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;

	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize 			= SharpFront_AllSamples;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;

	DMA_InitStructure.DMA_Priority 				= DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_HalfFull;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream1, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void SharpRear_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;

	DMA_InitStructure.DMA_Channel 				= DMA_Channel_1;

	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&ADC2 -> DR;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;

	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)ADC_SharpRearValues;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;

	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize 			= SharpRear_AllSamples;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;

	DMA_InitStructure.DMA_Priority 				= DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_HalfFull;
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream2, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void LightSensor_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;

	DMA_InitStructure.DMA_Channel 				= DMA_Channel_0;

	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&ADC1 -> DR;			//Adres rejestu zrodlowego z ktorego bedzie kopiowane
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;		//Adres ADC1_DR jest jeden to nie trzeba inkrementowac adresu zrodlowego
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_HalfWord;	//Okreslami wielkosc bajtu rejestru zrodlowego na 16-bitow
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;

	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)ADC1_Values;			//Adres rejestu docelowego do ktorego bedzie kopiowane
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;				//Odczytujemy 2 pomiary dlatego zwiekszam adres po kazdym zapisie (jak bedzie wylaczone to DMA bedzie nadpisywal dane)
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_HalfWord;		//Okreslami wielkosc bajtu rejestru docelowego na 16-bitow
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;

	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralToMemory;		//Ustawienie kierunku przesylo skad do kad
	DMA_InitStructure.DMA_BufferSize 			= 6;								//Ustawiam liczbe danych do przeslania (dwa bajty 16-bitowe)
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;				//Po zakonczeniu transmisji DMA zacznie transmitowac od poczatku

	DMA_InitStructure.DMA_Priority 				= DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_HalfFull;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}
