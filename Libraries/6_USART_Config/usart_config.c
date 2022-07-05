#include "stm32f4xx_usart.h"

#include "allDefines.h"
#include "usart_config.h"

/*--------------------	Start konfiguracja USART3 - Bluetooth --------------------*/
void BT_HC05_USART_Init(void)
{
	USART_InitTypeDef UART_InitStruct;	//Create structure for UART
	NVIC_InitTypeDef NVIC_InitStruct;	//Create structure for NVIC

	UART_InitStruct.USART_BaudRate = BT_baudrate;									//Predkosc przesylu
	UART_InitStruct.USART_WordLength = USART_WordLength_8b;						//Dlugosc danych 8bit
	UART_InitStruct.USART_StopBits = USART_StopBits_1;							//Jeden bit stopu
	UART_InitStruct.USART_Parity = USART_Parity_No;								//Bez bitu parystosci
	UART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//
	UART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;					//
	USART_Init(BT_UART, &UART_InitStruct);										//

	USART_ITConfig(BT_UART, USART_IT_RXNE, ENABLE);								//Enable interrupt UART4 from USART_IT_RXNE
	USART_Cmd(BT_UART, ENABLE);													//Enable peripheral UART4

	//Enable and set UART4 interrupt
	NVIC_InitStruct.NVIC_IRQChannel = BT_UART_IRQ;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*--------------------	Start konfiguracja USART6 - CMUCam5 Pixy --------------------*/
void CMUCamPixy_USART_Init(void)
{
	USART_InitTypeDef UART_InitStruct;	//Create structure for UART
	NVIC_InitTypeDef NVIC_InitStruct;	//Create structure for NVIC

	UART_InitStruct.USART_BaudRate = CMUCam_baudrate;									//Predkosc przesylu
	UART_InitStruct.USART_WordLength = USART_WordLength_8b;						//Dlugosc danych 8bit
	UART_InitStruct.USART_StopBits = USART_StopBits_1;							//Jeden bit stopu
	UART_InitStruct.USART_Parity = USART_Parity_No;								//Bez bitu parystosci
	UART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//
	UART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;					//
	USART_Init(CMUCam_UART, &UART_InitStruct);										//

	USART_ITConfig(CMUCam_UART, USART_IT_RXNE, ENABLE);								//Enable interrupt UART4 from USART_IT_RXNE
	USART_Cmd(CMUCam_UART, ENABLE);													//Enable peripheral UART4

	//Enable and set UART4 interrupt
	NVIC_InitStruct.NVIC_IRQChannel = CMUCam_UART_IRQ;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}
/*-------------------- Koniec konfiguracja --------------------*/
