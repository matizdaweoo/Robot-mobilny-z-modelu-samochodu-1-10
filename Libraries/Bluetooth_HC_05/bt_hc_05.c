#include <stdint.h>
#include <stdlib.h>

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_exti.h"

#include "allDefines.h"
#include "bt_hc_05.h"
#include "gpio_config.h"
#include "usart_config.h"
#include "lcd_tft.h"
#include "service_messages.h"
#include "delay.h"
#include "tm_stm32f4_ili9341.h"




volatile char BT_UART_RxBuf[BT_UART_RX_BUF_SIZE];		//Definiuje bufor odbiorczy
volatile uint8_t BT_UART_RxHead = 0;					//Deklaruje poczatek buforu odbiorczego (tzw. glowe weza)
volatile uint8_t BT_UART_RxTail = 0;					//Deklaruje koniec buforu odbiorczego (tzw. koniec weaz)	Head i Tail mog¹ byc rowne na poczatku bo bufor jest pusty.

extern uint8_t EnableDisplayLCD;
extern uint8_t BargraphValue;

void BT_SendString(volatile char* s, uint8_t len)						//Bluetooth send function
{
	while (USART_GetFlagStatus(BT_UART, USART_FLAG_TXE) == RESET);	//Sprawdzam czy bufor do wyslania jest wolny, jak nie to czekam
	USART_SendData(BT_UART, '!');
	for(uint8_t i = 0; i < len; i++)
	{
	//while(*s)															//Dopuki cos jest do wyslania
	//{
		while (USART_GetFlagStatus(BT_UART, USART_FLAG_TXE) == RESET);	//Sprawdzam czy bufor do wyslania jest wolny, jak nie to czekam
		USART_SendData(BT_UART, *s);									//Wysylam kolejno znak ze zdania
		(char)*s++;														//Zwiekszanie wskaznika na znak ((char) podkreslam doklanie jakiego tupu jest wskaznik)
	}
}


//void CMUCam_SendBuf(uint8_t *buf, uint8_t len)						//Funkcja wysylajaca dane do CMUCam
//{
	//for(uint8_t i = 0; i < len; i++)
	//{
	//	uint8_t data = *buf++;	//Zwiekszam wartosc wskaznika na ktora wskazuje wskaznik

	//	while (USART_GetFlagStatus(CMUCam_UART, USART_FLAG_TXE) == RESET);	//Sprawdzam czy bufor do wyslania jest wolny, jak nie to czekam
	//	USART_SendData(CMUCam_UART, data);									//Wysylam kolejno znak z tablicy
	//}

void USART3_IRQHandler(void)									//Handler for function UART3
{
	if(USART_GetITStatus(BT_UART, USART_IT_RXNE))				//sprawdza czy zdarzenie bylo na lini RX UARTa
	{
		uint8_t tmpHead = 0;
		char receiveData = USART_ReceiveData(BT_UART);			//Pobranie danych z UART do zmiennej

		tmpHead = (BT_UART_RxHead + 1) & BT_UART_RX_BUF_MASK;	//Obliczam nowy indeks dla 'glowy weza' (np. zapetlenie (31+1)&(31)=0 -> 0b00100000 & 0b00011111 = 0b00000000)

		if(tmpHead == BT_UART_RxTail)							//Jesli 'waz' zjada wlasny ogon, czyli zaczynamy gubic dane
		{

		}
		else													//Jesli bufor jest jeszcze wony
		{
			BT_UART_RxHead = tmpHead;							//Aktualizuje poczatek bufora
			BT_UART_RxBuf[tmpHead] = receiveData;				//Zapisuje nowe dane do bufora
		}															//Gdzies w programie glownym musze odczytywac bufor cyklcznie!!!

		USART_ClearITPendingBit(BT_UART, USART_IT_RXNE);		//Czysci flage zgloszenia przerwania na lini RX UARTa
	}
}

int8_t BT_ReadBuffer(void)
{
	char tabData[10];	//Tablica przechowujaca odebrane znaki
	char data;			//Zmienna przechowujaca aktuallnie odebrany znak
	int a = 0;

	while(1)	//Petle sie zeby odczytac caly bufor i wykonac instrukcje
	{
		if(BT_UART_RxHead != BT_UART_RxTail)	//Jesli sa dane w buforze do odczytania
		{
			BT_UART_RxTail = (BT_UART_RxTail + 1) & BT_UART_RX_BUF_MASK;	//Przemieszczam sie po buforze

			data = BT_UART_RxBuf[BT_UART_RxTail];	//Odczyt danej

			if(data != '\n')	//Odczytuje znaki rozkazu
			{
				tabData[a] = data;						//Przypis do tablicy
				++a;
			}
			else				//Jesli pojawil sie koniec lini
			{
				//przekazuje tab do funkcji realizujacej rozkaz
				ObslugaKomunikatow(tabData, (a+1));
				return 0;
			}
		}
		else									//Jesli bufor jest pusty
		{
			return -1;
		}
	}
}

void BT_HC05_Init(void)
{
	if(EnableDisplayLCD == 1)
	{
		LCD_ClearRow1();
		TM_ILI9341_Puts(110, 105, "bluetooth", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	}

	BT_HC05_GPIO_Init();
	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }

	BT_HC05_USART_Init();
	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('u'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; LCD_ClearAllRow(); }
}
