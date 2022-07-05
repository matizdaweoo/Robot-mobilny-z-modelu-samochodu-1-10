#include "stm32f4xx_usart.h"

#include "cmucam_pantilt.h"
#include "allDefines.h"
#include "gpio_config.h"
#include "tim_config.h"
#include "usart_config.h"
#include "delay.h"
#include "lcd_tft.h"
#include "tm_stm32f4_ili9341.h"

#include <stdio.h>		//sprintf
#include <math.h>		//round

volatile char CMUCam_UART_RxBuf[CMUCam_UART_RX_BUF_SIZE];	//Definiuje bufor odbiorczy
volatile uint8_t CMUCam_UART_RxHead = 0;					//Deklaruje poczatek buforu odbiorczego (tzw. glowe weza)
volatile uint8_t CMUCam_UART_RxTail = 0;					//Deklaruje koniec buforu odbiorczego (tzw. koniec weaz)	Head i Tail mog¹ byc rowne na poczatku bo bufor jest pusty.

uint8_t CMUCamEnable = 0;									//Zezwolenie na prace CMUCam
uint8_t CMUCamObjectIsFind = 0;								//Okreslenie czy obiekt zostal znaleziony

uint8_t CMUCamDataReady = 0;								//Okrsla czy dane sa juz gotowe (pomocne w okresleniu czy obiekt jest na obrazie czy nie)
uint8_t CMUCamIloscOdebranychBajtow = 0;					//W sumie waznych bajtow do odebrania jest 12
uint8_t CMUCamFlagaSynchro = 0;								//Flaga okreslajaca poprawnosc sekwencji synchro
uint8_t CMUCamFlagaOdczytu = 0;								//Flago okreslajaca ktory bajt zostal odebrany (bo odbieram dwa bajty i potem lacze w jedno)
uint8_t HighByte = 0;										//Starszy bajt
uint8_t LowByte = 0;										//Mlodszy bajt

uint8_t CMUCamFindDirectionMotion = 0;						//0-ruch w lewo, 1-ruch w prawo

uint8_t CMUCamDeadZonePxX = 8;											//Ustawienie strefy nieczylosci dla sterowania serwami (wart w stopniach!)
uint8_t CMUCamDeadZonePxY = 5;

int8_t CMUCamPositionStandX = 0;											//Pozycji serwa (wart w stopniach!)
int8_t CMUCamPositionStandY = 0;

int16_t CMUCamPositionObjectX = 0;										//Pozycja obiektu na obrazie (wart w stopniach!)
int16_t CMUCamPositionObjectY = 0;

volatile uint16_t CMUCamtabX[5] = {159, 159, 159, 159, 159};							//Tablica wartosci wsp X dla filtru medianowego
volatile uint16_t CMUCamtabY[5] = {99, 99, 99, 99, 99};								//Tablica wartosci wsp X dla filtru medianowego

uint8_t CMUCamIsObject = 0;
uint8_t CMUCamObjectRunLeft = 0;
uint8_t CMUCamObjectRunRight = 0;

uint8_t helpFlagCMUCam1 = 0;
uint8_t helpFlagCMUCam2 = 0;

uint8_t probkaRozmiaruObiektu = 0;
uint16_t sizeOfTheObjectTab[5] = {0, 0, 0, 0, 0};
uint16_t sizeOfTheObject = 0;
uint8_t CMUCamFlagaTestKamery = 0;

extern uint8_t EnableDisplayLCD;
extern uint8_t BargraphValue;

struct														//Struktura przechowujaca dane o obiekcie na obrazie
{
  uint16_t checksum;
  uint16_t signature;
  uint16_t x;
  uint16_t y;
  uint16_t width;
  uint16_t height;
} CMUCamBlock;

void CMUCam_FindObject(void)
{
	if(CMUCamFindDirectionMotion == 0)			//Wykonywanie ruchu statywu w lewo
	{
		CMUCamPositionStandX += 1;				//Obrot o 1' w lewo
		if(CMUCamPositionStandX >= 90)			//Przelaczenie obrotu z lewo na prawo
		{
			CMUCamFindDirectionMotion = 1;		//Polecenie wykonywania obrotu w prawo
		}
	}
	else if(CMUCamFindDirectionMotion == 1)		//Wykonywanie ruchu statywu w prawo
	{
		CMUCamPositionStandX -= 1;				//Obrot o 1' w prawo
		if(CMUCamPositionStandX <= -90)			//Przelaczenie obrotu z prawo na lewo
		{
			CMUCamFindDirectionMotion = 0;		//Polecenie wykonywania obrotu w lewo
		}
	}
	CMUCam_Pan_SetDegree(CMUCamPositionStandX);	//Wysterowanie obrotu statywu

	if(CMUCamPositionStandY > 0)	//Jesli przechyl nie jest w pozycji zerowej to niech wraca
	{
		CMUCamPositionStandY -= 1;
		CMUCam_Tilt_SetDegree(CMUCamPositionStandY);
	}
}

void CMUCam_TrackObject(void)
{
	if(CMUCamDataReady == 1)	//Jesli dane z CMAUCam s¹ gotowe
	{
		uint16_t NewX = 0, NewY = 0;				//Zmienna przechowujaca wart srednia z 3 wart srodkowych z tab
		//Aktualizacja tablicy z wart X i Y
		CMUCamtabX[4] = CMUCamtabX[3]; CMUCamtabX[3] = CMUCamtabX[2]; CMUCamtabX[2] = CMUCamtabX[1]; CMUCamtabX[1] = CMUCamtabX[0]; CMUCamtabX[0] = CMUCamBlock.x;
		CMUCamtabY[4] = CMUCamtabY[3]; CMUCamtabY[3] = CMUCamtabY[2]; CMUCamtabY[2] = CMUCamtabY[1]; CMUCamtabY[1] = CMUCamtabY[0]; CMUCamtabY[0] = CMUCamBlock.y;
		//Filtrowanie danych
		NewX = MedianAveragingFilter(CMUCamtabX[0], CMUCamtabX[1], CMUCamtabX[2], CMUCamtabX[3], CMUCamtabX[4]);
		NewY = MedianAveragingFilter(CMUCamtabY[0], CMUCamtabY[1], CMUCamtabY[2], CMUCamtabY[3], CMUCamtabY[4]);
		//Przeliczenie wsp px na stopnie dla kazdej osi (Os X -> 320px/74'=4,32[px/'])	(Os Y -> 200px/46'=4,34[px/'])
		CMUCamPositionObjectX = round(-0.2194 * (float)NewX + 34.963 );	//f liniowa z danych X (319px->-35', 159px->0, 0px-35')
		CMUCamPositionObjectY = round(-0.2312 * (float)NewY + 22.961 ); //f liniowa z danych Y (199px->-23',  99px->0, 0px-23')


		sizeOfTheObjectTab[4] = sizeOfTheObjectTab[3];
		sizeOfTheObjectTab[3] = sizeOfTheObjectTab[2];
		sizeOfTheObjectTab[2] = sizeOfTheObjectTab[1];
		sizeOfTheObjectTab[1] = sizeOfTheObjectTab[0];
		sizeOfTheObjectTab[0] = CMUCamBlock.width * CMUCamBlock.height;

		for(uint8_t i = 0; i < 5; i++) { sizeOfTheObject += sizeOfTheObjectTab[i]; }
		sizeOfTheObject /= 5;

			//Obliczan rozmiar powierzchni obiektu na obrazie

		CMUCamIsObject = 1;		//Obiekt znajduje sie na obrazie
		CMUCamDataReady = 0;	//Blokuje przyjscie nastepnych danych zeby obrobic aktualne
		helpFlagCMUCam1 = 0;	//Zmienna pomocnicza
	}
	else if(CMUCamDataReady == 0)
	{
		++helpFlagCMUCam1;
		if(helpFlagCMUCam1 >= 1)	//Bo petla spr jest co 10[ms] a dane przychodza co 20[ms] i zerowanko musi poczekac 2 petle
		{
			helpFlagCMUCam1 = 0;
			CMUCamIsObject = 0;  //Nie ma obiektu
		}
	}

//--------------- Okreslenie gdzie jest obiekt na obrazie
		if(CMUCamPositionObjectX < (int16_t)-CMUCamDeadZonePxX)		//Jesli obiekt jest z lewej strony
		{
			CMUCamObjectRunLeft = 1;	//Obiekt porusza sie w lewo
			CMUCamObjectRunRight = 0;
		}
		else if(CMUCamPositionObjectX > (int16_t)CMUCamDeadZonePxX)	//Jesli obiekt jest z prawej strony
		{
			CMUCamObjectRunRight = 1;
			CMUCamObjectRunLeft = 0;
		}
		else		//Jak obekt jest w widelkach to 'stoi'  miejscu
		{
			CMUCamObjectRunLeft = 0;
			CMUCamObjectRunRight = 0;
		}

//--------------- Ruch statywu w lewo
		if(CMUCamObjectRunLeft == 1)			//Sprawdzam czy obiekt porusza sie w lewo i jest w zasiegu 'wzroku'
		{
			CMUCamPositionStandX -= 1;								//Normalnie popierdziela
			if(CMUCamPositionStandX < -90)
			{
				CMUCamPositionStandX = -90;

				if(CMUCamIsObject == 0)
				{
					CMUCamObjectRunLeft = 0;	//Obiekt znikl na AMEN
					CMUCamPositionObjectX = 0;
				}
			}
			else
			{
				CMUCam_Pan_SetDegree(CMUCamPositionStandX);
			}
		}
//--------------- Ruch statywu w prawo
		else if(CMUCamObjectRunRight == 1)			//Sprawdzam czy obiekt porusza sie w lewo i jest w zasiegu 'wzroku'
		{
			CMUCamPositionStandX += 1;								//Normalnie popierdziela
			if(CMUCamPositionStandX > 90)
			{
				CMUCamPositionStandX = 90;

				if(CMUCamIsObject == 0)
				{
					CMUCamObjectRunRight = 0;	//Obiekt znikl na AMEN
					CMUCamPositionObjectX = 0;
				}
			}
			else
			{
				CMUCam_Pan_SetDegree(CMUCamPositionStandX);
			}
		}
		//sledzenie w osi y (gora i dol)
		++helpFlagCMUCam2;
		if(helpFlagCMUCam2 >= 2)
		{
			helpFlagCMUCam2 = 0;

			if(CMUCamPositionObjectY < (int16_t)-CMUCamDeadZonePxY)
			{
				CMUCamPositionStandY -= 1;	//to zmniejszac 2 razy wolniej
				//jesli statyw leci w prawo o nie ma obiektu to wroc do pozycj 0
					if(CMUCamPositionStandY < 0) { CMUCamPositionStandY = 0; }
				CMUCam_Tilt_SetDegree(CMUCamPositionStandY);
			}
			else if(CMUCamPositionObjectY > (int16_t)CMUCamDeadZonePxY)
			{
				CMUCamPositionStandY += 1;
					if(CMUCamPositionStandY > 35) { CMUCamPositionStandY = 35; }
				CMUCam_Tilt_SetDegree(CMUCamPositionStandY);
			}
		}
}

void CMUCam_PixyEnable(void)										//Funkcja aktywujaca prace CMUCam
{
	CMUCamEnable = 1;
	CMUCamDataReady = 0;
	USART_Cmd(CMUCam_UART, ENABLE);
}

void CMUCam_PixyDisable(void)										//Funkcja dezaktywujaca prace CMUCam
{
	CMUCamEnable = 0;
	USART_Cmd(CMUCam_UART, DISABLE);
	CMUCam_Tilt_SetHomePosition();
	CMUCam_Pan_SetHomePosition();
}

void CMUCam_Pan_SetDegree(int8_t degree)							//Funcja ustawiajaca kat obrotu statywu CMUCam (w stopniach)
{
	uint8_t a = 0;

	if(degree == 0) { a = 130; }
	else if(degree > 0 && degree <= 90 ) { a = roundf(0.9667 * (float)degree + 130); }
	else if(degree < 0 && degree >= -90) { a = roundf(0.8889 * (float)degree + 130); }

	if(a > 217) { a = 217; }
	if(a <  50) { a =  50; }

	TIM_SetCompare1(TIM9, a);	//CMUCam Pan (obrot) 217-lewo ( 90 stopni), 130-srodek, 50-prawo (-90 stopni)
}

void CMUCam_Pan_SetHomePosition(void)								//Funkcja ustawiajaca pozycje startowa obrotu
{
	TIM_SetCompare1(TIM9, 130);
}

void CMUCam_Tilt_SetDegree(int8_t degree)							//Funcja ustawiajaca kat pochylenia statywu CMUCam (w stopniach)
{
	uint8_t a = 0;

	if(degree <= 0) { a = 90; }
	else if(degree > 0 && degree <=  35) { a = round(3.4286 * (float)degree + 90); }

	if(a > 210) { a = 210; }

	TIM_SetCompare2(TIM9, a);	//CMUCam Tilt (pochylenie) 210-do tylu (35 stopni), 90-srodek, (60-do przodu (-5 stopni))
}

void CMUCam_Tilt_SetHomePosition(void)								//Funkcja ustawiajaca pozycje startowa pochylenia
{
	TIM_SetCompare2(TIM9, 90);
}

void CMUCam_SendBuf(uint8_t *buf, uint8_t len)						//Funkcja wysylajaca dane do CMUCam
{
	for(uint8_t i = 0; i < len; i++)
	{
		uint8_t data = *buf++;	//Zwiekszam wartosc wskaznika na ktora wskazuje wskaznik

		while (USART_GetFlagStatus(CMUCam_UART, USART_FLAG_TXE) == RESET);	//Sprawdzam czy bufor do wyslania jest wolny, jak nie to czekam
		USART_SendData(CMUCam_UART, data);									//Wysylam kolejno znak z tablicy
	}
}

void USART6_IRQHandler(void)										//Uchwyt dla przerwania z USART dla CMUCam
{
	if(USART_GetITStatus(CMUCam_UART, USART_IT_RXNE))				//sprawdza czy zdarzenie bylo na lini RX UARTa
	{
		USART_ClearITPendingBit(CMUCam_UART, USART_IT_RXNE);		//Czysci flage zgloszenia przerwania na lini RX UARTa

		uint8_t receiveData = USART_ReceiveData(CMUCam_UART);		//Pobranie danych z UART do zmiennej

		if((CMUCamDataReady == 0) && (CMUCamEnable != 0))
		{
				 if( receiveData == 0x55  && (CMUCamFlagaSynchro == 0))  	{ CMUCamFlagaSynchro = 1; }	//Odczytujemy drugi bajt synchronizacji (16-bitowe dane sa przesylane w trybie little endian)
			else if((receiveData == 0xaa) && (CMUCamFlagaSynchro == 1))	 	{ CMUCamFlagaSynchro = 2; }	//odczytujemy pierwszy bajt synchronizacji o ile drugi byl odczytany
				else if((receiveData != 0xaa) && (CMUCamFlagaSynchro == 1)) { CMUCamFlagaSynchro = 0; }	//Jesli w 2 ramce bedzie blad to resetuj synchro
			else if((receiveData == 0x55) && (CMUCamFlagaSynchro == 2)) 	{ CMUCamFlagaSynchro = 3; }	//Poczatek 2 ramki synchro
				else if((receiveData != 0x55) && (CMUCamFlagaSynchro == 2)) { CMUCamFlagaSynchro = 0; } //Jesli w 3 ramce bedzie blad to resetuj synchro
			else if((receiveData == 0xaa) && (CMUCamFlagaSynchro == 3)) 	{ CMUCamFlagaSynchro = 4; }
				else if((receiveData != 0xaa) && (CMUCamFlagaSynchro == 3)) { CMUCamFlagaSynchro = 0; } //Jesli w 4 ramce bedzie blad to resetuj synchro

			else if(CMUCamFlagaSynchro == 4)	//Jak synchronizacja ramki zostala odczytana poprawnie to lecimy z odczytem reszty danych
			{
				if(CMUCamFlagaOdczytu == 0)
				{
					LowByte = receiveData;
					CMUCamFlagaOdczytu = 1;
					++CMUCamIloscOdebranychBajtow;
				}
				else if(CMUCamFlagaOdczytu == 1)
				{
					HighByte = receiveData;
					CMUCamFlagaOdczytu = 0;
					++CMUCamIloscOdebranychBajtow;

					uint16_t PolecenieZCMUCam = (HighByte << 8) + LowByte;

					if(CMUCamIloscOdebranychBajtow == 2)		{ CMUCamBlock.checksum  = PolecenieZCMUCam; }
					else if(CMUCamIloscOdebranychBajtow == 4)   { CMUCamBlock.signature = PolecenieZCMUCam; }
					else if(CMUCamIloscOdebranychBajtow == 6)   { CMUCamBlock.x         = PolecenieZCMUCam; }
					else if(CMUCamIloscOdebranychBajtow == 8)   { CMUCamBlock.y         = PolecenieZCMUCam; }
					else if(CMUCamIloscOdebranychBajtow == 10)  { CMUCamBlock.width     = PolecenieZCMUCam; }
					else if(CMUCamIloscOdebranychBajtow == 12)
					{
						CMUCamBlock.height = PolecenieZCMUCam;
						CMUCamIloscOdebranychBajtow = 0;
						CMUCamFlagaSynchro = 0;
						CMUCamDataReady = 1;
						CMUCamObjectIsFind = 1;
					}
				}
			}
		}
	}
}

void CMUCam_SetLedColor(uint8_t red, uint8_t green, uint8_t blue)	//Funkcja do ustawiena koloru LED na CMUCam
{
	uint8_t setLedColor[5];		//Tablica 5 elementowa

	//LED sync (0xfd00) - slowo synchronizacji
	setLedColor[0] = 0x00;		//16-bitowe slowo przesylane jest w trybie little endian (najmniej znaczacy bajt pierwszy)
	setLedColor[1] = 0xfd;
	//Poszczegolne skladowe kolorow
	setLedColor[2] = red;
	setLedColor[3] = green;
	setLedColor[4] = blue;

	CMUCam_SendBuf(setLedColor, 5);
}

void CMUCam_CheckLedColor(void)										//Funcja testujaca dzialanie diody LED na CMUCam
{
	uint8_t jasnoscLed;

//Sprawdzenie kolorow diody LED na CMUCam
	CMUCam_SetLedColor(0, 0, 0);		//All off
	delay_ms(500);
	CMUCam_SetLedColor(255, 0, 0);		//On red
	delay_ms(500);
	CMUCam_SetLedColor(0, 255, 0);		//On green
	delay_ms(500);
	CMUCam_SetLedColor(0, 0, 255);		//On blue
	delay_ms(500);
	CMUCam_SetLedColor(0, 0, 0);		//All off
	delay_ms(500);

	for(uint8_t i = 0; i <= 100; i++)
	{
		jasnoscLed = FunkcjaJasnosciLed(i);
		CMUCam_SetLedColor(jasnoscLed, 0, 0);		//On red
		delay_ms(1);
	}
	delay_ms(200);
	for(uint8_t i = 0; i <= 100; i++)
	{
		jasnoscLed = FunkcjaJasnosciLed(i);
		CMUCam_SetLedColor(255, 0, jasnoscLed);		//On blue
		delay_ms(1);
	}
	delay_ms(200);
	for(uint8_t i = 101; i > 0; --i)
	{
		jasnoscLed = FunkcjaJasnosciLed(i);
		CMUCam_SetLedColor(jasnoscLed, 0, 255);		//Off red
		delay_ms(1);
	}
	delay_ms(200);
	for(uint8_t i = 0; i <= 100; i++)
	{
		jasnoscLed = FunkcjaJasnosciLed(i);
		CMUCam_SetLedColor(0, jasnoscLed, 255);		//On green
		delay_ms(1);
	}
	delay_ms(200);
	for(uint8_t i = 101; i > 0; --i)
	{
		jasnoscLed = FunkcjaJasnosciLed(i);
		CMUCam_SetLedColor(0, 255, jasnoscLed);		//Off blue
		delay_ms(1);
	}
	delay_ms(200);
	for(uint8_t i = 0; i <= 100; i++)
	{
		jasnoscLed = FunkcjaJasnosciLed(i);
		CMUCam_SetLedColor(jasnoscLed, 255, 0);		//On red
		delay_ms(1);
	}
	delay_ms(200);
	for(uint8_t i = 101; i > 0; --i)
	{
		jasnoscLed = FunkcjaJasnosciLed(i);
		CMUCam_SetLedColor(255, jasnoscLed, 0);		//Off green
		delay_ms(1);
	}
	delay_ms(200);
	for(uint8_t i = 101; i > 0; --i)
	{
		jasnoscLed = FunkcjaJasnosciLed(i);
		CMUCam_SetLedColor(jasnoscLed, 0, 0);		//Off red
		delay_ms(1);
	}
	delay_ms(200);
	for(uint8_t i = 0; i <= 100; i++)
	{
		jasnoscLed = FunkcjaJasnosciLed(i);
		CMUCam_SetLedColor(jasnoscLed, jasnoscLed, jasnoscLed);		//On white
		delay_ms(1);
	}
	delay_ms(200);
	for(uint8_t i = 101; i > 0; --i)
	{
		jasnoscLed = FunkcjaJasnosciLed(i);
		CMUCam_SetLedColor(jasnoscLed, jasnoscLed, jasnoscLed);		//Off white
		delay_ms(1);
	}
	CMUCam_SetLedColor(0, 0, 0);
}

void CMUCam_CheckPanTiltMechanism(void)								//Funkcja testujaca dzialanie statywu CMUCam
{
	if(EnableDisplayLCD == 1) { TM_ILI9341_Puts(109, 145, "(testing)", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }
// Obrot statywu CMUCam
	for(int8_t i =   1; i <=  90; i++) { CMUCam_Pan_SetDegree(i); delay_ms(5); }
	for(int8_t i =  89; i >= -90; i--) { CMUCam_Pan_SetDegree(i); delay_ms(5); }
	for(int8_t i = -89; i <=   0; i++) { CMUCam_Pan_SetDegree(i); delay_ms(5); }

// Pochylenie statywu CMUCam
	for(int8_t i =   1; i <=  35; i++) { CMUCam_Tilt_SetDegree(i); delay_ms(7); }
	for(int8_t i =  34; i >=   0; i--) { CMUCam_Tilt_SetDegree(i); delay_ms(7); }

	if(EnableDisplayLCD == 1) {LCD_ClearRow1(); LCD_ClearRow2(); TM_ILI9341_Puts(109, 145, "         ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); LCD_ClearAllRow(); }
}

void CMUCamSensor_Init(void)										//Funkcja incjalizacji CMUCam
{
	if(EnableDisplayLCD == 1) { LCD_ClearRow1(); TM_ILI9341_Puts(93, 105, "CMUCam5 Pixy", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }

	CMUCamPixy_GPIO_Init();
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	CMUCamPixy_USART_Init();
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('u'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; LCD_ClearAllRow(); }
}

void PanTiltMechanism_Init(void)									//Funkcja incjalizacji  mechanizmu statywu kamery
{
	if(EnableDisplayLCD == 1) { LCD_ClearRow1(); TM_ILI9341_Puts(104, 105, "pan & tilt", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
												 TM_ILI9341_Puts(109, 124, "mechanism", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);}

	Pixy_PanTilt_GPIO_Init();
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	Pixy_PanTilt_TIM_Init();
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('t'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
}

uint8_t FunkcjaJasnosciLed(uint8_t procent)	//Argumentem funkcji jest procent (0% - 100%)
{
	const float L = 257.0;
	const float k = 0.108;
	const float x0 = 60.0;
	uint8_t wynik = round(L / (1.0 + (float)exp(-k * ((float)procent - x0))));

	return wynik;
}

uint16_t MedianAveragingFilter(uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint16_t e)
{
	uint16_t AvgVal, schowek, zamiana;
	uint16_t sortowanie[5] = {a, b, c, d, e};

	do
		{
			zamiana=0;//przed ka¿dym "przejazdem" pêtli for zmienna zamiana jest zerowana
			for (int i = 0; i < 5; i++)
			{
				if (sortowanie[i] > sortowanie[i + 1])// jeœli element tablicy jest wiêkszy od nastêpnego elementu
				{
					zamiana = zamiana + 1; //jeœli jest jakaœ zmiana, to zmienne zamiana powiêksza swoj¹ wartoœæ
					schowek = sortowanie[i];//wartoœæ t[i] jest kopiowana do schowka
					sortowanie[i] = sortowanie[i + 1];//t[i] przyjmuje wartoœæ nastêpnego elementu, gdy¿ jest on mniejszy
					sortowanie[i + 1] = schowek;//kolejny element tablicy przejmuje wczeœniejsz¹ wartoœæ poprzedniego elementu, gdy¿ jest on wiêkszy
				}
			}
		}
		while(zamiana!=0);//jeœli zmienna zamiana mia³aby wartoœæ 0, oznacza³oby to ¿e nie dokonano ¿adnych zmian, a wiêc nie ma potrzeby dalszego sortowania

	return AvgVal = (sortowanie[1] + sortowanie[2] + sortowanie[3]) / 3;
}

/*int16_t medianAveragingFilter2(int16_t tabVal[], uint8_t len, uint8_t amountValToAvg)
{
	int16_t wynik, schowek, zamiana;
	float a = 0;

	do
	{
		zamiana=0;//przed ka¿dym "przejazdem" pêtli for zmienna zamiana jest zerowana
		for (int i = 0; i < len; i++)
		{
			if (tabVal[i] > tabVal[i + 1])// jeœli element tablicy jest wiêkszy od nastêpnego elementu
			{
				zamiana = zamiana + 1; //jeœli jest jakaœ zmiana, to zmienne zamiana powiêksza swoj¹ wartoœæ
				schowek = tabVal[i];//wartoœæ t[i] jest kopiowana do schowka
				tabVal[i] = tabVal[i + 1];//t[i] przyjmuje wartoœæ nastêpnego elementu, gdy¿ jest on mniejszy
				tabVal[i + 1] = schowek;//kolejny element tablicy przejmuje wczeœniejsz¹ wartoœæ poprzedniego elementu, gdy¿ jest on wiêkszy
			}
		}
	}
	while(zamiana!=0);//jeœli zmienna zamiana mia³aby wartoœæ 0, oznacza³oby to ¿e nie dokonano ¿adnych zmian, a wiêc nie ma potrzeby dalszego sortowania

	a = ((float)len - (float)amountValToAvg) / 2;

	for(uint8_t i = a; i < (a+amountValToAvg); i++)
	{
		wynik += tabVal[i];
	}

	return wynik = wynik / amountValToAvg;
}*/
