#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_tim.h"

#include "allDefines.h"
#include "encoder.h"
#include "gpio_config.h"
#include "tim_config.h"

#include "lighting.h"
#include "light_sensor.h"
#include "bt_hc_05.h"
#include "delay.h"
#include "lcd_tft.h"
#include "tm_stm32f4_ili9341.h"
#include "pid.h"

#include <stdio.h>		//sprintf
#include <stdlib.h>
#include <math.h>

volatile float Speed = 0;						//Zmienna zawierajaca predkosc
volatile uint8_t directionSpeed = 0;			//Kierunek obrotu 0 - przod, 1 - tyl
volatile uint16_t impCanalA, impCanalB;

extern uint8_t  EnableDisplayLCD;
extern uint8_t BargraphValue;

extern float SetSpeed;
extern float DefaultSpeed;

void SetSpeedMobileRobot(float a)
{
	if(a != 0.0)
	{
		if(a < 0.0)
		{
			directionSpeed = 1;	//kierunek obrotow do tylu
			a = a * -1;
			SetSpeed = a;
		}
		else
		{
			directionSpeed = 0;	//kierunek obrotow do przodu
			SetSpeed = a;
		}
		//PWM = f([m/s])		//Przedkosc przeliczona na sygnal PWM
		DefaultSpeed = (13.551 * pow(a, 3)) + (20.055 * pow(a, 2)) + (36.162 * a) + 48.233;
	}
	else
	{
		SetSpeed = 0.0;
		DefaultSpeed = 0.0;
		directionSpeed = 0;
	}
}

void ReadSpeedMobileRobot(void)
{
		//Zmienne odczytujace wartosc naliczone z impulsometru na wale silnika
	uint8_t dir = TIM1 -> CR1 & TIM_CR1_DIR;		//(x >> 4) Odczyt kierunku z rejestru i przesuniecie o 4 bity zeby pokazywal 1 lub 0 a nie 16 lub 0
	uint32_t sumaImp = impCanalA + impCanalB;

	impCanalA = 0;	impCanalB = 0;

	//Predkosc [m/s] = (Ilosc impulsow z enkodera * (Obw kola / Ilisc imp na pelny obrot kola) * 100 (przeliczenie z [m/10ms] na [m/s])
	Speed = (float)sumaImp * (0.22 / 1623.94);
	sumaImp = 0;

	if(dir == 0)
	{
		//directionSpeed = 0;
	}
	else if(dir == 16)	//Jak robot porusza sie do tylu
	{
		//directionSpeed = 1;
	}
}

void BT_Send_TempCPU(void)					//Funkcja wysylajaca zmierzona predkosc przez BT
{
	if(BT_State_ReadInputDataBit == 1)		//Jesli polaczony z BT
	{
		char znaki[10];						//Lokalna tablica znakow 10 elementowa

		float temp = ReadTempCPU();

		sprintf(znaki, "RTE%.2f\n", temp);	//Wyslanie predkosci w [m/s], *100 bo 'v' odswierzane jest co 10[ms] czyli Speed [m/10ms]
		BT_SendString(znaki, sizeof(znaki));				//Wyslanie znakow przez BT
	}
}

void BT_Send_Speed(void)					//Funkcja wysylajaca zmierzona predkosc przez BT
{
	if(BT_State_ReadInputDataBit == 1)		//Jesli polaczony z BT
	{
		char znaki[10];						//Lokalna tablica znakow 10 elementowa

		sprintf(znaki, "RSP%.2f\n", (Speed * 100));	//Wyslanie predkosci w [m/s], *100 bo 'v' odswierzane jest co 10[ms] czyli Speed [m/10ms]
		BT_SendString(znaki, sizeof(znaki));				//Wyslanie znakow przez BT
	}
}

void Clear_Impuls_Number(void)				//Funkcja zerujaca naliczone impulsy
{
	//Impuls = 0;								//Zerowanie zmiennej
}

void Encoder_Init(void)								//Initialize Encoder
{
	if(EnableDisplayLCD == 1) { LCD_ClearRow1(); TM_ILI9341_Puts(121, 105, "encoder", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }
	Encoder_GPIO_Init();
	Encoder_TIM_Init();
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	MotorImpulse_GPIO_Init();
	MotorImpulse_TIM_Init();
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('t'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; LCD_ClearAllRow(); }
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_CC3))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

		++impCanalA;
	}

	if(TIM_GetITStatus(TIM3, TIM_IT_CC4))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);

		++impCanalB;
	}
}
