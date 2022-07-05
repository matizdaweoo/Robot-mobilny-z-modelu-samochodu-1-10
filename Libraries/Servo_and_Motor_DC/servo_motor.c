#include <stdlib.h>
#include <stdint.h>

#include "allDefines.h"
#include "servo_motor.h"
#include "gpio_config.h"
#include "tim_config.h"
#include "delay.h"
#include "lcd_tft.h"
#include "tm_stm32f4_ili9341.h"
#include "encoder.h"

uint8_t enablePID = 1;

extern int LeftTurnSignalFlag;			//Flaga aktywacji migacza lewego
extern int RightTurnSignalFlag;		//Flaga aktywacji mogacza prawego
extern uint8_t EnableDisplayLCD;
extern uint8_t BargraphValue;
extern int8_t angleSteering;
extern volatile uint8_t directionSpeed;
extern volatile float Speed;

void TurnServo_SetHomePosition(void)
{
	while(angleSteering > 0)				//Jesli kola skrecone w lewo
	{
		angleSteering = angleSteering - 1;	//Skrecaj w prawo zeby wyprostowac
		TurnServo_SetDegree(angleSteering);
		delay_ms(15);					//Opuznienie zeby nie szarpal
	}
	while(angleSteering < 0)				//Jesli kola skrecone w prawo
	{
		angleSteering = angleSteering + 1;	//Skrecaj w lewo zeby wyprostowac
		TurnServo_SetDegree(angleSteering);
		delay_ms(15);					//Opuznienie zeby nie szarpal
	}
	angleSteering = 0;
	TurnServo_SetDegree(angleSteering);
}

void TurnServo_SetDegree(int a)							//Ustawienie kata skretu
{
	if(a >  22) { a =  22; }
	if(a < -22) { a = -22; }

	angleSteering = a;

	if(a <= -10) { RightTurnSignalFlag = 1; }	//Jesli skret <= -10 stopni (skret w prawo) to wlacz migacz prawy
	else 		 { RightTurnSignalFlag = 0; }	//W przeciwnym wypadku wylacz migacz prawy

	if(a >= 10)  { LeftTurnSignalFlag = 1;  }	//Jesli skret >= 10 stopni (skret w lewo) to wlacz migacz lewy
	else 		 { LeftTurnSignalFlag = 0;  }	//W przeciwnym wypadku wylacz migacz lewy

	a = 3785 - (38 * a);	//Org: a = 3785 - (38 * a); (3861 - zle nie jechal prosto)

	TIM14 -> CCR1 = a;
}

void MotorDC_SetVelocity(uint16_t velocity, uint8_t direction)			//Ustawiam predkosc. Wart we [m/s] !!!!!
{
	if (direction == 1) { GPIO_SetBits(Dir_L298_Port, Dir_L298_Pin); }		//Do tylu
	if (direction == 0) { GPIO_ResetBits(Dir_L298_Port, Dir_L298_Pin); }	//Do przodu

	TIM_SetCompare1(TIM13, velocity);
}

void MotorDC_Stop(void)
{
	SetSpeedMobileRobot(0.0);

	float dir = 0;
	float kierunek = 0;
	int8_t whileFlag = 1;
	StopLight_SET;

	enablePID = 0;	//Wylaczam PID zeby nie przeszkadzal

		 if(directionSpeed == 0) { MotorDC_SetVelocity(150, 1); kierunek = 0; }	//Jesli robot jedzie do przodu
	else if(directionSpeed == 1) { MotorDC_SetVelocity(150, 0); kierunek = 1; }	//Jesli robot jedzie do tylu

	while(whileFlag == 1)
	{
		for(uint8_t i = 0; i < 10; i++)
		{
			dir += (float)(TIM1 -> CR1 & TIM_CR1_DIR);
			delay_ms(10);
		}
		dir = dir / 10;		//0 - jazda do przodu, 16 - jazda do tylu

		if((kierunek == 0 && dir >= 10) || (kierunek == 1 && dir <= 6) || (kierunek == 0 && dir == 0))			//Jesli robot jedzie do przodu
		{
			whileFlag = 0;
			break;
		}
		dir = 0;
	}

	MotorDC_SetVelocity(0, 0);
	SetSpeedMobileRobot(0.0);

	while(Speed != 0.0)
	{

	}

	enablePID = 1;	//Wlaczam PID

	StopLight_RESET;
}

void TurnServo_MotorDC_Init(void)									//Initialize Servo and Motor DC
{
	if(EnableDisplayLCD == 1) { LCD_ClearRow1(); TM_ILI9341_Puts(104, 105, "turn servo", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	MotorDC_GPIO_Init();
	MotorDC_TIM_Init();
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('t'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; LCD_WriteTheTypeOfPeriphery(0); }

	if(EnableDisplayLCD == 1) { LCD_ClearRow1(); TM_ILI9341_Puts(115, 105, "motor DC", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }

	TurnServo_GPIO_Init();
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	TurnServo_TIM_Init();
		if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('t'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; LCD_ClearAllRow(); }
}
