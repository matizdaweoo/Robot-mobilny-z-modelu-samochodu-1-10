#include <stdio.h>		//sprintf
#include <math.h>
#include <stdint.h>

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "delay.h"

#include "allDefines.h"
#include "tim_config.h"
#include "cmucam_pantilt.h"
#include "bt_hc_05.h"
#include "lighting.h"
#include "light_sensor.h"
#include "encoder.h"
#include "pid.h"
#include "distance_measurement.h"
#include "odometry.h"
#include "imu.h"

volatile uint32_t timer_wait = 0;										//Zmienna do odliczenia czasu opuznienia

uint16_t RefreshTimeSendData = 1000;
uint16_t RefreshTimeSendDataFlag = 0;

unsigned int TimeToCheckLighting = 20000; 								//(20s) Czas [ms] sprawdzenia oswietlenia zewnetrznego dla wybrania rodzaju swiatel
unsigned int TimeToCheckLightingFlag = 0;								//Zmienna pomocnicza do odliczania czasu zeby okreslic kiedy sprawdzic oswietlenie
extern int CheckLightingFlag;

int UpdateRadarPosition = 500;											//Odswiezanie co 100ms (czas podawany w [ms])
int UpdateRadarPositionFlag = 0;
//extern int ChooseRadar;

int RefreshTimeSpeed = 10;												//Odswiezanie co 100ms (czas podawany w [ms])
int RefreshTimeSpeedFlag = 0;											//Zmienna pomocnicza do odliczania czasu kiedy zrobic pomiar predkosci
extern float Speed;
extern float SetSpeed;
extern uint8_t directionSpeed;
//extern int EnablePID;

const uint16_t RefreshTimeAcceGyro = 100;											//Odswierzanie co 100ms
int RefreshTimeAcceGyroFlag = 0;										//Zmienna pomocnicza do odliczania czasu kiedy zrobic pomiar akcelerometru i zyroskopu

const uint16_t TurnSignalTime = 500;												//Czas migania migaczami co 500ms
int TurnSignalTimeFlag = 0;												//Zmienna pomocnicza do odliczania czasu przelaczenia stanu migaczy
int LeftTurnSignalFlag = 0;												//Flaga aktywacji migacza lewego
int RightTurnSignalFlag = 0;											//Flaga aktywacji mogacza prawego

int EnableGoMobileRobot = 0;											//Zezwolenie wykonania petli z ODOMETRIA
const uint16_t RefreshTimeGoMobileRobot = 100;										//Co ile ms ma sie wykonywac petla z ODOMETRIA
int RefreshTimeGoMobileRobotFlag = 0;									//Zmienna pomocnicza do odliczania czasu kiedy ma zezwolic na wykonanie petli z ODOMETRIA

uint8_t EnableDelayFlag = 0;											//Zezwolenie pomiaru odleglosci, oswietlenia, predkosci, migaczy

const uint16_t RefreshBTReadBuffer = 100;								//Co ile [ms] ma sprawdzac czy przyszly dane przez BT
uint16_t RefreshBTReadBufferFlag = 60;									//Zmienna pomocnicza do odliczania czasu

const uint16_t RefreshCMUCamData = 10;									//Co ile [ms] ma sprawdzac czy przyszly dane z CMUCam
uint16_t RefreshCMUCamDataFlag = 0;										//Zmienna pomocnicza do odliczania czasu
uint16_t helpFlagCMUCamFindObjectAgain = 0;
extern uint8_t CMUCamEnable;
extern uint8_t CMUCamObjectIsFind;
extern uint8_t CMUCamIsObject;
extern uint16_t sizeOfTheObject;

const uint16_t RefreshSharpData = 300;
uint16_t RefreshSharpDataFlag = 0;
volatile uint8_t SharpFrontDistFromObst[5];

//extern float DistanceFromObstacle;

//extern uint8_t wybranyAlgorytm;
extern uint8_t startAlgorytm;

const uint16_t RefreshOdometry = 100;
uint16_t RefreshOdometryFlag = 0;
extern int8_t angleSteering;
extern uint8_t CMUCamFlagaTestKamery;
extern uint8_t sendVectorWanderFlag;
extern uint8_t sendVectorGoalFlag;

extern uint8_t enablePID;

void TIM7_IRQHandler(void)			//Obsluga przerwania co 1ms
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update))
	{
//---- Odswierzanie predkosci robota i dzialania regulatora PID ----*/
		++RefreshTimeSpeedFlag;												//Zwieksz zmienna pomocnicza
			if(RefreshTimeSpeedFlag >= RefreshTimeSpeed)					//Jesli ma sprawdzic predkosc
			{
				RefreshTimeSpeedFlag = 0;									//Zeruj zmienna pomocnicza

				ReadSpeedMobileRobot();										//Obliczam predkosc z enkoderow

				if(enablePID = 1)
				{
					PID_Controller(SetSpeed, Speed, directionSpeed);
				}
			}
//---- Sprawdzanie danych z CMUCam i szukanie/sledzenie obiektu ----*/
		if(CMUCamEnable == 1)	//Aktywacja pracy CMUCam
		{
			++RefreshCMUCamDataFlag;
			if(RefreshCMUCamDataFlag >= RefreshCMUCamData)
			{
				RefreshCMUCamDataFlag = 0;

				if(CMUCamObjectIsFind == 0)			//Jesli obiekt nie zostal znaleziony
				{
					CMUCam_FindObject();	//To szukaj go
				}
				else if(CMUCamObjectIsFind == 1)	//Jesli obiekt sie znalazl
				{
					CMUCam_TrackObject();	//To sledz go

					if(CMUCamIsObject == 0)  //Nie ma obiektu
					{
						++helpFlagCMUCamFindObjectAgain;	//ta zmienna * 10ms daje nam ile chcemy czekac zeby ponownie szukac obiektu
							if(helpFlagCMUCamFindObjectAgain >= 200)	//100 * 10ms = 1000ms
							{
								helpFlagCMUCamFindObjectAgain = 0;
								sizeOfTheObject = 0;
								CMUCamObjectIsFind = 0;	//Po tym czasie zaczyman znow szukac obiektu
							}
					}
				}
			}
		}
		else { RefreshCMUCamDataFlag = 0; }
//---- Sprawdzenie czy nie ma danych z BT ----*/
		++RefreshBTReadBufferFlag;
		if(RefreshBTReadBufferFlag >= RefreshBTReadBuffer) { RefreshBTReadBufferFlag = 0; BT_ReadBuffer(); }

		++RefreshSharpDataFlag;
		if(RefreshSharpDataFlag >= RefreshSharpData)
		{
			RefreshSharpDataFlag = 0;

			//if(wybranyAlgorytm == 1)
			//{
			//	SharpFront_ReadData();
			//}
		}

		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}

	/*++RefreshTimeAcceGyroFlag;
		if(RefreshTimeAcceGyroFlag >= RefreshTimeAcceGyro)
		{
			RefreshTimeAcceGyroFlag = 0;
			Read_Accelerometer_Data();
			Read_Gyroscope_Data();
			if(BT_State_ReadInputDataBit == 1)
			{
				//BT_Send_Accelerometer();
				//BT_Send_Gyroscope();
			}
		}*/

}

void SysTick_Handler()													//Uchwyt dla przerwania SysTick
{
	if(timer_wait) { timer_wait--; }

	++RefreshTimeSendDataFlag;
		if(RefreshTimeSendDataFlag >= RefreshTimeSendData)
		{
			RefreshTimeSendDataFlag = 0;

			if(CMUCamFlagaTestKamery == 1)
			{
				char znaki[10];
				sprintf(znaki, "RDG%d\n", sizeOfTheObject);	//
				BT_SendString(znaki, sizeof(znaki));
			}

			if(startAlgorytm == 0)
			{
				BT_Send_Speed();
				BT_Send_TempCPU();

				SharpFront_ReadData();
				BT_SendDistanceFromObstacle();
			}
		}

if(startAlgorytm == 1)
{
	++RefreshOdometryFlag;
		if(RefreshOdometryFlag >= RefreshOdometry)
		{
			RefreshOdometryFlag = 0;

			obliczOdometrie(Speed, angleSteering);
			BT_Send_Speed();
			BT_SendOdometry();
			BT_SendDistanceFromObstacle();

			char znakie[10];
			sprintf(znakie, "RDG%d\n", sizeOfTheObject);	//
			BT_SendString(znakie, sizeof(znakie));


			if(sendVectorWanderFlag == 1)
			{
				SendVectorsMS('w');
				sendVectorWanderFlag = 0;
			}

			if(sendVectorGoalFlag == 1)
			{
				SendVectorsMS('g');
				sendVectorGoalFlag = 0;
			}
		}
}
else
{
	RefreshOdometryFlag = 0;
}

//---- Odswierzanie migania migaczami ----*/
	++TurnSignalTimeFlag;												//Zwieksz zmienna pomocnicza
		if(TurnSignalTimeFlag >= TurnSignalTime)						//Jesli ma przelaczyc migacz
		{
			TurnSignalTimeFlag = 0;										//Zeruj zmienna pomocnicza
			if(LeftTurnSignalFlag == 1)  { LeftTurnSignal_TOGGLE;  }	//Jesli aktywny migacz lewy to migaj nim
			else 						 { LeftTurnSignal_RESET;   }	//W przeciwnym wypadku zgas migacz lewy
			if(RightTurnSignalFlag == 1) { RightTurnSignal_TOGGLE; }	//Jesli aktywny migacz prawy to migaj nim
			else 						 { RightTurnSignal_RESET;  }	//W przeciwnym wypadku zgas migacz prawy
		}

//---- Sprawdzenie oswietlenia jesli zezwolono na to ----*/
	if(CheckLightingFlag == 1)
	{
		++TimeToCheckLightingFlag;										//Zwieksz zmienna pomocnicza
		if(TimeToCheckLightingFlag >= TimeToCheckLighting)				//Jesli ma sprawdzic oswietlenie zewnetrzne
		{
			TimeToCheckLightingFlag = 0;								//Zeruj zmienna pomocnicza
			//LightSensor_ChoseLight();									//Sprawdzam oswietlenie
		}
	}
	else
	{
		TimeToCheckLightingFlag = 0;									//Zeruj zmienna pomocnicza
	}
}

void delay_ms(int time)													//Funkcja do opoznienia w [ms]
{
	timer_wait = time;
	while(timer_wait) {};
}

void delay_Init()														//Funkcja inicjujaca SysTick, przerwanie co 1ms
{
	Count_1us_TIM_Init();
	Count_1ms_TIM_Init();

	SysTick_Config(SystemCoreClock / 1000);								//(taktowanie zegara CPU / liczbe wywolan na sekunde) wywolanie co 1 ms
}
