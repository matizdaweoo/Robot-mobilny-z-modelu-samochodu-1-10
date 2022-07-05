#include <IMU/imu.h>
#include <IMU/tm_stm32f4_i2c.h>
#include <IMU/tm_stm32f4_mpu6050.h>
#include <stdlib.h>		//itoa
#include <stdio.h>		//sprintf
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "misc.h"

#include "allDefines.h"

#include "tm_stm32f4_gpio.h"
#include "tm_stm32f4_ili9341.h"

#include "rcc_config.h"
#include "gpio_config.h"
#include "exti_config.h"
#include "adc_config.h"

#include "bt_hc_05.h"
#include "delay.h"
#include "encoder.h"
#include "ext_interp.h"
#include "lcd_tft.h"
#include "light_sensor.h"
#include "lighting.h"
#include "pid.h"
#include "power_supply.h"
#include "servo_motor.h"
#include "cmucam_pantilt.h"
#include "distance_measurement.h"
#include "odometry.h"
#include "imu.h"

//#include "define.h"

void State_Start(void);
void State_Wander(void);
void State_GoToGoal(void);
void State_AvoidObstacle(void);
void State_Halt(void);
void SendVectorsMS(char WanderOrGoal);
void WyznaczWektorOutput(char a, float VmAvoid, float VdAvoid, float Vm, float Vd);
void WyznaczWektorAvoidObstacje(void);
float PrzeliczOdlNaPrednkosc(float odleglosc);
uint8_t ObliczWzmCzujnikaOdl(int8_t wart);
void WyznaczWspKierFunLin(float x1, float y1, float x2, float y2);
void TurnTheWheelsInTheSetDirection(int16_t newAngleSteering);
void MR_AvoidObstacle(float dist, int8_t angleSteering);
uint8_t checkObjectInFrontOfMobileRobot(void);

int Direction = 0;		//Kierunek obrotow silnika (0 - prod, 1 - tyl)
int Velocity = 0;		//Predkosc

uint8_t wybranyAlgorytm = 0;
uint8_t startAlgorytm = 0;

int8_t angleSteering = 0;
extern float Speed;						//Zmienna zawierajaca predkosc

//extern float SetSpeed, DefaultSpeed, ErrorPID, LastErrorPID, ErrorSumPID, ErrorDPID, AnsPID;

//Zmienne punktu poczatkowego !!!!!!!!!!!!!!!!
float xStart				= 0.0;		//Wspolrzedna x punktu startowego
float yStart				= 0.0;		//Wspolrzedna y punktu startowego
uint16_t fiStart			= 0;

//Zmienne punktu koncowego !!!!!!!!!!!!!!!!
float xStop					= 0.0;		//Wspolrzedna x punktu koncowego
float yStop 				= 0.0;		//Wspolrzedna y punktu koncowego
uint16_t fiStop				= 0;

float xActualLast			= 0.0;		//Ostatnia wartosc x
float xActual				= 0.0;		//Aktualna wartosc x
float dxActual 				= 0.0;		//Pochodna wartosci x
float yActualLast			= 0.0;		//Ostatnia wartosc y
float yActual				= 0.0;		//Aktualna wartosc y
float dyActual				= 0.0;		//Pochodna wartosci y

float thetaActualLast		= 0.0;		//Ostatnia wartosc thtey (Orientacja)
float thetaActual			= 0.0;		//Aktualna wartosc thety (Orientacja)
float dthetaActual			= 0.0;		//Pochodna wartosci thety (Orientacji)

const float WheelBase 		= 0.29;		//Rozstaw osi robota mobilnego

const float KpVelocity 		= 30.00;	// 33.00	//Wzmocnienie predkosci czlonu proporcionalnego
const float KpSteering 		=  0.50;	//  1.00	//Wzmocnienie skretu czlonu proporcionalnego

float Throttle 				= 0.0;				//Wartosc przepustnicy
float ThrottleLast 			= 10.0;			//Ostatnia wartosc przepustnicy
float Steering 				= 0.0;				//Wartosc skretu

float PWM_Motor_Velocity	= 0.0;		//Wartosc PWM podawana na silnik

char str[120];						//Tablica do funkcji sprintf

uint8_t helpFlag1 = 0;	//Zmienna pomocnicza do okreslenia co ile ma sie wysylac wsp robota do apki (co 1s)

TM_MPU6050_t MPU6050_Data0;			//Nazwa struktury dla MPU6050

uint8_t EnableDisplayLCD = 1;
uint8_t BargraphValue = 0;

float distanceToGoal 			= 0.0;
float distanceToGoalLast 		= 100.0;

float amIapproaching 			= 100.0;
float minDistanceToGoalLast 	= 0.0;

uint8_t aktualnyStanAutomatu = 0;
enum stanAutomatu {Start = 0, Wander = 1, GoToGoal = 2, AvoidObstacle = 3, Halt = 4};

extern uint8_t CMUCamObjectIsFind;
extern int8_t CMUCamPositionStandX;
extern int16_t CMUCamPositionObjectX;
extern uint16_t sizeOfTheObject;
uint16_t newSizeOfTheObject = 8000;

extern volatile uint8_t SharpFrontDistFromObst[5];

float maxSpeedForMobileRobot = 0.0;

//Algorytm Motor Schemas - Wektory sterowan [predkosc, kat wektora]
uint8_t sendVectorWanderFlag = 0;
uint8_t sendVectorGoalFlag = 0;

float vectorOutput[2]	  = { 0.0,   0.0 };	// { [m/s], deg }
float vectorWander[2]	  = { 0.5,   0.0 };
float vectorGoal[2]   	  = { 0.5,   0.0 };
float vectorAvoid[2]  	  = { 0.0,   0.0 };

float vectorSensFront[2]  = { 0.0, 180.0 };
float vectorSensFL[2]	  = { 0.0, 225.0 };
float vectorSensL[2]	  = { 0.0, 270.0 };
float vectorSensFR[2]	  = { 0.0, 135.0 };
float vectorSensR[2]	  = { 0.0,  90.0 };

float gainVectors[3]	  = { 1.0, 1.0, 1.0 };
uint8_t distSensActive[5] = { 15, 30, 30, 30, 15 };

float a_wspKier = 0.0, b_wspKier = 0.0;

char znaki[53];
char valueVectorM[3];
char valueVectorD[3];

int8_t katSkretuLast = 0;

uint8_t iloscOminiecZPrawej = 0;
uint8_t iloscOminiecZLewej = 0;

int main(void)
{
  	SystemInit();							//Inicjalizacja systemu (SysCoreClock, PLL, itp.)
	RCC_Config_Init();						//Wlaczenie wybranych peryferiow (GPIO, TIM, USART, itp.)
	delay_Init();							//Inicjalizacja opuznien
	Lighting_Init();						//Inicjalizacja oswietlenia
	LCD_Init();
	ADCCommon_Init();

	PowerSupply_Init();
	BT_HC05_Init();							//Inicjalizacja komunikacji BT
	TurnServo_MotorDC_Init();				//Inicjalizacja kontroli skretu i napedu dla robota
	Encoder_Init();
	PanTiltMechanism_Init();
		//CMUCam_CheckPanTiltMechanism();
	CMUCamSensor_Init();					//Inicjalizacja modulu CMUCam
		//CMUCam_CheckLedColor();
		CMUCam_PixyDisable();				//Wylaczenie CMUCam

	SharpSensor_Init();
	UltrasonicSensors_Init();
	IMUSensor_Init();
	//EXTI_Config_Init();						//Inicjalizacja zewnetrznych przerwan

	LightSensor_Init();
	//LightSensor_ChoseLight();

	if(EnableDisplayLCD == 1) { DisplayMainPage(); }

	delay_ms(100);
		BT_Enable_SET;						//Wlaczenie BT

	InfoLed_RESET;
	LedInsideCar(0);

	while(1)
	{
		if(BT_State_ReadInputDataBit == 0)
		{
			SetSpeedMobileRobot(0.0);
			wybranyAlgorytm = 0;
			startAlgorytm = 0;
		}

		if(BT_State_ReadInputDataBit == 1)
		{
//------------------------------------------------------ Algorytm pierwszy ------------------------------------------------------
			if(wybranyAlgorytm == 1 && startAlgorytm == 1)
			{
				wyczyscDaneOdometri(); xActual = xStart;	yActual = yStart;
				while(1)
				{
					//Obliczenie odleglosci od pkt koncowego i okreslenie czy robot zbliza sie do celu czy oddala
					distanceToGoal = sqrt(pow(xStop - xActual, 2) + pow(yStop - yActual, 2));		//Obliczam odleglosc miedzy pkt start i stop

					amIapproaching = distanceToGoal - distanceToGoalLast;
					distanceToGoalLast = distanceToGoal;

					if(amIapproaching <= 0)	//przypadek gdy zblizam sie do celu
					{
						//Obliczenie kata skrtu przedniej osi w zaleznosci od pozycji robota i pkt koncowego
						Steering = atan2((yStop - yActual), (xStop - xActual));		//Obliczenie kata skretu przednich kol
						Steering = (Steering - thetaActual) * KpSteering;			//Obliczenie bledu i czlon proporcionalny
							if(Steering >  0.38396) { Steering =  0.38396; }			//Ograniczenie max  22'
							if(Steering < -0.38396) { Steering = -0.38396; }			//Ograniczenie min -22'
						Steering = round(Steering * (180/liczbaPi));	//Przeliczenie rad na stopie oraz zaokraglenie

						++helpFlag1;
						if(helpFlag1 > 10)
						{
							helpFlag1 = 0;
							BT_Send_Speed();
						}
					}
					else	//jak jestem w celu lub zaczynam go mijac to STOP
					{
						SetSpeedMobileRobot(0.0);
						TurnServo_SetHomePosition();
						Speed = 0; Steering = 0.0;
						distanceToGoal = 0.0; distanceToGoalLast = 100.0;
						wybranyAlgorytm = 0; startAlgorytm = 0;
						wyczyscDaneOdometri();
						BT_SendString("PAS0\n", 5);
						break;
					}
					TurnServo_SetDegree(Steering);					//Ustawinie skretu
					SetSpeedMobileRobot(0.3);
					delay_ms(100);
				}
			}
//------------------------------------------------------ Algorytm koniec ------------------------------------------------------

//------------------------------------------------------ Algorytm drugi ------------------------------------------------------
			if(wybranyAlgorytm == 2 && startAlgorytm == 1)
			{
				wyczyscDaneOdometri(); xActual = xStart; yActual = yStart; CMUCamObjectIsFind = 0;
				aktualnyStanAutomatu = Start;
				BT_SendString("RSSS\n", 5);

				while(startAlgorytm == 1)	//Dopuki algorytm ma sie wykonywac, mozemy go przerwaz z apki
				{
					if(aktualnyStanAutomatu == Start)				//Stan - poczatkowy
					{
						delay_ms(400);
						State_Start();
					}
					else if(aktualnyStanAutomatu == Wander)			//Stan - bladzenie, szukanie celu
					{
						State_Wander();
					}
					else if(aktualnyStanAutomatu == GoToGoal)		//Stan - cel znaleziony, podarzam do celu
					{
						State_GoToGoal();
					}

					if(aktualnyStanAutomatu == AvoidObstacle)	//Stan - omin przeszkode
					{
						State_AvoidObstacle();
					}
					else if(aktualnyStanAutomatu == Halt)			//Stan - cel osiagniety, zakoncz algorytm
					{
						State_Halt();
						break;
					}

					delay_ms(100);
				}
				//Celowe zatrzymanie algorytmu z apki
				MotorDC_Stop();	//Motor stop
				Clear_PID_Values();
				TurnTheWheelsInTheSetDirection(0);
				DippedBeam_RESET; RearLight_RESET;
				CMUCam_PixyDisable();
			}
//------------------------------------------------------ Algorytm koniec ------------------------------------------------------

//------------------------------------------------------ Algorytm trzeci ------------------------------------------------------
			if(wybranyAlgorytm == 3 && startAlgorytm == 1)
			{
				int8_t katSkretu = 0;
				wyczyscDaneOdometri(); xActual = xStart; yActual = yStart; CMUCamObjectIsFind = 0;
				WyznaczWspKierFunLin(20.0, 0.1, 80.0, maxSpeedForMobileRobot); //Wyznaczenie wsp kier dla funkcji która przelicza odleg³oœc od przeszkodyw na predkosc
				vectorWander[0] = maxSpeedForMobileRobot;
				vectorGoal[0]   = maxSpeedForMobileRobot;
				aktualnyStanAutomatu = Start;

				while(startAlgorytm == 1)	//Dopuki algorytm ma sie wykonywac, mozemy go przerwaz z apki
				{
					if(aktualnyStanAutomatu == Start)				//Stan - poczatkowy
					{
						DippedBeam_SET; RearLight_SET;

						CMUCam_PixyEnable();	//Wlaczenie kamerki i algorytmu szukania celu

						aktualnyStanAutomatu = Wander;

						angleSteering = 0;
						TurnServo_SetDegree(angleSteering);
					}
					else if(aktualnyStanAutomatu == Wander)			//Stan - bladzenie, szukanie celu
					{
						float valRadToDegree = 180.0 / liczbaPi;

						WyznaczWektorAvoidObstacje();	//Wyznaczenie wektora Avoid Obstacle

						if(CMUCamObjectIsFind == 0)	//Wander
						{
							WyznaczWektorOutput('w', vectorAvoid[0], vectorAvoid[1], vectorWander[0], vectorWander[1]);

							sendVectorWanderFlag = 1;
						}
						else						//GoToGoal
						{
							int16_t eCam = 0;

							if((sizeOfTheObject > newSizeOfTheObject))	//Jesli przeszkoda jest dostatecznie duza to stop programu
							{
								SetSpeedMobileRobot(0.2);	//Zmniejszam predkosc

								while(1)	//dojezdzam do celu
								{
									SharpFront_ReadData();

									TurnTheWheelsInTheSetDirection(CMUCamPositionObjectX + (int16_t)CMUCamPositionStandX);

									if((SharpFrontDistFromObst[2] <= 25) && (SharpFrontDistFromObst[2] >= 8))
									{
										break;
									}
									delay_ms(100);
								}

								MotorDC_Stop();	//Motor stop
								TurnTheWheelsInTheSetDirection(0);
								vectorOutput[0] = 0.0;
								vectorOutput[1] = 0.0;
								wybranyAlgorytm = 0;
								startAlgorytm = 0;

								for(uint8_t i = 0; i < 3; i++) { Horn_SET; delay_ms(200); Horn_RESET; delay_ms(100); } //Sygnal dzwiekowy
								BT_SendString("PAS0\n", 5);

								TM_ILI9341_DrawFilledRectangle(0, 13, 319, 239, ILI9341_COLOR_BLACK);
							}
							else
							{
								eCam = CMUCamPositionObjectX + (int16_t)CMUCamPositionStandX;
								if(eCam >  22) { eCam =  22; }
								if(eCam < 0)
								{
									eCam = 360 + eCam;
									if(eCam < 338) { eCam = 338; }
								}

								vectorGoal[1] = (float)eCam;
								WyznaczWektorOutput('g', vectorAvoid[0], vectorAvoid[1], vectorGoal[0], vectorGoal[1]);

								sendVectorGoalFlag = 1;
							}
						}

						//Wysterowanie robota za pomoca wektora output

						float nowySkret = 0;

						if((vectorOutput[1] >= 0.0) && (vectorOutput[1] < 90.0))
						{
							nowySkret = vectorOutput[1];
						}
						else if((vectorOutput[1] >= 90.0) && (vectorOutput[1] < 180.0))
						{
							nowySkret = 180.0 - vectorOutput[1];
						}
						else if((vectorOutput[1] >= 180.0) && (vectorOutput[1] < 270.0))
						{
							nowySkret = 180.0 - vectorOutput[1];
						}
						else if((vectorOutput[1] >= 270.0) && (vectorOutput[1] < 360.0))
						{
							nowySkret = -(360.0 - vectorOutput[1]);
						}

						TurnTheWheelsInTheSetDirection((int16_t)nowySkret);

						if(((vectorOutput[1] >= 0.0) && (vectorOutput[1] < 90.0)) || ((vectorOutput[1] >= 270.0) && (vectorOutput[1] < 360.0)))
						{
							SetSpeedMobileRobot(vectorOutput[0]);
						}
						else if(((vectorOutput[1] >= 90.0) && (vectorOutput[1] < 180.0)) || ((vectorOutput[1] >= 180.0) && (vectorOutput[1] < 270.0)))
						{
							//SetSpeedMobileRobot(0.0);

							//vectorOutput[0] = 0.0;
							//vectorOutput[1] = 0.0;
							SetSpeedMobileRobot(vectorOutput[0] * -1);
						}
						else
						{
							SetSpeedMobileRobot(0.0);

							vectorOutput[0] = 0.0;
							vectorOutput[1] = 0.0;
						}

					}

					delay_ms(100);
				}
				//Celowe zatrzymanie algorytmu z apki
				MotorDC_Stop();	//Motor stop
				Clear_PID_Values();
				TurnTheWheelsInTheSetDirection(0);
				DippedBeam_RESET; RearLight_RESET;
				CMUCam_PixyDisable();
			}
//------------------------------------------------------ Algorytm koniec ------------------------------------------------------
		}
		delay_ms(100);
	}
	return 0;
}

void State_Start(void)
{
	DippedBeam_SET; RearLight_SET;

	CMUCam_PixyEnable();	//Wlaczenie kamerki i algorytmu szukania celu

	aktualnyStanAutomatu = Wander;
	BT_SendString("RSSW\n", 5);

	angleSteering = 0;
	TurnServo_SetDegree(angleSteering);
	SetSpeedMobileRobot(maxSpeedForMobileRobot);
}

void State_Wander(void)
{
	SharpFront_ReadData();					//Sprawdz odleglosci z SHARP Front

	if(checkObjectInFrontOfMobileRobot())	//Sprawdz czy jest przeszkoda przed autem
	{
		aktualnyStanAutomatu = AvoidObstacle;
		BT_SendString("RSSA\n", 5);
	}
	else	//Jesli nie wykryto przeszkody
	{
		SetSpeedMobileRobot(maxSpeedForMobileRobot);

		if(CMUCamObjectIsFind == 1)
		{
			MotorDC_Stop();
			TurnTheWheelsInTheSetDirection(0);

			//Wyznaczenie kierunku do przeszkody
			aktualnyStanAutomatu = GoToGoal;
			BT_SendString("RSSG\n", 5);
			Horn_SET; delay_ms(200); Horn_RESET;
		}
	}
}

void State_GoToGoal(void)
{
	SharpFront_ReadData();		//Sprawdz odleglosci z SHARP Front

	if((sizeOfTheObject >= newSizeOfTheObject))	//10000 Jesli przeszkoda jest dostatecznie duza to stop programu
	{
		SetSpeedMobileRobot(0.2);	//Zmniejszam predkosc

		while(1)	//dojezdzam do celu
		{
			SharpFront_ReadData();

			TurnTheWheelsInTheSetDirection(CMUCamPositionObjectX + (int16_t)CMUCamPositionStandX);

			if((SharpFrontDistFromObst[2] <= 25) && (SharpFrontDistFromObst[2] >= 8))
			{
				break;
			}
			delay_ms(100);
		}

		MotorDC_Stop();	//Motor stop

		aktualnyStanAutomatu = Halt;
		BT_SendString("RSSH\n", 5);
	}
	else
	{
		if(checkObjectInFrontOfMobileRobot())	//Sprawdz czy jest przeszkoda przed autem
		{
			aktualnyStanAutomatu = AvoidObstacle;
			BT_SendString("RSSA\n", 5);
		}
		else	//Jesli nie wykryto przeszkody
		{
			SetSpeedMobileRobot(maxSpeedForMobileRobot);
			TurnTheWheelsInTheSetDirection(CMUCamPositionObjectX + (int16_t)CMUCamPositionStandX);

			if(CMUCamObjectIsFind == 0)
			{
				TurnServo_SetHomePosition();	//Wyprostowac kola
				aktualnyStanAutomatu = Wander;
				BT_SendString("RSSW\n", 5);
				//TM_ILI9341_Puts(120, 20, "Wander       ", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
			}
		}
	}
}

void State_AvoidObstacle(void)
{
	uint8_t wykonanoSkret = 0; //1 - w lewo, 2 - w prawo
	uint8_t wylosowanaLiczba = 0;

	MotorDC_Stop();
	TurnTheWheelsInTheSetDirection(0);
	SharpFront_ReadData();

	if((SharpFrontDistFromObst[2] <= distSensActive[2]) && (SharpFrontDistFromObst[2] >= 8))			//Jesli przeszkoda wykryta z przodu
	{
		if((SharpFrontDistFromObst[1] < 8) && (SharpFrontDistFromObst[3] < 8))		//Przeszkoda wykryta tylko przez czujnik z przodu (1)
		{
			wylosowanaLiczba = (uint8_t)(rand() / (RAND_MAX + 1.0) * 2);

			if(wylosowanaLiczba == 0)
			{
				MR_AvoidObstacle(-0.25, -22);		//Jedz do tylu(25cm) i skrec w prawo(-22')
				wykonanoSkret = 2;
			}
			if(wylosowanaLiczba == 1)
			{
			MR_AvoidObstacle(-0.25, 22);			//Jedz do tylu(25cm) i skrec w prawo(-22')
				wykonanoSkret = 1;
			}
		}
		else if(SharpFrontDistFromObst[3] >= 8)												//Jesli przeszkoda bardziej z prawej (2)
		{
			MR_AvoidObstacle(-0.25, -22);			//Jedz do tylu(25cm) i skrec w prawo(-22')
			wykonanoSkret = 2;
		}
		else if(SharpFrontDistFromObst[1] >= 8)											//Jesli przeszkoda bardziej z lewej (3)
		{
			MR_AvoidObstacle(-0.25,  22);			//Jedz do tylu(25cm) i skrec w prawo( 22')
			wykonanoSkret = 1;
		}
		else if((SharpFrontDistFromObst[1] >= 8) && (SharpFrontDistFromObst[3] >= 8))	//Jesli wykryte przez 1, 2, 3
		{
			if(SharpFrontDistFromObst[3] == SharpFrontDistFromObst[1])					//Przeszkoda postawiona prostopadle do robota (4)
			{
				wylosowanaLiczba = (uint8_t)(rand() / (RAND_MAX + 1.0) * 2);

				if(wylosowanaLiczba == 0)
				{
					MR_AvoidObstacle(-0.25, -22);	//Jedz do tylu(25cm) i skrec w prawo(-22')
					wykonanoSkret = 2;
				}
				if(wylosowanaLiczba == 1)
				{
					MR_AvoidObstacle(-0.25, 22);	//Jedz do tylu(25cm) i skrec w prawo(-22')
					wykonanoSkret = 1;
				}
			}
			else if(SharpFrontDistFromObst[3] > SharpFrontDistFromObst[1])				//Ale przeszkoda blizej lewej strony (4.1)
			{
				MR_AvoidObstacle(-0.25,  22);		//Jedz do tylu(25cm) i skrec w prawo( 22')
				wykonanoSkret = 1;
			}
			else if(SharpFrontDistFromObst[3] < SharpFrontDistFromObst[1])				//Ale przeszkoda blizej prawej strony (4.2)
			{
				MR_AvoidObstacle(-0.25, -22);		//Jedz do tylu(25cm) i skrec w prawo(-22')
				wykonanoSkret = 2;
			}
		}

		SharpFront_ReadData();
		//Jesli robot wycofal, sprawdzam czy przeszkoda jest blisko robota i wybieram odleglosc najbluizsza i do niej dojazdzam
		if(((SharpFrontDistFromObst[1] <= distSensActive[1]) && (SharpFrontDistFromObst[1] >= 8)) || ((SharpFrontDistFromObst[2] <= distSensActive[2]) && (SharpFrontDistFromObst[2] >= 8)) || ((SharpFrontDistFromObst[3] <= distSensActive[3]) && (SharpFrontDistFromObst[3] >= 8)))
		{
			float minDist = 0.0;
			//Wybieram najmnejsza odleglosc
			if((SharpFrontDistFromObst[1] <= SharpFrontDistFromObst[2]) && (SharpFrontDistFromObst[1] <= SharpFrontDistFromObst[3])) { minDist = SharpFrontDistFromObst[1] - 10; }
			if((SharpFrontDistFromObst[2] <= SharpFrontDistFromObst[1]) && (SharpFrontDistFromObst[2] <= SharpFrontDistFromObst[3])) { minDist = SharpFrontDistFromObst[2] - 10; }
			if((SharpFrontDistFromObst[3] <= SharpFrontDistFromObst[2]) && (SharpFrontDistFromObst[3] <= SharpFrontDistFromObst[1])) { minDist = SharpFrontDistFromObst[3] - 10; }

			if(wykonanoSkret == 1) //Jesli wczesniej wykonano skret w lewo
			{
				MR_AvoidObstacle((minDist/100), -22);	//Jedz do przodu(o odleglosc do przeszkody w cm) i skrec w lewo(22')
				MR_AvoidObstacle(-0.25,  22);							//Jedz do tylu(25cm) i skrec w prawo(-22')
			}
			if(wykonanoSkret == 2) //Jesli wczesniej wykonano skret w prawo
			{
				MR_AvoidObstacle((minDist/100), 22);	//Jedz do przodu(o odleglosc do przeszkody w cm) i skrec w lewo(22')
				MR_AvoidObstacle(-0.25, -22);							//Jedz do tylu(25cm) i skrec w prawo(-22')
			}
		}
		else
		{
			if(wykonanoSkret == 1) //Jesli wczesniej wykonano skret w lewo
			{
				MR_AvoidObstacle( 0.25, -15);	//Jedz do przodu(25cm) i skrec w prawo(-22')
			}
			if(wykonanoSkret == 2) //Jesli wczesniej wykonano skret w prawo
			{
				MR_AvoidObstacle( 0.25,  15);	//Jedz do przodu(25cm) i skrec w lewo(22')
			}
		}
	}

	else if((SharpFrontDistFromObst[1] <= distSensActive[1]) && (SharpFrontDistFromObst[1] >= 8))			//Jesli przeszkoda wykryta z przodu z lewej
	{
		iloscOminiecZPrawej++;;

		if(iloscOminiecZPrawej > 3)
		{
			iloscOminiecZPrawej = 0;
			MR_AvoidObstacle(-0.15,  15);							//Jedz do tylu(20cm) i skrec w lewo( 15')
			MR_AvoidObstacle(0.2,  -6);							//Jedz do tylu(20cm) i skrec w prawo( 6')
		}
		else
		{
			MR_AvoidObstacle(-0.2,  15);							//Jedz do tylu(20cm) i skrec w lewo( 15')
		}

		char infosrinfo[20];
		sprintf(infosrinfo, "Proba omijania z prawej: %d", iloscOminiecZPrawej);
		TM_ILI9341_Puts(10, 160, infosrinfo, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	}
	else if((SharpFrontDistFromObst[3] <= distSensActive[3]) && (SharpFrontDistFromObst[3] >= 8))			//Jesli przeszkoda wykryta z przodu z prawej
	{
		iloscOminiecZLewej++;

		if(iloscOminiecZLewej > 3)
		{
			iloscOminiecZLewej = 0;
			MR_AvoidObstacle(-0.15, -15);							//Jedz do tylu(20cm) i skrec w prawo(-15')
			MR_AvoidObstacle(0.2,  6);							//Jedz do tylu(20cm) i skrec w lewo( 6')
		}
		else
		{
			MR_AvoidObstacle(-0.2,  -15);							//Jedz do tylu(20cm) i skrec w prawo(-15')
		}

		char infosrinfo[20];
		sprintf(infosrinfo, "Proba omijania z lewej:  %d", iloscOminiecZLewej);
		TM_ILI9341_Puts(10, 160, infosrinfo, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	}
	else if((SharpFrontDistFromObst[0] <= distSensActive[0]) && (SharpFrontDistFromObst[0] >= 8))			//Jesli przeszkoda wykryta z lewej
	{
		MR_AvoidObstacle(0.15, -10);							//Jedz do przodu(15cm) i skrec w prawo(-15')
	}
	else if((SharpFrontDistFromObst[4] <= distSensActive[4]) && (SharpFrontDistFromObst[4] >= 8))			//Jesli przeszkoda wykryta z prawej
	{
		MR_AvoidObstacle(0.15,  10);							//Jedz do przodu(15cm) i skrec w lewo( 15')
	}

	if(CMUCamObjectIsFind == 0)
	{
		aktualnyStanAutomatu = Wander;
		BT_SendString("RSSW\n", 5);
	}
	if(CMUCamObjectIsFind == 1)
	{
		aktualnyStanAutomatu = GoToGoal;
		BT_SendString("RSSG\n", 5);
	}
}

void State_Halt(void)
{
	MotorDC_Stop();	//Motor stop
	Clear_PID_Values();
	TurnTheWheelsInTheSetDirection(0);
	wybranyAlgorytm = 0;
	startAlgorytm = 0;

	for(uint8_t i = 0; i < 3; i++) { Horn_SET; delay_ms(200); Horn_RESET; delay_ms(100); } //Sygnal dzwiekowy
	BT_SendString("PAS0\n", 5);
	TM_ILI9341_DrawFilledRectangle(0, 13, 319, 239, ILI9341_COLOR_BLACK);
}

void SendVectorsMS(char WanderOrGoal)
{
	if(vectorOutput[0] <= 0.09) { sprintf(valueVectorM, "00%f", (vectorOutput[0] * 100)); }
	else if(vectorOutput[0] <= 0.99) { sprintf(valueVectorM,  "0%f", (vectorOutput[0] * 100)); }
	else if(vectorOutput[0] >= 1.00) { sprintf(valueVectorM,   "%f", (vectorOutput[0] * 100)); }

	if(vectorOutput[1] <=   9) { sprintf(valueVectorD, "00%f", vectorOutput[1]); }
	else if(vectorOutput[1] <=  99) { sprintf(valueVectorD,  "0%f", vectorOutput[1]); }
	else if(vectorOutput[1] >= 100) { sprintf(valueVectorD,   "%f", vectorOutput[1]); }

	for(uint8_t i = 0; i < 3; i++)	//Zapis Vm i Vd dla wktora Avoid Obstacle
	{
		znaki[4+i] = valueVectorM[i];
		znaki[7+i] = valueVectorD[i];
	}

	if(WanderOrGoal == 'w')
	{
		if(vectorWander[0] <= 0.09) { sprintf(valueVectorM, "00%f", (vectorWander[0] * 100)); }
		else if(vectorWander[0] <= 0.99) { sprintf(valueVectorM,  "0%f", (vectorWander[0] * 100)); }
		else if(vectorWander[0] >= 1.00) { sprintf(valueVectorM,   "%f", (vectorWander[0] * 100)); }

		if(vectorWander[1] <=   9) { sprintf(valueVectorD, "00%f", vectorWander[1]); }
		else if(vectorWander[1] <=  99) { sprintf(valueVectorD,  "0%f", vectorWander[1]); }
		else if(vectorWander[1] >= 100) { sprintf(valueVectorD,   "%f", vectorWander[1]); }
	}
	if(WanderOrGoal == 'g')
	{
		if(vectorGoal[0] <= 0.09) { sprintf(valueVectorM, "00%f", (vectorGoal[0] * 100)); }
		else if(vectorGoal[0] <= 0.99) { sprintf(valueVectorM,  "0%f", (vectorGoal[0] * 100)); }
		else if(vectorGoal[0] >= 1.00) { sprintf(valueVectorM,   "%f", (vectorGoal[0] * 100)); }

		if(vectorGoal[1] <=   9) { sprintf(valueVectorD, "00%f", vectorGoal[1]); }
		else if(vectorGoal[1] <=  99) { sprintf(valueVectorD,  "0%f", vectorGoal[1]); }
		else if(vectorGoal[1] >= 100) { sprintf(valueVectorD,   "%f", vectorGoal[1]); }
	}

	for(uint8_t i = 0; i < 3; i++)	//Zapis Vm i Vd dla wktora Wander lub Goal
	{
		znaki[10+i] = valueVectorM[i];
		znaki[13+i] = valueVectorD[i];
	}

	if(vectorAvoid[0] <= 0.09) { sprintf(valueVectorM, "00%f", (vectorAvoid[0] * 100)); }
	else if(vectorAvoid[0] <= 0.99) { sprintf(valueVectorM,  "0%f", (vectorAvoid[0] * 100)); }
	else if(vectorAvoid[0] >= 1.00) { sprintf(valueVectorM,   "%f", (vectorAvoid[0] * 100)); }

	if(vectorAvoid[1] <=   9) { sprintf(valueVectorD, "00%f", vectorAvoid[1]); }
	else if(vectorAvoid[1] <=  99) { sprintf(valueVectorD,  "0%f", vectorAvoid[1]); }
	else if(vectorAvoid[1] >= 100) { sprintf(valueVectorD,   "%f", vectorAvoid[1]); }

	for(uint8_t i = 0; i < 3; i++)	//Zapis Vm i Vd dla wktora Avoid Obstacle
	{
		znaki[16+i] = valueVectorM[i];
		znaki[19+i] = valueVectorD[i];
	}

	if(vectorSensL[0] <= 0.09) { sprintf(valueVectorM, "00%f", (vectorSensL[0] * 100)); }
	else if(vectorSensL[0] <= 0.99) { sprintf(valueVectorM,  "0%f", (vectorSensL[0] * 100)); }
	else if(vectorSensL[0] >= 1.00) { sprintf(valueVectorM,   "%f", (vectorSensL[0] * 100)); }

	if(vectorSensL[1] <=   9) { sprintf(valueVectorD, "00%f", vectorSensL[1]); }
	else if(vectorSensL[1] <=  99) { sprintf(valueVectorD,  "0%f", vectorSensL[1]); }
	else if(vectorSensL[1] >= 100) { sprintf(valueVectorD,   "%f", vectorSensL[1]); }

	for(uint8_t i = 0; i < 3; i++)	//Zapis Vm i Vd dla wktora Avoid Obstacle
	{
		znaki[22+i] = valueVectorM[i];
		znaki[25+i] = valueVectorD[i];
	}

	if(vectorSensFL[0] <= 0.09) { sprintf(valueVectorM, "00%f", (vectorSensFL[0] * 100)); }
	else if(vectorSensFL[0] <= 0.99) { sprintf(valueVectorM,  "0%f", (vectorSensFL[0] * 100)); }
	else if(vectorSensFL[0] >= 1.00) { sprintf(valueVectorM,   "%f", (vectorSensFL[0] * 100)); }

	if(vectorSensFL[1] <=   9) { sprintf(valueVectorD, "00%f", vectorSensFL[1]); }
	else if(vectorSensFL[1] <=  99) { sprintf(valueVectorD,  "0%f", vectorSensFL[1]); }
	else if(vectorSensFL[1] >= 100) { sprintf(valueVectorD,   "%f", vectorSensFL[1]); }

	for(uint8_t i = 0; i < 3; i++)	//Zapis Vm i Vd dla wktora Avoid Obstacle
	{
		znaki[28+i] = valueVectorM[i];
		znaki[31+i] = valueVectorD[i];
	}

	if(vectorSensFront[0] <= 0.09) { sprintf(valueVectorM, "00%f", (vectorSensFront[0] * 100)); }
	else if(vectorSensFront[0] <= 0.99) { sprintf(valueVectorM,  "0%f", (vectorSensFront[0] * 100)); }
	else if(vectorSensFront[0] >= 1.00) { sprintf(valueVectorM,   "%f", (vectorSensFront[0] * 100)); }

	if(vectorSensFront[1] <=   9) { sprintf(valueVectorD, "00%f", vectorSensFront[1]); }
	else if(vectorSensFront[1] <=  99) { sprintf(valueVectorD,  "0%f", vectorSensFront[1]); }
	else if(vectorSensFront[1] >= 100) { sprintf(valueVectorD,   "%f", vectorSensFront[1]); }

	for(uint8_t i = 0; i < 3; i++)	//Zapis Vm i Vd dla wktora Avoid Obstacle
	{
		znaki[34+i] = valueVectorM[i];
		znaki[37+i] = valueVectorD[i];
	}

	if(vectorSensFR[0] <= 0.09) { sprintf(valueVectorM, "00%f", (vectorSensFR[0] * 100)); }
	else if(vectorSensFR[0] <= 0.99) { sprintf(valueVectorM,  "0%f", (vectorSensFR[0] * 100)); }
	else if(vectorSensFR[0] >= 1.00) { sprintf(valueVectorM,   "%f", (vectorSensFR[0] * 100)); }

	if(vectorSensFR[1] <=   9) { sprintf(valueVectorD, "00%f", vectorSensFR[1]); }
	else if(vectorSensFR[1] <=  99) { sprintf(valueVectorD,  "0%f", vectorSensFR[1]); }
	else if(vectorSensFR[1] >= 100) { sprintf(valueVectorD,   "%f", vectorSensFR[1]); }

	for(uint8_t i = 0; i < 3; i++)	//Zapis Vm i Vd dla wktora Avoid Obstacle
	{
		znaki[40+i] = valueVectorM[i];
		znaki[43+i] = valueVectorD[i];
	}

	if(vectorSensR[0] <= 0.09) { sprintf(valueVectorM, "00%f", (vectorSensR[0] * 100)); }
	else if(vectorSensR[0] <= 0.99) { sprintf(valueVectorM,  "0%f", (vectorSensR[0] * 100)); }
	else if(vectorSensR[0] >= 1.00) { sprintf(valueVectorM,   "%f", (vectorSensR[0] * 100)); }

	if(vectorSensR[1] <=   9) { sprintf(valueVectorD, "00%f", vectorSensR[1]); }
	else if(vectorSensR[1] <=  99) { sprintf(valueVectorD,  "0%f", vectorSensR[1]); }
	else if(vectorSensR[1] >= 100) { sprintf(valueVectorD,   "%f", vectorSensR[1]); }

	for(uint8_t i = 0; i < 3; i++)	//Zapis Vm i Vd dla wktora Avoid Obstacle
	{
		znaki[46+i] = valueVectorM[i];
		znaki[49+i] = valueVectorD[i];
	}

	znaki[0] = 'R'; znaki[1] = 'M'; znaki[2] = 'S';	//Naglowek
	znaki[3] = WanderOrGoal;						//Rodzaj wektora bladzenie lub jazda do celu
	znaki[52] = '\n';

	BT_SendString(znaki, 53);
}

void WyznaczWektorOutput(char a, float VmAvoid, float VdAvoid, float Vm, float Vd)	//Dane wczodza jako "w lub g", Vm -> 0.3m/s, Vd kat 0 - 359
{
	float VOx = 0.0, VOy = 0.0;	//wsp wektora Output
	float VAx = 0.0, VAy = 0.0; //wsp wektora Avoid
	float Vx  = 0.0, Vy  = 0.0; //wsp wektora Wander lub Goal

	float valRadToDegree = 180.0 / liczbaPi;
	float valDegreeToRad = liczbaPi / 180.0;

	//Wybranie jaki wektor wzmacniac
	if(a == 'w') { Vm = Vm * gainVectors[0]; }
	if(a == 'g') { Vm = Vm * gainVectors[1]; }

	//Obliczenie wsp wektora Wander lub Goal
	if(Vd > 0.0)
	{
		Vx = cos(Vd * valDegreeToRad) * Vm;
		Vy = sin(Vd * valDegreeToRad) * Vm;
	}
	else if (Vd == 0.0)
	{
		Vx = Vm;
		Vy = 0.0;
	}
	else if (Vd > 270.0)
	{
		Vx = cos(Vd * valDegreeToRad) * Vm;
		Vy = sin(Vd * valDegreeToRad) * Vm;
	}

	//Obliczenie wsp wektora Avoid
	if (VdAvoid == 90.0)		//Wektor skierowany w lewo
	{
		VAx = 0.0;
		VAy = VmAvoid;
	}
	else if (VdAvoid == 180.0)	//Wektor skierowany w dol
	{
		VAx = -VmAvoid;
		VAy = 0.0;
	}
	else if (VdAvoid == 270.0)	//Wektor skierowany w prawo
	{
		VAx = 0.0;
		VAy = -VmAvoid;
	}
	else if ((VdAvoid > 90.0) && (VdAvoid < 180.0))	//Wektor w 3 cwiartce
	{
		float kat = 180 - VdAvoid;

		VAx = cos(kat * valDegreeToRad) * -VmAvoid;
		VAy = sin(kat * valDegreeToRad) * VmAvoid;
	}
	else if ((VdAvoid > 180.0) && (VdAvoid < 270.0))	//Wektor w 4 cwiartce
	{
		float kat = 270 - VdAvoid;

		VAx = sin(kat * valDegreeToRad) * -VmAvoid;
		VAy = cos(kat * valDegreeToRad) * -VmAvoid;
	}
	//Obliczenie wsp wektora Output
	VOx = VAx + Vx;		VOy = VAy + Vy;	//Sumowanie wektorow Avoid i Wander lub Goal
	vectorOutput[0] = sqrt(pow(VOx, 2) + pow(VOy, 2));	//Obliczenie wartosci wektora Avoid obstacle

	//sprawdzene czy wekt ster - wekt avoid jest

	//vectorOutput[0] musi byc dodatni!!!!!!!!!!!!!

	if(vectorOutput[0] > maxSpeedForMobileRobot) { vectorOutput[0] = maxSpeedForMobileRobot; }	//Ograniczenie prêdkoœci wyjœciowej
	if(vectorOutput[0] < 0.07) { vectorOutput[0] = 0.0; }		//Jak wyjdzie prêdkosc poni¿ej  0.07m/s to zatrzym silnik

	//Wyznaczenie kata wektora wyjsciowego
	float kat = 0.0;

	if(VOy == 0.0)	//Kierunek typowo do przodu lub do tylu
	{
		if(VOx >= 0.0) { kat =   0.0; }
		if(VOx <  0.0) { kat = 180.0; }
	}
	else if(VOy > 0.0)
	{
		kat = asin(VOy / vectorOutput[0]) * valRadToDegree;
		if(VOx >= 0.0)
		{
			if(kat >  22) { kat =  22; }
		}
		else
		{
			kat = 180 - kat;
			if(kat <  158) { kat =  158; }
		}
	}
	else if(VOy < 0.0)
	{
		kat = asin(VOy / vectorOutput[0]) * valRadToDegree;

		if(VOx >= 0.0)
		{
			kat = 360 + kat;
			if(kat <  338) { kat =  338; }
		}
		else
		{
			kat = 180 + kat;
			if(kat >  202) { kat =  202; }
		}
	}

	vectorOutput[1] = kat;
}

void WyznaczWektorAvoidObstacje(void)
{
	float Vx  = 0.0, Vy  = 0.0;	//wartosc i kierunek wektora avoid obstacle
	float V2x = 0.0, V2y = 0.0;
	float V4x = 0.0, V4y = 0.0;

	float valRadToDegree = 180.0 / liczbaPi;
	float valDegreeToRad = liczbaPi / 180.0;

	SharpFront_ReadData();					//Sprawdz odleglosci z SHARP Front

	vectorSensL[0] 	   = ObliczWzmCzujnikaOdl(SharpFrontDistFromObst[0]);	//Im blizej tym wieksza wartosc wektora
	vectorSensFL[0]	   = ObliczWzmCzujnikaOdl(SharpFrontDistFromObst[1]);
	vectorSensFront[0] = ObliczWzmCzujnikaOdl(SharpFrontDistFromObst[2]);
	vectorSensFR[0]    = ObliczWzmCzujnikaOdl(SharpFrontDistFromObst[3]);
	vectorSensR[0]     = ObliczWzmCzujnikaOdl(SharpFrontDistFromObst[4]);

	vectorSensL[0]     = PrzeliczOdlNaPrednkosc(vectorSensL[0]) * 0.3;		//Przeliczenie z cm na m/s
	vectorSensFL[0]    = PrzeliczOdlNaPrednkosc(vectorSensFL[0]);
	vectorSensFront[0] = PrzeliczOdlNaPrednkosc(vectorSensFront[0]);
	vectorSensFR[0]    = PrzeliczOdlNaPrednkosc(vectorSensFR[0]);
	vectorSensR[0]     = PrzeliczOdlNaPrednkosc(vectorSensR[0]) * 0.3;

	//Obliczenie wsp x i y dla wektorow V2 i V4
	V2x = sin(45.0 * valDegreeToRad) * vectorSensFL[0];
	V2y = cos(45.0 * valDegreeToRad) * vectorSensFL[0];
	V4x = sin(45.0 * valDegreeToRad) * vectorSensFR[0];
	V4y = cos(45.0 * valDegreeToRad) * vectorSensFR[0];

	//Sumowanie wektorow
	Vx = 0.0 - V2x;					Vy = vectorSensR[0] - V2y;	//Sumowanie wektorow V1    i V2
	Vx = Vx - vectorSensFront[0];	Vy = Vy + 0.0;				//Sumowanie wektorow V12   i V3
	Vx = Vx - V4x;					Vy = Vy + V4y;				//Sumowanie wektorow V123  i V4
	Vx = Vx + 0.0;					Vy = Vy - vectorSensL[0];	//Sumowanie wektorow V1234 i V5
	//Obliczenie wartosci wektora Avoid obstacle
	vectorAvoid[0] = sqrt(pow(Vx, 2) + pow(Vy, 2));

	vectorAvoid[0] = vectorAvoid[0] * gainVectors[2];	//Uwzglednienie wzmocnienia wektoru avoid obbstacle

	//Wyznaczenie kata wektora Avoid obstacle
	if(Vy > 0.0)
	{
		if(Vx == 0.0)
		{
			vectorAvoid[1] = 90.0; //270' - 90' - 90'
		}
		else
		{
			vectorAvoid[1] = 180.0 - (asin(Vy / vectorAvoid[0]) * valRadToDegree);
		}
	}
	else if (Vy == 0.0)
	{
		vectorAvoid[1] = 180; //270' - 90'
	}
	else if (Vy < 0.0)
	{
		if(Vx == 0.0)
		{
			vectorAvoid[1] = 270.0; //270'
		}
		else
		{
			vectorAvoid[1] = 270.0 + (asin(Vx / vectorAvoid[0]) * valRadToDegree);
		}
	}
}

float PrzeliczOdlNaPrednkosc(float odleglosc)
{
	odleglosc = (a_wspKier * odleglosc) + b_wspKier;
	if(odleglosc <= b_wspKier)
	{
		odleglosc = 0.0;
	}
	return odleglosc;
}

uint8_t ObliczWzmCzujnikaOdl(int8_t wart)
{
	uint8_t maxWartPomiaru = 80;	//Maksymalny zakres pomiaru czujnikow

	if((wart <= 8) || (wart >= maxWartPomiaru))
	{
		wart = 0;
	}
	else
	{
		wart = maxWartPomiaru - wart;
	}

	return wart;
}

void WyznaczWspKierFunLin(float x1, float y1, float x2, float y2)
{
	a_wspKier = (y2 - y1) / (x2 - x1);
	b_wspKier = y2 - (a_wspKier * x2);
}

void TurnTheWheelsInTheSetDirection(int16_t newAngleSteering)
{
	if(newAngleSteering != angleSteering)
	{
		int8_t e = newAngleSteering - angleSteering;

		if(newAngleSteering > angleSteering)
		{
			for(int8_t i = 1; i <= e; i++)
			{
				angleSteering = angleSteering + 1;
				if(angleSteering >  22) { angleSteering =  22; break; }

				TurnServo_SetDegree(angleSteering);
				delay_ms(10);
			}	//Skret w lewo
		}
		if(newAngleSteering < angleSteering)
		{
			for(int8_t i = -1; i >= e; i--)
			{
				angleSteering = angleSteering - 1;
				if(angleSteering < -22) { angleSteering = -22; break; }

				TurnServo_SetDegree(angleSteering);
				delay_ms(10);
			}	//Skret w prawo
		}
	}
}

void MR_AvoidObstacle(float dist, int8_t angSteer)
{
	float goalDist = 0.0;

	if(angSteer >  22) { angSteer =  22; }	//Zabezpieszenia przed max kontem skretu (+/- 22')
	if(angSteer < -22) { angSteer = -22; }

	if(angSteer > 0)	//Jesli ma skrecac w lewo
	{
		for(int8_t i = 0; i <= angSteer; i++) { TurnServo_SetDegree(i); delay_ms(10); }	//Skret w lewo
	}
	if(angSteer < 0)	//Jesli ma skrecac w prawo
	{
		for(int8_t i = 0; i >= angSteer; i--) { TurnServo_SetDegree(i); delay_ms(10); }	//Skret w prawo
	}

	if(dist > 0.0) { SetSpeedMobileRobot( 0.2); }	//Wybieram czy robot jedzie do przodu
	if(dist < 0.0) { SetSpeedMobileRobot(-0.2); ReversingLamps_SET; }	//czy do tylu w zaleznosci od dystansu

	while(1)
	{
		goalDist += Speed * 10;						//Wyznacam przejechan¹ drogê
		if(dist < 0.0) { dist = dist * -1.0; }	//Jesli do tylu ma jechac to zamieniam na + bo Speed jest zawsze na +
		if(goalDist >= dist) { break; }			//Jesli przejechal ile mial to koniec
		delay_ms(100);
	}

	MotorDC_Stop();
	if(GPIO_ReadInputDataBit(ReversingLamps_Port, ReversingLamps_Pin) == SET) { ReversingLamps_RESET; }

	//Wyprostuj kola
	if(angSteer > 0)	//Wyprostuj kola w prawo
	{
		for(int8_t i = angSteer; i >= 0; i--) { TurnServo_SetDegree(i); delay_ms(10); }	//Skret w lewo
	}
	if(angSteer < 0)	//Wyprostuj kola w lewo
	{
		for(int8_t i = angSteer; i <= 0; i++) { TurnServo_SetDegree(i); delay_ms(10); }	//Skret w prawo
	}

	TurnServo_SetDegree(0);
}

uint8_t checkObjectInFrontOfMobileRobot(void)
{
	//uint8_t czuloscAktywacji = 40;	//Okreslam od ilu [cm] robota od przeszkody zglaszam ze wykryto przeszkode

	 if((SharpFrontDistFromObst[0] <= distSensActive[0] && SharpFrontDistFromObst[0] >= 8) ||
		(SharpFrontDistFromObst[1] <= distSensActive[1] && SharpFrontDistFromObst[1] >= 8) ||
		(SharpFrontDistFromObst[2] <= distSensActive[2] && SharpFrontDistFromObst[2] >= 8) ||
		(SharpFrontDistFromObst[3] <= distSensActive[3] && SharpFrontDistFromObst[3] >= 8) ||
		(SharpFrontDistFromObst[4] <= distSensActive[4] && SharpFrontDistFromObst[4] >= 8) )

	/*if( (SharpFrontDistFromObst[1] <= distSensActive[1] && SharpFrontDistFromObst[1] >= 8) ||
		(SharpFrontDistFromObst[2] <= distSensActive[2] && SharpFrontDistFromObst[2] >= 8) ||
		(SharpFrontDistFromObst[3] <= distSensActive[3] && SharpFrontDistFromObst[3] >= 8) )*/
		 { return 1; }
	else { return 0; }
}

