#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"

#include "allDefines.h"
#include "ext_interp.h"
#include "encoder.h"
#include "lighting.h"
#include "bt_hc_05.h"
#include "light_sensor.h"
#include "servo_motor.h"

#include "delay.h"
#include "tm_stm32f4_ili9341.h"

#include <stdlib.h>

//extern int AngleGamma;

extern float DistanceFromObstacle;
//extern int DistanceMeasurement;
extern int DistanceMeasurementFlag;
extern int FrontRear;
extern int TimeOutDistanceMeasurementFlag;

extern int EnableDisplayLCD;
//extern int CheckLightingFlag;

// W ktorejs funkcji mam wlaczyc wyzerowany timer po wykryciu zbocza UP z jakiegos czujnika

// w kores funkcji wylaczyc timer, obliczyc odleglosc i wrzucic do zmiennej

/* Obsluga przerwania z:
 * - Echo Rear
 */

void Start_Distance_Measurement(void)		/* Funkcja rozpoczynaj¹ca pomiar odleglosci */
{
	TIM6 -> CNT = 0;										//Zeruje wartosc licznika
	TIM6 -> CR1 |= TIM_CR1_CEN;								//Licznik zaczyna liczyc co 1 us
}

float Stop_Distance_Measurment(void)		/* Funkcja konczaca pomiar odleglosci i zwraca wynik */
{
	float Distance = 0;

	Distance = ((float)(TIM6 -> CNT & TIM_CNT_CNT) / (float)58) / 100;	//Obliczenie odleglosci w [m]
	TIM6 -> CR1 &= ~TIM_CR1_CEN;							//Wlaczenie timera 6
	FrontRear = 0;											//Zaden czujnik aktywny
	DistanceMeasurementFlag = 0;							//Blokowanie pomiaru czasu trwania sygnalu z czujnika

	return Distance;
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)						//Sprawdzam czy przerwanie bylo na liniach 2
	{
		EXTI_ClearITPendingBit(EXTI_Line1);							//Czysci flage zgloszenia przerwania na liniach 2

		if(HC_SR04_REAR_ECHO_READ_INPUT == 1)						//Jesli czyjnik zacznie odpowiedz (zrobi sie stan HIGH)
		{
			Start_Distance_Measurement();
		}
		if(HC_SR04_REAR_ECHO_READ_INPUT == 0)						//Jesli czujnik skonczy odpowiedz (zrobi sie stan LOW)
		{
			if(TimeOutDistanceMeasurementFlag == 0)					//Jesli nie bylo timeout to zmierz odleglosc
			{
				DistanceFromObstacle = Stop_Distance_Measurment();
			}
		}
	}
}

void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);

		if(HC_SR04_FRONT_ECHO_READ_INPUT == 1)						//Jesli czyjnik zacznie odpowiedz (zrobi sie stan HIGH)
		{
			Start_Distance_Measurement();
		}
		if(HC_SR04_FRONT_ECHO_READ_INPUT == 0)						//Jesli czujnik skonczy odpowiedz (zrobi sie stan LOW)
		{
			if(TimeOutDistanceMeasurementFlag == 0)					//Jesli nie bylo timeout to zmierz odleglosc
			{
				DistanceFromObstacle = Stop_Distance_Measurment();
			}
		}
	}
}

void EXTI15_10_IRQHandler(void)										//Uchwyt dla przerwania na liniach od EXTI15 do EXTI10
{
	if(EXTI_GetITStatus(EXTI_Line15) != RESET)						//Sprawdzam czy zdarzenie bylo na liniach 15
	{
		EXTI_ClearITPendingBit(EXTI_Line15);						//Czysci flage zgloszenia przerwania na liniach 15

		if(BT_State_ReadInputDataBit == 1)						//Jesli BT podlaczony
		{

			//BT_SendString("Hello Porsche Panamera!\r\n");		//Wyslij przywitanie
			//LightSensor_ChoseLight(); 							//Wybierz oswietlenie
			//CheckLightingFlag = 1;								//Wlaczenie sprawdzenia oswietlenia

			//DaytimeRunningLamp_SET();

			if(EnableDisplayLCD == 1) {	TM_ILI9341_Puts(98, 1, "BT-Connect   ", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }
		}
		if(BT_State_ReadInputDataBit == 0)						//Jesli BT rozlaczony
		{
			SetSpeedMobileRobot(0.0);

			TurnServo_SetHomePosition();

			//DaytimeRunningLamp_RESET();

			//Clear_PID_Values();									//Zerowanie wartosci regulatora PID
			/*MotorDC_SetVelocity(0);								//Zatrzymanie silnika napedowego

			//CheckLightingFlag = 0;								//Wylaczenie sprawdzenia oswietlenia


			OffAllLamps();		*/					//Zgaszenie wszystkich swiatel
			if(EnableDisplayLCD == 1) {	TM_ILI9341_Puts(98, 1, "BT-Disconnect", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }
		}
	}
}

void Ext_Interp_Init(void)
{

	if(EnableDisplayLCD == 1)
	{
		TM_ILI9341_Puts(50, 124, "                    ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		TM_ILI9341_Puts(50, 147, "                    ", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		TM_ILI9341_Puts(61, 124, "external interrupt", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		TM_ILI9341_DrawFilledRectangle(274, 185, 294, 215, ILI9341_COLOR_BROWN);
		delay_ms(100+random()%1000);		//Czekanie o wylosowana wartosc z przedzialu od 100 do 1500
	}
}
