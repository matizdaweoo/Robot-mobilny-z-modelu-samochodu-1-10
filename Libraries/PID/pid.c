#include "stm32f4xx_gpio.h"

#include "allDefines.h"
#include "bt_hc_05.h"
#include "servo_motor.h"
#include "lighting.h"

float SetSpeed		=    0.0;					//Zadana predkosc
float DefaultSpeed 	=    0.0;				//Wartosc PWM obliczona ze wzoru na podstawie SetSpeed
float KpMotor 		=  190;  //193.09;
float KiMotor 		=   70;  //112.29;
float KdMotor 		=    1.31;	  //1.31;
float ErrorPID 		=    0.0;
float LastErrorPID 	=    0.0;
float ErrorSumPID 	=    0.0;
float ErrorDPID 	=    0.0;
float P = 0.0, I = 0.0, D = 0.0;
float AnsPID 		=    0.0;

void PID_Controller(float SetSpeed, float Speed, uint8_t dir)
{
		ErrorPID = SetSpeed - (Speed * 100);						//Wyznaczenie bledu predkosci (Speed * 100 bo obliczam blad w [m/s], a odswierzanie jest co 10ms

		ErrorSumPID += ErrorPID;							//Obliczenie calki (sumowanie)
			if(ErrorSumPID >  50) { ErrorSumPID =  50; }	//Ograniczenie dla nadmiernego przyrostu calki
			if(ErrorSumPID < -50) { ErrorSumPID = -50; }	//Tzw WINDUP (zdmuchiwanie)
		ErrorDPID = (ErrorPID - LastErrorPID) / 0.01;				//

		P = KpMotor * ErrorPID;								//Odpoweidz czlonu P
			if(P >  500) { P =  500; }
			if(P < -500) { P = -500; }

		I = KiMotor * ErrorSumPID;							//Odpowiedz czlonu I
			if(I >  500) { I =  500; }
			if(I < -500) { I = -500; }

		D = KdMotor * ErrorDPID;							//Odpowiedz czlonu D
			//if(D < 0) { D =  0; }

		AnsPID = P + I + D + DefaultSpeed;					//Wypracowana odpowiedz + OFFSET !!!!!!!!

		if(AnsPID >  500) { AnsPID =  500; }				//Ograniczenie odpowiedzi
		if(AnsPID <    0) { AnsPID =    0; }				//Ograniczenie odpowiedzi

		LastErrorPID = ErrorPID;							//Zapis starego bledu do zmiennej

		MotorDC_SetVelocity(AnsPID, dir);							//Wysterowanie silnika

		if(BT_State_ReadInputDataBit == 1)
		{
			//char str[20];
			//BT_Send_Speed();	//Jak jestem polaczony z BT to wysylam przez BT predkosc

			//sprintf(str, "Set Speed: %.2f\n", SetSpeed); 		BT_SendString(str);
			//sprintf(str, "Def Speed: %.2f\n", DefaultSpeed); 	BT_SendString(str);
			//sprintf(str, "Speed: %.2f\n", Speed);   	 		BT_SendString(str);
			//sprintf(str, "Error: %.2f\n", ErrorPID);     		BT_SendString(str);
			//sprintf(str, "P: %.2f\n", P);						BT_SendString(str);
			//sprintf(str, "ErrorSum: %.6f\n", ErrorSumPID);	BT_SendString(str);
			//sprintf(str, "I: %.6f\n", I);						BT_SendString(str);
			//sprintf(str, "ErrorD: %.6f\n", ErrorDPID);		BT_SendString(str);
			//sprintf(str, "D: %.2f\n", D);						BT_SendString(str);
			//sprintf(str, "AnsPID: %.2f\n", AnsPID);			BT_SendString(str);
			//sprintf(str, "Last Error: %.5f\n", LastErrorPID);	BT_SendString(str);

/*
			sprintf(str, "%.2f\n", SetSpeed); 		BT_SendString(str);
			sprintf(str, "%.4f\n", DefaultSpeed); 	BT_SendString(str);
			sprintf(str, "%.2f\n", Speed);   	 		BT_SendString(str);
			sprintf(str, "%.3f\n", ErrorPID);     		BT_SendString(str);
			sprintf(str, "%.6f\n", ErrorSumPID);		BT_SendString(str);
			sprintf(str, "%.6f\n", ErrorDPID);			BT_SendString(str);
			sprintf(str, "%.2f\n", P);						BT_SendString(str);
			sprintf(str, "%.6f\n", I);						BT_SendString(str);
			sprintf(str, "%.6f\n", D);						BT_SendString(str);
			sprintf(str, "%.2f\n", AnsPID);				BT_SendString(str);
			sprintf(str, "%.5f\n", LastErrorPID);	BT_SendString(str); */
		}
}

void Clear_PID_Values(void)
{
	SetSpeed		= 0.0;
	DefaultSpeed	= 0.0;
	ErrorPID		= 0.0;
	LastErrorPID	= 0.0;
	ErrorDPID		= 0.0;
	ErrorSumPID		= 0.0;
	P				= 0.0;
	I				= 0.0;
	D				= 0.0;
	AnsPID			= 0.0;
}
