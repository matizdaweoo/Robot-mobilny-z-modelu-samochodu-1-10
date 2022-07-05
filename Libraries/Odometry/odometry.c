#include <stdio.h>		//sprintf
#include <math.h>

#include "allDefines.h"
#include "odometry.h"
#include "bt_hc_05.h"
#include "tm_stm32f4_ili9341.h"

extern float xActualLast;
extern float xActual;
extern float dxActual;
extern float yActualLast;
extern float yActual;
extern float dyActual;
extern float thetaActualLast;
extern float thetaActual;
extern float dthetaActual;

//extern uint8_t EnableDisplayLCD;
extern uint8_t directionSpeed;
extern const float WheelBase;

void BT_SendOdometry(void)
{
	char znakiOdometrii[26];
	sprintf(znakiOdometrii, "PAX%.2fY%.2fO%.5f\n",  xActual, yActual, thetaActual);
	BT_SendString(znakiOdometrii, sizeof(znakiOdometrii));
}

void obliczOdometrie(float v, int8_t gammaDegree)	//Predkosc (v) [m/s], kat skretu (gamma) [']
{
	const float korektaSpeed = 10.0;	//!!!!!!!! Speed * 10 bo petla leci co 100ms a odswierzanie v jest co 10ms
	float gammaRadian = (float)gammaDegree * (liczbaPi/180);

	if(directionSpeed == 1) { v = v * -1; }

	dthetaActual 	= ((v * korektaSpeed) / WheelBase) * tan(gammaRadian);		//Obliczenie pochodnej theta (z modelu)
	thetaActual  	= dthetaActual + thetaActualLast; 							//Calkowanie i otrzymanie kata theta

	dxActual	 	= (v * korektaSpeed) * cos(thetaActual);					//Obliczenie pochodnej x (z modelu)
	xActual			= dxActual + xActualLast; 									//Calkowanie i otrzymanie wsp. x

	dyActual 	 	= (v * korektaSpeed) * sin(thetaActual);					//Obliczenie pochodnej y (z modelu)
	yActual 	 	= dyActual + yActualLast; 									//Calkowanie i otrzymanie wsp. y

	thetaActualLast = thetaActual;
	xActualLast 	= xActual;													//Zapamiêtanie wartosci
	yActualLast 	= yActual;

	/*if(EnableDisplayLCD == 1)
	{
		char wspLCD[10];

		sprintf(wspLCD, "x = %.5f", xActual);
		TM_ILI9341_Puts(10, 20, wspLCD, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		sprintf(wspLCD, "y = %.5f", yActual);
		TM_ILI9341_Puts(10, 35, wspLCD, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		sprintf(wspLCD, "t = %.5f", thetaActual);
		TM_ILI9341_Puts(10, 50, wspLCD, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	}*/
}

void wyczyscDaneOdometri(void)
{
	dxActual 	 = 0.0; xActual		= 0.0; xActualLast	   = 0.0;
	dyActual	 = 0.0; yActual 	= 0.0; yActualLast	   = 0.0;
	dthetaActual = 0.0; thetaActual = 0.0; thetaActualLast = 0.0;
}
