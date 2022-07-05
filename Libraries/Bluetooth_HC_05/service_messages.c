#include <stdio.h>		//sscanf

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

#include "allDefines.h"
#include "service_messages.h"
#include "lighting.h"
#include "bt_hc_05.h"
#include "servo_motor.h"
#include "pid.h"
#include "delay.h"
#include "encoder.h"
#include "cmucam_pantilt.h"
#include "tm_stm32f4_ili9341.h"
#include <stdio.h>		//sprintf
#include <math.h>

float activeGear = 0.0;
uint8_t typeOfControl = 0;
enum controlTypeMobileRobot { ControlAutomatic = 1, ControlManual = 2 };

uint8_t typeGear = 0;
enum gearTypeMobileRobot { GearTypeAutomatic = 1, GearTypeManual = 2 };

//extern float SetSpeed;
extern float DefaultSpeed;
extern int8_t angleSteering;
//extern char str[120];

uint8_t flagaSwaitelDziennych = 0;
uint8_t flagaSwiatlaPostojowe = 0;
uint8_t flagaSwiatlaKrotkie = 0;
uint8_t flagaSwiatlaDlugie = 0;

extern int LeftTurnSignalFlag;
extern int RightTurnSignalFlag;

//extern void Clear_Odometry_Values();

extern float xStart;
extern float yStart;
extern uint16_t fiStart;
extern float xStop;
extern float yStop;
extern uint16_t fiStop;
//extern float xActual;
//extern float yActual;

extern uint8_t wybranyAlgorytm;
extern uint8_t startAlgorytm;
extern float maxSpeedForMobileRobot;
extern float gainVectors[3];
extern uint8_t distSensActive[5];
extern uint8_t CMUCamFlagaTestKamery;
extern uint16_t newSizeOfTheObject;

void ObslugaKomunikatow(char tabInstr[], int Rozmiar)	//
{
	char informacje[30];

	char tabKomunikatWsp[9];
	char tabWartoscWsp[5];
	char tabWartoscOrient[3];
	char tabKomunikatOrient[7];
	char tabWybrAlgor[5];

	char znakiPoleObiektu[6];

	switch (tabInstr[0])		//wybor zalezny od zmiennej
	{
		case 'T':
			switch (tabInstr[1])
			{
				case 'C':
					switch (tabInstr[2])
					{
						case '0':
							CMUCamFlagaTestKamery = 0;
							CMUCam_PixyDisable();
						break;

						case '1':
							CMUCamFlagaTestKamery = 1;
							CMUCam_PixyEnable();
						break;
					}
				break;
			}
		break;

		case 'P':	//Komendy odbierane przez robota
			switch (tabInstr[1])
			{
				case 'A':
					switch (tabInstr[2])
					{
						case 'S':
								 if(tabInstr[3] == '0') { startAlgorytm = 0; SetSpeedMobileRobot(0.0); TM_ILI9341_DrawFilledRectangle(0, 13, 319, 239, ILI9341_COLOR_BLACK);}
							else if(tabInstr[3] == '1')
							{
								BT_SendString("PAS1\n", 5);	//Wyslanie potwierdzenia
								startAlgorytm = 1;
							}
						break;
					}
				break;

				case 'D':
					switch (tabInstr[2])
					{
						case 'G':
							for(uint8_t i = 0; i < 6; i++)
							{
								znakiPoleObiektu[i] = tabInstr[i + 3];
							}
							sscanf(znakiPoleObiektu, "%d", &newSizeOfTheObject);

							TM_ILI9341_Puts(120, 20, "          ", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
							sprintf(informacje, "Min. pole obiektu: %d", newSizeOfTheObject);
							TM_ILI9341_Puts(10, 20, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
						break;
					}
				break;

				case 'M':
					switch (tabInstr[2])
					{
						case 'V':
						{
							char tab1[3]; char tab2[7];

							for(uint8_t i = 0; i < 3; i++) { tab1[i] = tabInstr[i + 3]; }
							sscanf(tab1, "%f", &maxSpeedForMobileRobot);
							maxSpeedForMobileRobot = maxSpeedForMobileRobot / 100;

							for(uint8_t i = 0; i < 6; i++) { tab2[i] = tabInstr[i]; } tab2[6] = '\n';
							BT_SendString(tab2, sizeof(tab2));
						}
					}
				break;

				case 'S':
					switch (tabInstr[2])
					{
						case 'A':
							switch (tabInstr[3])
							{
								char tab1[2]; char tab2[7];
								case '1':
									for(uint8_t i = 0; i < 2; i++) { tab1[i] = tabInstr[i + 4]; }
									sscanf(tab1, "%d", &distSensActive[0]);
									for(uint8_t i = 0; i < 6; i++) { tab2[i] = tabInstr[i]; } tab2[6] = '\n';
									BT_SendString(tab2, sizeof(tab2));
								break;

								case '2':
									for(uint8_t i = 0; i < 2; i++) { tab1[i] = tabInstr[i + 4]; }
									sscanf(tab1, "%d", &distSensActive[1]);
									for(uint8_t i = 0; i < 6; i++) { tab2[i] = tabInstr[i]; } tab2[6] = '\n';
									BT_SendString(tab2, sizeof(tab2));
								break;

								case '3':
									for(uint8_t i = 0; i < 2; i++) { tab1[i] = tabInstr[i + 4]; }
									sscanf(tab1, "%d", &distSensActive[2]);
									for(uint8_t i = 0; i < 6; i++) { tab2[i] = tabInstr[i]; } tab2[6] = '\n';
									BT_SendString(tab2, sizeof(tab2));
								break;

								case '4':
									for(uint8_t i = 0; i < 2; i++) { tab1[i] = tabInstr[i + 4]; }
									sscanf(tab1, "%d", &distSensActive[3]);
									for(uint8_t i = 0; i < 6; i++) { tab2[i] = tabInstr[i]; } tab2[6] = '\n';
									BT_SendString(tab2, sizeof(tab2));
								break;

								case '5':
									for(uint8_t i = 0; i < 2; i++) { tab1[i] = tabInstr[i + 4]; }
									sscanf(tab1, "%d", &distSensActive[4]);
									for(uint8_t i = 0; i < 6; i++) { tab2[i] = tabInstr[i]; } tab2[6] = '\n';
									BT_SendString(tab2, sizeof(tab2));
								break;
							}
						break;

						case 'D':
							TM_ILI9341_DrawFilledRectangle(0, 13, 319, 239, ILI9341_COLOR_BLACK);

								sprintf(informacje, "Wybrany algorytm: %d", wybranyAlgorytm);
							TM_ILI9341_Puts(10, 40, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

							TM_ILI9341_Puts( 20, 60, "Punkt poczatkowy", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

								sprintf(informacje, "X start:    %.2f", xStart);
							TM_ILI9341_Puts(10, 71, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
								sprintf(informacje, "Y start:    %.2f", yStart);
							TM_ILI9341_Puts(10, 82, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
								sprintf(informacje, "Orientacja: %d", fiStart);
							TM_ILI9341_Puts(10, 93, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

							if(wybranyAlgorytm == 1)
							{
								TM_ILI9341_Puts(180, 60, "Punkt koncowy",    &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
									sprintf(informacje, "X stop:     %.2f", xStop);
								TM_ILI9341_Puts(170, 71, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
									sprintf(informacje, "Y stop:     %.2f", yStop);
								TM_ILI9341_Puts(170, 82, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
									sprintf(informacje, "Orientacja: %d", fiStop);
								TM_ILI9341_Puts(170, 93, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
							}
							if(wybranyAlgorytm == 2)
							{
								TM_ILI9341_Puts(20, 135, "Aktywacja reakcji na przeszkode [cm]",    &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
									sprintf(informacje, "Sensor. 1: %d", distSensActive[0]);
								TM_ILI9341_Puts(10, 146, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
									sprintf(informacje, "Sensor. 2: %d", distSensActive[1]);
								TM_ILI9341_Puts(10, 157, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
									sprintf(informacje, "Sensor. 3: %d", distSensActive[2]);
								TM_ILI9341_Puts(10, 168, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
									sprintf(informacje, "Sensor. 4: %d", distSensActive[3]);
								TM_ILI9341_Puts(10, 179, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
									sprintf(informacje, "Sensor. 5: %d", distSensActive[4]);
								TM_ILI9341_Puts(10, 190, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
							}
							if(wybranyAlgorytm == 3)
							{
								TM_ILI9341_Puts(20, 135, "Wzmocnenie zachwania",    &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
									sprintf(informacje, "Wander:         *%.1f", gainVectors[0]);
								TM_ILI9341_Puts(10, 146, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
									sprintf(informacje, "Go to goal:     *%.1f", gainVectors[1]);
								TM_ILI9341_Puts(10, 157, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
									sprintf(informacje, "Avoid obstacle: *%.1f", gainVectors[2]);
								TM_ILI9341_Puts(10, 168, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
							}
							if(wybranyAlgorytm == 2 || wybranyAlgorytm == 3)
							{
								TM_ILI9341_Puts(120, 20, "          ", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
									sprintf(informacje, "Min. pole obiektu: %d", newSizeOfTheObject);
								TM_ILI9341_Puts(10, 20, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

									sprintf(informacje, "V max: %.2f [m/s]", maxSpeedForMobileRobot);
								TM_ILI9341_Puts(10, 114, informacje, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
							}
						break;

						case 'X':
							for(uint8_t i = 0; i < 5; i++) { tabWartoscWsp[i] = tabInstr[i + 3]; }
							sscanf(tabWartoscWsp, "%f", &xStart);
							for(uint8_t i = 0; i < 8; i++) { tabKomunikatWsp[i] = tabInstr[i]; } tabKomunikatWsp[8] = '\n';
							BT_SendString(tabKomunikatWsp, sizeof(tabKomunikatWsp));
						break;

						case 'Y':
							for(uint8_t i = 0; i < 5; i++) { tabWartoscWsp[i] = tabInstr[i + 3]; }
							sscanf(tabWartoscWsp, "%f", &yStart);
							for(uint8_t i = 0; i < 8; i++) { tabKomunikatWsp[i] = tabInstr[i]; } tabKomunikatWsp[8] = '\n';
							BT_SendString(tabKomunikatWsp, sizeof(tabKomunikatWsp));
						break;

						case 'O':
							for(uint8_t i = 0; i < 3; i++) { tabWartoscOrient[i] = tabInstr[i + 3]; }
							sscanf(tabWartoscOrient, "%u", &fiStart);
							for(uint8_t i = 0; i < 6; i++) { tabKomunikatOrient[i] = tabInstr[i]; } tabKomunikatOrient[6] = '\n';
							BT_SendString(tabKomunikatOrient, sizeof(tabKomunikatOrient));
						break;
					}
				break;

				case 'G':
					switch (tabInstr[2])
					{
						case 'X':
							for(uint8_t i = 0; i < 5; i++) { tabWartoscWsp[i] = tabInstr[i + 3]; }
							sscanf(tabWartoscWsp, "%f", &xStop);
							for(uint8_t i = 0; i < 8; i++) { tabKomunikatWsp[i] = tabInstr[i]; } tabKomunikatWsp[8] = '\n';
							BT_SendString(tabKomunikatWsp, sizeof(tabKomunikatWsp));
						break;

						case 'Y':
							for(uint8_t i = 0; i < 5; i++) { tabWartoscWsp[i] = tabInstr[i + 3]; }	//Odczyt wsp
							sscanf(tabWartoscWsp, "%f", &yStop);			//Zamiana tablicy char na float
							for(uint8_t i = 0; i < 8; i++) { tabKomunikatWsp[i] = tabInstr[i]; } tabKomunikatWsp[8] = '\n';	//Odeslanie potwierdzenia
							BT_SendString(tabKomunikatWsp, sizeof(tabKomunikatWsp));
						break;

						case 'O':
							for(uint8_t i = 0; i < 3; i++) { tabWartoscOrient[i] = tabInstr[i + 3]; }
							sscanf(tabWartoscOrient, "%u", &fiStop);
							for(uint8_t i = 0; i < 6; i++) { tabKomunikatOrient[i] = tabInstr[i]; } tabKomunikatOrient[6] = '\n';
							BT_SendString(tabKomunikatOrient, sizeof(tabKomunikatOrient));
						break;

						case 'S':
							switch (tabInstr[3])
							{
								char tab1[3]; char tab2[8];
								case 'A':
									for(uint8_t i = 0; i < 3; i++) { tab1[i] = tabInstr[i + 4]; }
									sscanf(tab1, "%f", &gainVectors[2]); gainVectors[2] = gainVectors[2] / 10;
									for(uint8_t i = 0; i < 7; i++) { tab2[i] = tabInstr[i]; } tab2[7] = '\n';
									BT_SendString(tab2, sizeof(tab2));
								break;

								case 'W':
									for(uint8_t i = 0; i < 3; i++) { tab1[i] = tabInstr[i + 4]; }
									sscanf(tab1, "%f", &gainVectors[0]); gainVectors[0] = gainVectors[0] / 10;
									for(uint8_t i = 0; i < 7; i++) { tab2[i] = tabInstr[i]; } tab2[7] = '\n';
									BT_SendString(tab2, sizeof(tab2));
								break;

								case 'G':
									for(uint8_t i = 0; i < 3; i++) { tab1[i] = tabInstr[i + 4]; }
									sscanf(tab1, "%f", &gainVectors[1]); gainVectors[1] = gainVectors[1] / 10;
									for(uint8_t i = 0; i < 7; i++) { tab2[i] = tabInstr[i]; } tab2[7] = '\n';
									BT_SendString(tab2, sizeof(tab2));
								break;
							}
						break;
					}
				break;

				case 'W':
					switch (tabInstr[2])
					{
						case 'A':
							if(tabInstr[3] == '1' || tabInstr[3] == '2' || tabInstr[3] == '3')
							{
								if(tabInstr[3] == '1') { wybranyAlgorytm = 1; }
								else if(tabInstr[3] == '2') { wybranyAlgorytm = 2; }
								else if(tabInstr[3] == '3') { wybranyAlgorytm = 3; }

								for(uint8_t i = 0; i < 4; i++) { tabWybrAlgor[i] = tabInstr[i]; } tabWybrAlgor[4] = '\n';
								BT_SendString(tabWybrAlgor, sizeof(tabWybrAlgor));
							}
						break;
					}
				break;
			}
		break;

//Automotive lighting (oswietlenie samochodowe)
		case AutomotiveLighting:
			switch (tabInstr[1])
			{
				case DaytimeRunningLamps:		//Swiatla do jazdy dziennej
					if(flagaSwaitelDziennych == 0)
					{
						flagaSwaitelDziennych = 1;

							 if(LeftTurnSignalFlag == 1) { DaytimeRunningLampLeft(6);   }
						else if(LeftTurnSignalFlag == 0) { DaytimeRunningLampLeft(100); }

							 if(RightTurnSignalFlag == 1) { DaytimeRunningLampRight(6);   }
						else if(RightTurnSignalFlag == 0) { DaytimeRunningLampRight(100); }

						BT_SendString("L01\n", 4);		//INFO - swiatla dzinne ON
					}
					else if(flagaSwaitelDziennych == 1)
					{
						flagaSwaitelDziennych = 0;
						DaytimeRunningLamp_RESET();
						BT_SendString("L00\n", 4);		//INFO - swiatla dzinne OFF
					}
				break;

				case FrontPositionLampss:		//Swiatla postojowe
				//Przypadki kiedy swaitla dlugie sa OFF
					if(flagaSwiatlaPostojowe == 0 && flagaSwiatlaDlugie == 0)				//Jak pozycyjne i dlugie OFF to ON pozycyjne
					{
						flagaSwiatlaPostojowe = 1;
						FrontPositionLamps();
						RearLight_SET;
						BT_SendString("L11\n", 4);		//INFO - swiatla pozycyjne ON
					}
					else if(flagaSwiatlaPostojowe == 1 && flagaSwiatlaDlugie == 0)		//Jak pozycyjne ON i dlugie OFF to OFF pozycyjne
					{
						flagaSwiatlaPostojowe = 0;
						FrontPositionMainLamps_RESET();
						if(flagaSwiatlaKrotkie == 0)
						{
							RearLight_RESET;
						}
						BT_SendString("L10\n", 4);		//INFO - swiatla pozycyjne OFF
					}
				//Przypadki kiedy swaitla dlugie sa ON
					else if(flagaSwiatlaPostojowe == 0 && flagaSwiatlaDlugie == 1)		//Jak pozycyjne OFF i dlugie ON to ustaw flage pozycyjne
					{
						flagaSwiatlaPostojowe = 1;
						BT_SendString("L11\n", 4);		//INFO - swiatla pozycyjne ON
					}
					else if(flagaSwiatlaPostojowe == 1 && flagaSwiatlaDlugie == 1)		//Jak pozycyjne OFF i dlugie ON to zeruj flage pozycyjne
					{
						flagaSwiatlaPostojowe = 0;
						BT_SendString("L10\n", 4);		//INFO - swiatla pozycyjne ON
					}
				break;

				case DippedBeam:				//Swiatla krotkie
					if(flagaSwiatlaKrotkie == 0)
					{
						flagaSwiatlaKrotkie = 1;
						DippedBeam_SET;
						RearLight_SET;
						BT_SendString("L21\n", 4);		//INFO - swiatla krotkie ON
					}
					else if(flagaSwiatlaKrotkie == 1)
					{
						flagaSwiatlaKrotkie = 0;
						DippedBeam_RESET;
						if(flagaSwiatlaPostojowe == 0)
						{
							RearLight_RESET;
						}

						BT_SendString("L20\n", 4);		//INFO - swiatla krotkie OFF
					}
				break;

				case MainBeam:					//Swiatla dlugie
					if(flagaSwiatlaDlugie == 0)				//Jak pozycyjne i dlugie OFF to ON pozycyjne
					{
						flagaSwiatlaDlugie = 1;

						FrontMainLamps();
						BT_SendString("L31\n", 4);		//INFO - swiatla dlugie ON
					}
					else if(flagaSwiatlaDlugie == 1)		//Jak pozycyjne ON i dlugie OFF to OFF pozycyjne
					{
						flagaSwiatlaDlugie = 0;

						if(flagaSwiatlaPostojowe == 1)
						{
							FrontPositionLamps();
						}
						else if(flagaSwiatlaPostojowe == 0)
						{
							FrontPositionMainLamps_RESET();
						}
						BT_SendString("L30\n", 4);		//INFO - swiatla dlugie OFF
					}
				break;

				case FrontFogLamps:

				break;

				case RearFogLamps:				//Swiatla przeciwmglowe tylne
					if(GPIO_ReadInputDataBit(RearFogLamps_Port, RearFogLamps_Pin) == 0)
					{
						RearFogLamps_SET;
						BT_SendString("L51\n", 4);		//INFO - swiatla przeciwmglowe tylne ON
					}
					else if(GPIO_ReadInputDataBit(RearFogLamps_Port, RearFogLamps_Pin) == 1)
					{
						RearFogLamps_RESET;
						BT_SendString("L50\n", 4);		//INFO - swiatla przeciwmglowe tylne OFF
					}
				break;

				case HazardFlashers:		//Swiatla awaryjne
					if((LeftTurnSignalFlag == 0) || (RightTurnSignalFlag == 0))
					{
						LeftTurnSignalFlag = 1;
						RightTurnSignalFlag = 1;

						if(GPIO_ReadInputDataBit(LeftTurnSignal_Port, LeftTurnSignal_Pin) != GPIO_ReadInputDataBit(RightTurnSignal_Port, RightTurnSignal_Pin))
						{	//Synchronizuj migacze
							LeftTurnSignal_TOGGLE;
						}

						if(flagaSwaitelDziennych == 1) { DaytimeRunningLampLeft(6); }
						if(flagaSwaitelDziennych == 1) { DaytimeRunningLampRight(6); }

						//BT_SendString("L70\n");		//INFO - migacz lewy  OFF
						//BT_SendString("L80\n");		//INFO - migacz prawy OFF
						BT_SendString("L61\n", 4);		//INFO - swiatla awaryjne ON
					}
					else if((LeftTurnSignalFlag == 1) && (RightTurnSignalFlag == 1))
					{
						LeftTurnSignalFlag = 0;
						RightTurnSignalFlag = 0;

						if(flagaSwaitelDziennych == 1) { DaytimeRunningLampLeft(100); }
						if(flagaSwaitelDziennych == 1) { DaytimeRunningLampRight(100); }

						BT_SendString("L60\n", 4);		//INFO - swiatla awaryjne OFF
					}
				break;

				case LeftTurnSignal:		//Migacz lewy
					if(LeftTurnSignalFlag == 0 || (LeftTurnSignalFlag == 1 && RightTurnSignalFlag == 1))
					{
						LeftTurnSignalFlag = 1;
						RightTurnSignalFlag = 0;

						if(flagaSwaitelDziennych == 1) { DaytimeRunningLampLeft(6); }
						if(flagaSwaitelDziennych == 1) { DaytimeRunningLampRight(100); }

						//BT_SendString("L60\n");		//INFO - swiatla awaryjne OFF
						//BT_SendString("L80\n");		//INFO - migacz prawy OFF
						BT_SendString("L71\n", 4);		//INFO - migacz lewy ON
					}
					else if(LeftTurnSignalFlag == 1)
					{
						LeftTurnSignalFlag = 0;

						if(flagaSwaitelDziennych == 1) { DaytimeRunningLampLeft(100); }

						BT_SendString("L70\n", 4);		//INFO - migacz lewy  OFF
					}
				break;

				case RightTurnSignal:		//Migacz prawy
					if(RightTurnSignalFlag == 0 || (LeftTurnSignalFlag == 1 && RightTurnSignalFlag == 1))
					{
						RightTurnSignalFlag = 1;
						LeftTurnSignalFlag = 0;

						if(flagaSwaitelDziennych == 1) { DaytimeRunningLampLeft(100); }
						if(flagaSwaitelDziennych == 1) { DaytimeRunningLampRight(6); }

						//BT_SendString("L60\n");		//INFO - swiatla awaryjne OFF
						//BT_SendString("L70\n");		//INFO - migacz lewy  OFF
						BT_SendString("L81\n", 4);		//INFO - migacz prawy ON
					}
					else if(RightTurnSignalFlag == 1)
					{
						RightTurnSignalFlag = 0;

						if(flagaSwaitelDziennych == 1) { DaytimeRunningLampRight(100); }

						BT_SendString("L80\n", 4);		//INFO - migacz prawy OFF
					}
				break;

				case Horn:			//Klakson
					if(GPIO_ReadInputDataBit(Horn_Port, Horn_Pin) == 0)
					{
						Horn_SET;
						BT_SendString("L91\n", 4);		//INFO - klakson ON
					}
					else if(GPIO_ReadInputDataBit(Horn_Port, Horn_Pin) == 1)
					{
						Horn_RESET;
						BT_SendString("L90\n", 4);		//INFO - klakson OFF
					}
				break;
			}
		break;
//Control mobile robot (sterowanie robota mobilnego)
		case RobotControl:
			switch (tabInstr[1])
			{
				case AutoControl:
					typeOfControl = ControlAutomatic;
					BT_SendString("CA1\n", 4);		//INFO - Sterowanie automatyczne
				break;

				case ManualControl:
					typeOfControl = ControlManual;
					BT_SendString("CM1\n", 4);		//INFO - Sterowanie manualne
				break;

				if(typeGear == GearTypeManual)
				{
					case ForwardDirection:
						ResetStopReversLamps();
						SetSpeedMobileRobot(activeGear);
					break;

					case ReverseDirection:
						if(activeGear < 0.0)
						{
							ResetStopSetReversLamps();
							SetSpeedMobileRobot(activeGear);
						}
					break;

					case TurnLeftWheels:
						angleSteering = angleSteering + 2;
						TurnServo_SetDegree(angleSteering);
					break;

					case TurnRightWheels:
						angleSteering = angleSteering - 2;
						TurnServo_SetDegree(angleSteering);
					break;

					case FDTLW:
						ResetStopReversLamps();
						angleSteering = angleSteering + 2;
						TurnServo_SetDegree(angleSteering);
						SetSpeedMobileRobot(activeGear);
					break;

					case FDTRW:
						ResetStopReversLamps();
						angleSteering = angleSteering - 2;
						TurnServo_SetDegree(angleSteering);
						SetSpeedMobileRobot(activeGear);
					break;

					case RDTLW:
						if(activeGear < 0.0)
						{
							ResetStopSetReversLamps();
							angleSteering = angleSteering + 2;
							TurnServo_SetDegree(angleSteering);
							SetSpeedMobileRobot(activeGear);
						}
					break;

					case RDTRW:
						if(activeGear < 0.0)
						{
							ResetStopSetReversLamps();
							angleSteering = angleSteering - 2;
							TurnServo_SetDegree(angleSteering);
							SetSpeedMobileRobot(activeGear);
						}
					break;

					case Break:
						StopLight_SET;
						SetSpeedMobileRobot(0.0);
						DefaultSpeed = 0;
					break;

					case Neutral:
						SetSpeedMobileRobot(0.0);
						DefaultSpeed = 0;
					break;
				}
			}
		break;
//Gear stick (drazek zmiany biegow)
		case Gear:
			switch (tabInstr[1])
			{
				case GearAutomatic:
					typeGear = GearTypeAutomatic;
					BT_SendString("GA1\n", 4);		//INFO - Aktywny bieg N
				break;

				case GearManual:
					typeGear = GearTypeManual;
					BT_SendString("GM1\n", 4);		//INFO - Aktywny bieg N
					//BT_SendString("\n");
				break;

				case GearN:
					ResetStopReversLamps();
					activeGear = 0.0;
					BT_SendString("GN1\n", 4);		//INFO - Aktywny bieg N
				break;

				case Gear1:
					ResetStopReversLamps();
					activeGear = 0.3;
					BT_SendString("G11\n", 4);		//INFO - Aktywny bieg 1
				break;

				case Gear2:
					ResetStopReversLamps();
					activeGear = 0.5;
					BT_SendString("G21\n", 4);		//INFO - Aktywny bieg 2
				break;

				case Gear3:
					ResetStopReversLamps();
					activeGear = 0.7;
					BT_SendString("G31\n", 4);		//INFO - Aktywny bieg 3
				break;

				case Gear4:
					ResetStopReversLamps();
					activeGear = 1.0;
					BT_SendString("G41\n", 4);		//INFO - Aktywny bieg 4
				break;

				case Gear5:
					ResetStopReversLamps();
					activeGear = 1.7;
					BT_SendString("G51\n", 4);		//INFO - Aktywny bieg 5
				break;

				case Gear6:
					ResetStopReversLamps();
					activeGear = 2.5;
					BT_SendString("G61\n", 4);		//INFO - Aktywny bieg 6
				break;

				case GearR:
					ResetStopSetReversLamps();
					activeGear = -0.3;
					BT_SendString("GR1\n", 4);		//INFO - Aktywny bieg R
				break;
			}
		break;

		default:
			//BT_SendString("Error!");
		break;

	}
}

void ResetStopReversLamps(void)
{
	StopLight_RESET;
	ReversingLamps_RESET;
}

void ResetStopSetReversLamps(void)
{
	StopLight_RESET;
	ReversingLamps_SET;
}
