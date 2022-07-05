#include "stm32f4xx_gpio.h"

#include "allDefines.h"
#include "gpio_config.h"

/*-------------------- Start konfiguracja - Bluetooth --------------------*/
void BT_HC05_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	//Configure UART4_TX_Pin and UART4_RX_Pin as alternative function
	GPIO_InitStruct.GPIO_Pin = BT_UART_TX_Pin | BT_UART_RX_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(BT_UART_GPIO_Port, &GPIO_InitStruct);

	GPIO_PinAFConfig(BT_UART_GPIO_Port, BT_UART_TX_Pin_Source, BT_UART_GPIO_AF);	//Configure pin as AF
	GPIO_PinAFConfig(BT_UART_GPIO_Port, BT_UART_RX_Pin_Source, BT_UART_GPIO_AF);

	//Configure Enable_BT_Pin as output
	GPIO_InitStruct.GPIO_Pin = BT_Enable_Pin;		//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;		//Tryb
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;	//Szybkosc dzialania (narostu sygnalu)
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;		//Tryb dwu wartosciowy logiczne 0 lub 1
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;		//Stan poczatkowy
	GPIO_Init(BT_Enable_Port, &GPIO_InitStruct);	//Inicjalizacja struktury

	//Configure State_BT_Pin as input
	GPIO_InitStruct.GPIO_Pin = BT_State_Pin;		//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;		//Tryb
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;	//Szybkosc dzialania (narostu sygnalu)
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;		//Tryb dwu wartosciowy logiczne 0 lub 1
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;	//Stan poczatkowy
	GPIO_Init(BT_State_Port, &GPIO_InitStruct);		//Inicjalizacja struktury
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - CMUCam5 Pixy --------------------*/
void CMUCamPixy_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	//Configure UART4_TX_Pin and UART4_RX_Pin as alternative function
	GPIO_InitStruct.GPIO_Pin = CMUCam_UART_TX_Pin | CMUCam_UART_RX_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(CMUCam_UART_GPIO_Port, &GPIO_InitStruct);

	GPIO_PinAFConfig(CMUCam_UART_GPIO_Port, CMUCma_UART_TX_Pin_Source, CMUCam_UART_GPIO_AF);	//Configure pin as AF
	GPIO_PinAFConfig(CMUCam_UART_GPIO_Port, CMUCam_UART_RX_Pin_Source, CMUCam_UART_GPIO_AF);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - Statyw do CMUCam5 Pixy --------------------*/
void Pixy_PanTilt_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	//Configure Servo_Pin as alternative function
	GPIO_InitStruct.GPIO_Pin = CMUCam_Pan_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CMUCam_Pan_Port , &GPIO_InitStruct);

	GPIO_PinAFConfig(CMUCam_Pan_Port, CMUCam_Pan_PinSource, CMUCam_PanTilt_GPIO_AF);	//Configure pin as AF

	//Configure Servo_Pin as alternative function
	GPIO_InitStruct.GPIO_Pin = CMUCam_Tilt_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CMUCam_Tilt_Port, &GPIO_InitStruct);

	GPIO_PinAFConfig(CMUCam_Tilt_Port, CMUCam_Tilt_PinSource, CMUCam_PanTilt_GPIO_AF);	//Configure pin as AF
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - SharpFront --------------------*/
void SharpFront_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin = SharpFront1_Pin;	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; GPIO_Init(SharpFront1_Port, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = SharpFront2_Pin;	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; GPIO_Init(SharpFront2_Port, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = SharpFront3_Pin;	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; GPIO_Init(SharpFront3_Port, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = SharpFront4_Pin;	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; GPIO_Init(SharpFront4_Port, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = SharpFront5_Pin;	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; GPIO_Init(SharpFront5_Port, &GPIO_InitStruct);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - SharpRear --------------------*/
void SharpRear_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin = SharpRear1_Pin;	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; GPIO_Init(SharpRear1_Port, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = SharpRear2_Pin;	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; GPIO_Init(SharpRear2_Port, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = SharpRear3_Pin;	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; GPIO_Init(SharpRear3_Port, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = SharpRear4_Pin;	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; GPIO_Init(SharpRear4_Port, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = SharpRear5_Pin;	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; GPIO_Init(SharpRear5_Port, &GPIO_InitStruct);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - Ultrasonic sensor --------------------*/
void UltrasonicSensors_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;							//Struktura dla konfiguracji GPIO

	//-------------------- TRIGGER --------------------
	GPIO_InitStruct.GPIO_Pin = HC_SR04_FRONT_TRIGGER_Pin; 		//Trigger front
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;					//Tryb
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;				//Szybkosc dzialania (narostu sygnalu)
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;					//Tryb dwu wartosciowy logiczne 0 lub 1
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;					//Stan poczatkowy
	GPIO_Init(HC_SR04_FRONT_TRIGGER_Port, &GPIO_InitStruct);	//Inicjalizacja struktury

	GPIO_InitStruct.GPIO_Pin = HC_SR04_REAR_TRIGGER_Pin; 		//Trigger rear
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;					//Tryb
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;				//Szybkosc dzialania (narostu sygnalu)
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;					//Tryb dwu wartosciowy logiczne 0 lub 1
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;					//Stan poczatkowy
	GPIO_Init(HC_SR04_REAR_TRIGGER_Port, &GPIO_InitStruct);		//Inicjalizacja struktury

	//-------------------- ECHO --------------------
	GPIO_InitStruct.GPIO_Pin = HC_SR04_FRONT_ECHO_Pin; 			//Echo rear
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;					//Tryb
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;				//Szybkosc dzialania
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;					//Tryb dwu wartosciowy logiczne 0 lub 1
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;				//Stan poczatkowy
	GPIO_Init(HC_SR04_FRONT_ECHO_Port, &GPIO_InitStruct);		//Inicjalizacja struktury

	//Ustawiam przerwanie aktywowane z pinu czujnika przedniego
	SYSCFG_EXTILineConfig(HC_SR04_FRONT_ECHO_Port_Source, HC_SR04_FRONT_ECHO_Pin_Source);

	GPIO_InitStruct.GPIO_Pin = HC_SR04_REAR_ECHO_Pin; 			//Echo front
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;					//Tryb
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;				//Szybkosc dzialania
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;					//Tryb dwu wartosciowy logiczne 0 lub 1
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;				//Stan poczatkowy
	GPIO_Init(HC_SR04_REAR_ECHO_Port, &GPIO_InitStruct);		//Inicjalizacja struktury

	//Ustawiam przerwanie aktywowane z pinu czujnika tylnego
	SYSCFG_EXTILineConfig(HC_SR04_REAR_ECHO_Port_Source, HC_SR04_REAR_ECHO_Pin_Source);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - Enkoder --------------------*/
void Encoder_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	//Configure Encoder canal A
	GPIO_InitStruct.GPIO_Pin = Encoder_A_Pin;			//Jaki nr pinu
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			//Tryb
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		//Szybkosc dzialania (narostu sygnalu)
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			//Tryb dwu wartosciowy logiczne 0 lub 1
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		//Stan poczatkowy
	GPIO_Init(Encoder_A_Port, &GPIO_InitStruct);		//Inicjalizacja struktury

	//Configure Encoder canal B
	GPIO_InitStruct.GPIO_Pin = Encoder_B_Pin;			//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			//Tryb
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		//Szybkosc dzialania (narostu sygnalu)
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			//Tryb dwu wartosciowy logiczne 0 lub 1
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		//Stan poczatkowy
	GPIO_Init(Encoder_B_Port, &GPIO_InitStruct);		//Inicjalizacja struktury

	GPIO_PinAFConfig(Encoder_A_Port, Encoder_A_Pin_Source, Encoder_A_GPIO_AF);
	GPIO_PinAFConfig(Encoder_B_Port, Encoder_B_Pin_Source, Encoder_B_GPIO_AF);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - Impulsator silnika --------------------*/
void MotorImpulse_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	//Configure pulser canal A
	GPIO_InitStruct.GPIO_Pin = Pulser_A_Pin;			//Jaki nr pinu
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			//Tryb
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		//Szybkosc dzialania (narostu sygnalu)
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			//Tryb dwu wartosciowy logiczne 0 lub 1
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		//Stan poczatkowy
	GPIO_Init(Pulser_A_Port, &GPIO_InitStruct);		//Inicjalizacja struktury

	//Configure pulser canal B
	GPIO_InitStruct.GPIO_Pin = Pulser_B_Pin;			//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			//Tryb
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		//Szybkosc dzialania (narostu sygnalu)
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			//Tryb dwu wartosciowy logiczne 0 lub 1
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		//Stan poczatkowy
	GPIO_Init(Pulser_B_Port, &GPIO_InitStruct);		//Inicjalizacja struktury

	GPIO_PinAFConfig(Pulser_A_Port, Pulser_A_Pin_Source, Pulser_A_GPIO_AF);
	GPIO_PinAFConfig(Pulser_B_Port, Pulser_B_Pin_Source, Pulser_B_GPIO_AF);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - Motor DC --------------------*/
void MotorDC_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin = Enable_L298_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Enable_L298_Port, &GPIO_InitStruct);

	GPIO_PinAFConfig(Enable_L298_Port, Enable_L298_PinSource, Enable_L298_GPIO_AF);

	GPIO_InitStruct.GPIO_Pin = Dir_L298_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(Dir_L298_Port, &GPIO_InitStruct);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - Servo --------------------*/
void TurnServo_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	//Configure Servo_Pin as alternative function
	GPIO_InitStruct.GPIO_Pin = Servo_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Servo_Port, &GPIO_InitStruct);

	GPIO_PinAFConfig(Servo_Port, Servo_PinSource, Servo_GPIO_AF);	//Configure pin as AF
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - --------------------*/

/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - Czujnik oswietlenie --------------------*/
void LightSensor_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin = LightSensor_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LightSensor_Port, &GPIO_InitStruct);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - Podswietlenie LCD TFT --------------------*/
void LedLCD_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin = LedLCD_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(LedLCD_Port, &GPIO_InitStruct);
/*
	//Configure LedLCD_Pin as alternative function
	GPIO_InitStruct.GPIO_Pin = LedLCD_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LedLCD_Port, &GPIO_InitStruct);

	GPIO_PinAFConfig(LedLCD_Port, LedLCD_PinSource, LedLCD_GPIO_AF); */	//Configure pin as AF
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja - Oswietlenie zewnetrzne --------------------*/
void Lighting_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;	//Create structure for GPIO

	//Dipped beam ---------------------------------------------------------------------------------------------------------------------------------------------------------------
		GPIO_InitStruct.GPIO_Pin = DippedBeam_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(DippedBeam_Port, &GPIO_InitStruct);
	//Left turn signal ---------------------------------------------------------------------------------------------------------------------------------------------------------------
		GPIO_InitStruct.GPIO_Pin = LeftTurnSignal_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(LeftTurnSignal_Port, &GPIO_InitStruct);
	//Right turn signal ---------------------------------------------------------------------------------------------------------------------------------------------------------------
		GPIO_InitStruct.GPIO_Pin = RightTurnSignal_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(RightTurnSignal_Port, &GPIO_InitStruct);
	//Rear fog lamps ---------------------------------------------------------------------------------------------------------------------------------------------------------------
		GPIO_InitStruct.GPIO_Pin = RearFogLamps_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(RearFogLamps_Port, &GPIO_InitStruct);
	//Rear light ---------------------------------------------------------------------------------------------------------------------------------------------------------------
		GPIO_InitStruct.GPIO_Pin = RearLight_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(RearLight_Port, &GPIO_InitStruct);
	//Reversing lamps ---------------------------------------------------------------------------------------------------------------------------------------------------------------
		GPIO_InitStruct.GPIO_Pin = ReversingLamps_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(ReversingLamps_Port, &GPIO_InitStruct);
	//Stop light ---------------------------------------------------------------------------------------------------------------------------------------------------------------
		GPIO_InitStruct.GPIO_Pin = StopLight_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(StopLight_Port, &GPIO_InitStruct);
	//Horn ---------------------------------------------------------------------------------------------------------------------------------------------------------------
		GPIO_InitStruct.GPIO_Pin = Horn_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(Horn_Port, &GPIO_InitStruct);
	//Info LED ---------------------------------------------------------------------------------------------------------------------------------------------------------------
		GPIO_InitStruct.GPIO_Pin = InfoLed_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(InfoLed_Port, &GPIO_InitStruct);
		InfoLed_SET;	//Informacja o tym ze robot sie wlacza i konfiguruje

	//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		//Daytime running lamp left ---------------------------------------------------------------------------------------------------------------------------------------------------------------
			GPIO_InitStruct.GPIO_Pin = DaytimeRunningLampLeft_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(DaytimeRunningLampLeft_Port, &GPIO_InitStruct);
		//Daytime running lamp right ---------------------------------------------------------------------------------------------------------------------------------------------------------------
			GPIO_InitStruct.GPIO_Pin = DaytimeRunningLampRight_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(DaytimeRunningLampRight_Port, &GPIO_InitStruct);
		//Led inside car ---------------------------------------------------------------------------------------------------------------------------------------------------------------
			GPIO_InitStruct.GPIO_Pin = LedInsideCar_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(LedInsideCar_Port, &GPIO_InitStruct);
		//Front position/main lamps ---------------------------------------------------------------------------------------------------------------------------------------------------------------
			GPIO_InitStruct.GPIO_Pin = FrontPositionMainLamps_Pin; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(FrontPositionMainLamps_Port, &GPIO_InitStruct);

		GPIO_PinAFConfig(DaytimeRunningLampLeft_Port,  DaytimeRunningLampLeft_PinSource,  DaytimeRunningLampLeft_GPIO_AF);
		GPIO_PinAFConfig(DaytimeRunningLampRight_Port, DaytimeRunningLampRight_PinSource, DaytimeRunningLampRight_GPIO_AF);
		GPIO_PinAFConfig(LedInsideCar_Port,            LedInsideCar_PinSource,            LedInsideCar_GPIO_AF);
		GPIO_PinAFConfig(FrontPositionMainLamps_Port,  FrontPositionMainLamps_PinSource,  FrontPositionMainLamps_GPIO_AF);
}
/*-------------------- Koniec konfiguracja --------------------*/
