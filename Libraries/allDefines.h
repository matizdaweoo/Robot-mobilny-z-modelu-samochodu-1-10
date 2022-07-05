#define liczbaPi 3.141592653589793

/*-------------------- Start Definicje - Bluetooth --------------------*/
	#define BT_baudrate							9600							//Predkosc transmisji (9600, 14400, 19200, 38400, 56000)
	#define BT_UART_RX_BUF_SIZE 				64								//Definiuje rozmiar bufora odbiorczego na 32 bajty
	#define BT_UART_RX_BUF_MASK					(BT_UART_RX_BUF_SIZE - 1)		//Definiuje maske dla bufora odbiorczego (pomocna przy obliczeniu zapetlenia bufora)

	#define BT_UART_GPIO_Port					GPIOD
	#define BT_UART_TX_Pin						GPIO_Pin_8
	#define BT_UART_RX_Pin						GPIO_Pin_9
	#define BT_UART_TX_Pin_Source				GPIO_PinSource8
	#define BT_UART_RX_Pin_Source				GPIO_PinSource9
	#define BT_UART_GPIO_AF						GPIO_AF_USART3
	#define BT_UART								USART3
	#define BT_UART_IRQ							USART3_IRQn

	#define BT_Enable_Pin						GPIO_Pin_14
	#define BT_Enable_Port						GPIOB
	#define BT_Enable_SET						GPIO_SetBits(BT_Enable_Port, BT_Enable_Pin)
	#define BT_Enable_RESET						GPIO_ResetBits(BT_Enable_Port, BT_Enable_Pin)
	#define BT_Enable_TOGGLE					GPIO_ToggleBits(BT_Enable_Port, BT_Enable_Pin)

	#define BT_State_Pin						GPIO_Pin_15
	#define BT_State_Port						GPIOB
	#define BT_EXTI_Pin_Source					EXTI_PinSource15
	#define BT_EXTI_Port_Source					EXTI_PortSourceGPIOB
		#define BT_State_ReadInputDataBit		GPIO_ReadInputDataBit(BT_State_Port, BT_State_Pin)
/*-------------------- Koniec Definicje --------------------*/

/*-------------------- Start Definicje - CMUCam5 Pixy --------------------*/
	#define CMUCam_baudrate 					9600		//19200
	#define CMUCam_UART_RX_BUF_SIZE 			32								//Definiuje rozmiar bufora odbiorczego na 12 bajty bo tyle pluje CMUCam
	#define CMUCam_UART_RX_BUF_MASK				(CMUCam_UART_RX_BUF_SIZE - 1)	//Definiuje maske dla bufora odbiorczego (pomocna przy obliczeniu zapetlenia bufora)

	#define CMUCam_UART_GPIO_Port				GPIOC
	#define CMUCam_UART_TX_Pin					GPIO_Pin_6
	#define CMUCam_UART_RX_Pin					GPIO_Pin_7
	#define CMUCma_UART_TX_Pin_Source			GPIO_PinSource6
	#define CMUCam_UART_RX_Pin_Source			GPIO_PinSource7
	#define CMUCam_UART_GPIO_AF					GPIO_AF_USART6
	#define CMUCam_UART							USART6
	#define CMUCam_UART_IRQ						USART6_IRQn
/*-------------------- Koniec Definicje --------------------*/

/*-------------------- Start Definicje - Statyw do CMUCam5 Pixy --------------------*/
	#define CMUCam_Pan_Pin						GPIO_Pin_5		//Pan - Obrot kamery (PWM)
	#define CMUCam_Pan_Port						GPIOE
	#define CMUCam_Pan_PinSource				GPIO_PinSource5	//Pin zrodlowy

	#define CMUCam_Tilt_Pin						GPIO_Pin_6		//Tilt - Pochylenie kamery (PWM)
	#define CMUCam_Tilt_Port					GPIOE
	#define CMUCam_Tilt_PinSource				GPIO_PinSource6	//Pin zrodlowy

	#define CMUCam_PanTilt_GPIO_AF				GPIO_AF_TIM9	//Jakie zadanie bedzie miec ten pin
	#define CMUCam_PanTilt_TIM					TIM9
/*-------------------- Koniec Definicje --------------------*/

/*-------------------- Start Definicje - Czujniki odleglosci Sharp - przod --------------------*/
	#define SharpFront_NrSensors				5
	#define SharpFront_NrSamples				5
	#define SharpFront_AllSamples				(SharpFront_NrSensors * SharpFront_NrSamples)

	#define SharpFront1_Pin						GPIO_Pin_0
	#define SharpFront1_Port					GPIOC
	#define SharpFront2_Pin						GPIO_Pin_1
	#define SharpFront2_Port					GPIOC
	#define SharpFront3_Pin						GPIO_Pin_2
	#define SharpFront3_Port					GPIOC
	#define SharpFront4_Pin						GPIO_Pin_3
	#define SharpFront4_Port					GPIOC
	#define SharpFront5_Pin						GPIO_Pin_0
	#define SharpFront5_Port					GPIOA
/*-------------------- Koniec Definicje --------------------*/

/*-------------------- Start Definicje - Czujniki odleglosci Sharp - tyl --------------------*/
	#define SharpRear_NrSensors					5
	#define SharpRear_NrSamples					5
	#define SharpRear_AllSamples				(SharpRear_NrSensors * SharpRear_NrSamples)

	#define SharpRear1_Pin						GPIO_Pin_1
	#define SharpRear1_Port						GPIOA
	#define SharpRear2_Pin						GPIO_Pin_2
	#define SharpRear2_Port						GPIOA
	#define SharpRear3_Pin						GPIO_Pin_3
	#define SharpRear3_Port						GPIOA
	#define SharpRear4_Pin						GPIO_Pin_4
	#define SharpRear4_Port						GPIOA
	#define SharpRear5_Pin						GPIO_Pin_5
	#define SharpRear5_Port						GPIOA
/*-------------------- Koniec Definicje --------------------*/

/*-------------------- Start Definicje - Czujnik odleglosci ultradzwiekowe - przod i tyl --------------------*/
/* Sonar centralny z przodu */
	#define HC_SR04_FRONT_TRIGGER_Pin			GPIO_Pin_11
	#define HC_SR04_FRONT_TRIGGER_Port			GPIOE
		#define HC_SR04_FRONT_TRIGGER_SET			GPIO_SetBits(HC_SR04_FRONT_TRIGGER_Port, HC_SR04_FRONT_TRIGGER_Pin);
		#define HC_SR04_FRONT_TRIGGER_RESET			GPIO_ResetBits(HC_SR04_FRONT_TRIGGER_Port, HC_SR04_FRONT_TRIGGER_Pin);

	#define HC_SR04_FRONT_ECHO_Pin				GPIO_Pin_3
	#define HC_SR04_FRONT_ECHO_Port				GPIOD
	#define HC_SR04_FRONT_ECHO_Pin_Source		EXTI_PinSource3
	#define HC_SR04_FRONT_ECHO_Port_Source		EXTI_PortSourceGPIOD
		#define HC_SR04_FRONT_ECHO_READ_INPUT		GPIO_ReadInputDataBit(HC_SR04_FRONT_ECHO_Port, HC_SR04_FRONT_ECHO_Pin)

	/* Sonar centralny z ty³u */
	#define HC_SR04_REAR_TRIGGER_Pin			GPIO_Pin_10
	#define HC_SR04_REAR_TRIGGER_Port			GPIOE
		#define HC_SR04_REAR_TRIGGER_SET			GPIO_SetBits(HC_SR04_REAR_TRIGGER_Port, HC_SR04_REAR_TRIGGER_Pin);
		#define HC_SR04_REAR_TRIGGER_RESET			GPIO_ResetBits(HC_SR04_REAR_TRIGGER_Port, HC_SR04_REAR_TRIGGER_Pin);

	#define HC_SR04_REAR_ECHO_Pin				GPIO_Pin_1
	#define HC_SR04_REAR_ECHO_Port				GPIOD
	#define HC_SR04_REAR_ECHO_Pin_Source		EXTI_PinSource1
	#define HC_SR04_REAR_ECHO_Port_Source		EXTI_PortSourceGPIOD
		#define HC_SR04_REAR_ECHO_READ_INPUT		GPIO_ReadInputDataBit(HC_SR04_REAR_ECHO_Port, HC_SR04_REAR_ECHO_Pin)
/*-------------------- Koniec Definicje --------------------*/

/*-------------------- Start Definicje - Enkoder --------------------*/
//Encoder canal A
	#define Encoder_A_Pin						GPIO_Pin_8
	#define Encoder_A_Port						GPIOA
	#define Encoder_A_Pin_Source				GPIO_PinSource8
	#define Encoder_A_GPIO_AF					GPIO_AF_TIM1
	//#define Encoder_A_EXTI_IRQ				EXTI3_IRQn
	//	#define Encoder_A_ReadInputDataBit		GPIO_ReadInputDataBit(Encoder_A_Port, Encoder_A_Pin)

//Encoder canal B
	#define Encoder_B_Pin						GPIO_Pin_9
	#define Encoder_B_Port						GPIOA
	#define Encoder_B_Pin_Source				GPIO_PinSource9
	#define Encoder_B_GPIO_AF					GPIO_AF_TIM1
	//#define Encoder_B_EXTI_IRQ				EXTI2_IRQn
	//	#define Encoder_B_ReadInputDataBit		GPIO_ReadInputDataBit(Encoder_B_Port, Encoder_B_Pin)
/*-------------------- Koniec Definicje --------------------*/

/*-------------------- Start konfiguracja - Impulsator silnika --------------------*/
//Pulser canal A
	#define Pulser_A_Pin						GPIO_Pin_8
	#define Pulser_A_Port						GPIOC
	#define Pulser_A_Pin_Source					GPIO_PinSource8
	#define Pulser_A_GPIO_AF					GPIO_AF_TIM3

//Pulser canal B
	#define Pulser_B_Pin						GPIO_Pin_9
	#define Pulser_B_Port						GPIOC
	#define Pulser_B_Pin_Source					GPIO_PinSource9
	#define Pulser_B_GPIO_AF					GPIO_AF_TIM3
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start Definicje - Servo and Motor DC --------------------*/
	#define Servo_Pin							GPIO_Pin_7
	#define Servo_Port							GPIOA
	#define Servo_PinSource						GPIO_PinSource7	//Pin zrodlowy
	#define Servo_GPIO_AF						GPIO_AF_TIM14	//Jakie zadanie bedzie miec ten pin
	#define Servo_TIM							TIM14

	#define Enable_L298_Pin						GPIO_Pin_6
	#define Enable_L298_Port					GPIOA
	#define Enable_L298_PinSource				GPIO_PinSource6	//Pin zrodlowy
	#define Enable_L298_GPIO_AF					GPIO_AF_TIM13	//Jakie zadanie bedzie miec ten pin
	#define Enable_L298_TIM						TIM13

	#define Dir_L298_Pin						GPIO_Pin_0
	#define Dir_L298_Port						GPIOB
/*-------------------- Koniec Definicje --------------------*/

/*-------------------- Start Definicje - Czujnik oswietlenia --------------------*/
	#define LightSensor_Pin						GPIO_Pin_1
	#define LightSensor_Port					GPIOB
/*-------------------- Koniec Definicje --------------------*/

/*-------------------- Start Definicje - Oswietlenie zewnetrzne --------------------*/
	//PORT D !!!
	#define DaytimeRunningLampLeft_Pin			GPIO_Pin_15			//Dzienne lewe (PWM)
	#define DaytimeRunningLampLeft_Port			GPIOD
	#define DaytimeRunningLampLeft_PinSource	GPIO_PinSource15	//Pin zrodlowy
	#define DaytimeRunningLampLeft_GPIO_AF		GPIO_AF_TIM4		//Jakie zadanie bedzie miec ten pin

	#define DaytimeRunningLampRight_Pin			GPIO_Pin_14			//Dzienne prawe (PWM)
	#define DaytimeRunningLampRight_Port		GPIOD
	#define DaytimeRunningLampRight_PinSource	GPIO_PinSource14	//Pin zrodlowy
	#define DaytimeRunningLampRight_GPIO_AF		GPIO_AF_TIM4		//Jakie zadanie bedzie miec ten pin

	#define LedInsideCar_Pin					GPIO_Pin_13			//Podswietlenie wnetrza (PWM)
	#define LedInsideCar_Port					GPIOD
	#define LedInsideCar_PinSource				GPIO_PinSource13	//Pin zrodlowy
	#define LedInsideCar_GPIO_AF				GPIO_AF_TIM4		//Jakie zadanie bedzie miec ten pin

	#define FrontPositionMainLamps_Pin			GPIO_Pin_12			//Swiatla postojowe i dlugie (PWM)
	#define FrontPositionMainLamps_Port			GPIOD
	#define FrontPositionMainLamps_PinSource	GPIO_PinSource12	//Pin zrodlowy
	#define FrontPositionMainLamps_GPIO_AF		GPIO_AF_TIM4		//Jakie zadanie bedzie miec ten pin

	#define TimerForLight						TIM4

	#define DippedBeam_Pin						GPIO_Pin_13			//Swiatla krotkie
	#define DippedBeam_Port						GPIOB
	#define LeftTurnSignal_Pin					GPIO_Pin_10			//Lewy migacz
	#define LeftTurnSignal_Port					GPIOD
	#define RightTurnSignal_Pin					GPIO_Pin_11			//Prawy migacz
	#define RightTurnSignal_Port				GPIOD
	#define RearFogLamps_Pin					GPIO_Pin_12			//Lampa przeciwmglowa tylnia
	#define RearFogLamps_Port					GPIOB
	//PORT B !!!
	#define RearLight_Pin						GPIO_Pin_11			//Swiatlo pozycyjne tylne
	#define RearLight_Port						GPIOB
	#define ReversingLamps_Pin					GPIO_Pin_10			//Swiatlo cofania
	#define ReversingLamps_Port					GPIOB
	#define StopLight_Pin						GPIO_Pin_15			//Swiatlo stop
	#define StopLight_Port						GPIOE
	#define Horn_Pin							GPIO_Pin_14			//Klakson
	#define Horn_Port							GPIOE
	#define InfoLed_Pin							GPIO_Pin_13			//Dioda LED czerwona informacyjna w srodku auta
	#define InfoLed_Port						GPIOE

	#define LedLCD_Pin							GPIO_Pin_9			//Podswietlenie LCD	(PWM)
	#define	LedLCD_Port							GPIOB
	#define LedLCD_PinSource					GPIO_PinSource9		//Pin zrodlowy
	#define LedLCD_GPIO_AF						GPIO_AF_TIM11		//Jakie zadanie bedzie miec ten pin
	#define LedLCD_TIM							TIM11

//----------------------------------------------------------------------------------------------------------------
	//Wlaczenie / wylaczenie oswietlenia
	#define DippedBeam_SET						GPIO_SetBits(DippedBeam_Port, DippedBeam_Pin)
	#define DippedBeam_RESET					GPIO_ResetBits(DippedBeam_Port, DippedBeam_Pin)
	#define DippedBeam_TOGGLE					GPIO_ToggleBits(DippedBeam_Port, DippedBeam_Pin)

	#define LeftTurnSignal_SET					GPIO_SetBits(LeftTurnSignal_Port, LeftTurnSignal_Pin)
	#define LeftTurnSignal_RESET				GPIO_ResetBits(LeftTurnSignal_Port, LeftTurnSignal_Pin)
	#define LeftTurnSignal_TOGGLE				GPIO_ToggleBits(LeftTurnSignal_Port, LeftTurnSignal_Pin)

	#define RightTurnSignal_SET					GPIO_SetBits(RightTurnSignal_Port, RightTurnSignal_Pin)
	#define RightTurnSignal_RESET				GPIO_ResetBits(RightTurnSignal_Port, RightTurnSignal_Pin)
	#define RightTurnSignal_TOGGLE				GPIO_ToggleBits(RightTurnSignal_Port, RightTurnSignal_Pin)

	#define RearFogLamps_SET					GPIO_SetBits(RearFogLamps_Port, RearFogLamps_Pin)
	#define RearFogLamps_RESET					GPIO_ResetBits(RearFogLamps_Port, RearFogLamps_Pin)
	#define RearFogLamps_TOGGLE					GPIO_ToggleBits(RearFogLamps_Port, RearFogLamps_Pin)

	#define RearLight_SET						GPIO_SetBits(RearLight_Port, RearLight_Pin)
	#define RearLight_RESET						GPIO_ResetBits(RearLight_Port, RearLight_Pin)
	#define RearLight_TOGGLE					GPIO_ToggleBits(RearLight_Port, RearLight_Pin)

	#define ReversingLamps_SET					GPIO_SetBits(ReversingLamps_Port, ReversingLamps_Pin)
	#define ReversingLamps_RESET				GPIO_ResetBits(ReversingLamps_Port, ReversingLamps_Pin)
	#define ReversingLamps_TOGGLE				GPIO_ToggleBits(ReversingLamps_Port, ReversingLamps_Pin)

	#define StopLight_SET						GPIO_SetBits(StopLight_Port, StopLight_Pin)
	#define StopLight_RESET						GPIO_ResetBits(StopLight_Port, StopLight_Pin)
	#define StopLight_TOGGLE					GPIO_ToggleBits(StopLight_Port, StopLight_Pin)

	#define Horn_SET							GPIO_SetBits(Horn_Port, Horn_Pin)
	#define Horn_RESET							GPIO_ResetBits(Horn_Port, Horn_Pin)
	#define Horn_TOGGLE							GPIO_ToggleBits(Horn_Port, Horn_Pin)

	#define InfoLed_SET							GPIO_SetBits(InfoLed_Port, InfoLed_Pin)
	#define InfoLed_RESET						GPIO_ResetBits(InfoLed_Port, InfoLed_Pin)
	#define InfoLed_TOGGLE						GPIO_ToggleBits(InfoLed_Port, InfoLed_Pin)

	#define LedLCD_SET							GPIO_SetBits(LedLCD_Port, LedLCD_Pin)
	#define LedLCD_RESET						GPIO_ResetBits(LedLCD_Port, LedLCD_Pin)
	#define LedLCD_TOGGLE						GPIO_ToggleBits(LedLCD_Port, LedLCD_Pin)
/*-------------------- Koniec Definicje --------------------*/
