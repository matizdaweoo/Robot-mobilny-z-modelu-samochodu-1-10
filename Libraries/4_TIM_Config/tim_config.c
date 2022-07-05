#include "stm32f4xx.h"

#include "allDefines.h"
#include "tim_config.h"

//timer_tick_freq = Timer_default_freq / (prescaller_set + 1)	Czestotliwosc taktowania timera = Podstawowa czestotliwosc timera / (Prescaler + 1)
//PWM_freq = timer_tick_freq / (TIM_Period + 1)					Czestotliwosc PWM = Czestotliwosc taktowania timera / (okres timera +1)
//TIM_Period = (timer_tick_freq / PWM_freq) - 1					Okres timera = (Czestotliwosc taktowania timera / Czestotliwosc PWM) - 1

//pulse_length = ((TIM_Period + 1) * DutyCycle (percent between 0 and 100%)) / 100 - 1
//pulse_length = ((524 + 1) * 0) / 100 - 1 = 0
//pulse_length = ((524 + 1) * 100) / 100 - 1 = 524

/*-------------------- Start konfiguracja TIM1 - Encoder --------------------*/
void Encoder_TIM_Init(void)
{
	/*TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_Cmd(TIM1, ENABLE); */

	// Konfiguracja Channel 1 dla TIM1
	TIM1 -> CCMR1 &= ~TIM_CCMR1_CC1S_1;							//CC1S = 01
	TIM1 -> CCMR1 |=  TIM_CCMR1_CC1S_0;							//CC1 channel is configured as input, IC1 is mapped on TI1
	TIM1 -> CCMR1 &= ~TIM_CCMR1_IC1PSC; 						//IC1 prescaler, IC1PSC = 00, no prescaler
	TIM1 -> CCMR1 &= ~TIM_CCMR1_IC1F;							//IC1 filter, IC1F = 0000, no filter
	TIM1 -> CCER  &= ~TIM_CCER_CC1P | TIM_CCER_CC1NP; 			//CC1 polarity, noninverted / rising edge

	// Konfiguracja Channel 2 dla TIM1
	TIM1 -> CCMR1 &= ~TIM_CCMR1_CC2S_1;							//CC2S = 01
	TIM1 -> CCMR1 |=  TIM_CCMR1_CC2S_0;							//CC2 channel is configured as input, IC2 is mapped on TI2
	TIM1 -> CCMR1 &= ~TIM_CCMR1_IC2PSC; 						//IC2 prescaler, IC2PSC = 00, no prescaler
	TIM1 -> CCMR1 &= ~TIM_CCMR1_IC2F;							//IC2 filter, IC2F = 0000, no filter
	TIM1 -> CCER  &= ~TIM_CCER_CC2P | TIM_CCER_CC2NP; 			//CC2 polarity, noninverted / rising edge

	TIM1 -> SMCR  &= ~TIM_SMCR_SMS_2;							//Slave mode selection, SMS = 011
	TIM1 -> SMCR  |=  TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;			//Encoder mode 3, Counter count up/down on both TI1FP1 and TI2FP2 edges

	TIM1 -> CR1   |=  TIM_CR1_CEN;								//Counter enable
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM2 - --------------------*/

/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM3 - Licznik impulsow --------------------*/
void MotorImpulse_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_ICInitTypeDef TIM_ICStruct;

	TIM_BaseStruct.TIM_Prescaler = 8400;		//84MHz / 8 400Hz = 10 000 Hz
	TIM_BaseStruct.TIM_Period = 65000;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);

	TIM_ICStruct.TIM_Channel = TIM_Channel_3;
	TIM_ICStruct.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICStruct.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM3, &TIM_ICStruct);

	TIM_ICStruct.TIM_Channel = TIM_Channel_4;
	TIM_ICStruct.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICStruct.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM3, &TIM_ICStruct);

	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);

	TIM_Cmd(TIM3, ENABLE);

	NVIC_EnableIRQ(TIM3_IRQn);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM4 - Oswietlenie zewnetrzne --------------------*/
void Lighting_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_OCInitTypeDef TIM_OCStruct;

	TIM_BaseStruct.TIM_Prescaler = 419;					 		//Dzielnik zegara (84 000 000 [Hz] / (419 + 1) = 200 000 [Hz])
	TIM_BaseStruct.TIM_Period = 99;					 			//TIM_Period = 200 000 (TIM4 Clock) / 2000 Hz(Period PWM) - 1 = 99
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up; 		//Zliczanie w gore
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;	 		//Dodatkowy dzielnik
	TIM_BaseStruct.TIM_RepetitionCounter = 0;			 		//
	TIM_TimeBaseInit(TimerForLight, &TIM_BaseStruct);	 		//Inicjalizacja timera z podanymi ustawieniami
		TIM_Cmd(TimerForLight, ENABLE);						 	//Wlaczenie timera

	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;			 		//Po odliczeniu ma wyzerowac wyjscie PWM
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;		//Stan wysoki na wyjsciu PWM gdy zaczyna sie okres
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;			//Polaryzacja wyjsciowa

	TIM_OCStruct.TIM_Pulse = 0;									//Front position/main lamps
	TIM_OC1Init(TimerForLight, &TIM_OCStruct);
	TIM_OC1PreloadConfig(TimerForLight, TIM_OCPreload_Enable);

	TIM_OCStruct.TIM_Pulse = 100;								//Led inside car
	TIM_OC2Init(TimerForLight, &TIM_OCStruct);
	TIM_OC2PreloadConfig(TimerForLight, TIM_OCPreload_Enable);

	TIM_OCStruct.TIM_Pulse = 0;									//Daytime running lamp right
	TIM_OC3Init(TimerForLight, &TIM_OCStruct);
	TIM_OC3PreloadConfig(TimerForLight, TIM_OCPreload_Enable);

	TIM_OCStruct.TIM_Pulse = 0;									//Daytime running lamp left
	TIM_OC4Init(TimerForLight, &TIM_OCStruct);
	TIM_OC4PreloadConfig(TimerForLight, TIM_OCPreload_Enable);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM5 - --------------------*/

/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM6 - Odliczenie 1us --------------------*/
void Count_1us_TIM_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;								//Struktura dla konfiguracji NVIC

	TIM6 -> DIER |=  TIM_DIER_UIE;								//Wlaczenie przrwania update
	TIM6 -> CR1  |=  TIM_CR1_ARPE; 								//Auto-reload preload
	TIM6 -> CR1  &= ~TIM_CR1_OPM; 								//NO One pulse mode
	TIM6 -> CNT   =  0; 										//Wartosc naliczona
	TIM6 -> PSC   =  83; 										//84MHz / (83 + 1) = 1MHz
	TIM6 -> ARR   =  11600; 									//wartosc max do jakiem ma liczyc, czyli czujnik zmierzy > 2m
	TIM6 -> CR1  &= ~TIM_CR1_CEN; 								//Counter disable

	//Enable and set EXTI0 interrupt
	NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	//NVIC_EnableIRQ(TIM6_DAC_IRQn);								//Aktywacja przerwanie dla TIM6
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM7 - Odliczanie 1ms --------------------*/
void Count_1ms_TIM_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;

	/* TIMER 7, Taktowanie magistrali 42MHz * 2 = 84MHz */
	TIM7 -> DIER |= TIM_DIER_UIE;								//Wlaczenie przrwania update
	TIM7 -> CR1 |= TIM_CR1_ARPE;								//Auto-reload preload
	TIM7 -> CR1 &= ~TIM_CR1_OPM;								//NO One pulse mode
	TIM7 -> CNT = 0;											//Wartosc naliczona
	TIM7 -> PSC = 8399;											//84MHz / (8 399 + 1) = 10kHz (T = 100ns)
	TIM7 -> ARR = 9;											//T * (ARR + 1) = 1ms
	TIM7 -> CR1 |= TIM_CR1_CEN;										//Counter enable

	//Enable and set EXTI0 interrupt
	NVIC_InitStruct.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	//NVIC_EnableIRQ(TIM7_IRQn);									//Aktywacja przerwanie dla TIM7
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM8 -  --------------------*/

/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM9 - Sterowanie kamera --------------------*/
void Pixy_PanTilt_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_OCInitTypeDef TIM_OCStruct;

	TIM_BaseStruct.TIM_Prescaler = 1679;					 		//Dzielnik zegara (168 000 000 [Hz] / (1679 + 1) = 100 000 [Hz])
	TIM_BaseStruct.TIM_Period = 1999;					 		//TIM_Period = 100 000 (TIM4 Clock) / 50 Hz(Period PWM) - 1 = 1999
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up; 		//Zliczanie w gore
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;	 		//Dodatkowy dzielnik
	TIM_BaseStruct.TIM_RepetitionCounter = 0;			 		//
	TIM_TimeBaseInit(CMUCam_PanTilt_TIM, &TIM_BaseStruct);	 		//Inicjalizacja timera z podanymi ustawieniami
		TIM_Cmd(CMUCam_PanTilt_TIM, ENABLE);						 	//Wlaczenie timera

	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;			 		//Po odliczeniu ma wyzerowac wyjscie PWM
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;		//Stan wysoki na wyjsciu PWM gdy zaczyna sie okres
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;			//Polaryzacja wyjsciowa

	TIM_OCStruct.TIM_Pulse = 130;									//CMUCam Pan (obrot) 217-lewo, 130-srodek, 50-prawo
	TIM_OC1Init(CMUCam_PanTilt_TIM, &TIM_OCStruct);
	TIM_OC1PreloadConfig(CMUCam_PanTilt_TIM, TIM_OCPreload_Enable);

	TIM_OCStruct.TIM_Pulse = 90;									//CMUCam Tilt (przechylenie) 210-przechyl do tylu, 90-srodek, 60-przechyl do przodu
	TIM_OC2Init(CMUCam_PanTilt_TIM, &TIM_OCStruct);
	TIM_OC2PreloadConfig(CMUCam_PanTilt_TIM, TIM_OCPreload_Enable);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM10 --------------------*/

/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM11 - Podswietlenie LCD --------------------*/
void LedLCD_TIM_Init(void)
{
	/*TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_OCInitTypeDef TIM_OCStruct;

	TIM_BaseStruct.TIM_Prescaler = 83;						//84 000 000 [Hz] / 84 = 1 000 000 [Hz]
	TIM_BaseStruct.TIM_Period = 499;							//TIM_Period = 1 000 000 (TIM13 Clock) / 2000 Hz(Period PWM) - 1 = 499
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(LedLCD_TIM, &TIM_BaseStruct);

	TIM_Cmd(LedLCD_TIM, ENABLE);

	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OCStruct.TIM_Pulse = 0;
	TIM_OC1Init(LedLCD_TIM, &TIM_OCStruct);
	TIM_OC1PreloadConfig(LedLCD_TIM, TIM_OCPreload_Enable); */
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM12 -  --------------------*/

/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM13 - Motor DC --------------------*/
void MotorDC_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_OCInitTypeDef TIM_OCStruct;

	TIM_BaseStruct.TIM_Prescaler = 83;							//84 000 000 [Hz] / (83 + 1) = 1 000 000 [Hz]
	TIM_BaseStruct.TIM_Period = 499;							//TIM_Period = (1 000 000 (TIM10 Clock) / 2000 Hz(Period PWM)) - 1 = 499
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(Enable_L298_TIM, &TIM_BaseStruct);
		TIM_Cmd(Enable_L298_TIM, ENABLE);

	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OCStruct.TIM_Pulse = 0;									//Zakres od 0 do 500
	TIM_OC1Init(Enable_L298_TIM, &TIM_OCStruct);
	TIM_OC1PreloadConfig(Enable_L298_TIM, TIM_OCPreload_Enable);
}
/*-------------------- Koniec konfiguracja --------------------*/

/*-------------------- Start konfiguracja TIM14 - Servo --------------------*/
void TurnServo_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_OCInitTypeDef TIM_OCStruct;

	TIM_BaseStruct.TIM_Prescaler = 31;						//Wartosc dzielnika wewnetrznego zegara, ktorym bedzie taktowany timer (od 0 do 65535), 84 [kHz] / (31 + 1) = 2 625 [kHz]
	TIM_BaseStruct.TIM_Period = 52499;							//Wartosc do ktorej ma zliczac timer, TIM_Period = (2 625kHz (TIM13 Clock) / 50 Hz(Period PWM)) - 1 = 52499
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;		//zliczanie w gore lub w dol
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;			//jeszcze jedno miejsce gdzie mozna podzielic sygnal taktujacy timer
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(Servo_TIM, &TIM_BaseStruct);
		TIM_Cmd(Servo_TIM, ENABLE);

	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OCStruct.TIM_Pulse = 3785;								//Zakres od 99 do 199, srodek to 149
	TIM_OC1Init(Servo_TIM, &TIM_OCStruct);
	TIM_OC1PreloadConfig(Servo_TIM, TIM_OCPreload_Enable);
}
/*-------------------- Koniec konfiguracja --------------------*/
