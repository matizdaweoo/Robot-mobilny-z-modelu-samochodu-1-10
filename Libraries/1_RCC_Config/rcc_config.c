#include "stm32f4xx.h"

void RCC_Config_Init(void)	//Inicjalize function for RCC
{
	/* Wlaczenie: portow A, B, C, D, E */
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN  |
					  RCC_AHB1ENR_GPIOBEN  |
					  RCC_AHB1ENR_GPIOCEN  |
					  RCC_AHB1ENR_GPIODEN  |
					  RCC_AHB1ENR_GPIOEEN  |
					  RCC_AHB1ENR_DMA1EN   |
					  RCC_AHB1ENR_DMA2EN   ;

	/** Wlaczenie:
	  * TIM3   (Zliczanie impulsow z enkodera),
	  * TIM4   (Daytime light left/fight, front position/main light, light inside car,
	  * TIM6   (Odmierzanie czasu co 1us) dla czujników odleg³osci
	  * TIM7   (Odmierzanie czasu co 1ms) dla odswierzania predkosci, migaczy itp.
	  * TIM13  (Servo)
	  * TIM14  (Motor DC)
	  * USART3 (komunikacja z BT),
	  * SPI3   (Wyswietlacz LCD i Touchpad)
	  * I2C    (Modul zasilania)
	  */
	RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN   |
					  RCC_APB1ENR_TIM4EN   |
					  RCC_APB1ENR_TIM6EN   |
					  RCC_APB1ENR_TIM7EN   |
					  RCC_APB1ENR_TIM13EN  |
					  RCC_APB1ENR_TIM14EN  |
					  RCC_APB1ENR_USART3EN |
					  RCC_APB1ENR_SPI3EN   |
					  RCC_APB1ENR_I2C1EN   ;

	/** Wlaczenie:
	  * SYSCFG (Uruchomienie przerwan)
	  * TIM1   (Encoder channel A and B),
	  * TIM9   (Servos for Pan/Tilt CMUCam5 Pixy)
	  * TIM11  (Led_LCD Motor DC)
	  * USART6 (Komunikacja z CMUCam5 Pixy)
	  * ADC1   (Light sensor)
	  */
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN |
					  RCC_APB2ENR_TIM1EN   |
					  RCC_APB2ENR_TIM9EN   |
					  RCC_APB2ENR_TIM11EN  |
					  RCC_APB2ENR_USART6EN |
					  RCC_APB2ENR_ADC1EN   |
					  RCC_APB2ENR_ADC2EN   |
					  RCC_APB2ENR_ADC3EN   ;
}
