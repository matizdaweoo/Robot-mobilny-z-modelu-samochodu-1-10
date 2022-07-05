#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

#include "allDefines.h"
#include "lighting.h"
#include "gpio_config.h"
#include "tim_config.h"

void OffAllLamps(void)
{
	DaytimeRunningLamp_RESET();
	FrontPositionMainLamps_RESET();
	DippedBeam_RESET;
	LeftTurnSignal_RESET;
	RightTurnSignal_RESET;
	RearFogLamps_RESET;
	RearLight_RESET;
	ReversingLamps_RESET;
	StopLight_RESET;
}

void FrontMainLamps(void)									//Wlaczenie swiatel dlugich, zakres od 0 do 100
{
	TIM_SetCompare1(TimerForLight, 100);
}

void FrontPositionLamps(void)								//Wlaczenie swiatel postojowych
{
	TIM_SetCompare1(TimerForLight, 6);
}

void FrontPositionMainLamps_RESET(void)						//Wylaczenie swiatel dlugich lub postojowych
{
	TIM_SetCompare1(TimerForLight, 0);
}

void DaytimeRunningLamp_SET(void)							//Wlaczenie swiatel do jazdy dziennej
{
	TIM_SetCompare4(TimerForLight, 100);	//Left
	TIM_SetCompare3(TimerForLight, 100);	//Right
}

void DaytimeRunningLamp_RESET(void)							//Wylaczenie swiatel do jazdy dziennej
{
	TIM_SetCompare4(TimerForLight, 0);		//Left
	TIM_SetCompare3(TimerForLight, 0);		//Right
}

void DaytimeRunningLampLeft(uint8_t percent)					//Lekkie zapalenie lewej lampy dziennej
{
	TIM_SetCompare4(TimerForLight, percent);		//Left
}

void DaytimeRunningLampRight(uint8_t percent)				//Lekkie zapalenie pawej lampy dziennej
{
	TIM_SetCompare3(TimerForLight, percent);		//Right
}

void LedInsideCar(uint8_t percent)
{
	TIM_SetCompare2(TimerForLight, percent);
}

void Lighting_Init(void)
{
	Lighting_GPIO_Init();
	Lighting_TIM_Init();
}
