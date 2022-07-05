#include "imu.h"
#include "tm_stm32f4_gpio.h"
#include "tm_stm32f4_i2c.h"
#include "tm_stm32f4_mpu6050.h"
#include "bt_hc_05.h"
#include "lcd_tft.h"
#include "tm_stm32f4_ili9341.h"

#include <stdio.h>		//sprintf

int AcceXData = 0, AcceYData = 0, AcceZData = 0;	//Dane z akcelerometru os X, Y, Z
int GyroXData = 0, GyroYData = 0, GyroZData = 0;	//Dane z zyroskopu os X, Y, Z

TM_MPU6050_t MPU6050_Data0;			//Nazwa struktury dla MPU6050

extern uint8_t EnableDisplayLCD;
extern uint8_t BargraphValue;

void Read_Accelerometer_Data(void)
{
	TM_MPU6050_ReadAccelerometer(&MPU6050_Data0);
		AcceXData = MPU6050_Data0.Accelerometer_X;
		AcceYData = MPU6050_Data0.Accelerometer_Y;
		AcceZData = MPU6050_Data0.Accelerometer_Z;
}

void BT_Send_Accelerometer(void)
{
	//char znaki[30];

	//sprintf(znaki, "%d\n%d\n%d\n", AcceXData, AcceYData, AcceZData); //konwersja zmiennej na znaki
	//BT_SendString(znaki);//wyslanie impulsow przez BT
}

void Read_Gyroscope_Data(void)
{
	TM_MPU6050_ReadGyroscope(&MPU6050_Data0);
		GyroXData = MPU6050_Data0.Gyroscope_X;
		GyroYData = MPU6050_Data0.Gyroscope_Y;
		GyroZData = MPU6050_Data0.Gyroscope_Z;
}

void BT_Send_Gyroscope(void)
{
	//char znaki[30];

	//sprintf(znaki, "%d\n%d\n%d\n", GyroXData, GyroYData, GyroZData); //konwersja zmiennej na znaki
	//BT_SendString(znaki);//wyslanie impulsow przez BT
}

void IMUSensor_Init(void)
{
	if(EnableDisplayLCD == 1) { LCD_ClearRow1(); TM_ILI9341_Puts(104, 105, "IMU sensor", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
												 TM_ILI9341_Puts(49, 124, "gryo & accelerometer", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }
	//TM_MPU6050_Init(&MPU6050_Data0, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_250s);
	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('s'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('e'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; LCD_WriteTheTypeOfPeriphery(0); LCD_ClearRow2(); }

	if(EnableDisplayLCD == 1) { LCD_ClearRow2(); TM_ILI9341_Puts(93, 124, "magnetometer", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK); }
	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('g'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('s'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; }
	if(EnableDisplayLCD == 1) { LCD_WriteTheTypeOfPeriphery('e'); LCD_DrawBargraph(BargraphValue); ++BargraphValue; LCD_ClearAllRow(); }

}
