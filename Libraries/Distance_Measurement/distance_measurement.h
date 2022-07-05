void BT_SendDistanceFromObstacle(void);

uint8_t SharpFront_ReadData(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);

void US_ReadDistanceFromObstacle(int a);
void TIM6_DAC_IRQHandler(void);

void SharpSensor_Init(void);
void UltrasonicSensors_Init(void);
