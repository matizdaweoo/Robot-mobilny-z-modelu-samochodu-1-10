#include <stdint.h>

void BT_SendString(volatile char* s, uint8_t len);
void USART3_IRQHandler(void);
int8_t BT_ReadBuffer(void);
void BT_HC05_Init(void);
