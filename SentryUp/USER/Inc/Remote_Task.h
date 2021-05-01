#ifndef _USER_REMOTE_H_
#define _USER_REMOTE_H_

#include "RMLibHead.h"
#include "Remote.h"

typedef struct __PACKED_REMOTE_STRUCT {
    int16_t ch2;
    int16_t ch3;
    int8_t s1;
    int8_t s2;
    int16_t Null;
}Remote_Rx_PACK_t;

extern uint8_t usart1_dma_buff[30];

extern SemaphoreHandle_t Remote_Semaphore;
extern TaskHandle_t Remote_task_Handler;

void Remote_task(void *pvParameters);

#endif

