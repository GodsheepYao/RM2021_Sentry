#ifndef _USER_REMOTE_H_
#define _USER_REMOTE_H_

#include "FreeRTOS_Task.h"
#include "Remote.h"

extern uint8_t usart1_dma_buff[30];

extern SemaphoreHandle_t Remote_Semaphore;
extern TaskHandle_t Remote_task_Handler;

void Remote_task(void *pvParameters);

#endif

