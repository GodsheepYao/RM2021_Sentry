#ifndef _PID1_task_
#define _PID1_task_

#include "FreeRTOS_Task.h"

#include "RMLibHead.h"

extern TaskHandle_t Fire_Mechanism_task_Handler;
void Fire_Mechanism_Task(void *pvParameters);

#endif
