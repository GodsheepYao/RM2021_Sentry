#ifndef _PTZ_Init_task_
#define _PTZ_Init_task_

#include "RMLibHead.h"

#include "ramp.h"

#include "FreeRTOS_Task.h"

extern TaskHandle_t PTZ_Init_Handler;
void PTZ_Init_task(void *pvParameters);

#endif
