#ifndef _PTZ_Init_task_
#define _PTZ_Init_task_

#include "RMLibHead.h"
#include "ramp.h"

extern TaskHandle_t PTZ_Init_Handler;
extern uint8_t PTZ_Init_Ready;

void PTZ_Init_task(void *pvParameters);

#endif
