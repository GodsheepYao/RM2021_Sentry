#ifndef _PTZ_Runtime_task_
#define _PTZ_Runtime_task_

#include "RMLibHead.h"

RMLIB_CPP_BEGIN

extern TaskHandle_t PTZ_Runtime_Handler;

void PTZ_Runtime_task(void *pvParameters);

RMLIB_CPP_END

#endif
