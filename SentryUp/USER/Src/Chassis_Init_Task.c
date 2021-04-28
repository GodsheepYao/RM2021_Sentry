#include "Chassis_Init_Task.h"
#include "FreeRTOS_Task.h"
#include "Remote_Task.h"

TaskHandle_t Chassis_Init_Handler;
void Chassis_Init_task(void *pvParameters) {
#if Down_Remote == 0
        xTaskCreate((TaskFunction_t)Remote_task,
            (const char *)"Remote_task",
            (uint16_t)256,
            (void *)NULL,
            (UBaseType_t)2,
            (TaskHandle_t *)&Remote_task_Handler);
#endif
        Robot_Status.RS_Ready = STATUS_TURN_ON;
        vTaskDelete(NULL);
}
