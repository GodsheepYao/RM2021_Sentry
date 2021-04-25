#include "Power_Protection_Task.h"
#include "PTZ_Init_Task.h"
#include "PTZ_Runtime_Task.h"
#include "Fire_Mechanism_Task.h"

SemaphoreHandle_t Power_Semaphore;
TaskHandle_t Power_Protection_Handler;
WatchDog_Flag_t DownDog_Flag;

void Power_Protection_task(void *pvParameters) {
    /* vTaskDelayUntil延时初始化 */
    portTickType xLastWakeTime = xTaskGetTickCount();
    for( ;; ) {
        WatchDog_Polling();
        if(!Robot_Status.RS_Dead && (Robot_Status.RS_Kill || DownDog_Flag == 0)) {
            Robot_Status.RS_Dead = 1;
            vTaskDelete(PTZ_Runtime_Handler);
            vTaskDelete(Fire_Mechanism_task_Handler); 
            } else if (Robot_Status.RS_Dead && (!Robot_Status.RS_Kill && (DownDog_Flag == 0x07))) { //忽略摩擦轮电机掉线
            Robot_Status.RS_Dead = 0;
            xTaskCreate((TaskFunction_t)PTZ_Init_task,
                (const char *)"PTZ_Init_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)1,
                (TaskHandle_t *)&PTZ_Init_Handler);
        }
        vTaskDelayUntil(&xLastWakeTime, 20);
    }
}

void WatchDog_CallBack(WatchDogp handle) {
    if(IS_Dog(handle, Yaw_Dog))       Dog_ClearStatus(DownDog_Flag, YawDog_Flag);
    if(IS_Dog(handle, Pitch_Dog))     Dog_ClearStatus(DownDog_Flag, PitchDog_Flag);
    if(IS_Dog(handle, Pluck_Dog))     Dog_ClearStatus(DownDog_Flag, PluckDog_Flag);
    if(IS_Dog(handle, Friction1_Dog)) Dog_ClearStatus(DownDog_Flag, Friction1Dog_Flag);
    if(IS_Dog(handle, Friction2_Dog)) Dog_ClearStatus(DownDog_Flag, Friction2Dog_Flag);
}

void FeedDog_CallBack(WatchDogp handle) {
    if(IS_Dog(handle, Yaw_Dog))       Dog_SetStatus(DownDog_Flag, YawDog_Flag);
    if(IS_Dog(handle, Pitch_Dog))     Dog_SetStatus(DownDog_Flag, PitchDog_Flag);
    if(IS_Dog(handle, Pluck_Dog))     Dog_SetStatus(DownDog_Flag, PluckDog_Flag);
    if(IS_Dog(handle, Friction1_Dog)) Dog_SetStatus(DownDog_Flag, Friction1Dog_Flag);
    if(IS_Dog(handle, Friction2_Dog)) Dog_SetStatus(DownDog_Flag, Friction2Dog_Flag);
}

