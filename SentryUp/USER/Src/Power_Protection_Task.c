#include "Power_Protection_Task.h"
#include "PTZ_Init_Task.h"
#include "PTZ_Runtime_Task.h"
#include "FreeRTOS_Task.h"
#include "Chassis_Init_Task.h"
#include "Remote_Task.h"
#include "Chassis_Fire_Task.h"

TaskHandle_t Power_Protection_Handler;
WatchDog_Flag_t UpDog_Flag;

void Power_Protection_task(void *pvParameters) {
    portTickType xLastWakeTime = xTaskGetTickCount();
    for( ;; ) {
        WatchDog_Polling();
        if(!Robot_Status.RS_Dead && (Robot_Status.RS_Kill || UpDog_Flag == 0)) {
            Robot_Status.RS_Dead = 1;
            vTaskDelete(PTZ_Runtime_Handler);
            vTaskDelete(Chassis_Fire_Handler);
        } else if (Robot_Status.RS_Dead && (!Robot_Status.RS_Kill && (UpDog_Flag == ALL_Flag))) { 
            Robot_Status.RS_Dead = 0;
            PTZAngle_Ref.Pitch = PTZ_Pitch_median;
            PTZAngle_Ref.Yaw = PTZ_Yaw_median;
            xTaskCreate((TaskFunction_t)PTZ_Init_task,
                (const char *)"PTZ_Init_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)1,
                (TaskHandle_t *)&PTZ_Init_Handler);
            xTaskCreate((TaskFunction_t)Chassis_Fire_task,
                (const char *)"Chassis_Fire_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)2,
                (TaskHandle_t *)&Chassis_Fire_Handler);
        }
        vTaskDelayUntil(&xLastWakeTime, 20);
    }
}

void WatchDog_CallBack(WatchDogp handle) {
   
    if(IS_Dog(handle, Chassis_Dog))   Dog_ClearStatus(UpDog_Flag,  ChassisDog_Flag);
    if(IS_Dog(handle, Yaw_Dog))       Dog_ClearStatus(UpDog_Flag, YawDog_Flag);
    if(IS_Dog(handle, Pitch_Dog))     Dog_ClearStatus(UpDog_Flag, PitchDog_Flag);
    if(IS_Dog(handle, Pluck1_Dog))    Dog_ClearStatus(UpDog_Flag, Pluck1Dog_Flag);
    if(IS_Dog(handle, Pluck2_Dog))    Dog_ClearStatus(UpDog_Flag, Pluck2Dog_Flag);
    if(IS_Dog(handle, Friction1_Dog)) Dog_ClearStatus(UpDog_Flag, Friction1Dog_Flag);
    if(IS_Dog(handle, Friction2_Dog)) Dog_ClearStatus(UpDog_Flag, Friction2Dog_Flag);
    if(IS_Dog(handle, Remote_Dog))    {
        RemoteClear();
        RemoteControlProcess(&(RC_CtrlData.rc));
    }
}

void FeedDog_CallBack(WatchDogp handle) {
    if(IS_Dog(handle, Chassis_Dog))   Dog_SetStatus(UpDog_Flag,  ChassisDog_Flag);
    if(IS_Dog(handle, Yaw_Dog))       Dog_SetStatus(UpDog_Flag, YawDog_Flag);
    if(IS_Dog(handle, Pitch_Dog))     Dog_SetStatus(UpDog_Flag, PitchDog_Flag);
    if(IS_Dog(handle, Pluck1_Dog))    Dog_SetStatus(UpDog_Flag, Pluck1Dog_Flag);
    if(IS_Dog(handle, Pluck2_Dog))    Dog_SetStatus(UpDog_Flag, Pluck2Dog_Flag);
    if(IS_Dog(handle, Friction1_Dog)) Dog_SetStatus(UpDog_Flag, Friction1Dog_Flag);
    if(IS_Dog(handle, Friction2_Dog)) Dog_SetStatus(UpDog_Flag, Friction2Dog_Flag);
}
