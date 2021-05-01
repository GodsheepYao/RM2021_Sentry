#include "Remote_Task.h"
#include "PC_Task.h"
uint8_t usart1_dma_buff[30] = {0};

SemaphoreHandle_t Remote_Semaphore;
TaskHandle_t Remote_task_Handler;
Remote_Rx_PACK_t Remote_Rx_PACK;

void Remote_task(void *pvParameters) {
    BaseType_t err = pdFALSE;
    for( ;; ) {
        err = xSemaphoreTake(Remote_Semaphore, portMAX_DELAY);
        if (err == pdTRUE) {
            Remote_Rx(usart1_dma_buff);
        }
        else if (err == pdFALSE){
            vTaskDelay(10);
        }
    }
}

void RemoteControlProcess(Remote *rc){
    static uint8_t Up_count = 0;
    static uint8_t Down_count = 0;
    
    if(rc->ch1 > 1600 && Robot_Status.RS_Downctl)
    {
        ++Up_count;
        if(Up_count > 70)
        {
            Up_count = 0;
            Robot_Status.RS_Downctl = STATUS_TURN_OFF;
            //TODO 切换灯变换
        }
    }
    else
        Up_count = 0;
    
    if(rc->ch1 < 400 && !Robot_Status.RS_Downctl)
    {
        ++Down_count;
        if(Down_count > 70)
        {
            Down_count = 0;
            Robot_Status.RS_Downctl = STATUS_TURN_ON;
            //TODO 切换灯变换
        }
    }
    else
        Down_count = 0;
    
    if(Robot_Status.RS_Downctl && !Robot_Status.RS_Auto) {
        PTZAngle_Ref.Pitch += (float)(rc->ch3 - REMOTE_CONTROLLER_STICK_OFFSET) * Sensitivity_Pitch;
        limit(PTZAngle_Ref.Pitch, PTZ_Pitch_MAX, PTZ_Pitch_MIN);
        int32_t t = (int32_t) (PTZAngle_Ref.Yaw + (rc->ch2 - REMOTE_CONTROLLER_STICK_OFFSET) * Sensitivity_Yaw) % 8192;
        PTZAngle_Ref.Yaw = t < 0 ? t + 8192 : t;
        
        switch(rc->s1){
            case 1:
                Robot_Status.RS_Fire = STATUS_TURN_ON;
                break;
            case 3:
                Robot_Status.RS_Loaded = STATUS_TURN_ON;
                Robot_Status.RS_Fire = STATUS_TURN_OFF;
                break;
            case 2:
                Robot_Status.RS_Loaded = STATUS_TURN_OFF;
                Robot_Status.RS_Fire = STATUS_TURN_OFF;
                break;
    }
    }
    
    if(rc->s2 != 3) Robot_Status.RS_Auto = 0;   

}

void MouseKeyControlProcess(Mouse *mouse, Key_t key, Key_t Lastkey)
{
    Robot_Status.RS_Auto = 1;
}

void CAN_Remote_Rx(uint8_t* Data) {
    memcpy(&Remote_Rx_PACK, Data, 8);
    if(Robot_Status.RS_Downctl && !Robot_Status.RS_Auto) {
        PTZAngle_Ref.Pitch += (float)(Remote_Rx_PACK.ch3 - REMOTE_CONTROLLER_STICK_OFFSET) * Sensitivity_Pitch;
        limit(PTZAngle_Ref.Pitch, PTZ_Pitch_MAX, PTZ_Pitch_MIN);
        int32_t t = (int32_t) (PTZAngle_Ref.Yaw + (Remote_Rx_PACK.ch2 - REMOTE_CONTROLLER_STICK_OFFSET) * Sensitivity_Yaw) % 8192;
        PTZAngle_Ref.Yaw = t < 0 ? t + 8192 : t;
        switch(Remote_Rx_PACK.s1){
            case 1:
                Robot_Status.RS_Fire = STATUS_TURN_ON;
                break;
            case 3:
                Robot_Status.RS_Loaded = STATUS_TURN_ON;
                Robot_Status.RS_Fire = STATUS_TURN_OFF;
                break;
            case 2:
                Robot_Status.RS_Loaded = STATUS_TURN_OFF;
                Robot_Status.RS_Fire = STATUS_TURN_OFF;
                break;
        }
    }
}
