#include "Remote_Task.h"
#include "FreeRTOS_Task.h"

uint8_t usart1_dma_buff[30] = {0};

SemaphoreHandle_t Remote_Semaphore;
TaskHandle_t Remote_task_Handler;
uint8_t remote_flag1 = 0;

void Remote_task(void *pvParameters) {
    BaseType_t err = pdFALSE;
    for( ;; ) {
        err = xSemaphoreTake(Remote_Semaphore, portMAX_DELAY);
        if (err == pdTRUE) {
            Remote_Rx(usart1_dma_buff);
            if(Robot_Status.RS_Downctl && !Robot_Status.RS_Auto) { //处于下板操控模式，发送遥控器数据
            
                Remote_Rx_PACK_t Remote_Rx_PACK;

                Remote_Rx_PACK.ch2 = RC_CtrlData.rc.ch2;
                Remote_Rx_PACK.ch3 = RC_CtrlData.rc.ch3;
                Remote_Rx_PACK.s1 = RC_CtrlData.rc.s1;
                Remote_Rx_PACK.s2 = RC_CtrlData.rc.s2;
                Remote_Rx_PACK.Null = 0;

                CAN_Send_StdDataFrame(&hcan2, 0x0ff, (uint8_t*)&Remote_Rx_PACK);
            }
        }
        else if (err == pdFALSE){
            vTaskDelay(10);
        }
    }
}

void RemoteControlProcess(Remote *rc) {
    static uint8_t Up_count = 0;
    static uint8_t Down_count = 0;
//    static uint8_t pluck_flag = 0;
    
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
    
    if(!Robot_Status.RS_Auto && !Robot_Status.RS_Downctl){
        PTZAngle_Ref.Pitch += (float)(rc->ch3 - REMOTE_CONTROLLER_STICK_OFFSET) * Sensitivity_Pitch;
        limit(PTZAngle_Ref.Pitch, PTZ_Pitch_MAX, PTZ_Pitch_MIN);
        PTZAngle_Ref.Yaw += (float)(rc->ch2 - REMOTE_CONTROLLER_STICK_OFFSET) * Sensitivity_Yaw;
        limit(PTZAngle_Ref.Yaw, PTZ_Yaw_MAX, PTZ_Yaw_MIN);
        
        switch(rc->s1){
            case 1:
//                if(remote_flag1 == 2) 
                    Robot_Status.RS_Fire = STATUS_TURN_ON;
                break;
            case 3:
                Robot_Status.RS_Loaded = STATUS_TURN_ON;
                Robot_Status.RS_Fire = STATUS_TURN_OFF;
                remote_flag1++;
                break;
            case 2:
                remote_flag1 = 0;
                Robot_Status.RS_Loaded = STATUS_TURN_OFF;
                Robot_Status.RS_Fire = STATUS_TURN_OFF;
                remote_flag1++;
                break;
        }
       
    }
    ChassisSpeedExp = (rc->ch0 - REMOTE_CONTROLLER_STICK_OFFSET) * Sensitivity_Chassis;

    
    if(rc->s2 == 3) Robot_Status.RS_Auto = STATUS_TURN_ON;
    else Robot_Status.RS_Auto = STATUS_TURN_OFF;

}

void MouseKeyControlProcess(Mouse *mouse, Key_t key, Key_t Lastkey)
{
    Robot_Status.RS_Auto = STATUS_TURN_ON;
}
