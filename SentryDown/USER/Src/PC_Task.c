#include "PC_Task.h"

uint8_t test[15] = "helloworld\r\n";

/*PC通信任务*/
TaskHandle_t PC_task_Handler;
void PC_task(void *pvParameters){
    Robot_Info_t robot_info = { 0 };
    
    portTickType xLastWakeTime = xTaskGetTickCount();
    for ( ;; ) {
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            robot_info.Yaw_MchanicalAngle    = GM6020_Yaw.MchanicalAngle - PTZ_Yaw_median;           
            robot_info.Yaw_AngularVelocity   = GM6020_Yaw.Speed;
            robot_info.Pitch_MchanicalAngle  = GM6020_Pitch.MchanicalAngle - PTZ_Pitch_median;
            robot_info.Pitch_AngularVelocity = GM6020_Pitch.Speed;
            robot_info.Status = Robot_Status;
        
            portENTER_CRITICAL();
            VCOMM_Transmit(0x01, 0x01, (uint8_t*)&robot_info, sizeof(robot_info));
            if(blocked_flag1 == 1) VCOMM_Transmit(0x03, 0x80, NULL, 0);
//            VCOMM_Transmit(0x01, 0x01, (uint8_t*)&test, sizeof(test));
            portEXIT_CRITICAL();
        }
        
        CAN_Send_StdDataFrame(&hcan2, 0x101, (uint8_t*)&Robot_Status);
        
        if (Robot_Status.RS_Downctl)
            HAL_GPIO_WritePin(DownMode_LED_GPIO_Port, DownMode_LED_Pin, GPIO_PIN_RESET);   
        else 
            HAL_GPIO_WritePin(DownMode_LED_GPIO_Port, DownMode_LED_Pin, GPIO_PIN_SET);
        
        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}

void VCOMM_CallBack(uint8_t fun_code, uint16_t id, uint8_t *data, uint8_t len) {
    if (Robot_Status.RS_Auto && Robot_Status.RS_Ready) {
        if (fun_code == 0x01 && id == 0x01 && len == sizeof(Control_Info_t)) {
            Control_Info_t control_info;
            memcpy(&control_info, data, sizeof(Control_Info_t));
            
            PTZAngle_Ref.Pitch = (control_info.Pitch_Angle + PTZ_Pitch_median) % 8192;
            limit(PTZAngle_Ref.Pitch, PTZ_Pitch_MAX, PTZ_Pitch_MIN);           
            PTZAngle_Ref.Yaw = (control_info.Yaw_Angle + PTZ_Yaw_median) % 8192;
            
            Robot_Status.RS_Loaded = control_info.Status.RS_Loaded;
            Robot_Status.RS_Fire = control_info.Status.RS_Fire;
        }
    }
}


