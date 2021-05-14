#include "PC_Task.h"
#include "FreeRTOS_Task.h"
#include "Referee_offcial.h"

uint8_t test[15] = "helloworld\r\n";
TaskHandle_t PC_task_Handler;
Robot_Info_t robot_info = { 0 };
void PC_task(void *pvParameters){
//    Robot_Info_t robot_info = { 0 };
    UpBoard_Data_t UpBoard_Data;
    float last_Encoder_Locat = 18.84f * (float)(Encoder_Rand * 1600.0f + TIM1->CNT) / 1600.0f;

    portTickType xLastWakeTime = xTaskGetTickCount();
    for( ;; ) {
        Encoder_Locat = 18.84f * (float)(Encoder_Rand * 1600.0f + TIM1->CNT) / 1600.0f;
        Encoder_Speed = Encoder_Locat - last_Encoder_Locat;
        
        UpBoard_Data.Status = Robot_Status;
        
        if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
            robot_info.Yaw_MchanicalAngle    = -GM6020_Yaw.MchanicalAngle + PTZ_Yaw_median;         
            robot_info.Yaw_AngularVelocity   = GM6020_Yaw.Speed;                     	
            robot_info.Pitch_MchanicalAngle  = GM6020_Pitch.MchanicalAngle - PTZ_Pitch_median;     	
            robot_info.Pitch_AngularVelocity = GM6020_Pitch.Speed;                   	
            robot_info.Location              = Encoder_Locat * 10.0f;                       
            robot_info.Speed                 = Encoder_Speed * 100.0f;										   
            robot_info.Status = UpBoard_Data.Status;
            
//            portENTER_CRITICAL();    
            VCOMM_Transmit(0x01, 0x01, (uint8_t*)&robot_info, sizeof(robot_info));
//            VCOMM_Transmit(0x01, 0x01, (uint8_t*)&test, sizeof(test));
//            portEXIT_CRITICAL();
        }
        
        unpack_refree_system_data(&Referee_Queue, 0.01);
        UpBoard_Data.Radiofreq_Limit = (int16_t)(shooter_bili1 * 100);
        CAN_Send_StdDataFrame(&hcan2, 0x100, (uint8_t*)&UpBoard_Data);
        last_Encoder_Locat = Encoder_Locat;
        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}

void VCOMM_CallBack(uint8_t fun_code, uint16_t id, uint8_t *data, uint8_t len) {
    if (Robot_Status.RS_Auto && Robot_Status.RS_Ready) {
        if (fun_code == 0x01 && id == 0x01 && len == sizeof(Control_Info_t)) {
            Control_Info_t control_info = *(Control_Info_t*) data;
            
            PTZAngle_Ref.Pitch = (control_info.Pitch_Angle + PTZ_Pitch_median) % 8192;
            limit(PTZAngle_Ref.Pitch, PTZ_Pitch_MAX, PTZ_Pitch_MIN);           
            PTZAngle_Ref.Yaw = (8191 - control_info.Yaw_Angle + PTZ_Yaw_median) % 8192;
            limit(PTZAngle_Ref.Yaw, PTZ_Yaw_MAX, PTZ_Yaw_MIN);           
        }
    }
    
    if(fun_code == 0x01 && id == 0x02) {
        uint8_t Enemy_Camp;
        if(ext_game_robot_state.robot_id == 7) Enemy_Camp = 'b';
        else Enemy_Camp = 'r';
        VCOMM_Transmit(0x01, 0x02, &Enemy_Camp, sizeof(Enemy_Camp));
    }
}


