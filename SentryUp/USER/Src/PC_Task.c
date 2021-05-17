#include "PC_Task.h"
#include "FreeRTOS_Task.h"
#include "Referee_offcial.h"

UpBoard_Data_t UpBoard_Data;
uint8_t test[15] = "helloworld\r\n";
TaskHandle_t PC_task_Handler;

void PC_task(void *pvParameters){
    Robot_Info_t robot_info = { 0 };
    float last_Encoder_Locat = 18.84f * (float)(Encoder_Rand * 1600.0f + TIM1->CNT) / 1600.0f;

    portTickType xLastWakeTime = xTaskGetTickCount();
    for( ;; ) {
        Encoder_Locat = 18.84f * (float)(Encoder_Rand * 1600.0f + TIM1->CNT) / 1600.0f;
        Encoder_Speed = Encoder_Locat - last_Encoder_Locat;
        
        UpBoard_Data.Status = Robot_Status;
        if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
            int16_t temp_yaw = -GM6020_Yaw.MchanicalAngle + PTZ_Yaw_median ;
            robot_info.Yaw_MchanicalAngle    = temp_yaw < 0 ? 8191 + temp_yaw : temp_yaw;
            robot_info.Yaw_AngularVelocity   = GM6020_Yaw.Speed;                     	
            robot_info.Pitch_MchanicalAngle  = GM6020_Pitch.MchanicalAngle - PTZ_Pitch_median;     	
            robot_info.Pitch_AngularVelocity = GM6020_Pitch.Speed;                   	
            robot_info.Location              = (Encoder_Locat - Encoder_offsef) * 5000.0f / Encoder_Max;                       
            robot_info.Speed                 = Encoder_Speed * 100.0f;										   
            robot_info.Status                = UpBoard_Data.Status;
            robot_info.HP                    = ext_game_robot_state.remain_HP;
            
//            portENTER_CRITICAL();    
            VCOMM_Transmit(0x01, 0x01, (uint8_t*)&robot_info, sizeof(robot_info));
            if(blocked_flag1 == 1) VCOMM_Transmit(0x01, 0x80, NULL, 0);            
            if(blocked_flag2 == 1) VCOMM_Transmit(0x02, 0x80, NULL, 0);
//            VCOMM_Transmit(0x01, 0x01, (uint8_t*)&test, sizeof(test));
//            portEXIT_CRITICAL();
        }
        
        while(unpack_refree_system_data(&Referee_Queue));
        UpBoard_Data.Radiofreq_Limit = (int16_t)(shooter_bili1 * 100);
        CAN_Send_StdDataFrame(&hcan2, 0x100, (uint8_t*)&UpBoard_Data);
        last_Encoder_Locat = Encoder_Locat;
        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}

void VCOMM_CallBack(uint8_t fun_code, uint16_t id, uint8_t *data, uint8_t len) {
    if (Robot_Status.RS_Auto && Robot_Status.RS_Ready) {
        if (fun_code == 0x01 && id == 0x01 && len == sizeof(Control_Info_t)) {
            Control_Info_t control_info;
            memcpy(&control_info, data, sizeof(Control_Info_t));  
    
            if(control_info.Yaw_Angle > 4095) control_info.Yaw_Angle -= 8191;
            
            PTZAngle_Ref.Pitch = (control_info.Pitch_Angle + PTZ_Pitch_median) % 8192;
            limit(PTZAngle_Ref.Pitch, PTZ_Pitch_MAX, PTZ_Pitch_MIN);       

            PTZAngle_Ref.Yaw = (-control_info.Yaw_Angle + PTZ_Yaw_median) % 8192;
            limit(PTZAngle_Ref.Yaw, PTZ_Yaw_MAX, PTZ_Yaw_MIN); 

            Chasssis_Auto_Speed = control_info.Speed;

            Robot_Status.RS_Loaded = control_info.Status.RS_Loaded;
            Robot_Status.RS_Fire = control_info.Status.RS_Fire;            
        }
    }
    
    if(fun_code == 0x01 && id == 0x02) {
        uint8_t Enemy_Camp;
        if(ext_game_robot_state.robot_id == 7) Enemy_Camp = 'b';
        else Enemy_Camp = 'r';
        VCOMM_Transmit(0x01, 0x02, &Enemy_Camp, sizeof(Enemy_Camp));
    }
}


