#include "PTZ_Runtime_Task.h"
#include "FreeRTOS_Task.h"
#include "kalman.h"
#include "Chassis.h"

TaskHandle_t PTZ_Runtime_Handler;
void PTZ_Runtime_task(void *pvParameters)
{
    int16_t Send_buff[4] = { 0 };
    
    PTZAngle_Ref_t Temp;
    
    uint16_t blocked_count1 = 0, blocked_count2 = 0;
    uint8_t blocked_flag1 = 0, blocked_flag2 = 0;
    int16_t PluckSpeed1 = 0, PluckSpeed2 = 0;
    
    kalman_filter_t PTZAngleFilter_Yaw;
    kalman_filter_t PTZAngleFilter_Pitch;
    
    kalman_Init(&PTZAngleFilter_Yaw,20,200);
    kalman_Init(&PTZAngleFilter_Pitch,20,250);
    
    for(uint8_t i = 0; i < 100; i++) {
		Kalman_Filter(&PTZAngleFilter_Yaw, PTZAngle_Ref.Yaw);
        Kalman_Filter(&PTZAngleFilter_Pitch, PTZAngle_Ref.Pitch);
	}
    
    portTickType xLastWakeTime = xTaskGetTickCount();
    for ( ;; ) {
        limit(PTZAngle_Ref.Pitch, PTZ_Pitch_MAX, PTZ_Pitch_MIN);
        limit(PTZAngle_Ref.Yaw, PTZ_Yaw_MAX, PTZ_Yaw_MIN);
        Temp.Yaw = Kalman_Filter(&PTZAngleFilter_Yaw,PTZAngle_Ref.Yaw);
        Temp.Pitch = Kalman_Filter(&PTZAngleFilter_Pitch,PTZAngle_Ref.Pitch);
        																	
        if(HAL_GPIO_ReadPin(Pill_Count1_GPIO_Port, Pill_Count1_Pin) == GPIO_PIN_RESET) {
            Pill_SupplyDown++;
        }
        
        if(Robot_Status.RS_Fire) {
            if(Pluck_Select == 1 || Pluck_Select == 3) {
                PluckSpeed1 = PluckSpeedExp;
                if(blocked_flag1 == 0) {
                    if(Pluck1.Speed < 500) {
                        ++blocked_count1;
                        if(blocked_count1 == 250 ) {
                            blocked_flag1 = 1;
                            PluckSpeed1 = -PluckSpeedExp;
                        }
                    }
                }
                else {
                    PluckSpeed1 = -PluckSpeedExp;
                    --blocked_count1;
                    if(blocked_count1 == 0)blocked_flag1 = 0;
                }
            }
            else {
                PluckSpeed1 = 0;
            }

            if(Pluck_Select == 2 || Pluck_Select == 3) {
                PluckSpeed2 = PluckSpeedExp;
                if(blocked_flag2 == 0) {
                    if(Pluck2.Speed < 500) {
                        ++blocked_count2;
                        if(blocked_count2 == 250) {
                            blocked_flag2 = 1;
                            PluckSpeed2 = -PluckSpeedExp;
                        }
                    }
                }
                else {
                    PluckSpeed2 = -PluckSpeedExp;
                    --blocked_count2;
                    if(blocked_count2 == 0)blocked_flag2 = 0;
                }
            }
            else {
                PluckSpeed2 = 0;
            }
        }
        else {
            blocked_flag1 = 0;
            blocked_flag2 = 0;
            blocked_count1 = 0;
            blocked_count2 = 0;
            PluckSpeed1 = 0;
            PluckSpeed2 = 0;
        }
        
        PID_Control_Smis(GM6020_Pitch.MchanicalAngle, Temp.Pitch, &GM6020_Pitch_PID, GM6020_Pitch.Speed);
        PID_Control(GM6020_Pitch.Speed, GM6020_Pitch_PID.pid_out, &GM6020_Pitch_SPID);
        limit(GM6020_Pitch_SPID.pid_out, 29000, -29000);

        PID_Control_Smis(GM6020_Yaw.MchanicalAngle, Temp.Yaw, &GM6020_Yaw_PID, GM6020_Yaw.Speed);
        PID_Control(GM6020_Yaw.Speed, GM6020_Yaw_PID.pid_out, &GM6020_Yaw_SPID);
        limit(GM6020_Yaw_SPID.pid_out, 29000, -29000);
        
        PID_Control(Pluck1.Speed, PluckSpeed1, &Pluck1_SPID);
        limit(Pluck1_SPID.pid_out, 10000, -10000);

        PID_Control(Pluck2.Speed, PluckSpeed2, &Pluck2_SPID);
        limit(Pluck2_SPID.pid_out, 10000, -10000);
        
        refree_shooter_limit_bili();
        Send_buff[0] = GM6020_Yaw_SPID.pid_out;
        Send_buff[1] = GM6020_Pitch_SPID.pid_out;
        Send_buff[2] = Pluck1_SPID.pid_out;
        Send_buff[3] = Pluck2_SPID.pid_out;

#if TEST == 0
        MotorSend(&hcan1, 0x1ff, Send_buff);
#else
        UNUSED(Send_buff);
#endif
        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}
