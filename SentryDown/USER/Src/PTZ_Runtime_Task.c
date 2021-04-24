#include "PTZ_Runtime_Task.h"
#include "FreeRTOS_Task.h"
#include "kalman.h"
#include "Chassis.h"

TaskHandle_t PTZ_Runtime_Handler;
void PTZ_Runtime_task(void *pvParameters) {
    /* 电机数据发送缓冲区 */
    int16_t Send_buff[4] = { 0, 0, 0, 0 };
    
    /* 云台滤波后期望临时变量 */
    PTZAngle_Ref_t Temp;
    
    /* vTaskDelayUntil延时初始化 */
    portTickType xLastWakeTime = xTaskGetTickCount();
    
    //云台输入卡尔曼滤波
    kalman_filter_t PTZAngleFilter_Yaw;
    kalman_filter_t PTZAngleFilter_Pitch;
    
    kalman_Init(&PTZAngleFilter_Yaw, 20, 200);
    kalman_Init(&PTZAngleFilter_Pitch, 20, 250);
    
    for(uint8_t i = 0; i < 100; i++) {
		Kalman_Filter(&PTZAngleFilter_Yaw, PTZAngle_Ref.Yaw);
        Kalman_Filter(&PTZAngleFilter_Pitch, PTZAngle_Ref.Pitch);
	}
    
    int32_t Yaw_offset = GM6020_Yaw.Angle;
    
    uint16_t blocked_count1 = 0;
    uint8_t blocked_flag1 = 0;
    int16_t PluckSpeed1 = 0;
    
    for (;;) {
        /* 云台输入滤波平滑化 */
        
        /* P轴限幅 */
        limit(PTZAngle_Ref.Pitch, PTZ_Pitch_MAX, PTZ_Pitch_MIN);
        
        Temp.Yaw = Kalman_Filter(&PTZAngleFilter_Yaw, PTZAngle_Ref.Yaw);
        Temp.Pitch = Kalman_Filter(&PTZAngleFilter_Pitch, PTZAngle_Ref.Pitch);
        
        if(Robot_Status.RS_Fire){
            PluckSpeed1 = PluckSpeedExp;
            if(blocked_flag1 == 0){
                if(Pluck1.Speed > -500){
                    blocked_count1++;
                    if(blocked_count1 == 250){
                        blocked_flag1 = 1;
                        PluckSpeed1 = -PluckSpeedExp;
                    }
                }
            }
            else{
                PluckSpeed1 = -PluckSpeedExp;
                blocked_count1--;
                if(blocked_count1 == 0) blocked_flag1 = 0;
            }
        }
        else{
            blocked_count1 = 0;
            blocked_flag1 = 0;
            PluckSpeed1 = 0;
        }
        
        PID_Control(Pluck1.Speed, PluckSpeed1, &Pluck1_SPID);
        limit(Pluck1_SPID.pid_out, 10000, -10000);
        
        PID_Control_Smis(GM6020_Pitch.MchanicalAngle, Temp.Pitch, &GM6020_Pitch_PID, GM6020_Pitch.Speed);
        PID_Control(GM6020_Pitch.Speed, GM6020_Pitch_PID.pid_out, &GM6020_Pitch_SPID);
        limit(GM6020_Pitch_SPID.pid_out, 29000, -29000);

        PID_Control_Smis(GM6020_Yaw.Angle - Yaw_offset, Temp.Yaw, &GM6020_Yaw_PID, GM6020_Yaw.Speed);
        PID_Control(GM6020_Yaw.Speed, GM6020_Yaw_PID.pid_out, &GM6020_Yaw_SPID);
        limit(GM6020_Yaw_SPID.pid_out, 29000, -29000);
        
        Send_buff[0] = GM6020_Pitch_SPID.pid_out;
        Send_buff[1] = GM6020_Yaw_SPID.pid_out;
        Send_buff[2] = Pluck1_SPID.pid_out;
        
#if TEST == 0
        MotorSend(&hcan1, 0X1ff, Send_buff);
#else
        UNUSED(Send_buff);
#endif
        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}
