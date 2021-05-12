#include "PTZ_Runtime_Task.h"
#include "FreeRTOS_Task.h"
#include "kalman.h"
#include "Chassis.h"

float pitch_exp = 0;

TaskHandle_t PTZ_Runtime_Handler;
void PTZ_Runtime_task(void *pvParameters) {
    /* 电机数据发送缓冲区 */
    int16_t Send_buff[4] = { 0, 0, 0, 0 };
    
    /* vTaskDelayUntil延时初始化 */
    portTickType xLastWakeTime = xTaskGetTickCount();
    
    uint16_t blocked_count1 = 0;
    uint8_t blocked_flag1 = 0;
    int16_t PluckSpeed1 = 0;
    uint16_t pill_count_time = 0;
    
    for (;;) {    
        /* P轴限幅 */
        limit(PTZAngle_Ref.Pitch, PTZ_Pitch_MAX, PTZ_Pitch_MIN);
        
        if(HAL_GPIO_ReadPin(Pill_Count2_GPIO_Port, Pill_Count2_Pin) == GPIO_PIN_RESET && pill_count_time < 5) {
            Pill_Out++;
            pill_count_time++;
        }
        else 
            pill_count_time = 0;
        
        if(Robot_Status.RS_Fire && UpBoard_Data.Radiofreq_Limit != 0){
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
        else {
            blocked_count1 = 0;
            blocked_flag1 = 0;
            PluckSpeed1 = 0;
        }
        
        PluckSpeed1 *= UpBoard_Data.Radiofreq_Limit / 100;
        PID_Control(Pluck1.Speed, PluckSpeed1, &Pluck1_SPID);
        limit(Pluck1_SPID.pid_out, 10000, -10000);
        
        pitch_exp = QuickCentering(GM6020_Pitch.MchanicalAngle, PTZAngle_Ref.Pitch);
        PID_Control_Smis(GM6020_Pitch.MchanicalAngle, pitch_exp, &GM6020_Pitch_PID, GM6020_Pitch.Speed);
        PID_Control(GM6020_Pitch.Speed, GM6020_Pitch_PID.pid_out, &GM6020_Pitch_SPID);
        limit(GM6020_Pitch_SPID.pid_out, 29000, -29000);
        
        float yaw_exp = QuickCentering(GM6020_Yaw.MchanicalAngle, PTZAngle_Ref.Yaw);
        PID_Control_Smis(GM6020_Yaw.MchanicalAngle, yaw_exp, &GM6020_Yaw_PID, GM6020_Yaw.Speed);
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
