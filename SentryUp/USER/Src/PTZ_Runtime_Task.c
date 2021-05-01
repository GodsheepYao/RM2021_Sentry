#include "PTZ_Runtime_Task.h"
#include "FreeRTOS_Task.h"
#include "kalman.h"
#include "Chassis.h"

/*PID进程*/
TaskHandle_t PTZ_Runtime_Handler;
void PTZ_Runtime_task(void *pvParameters)
{
    int16_t Send_buff[4] = { 0 };
    
    PTZAngle_Ref_t Temp;
    
    //云台输入卡尔曼滤波
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
        
        PID_Control_Smis(GM6020_Pitch.MchanicalAngle, Temp.Pitch, &GM6020_Pitch_PID, GM6020_Pitch.Speed);
        PID_Control(GM6020_Pitch.Speed, GM6020_Pitch_PID.pid_out, &GM6020_Pitch_SPID);
        limit(GM6020_Pitch_SPID.pid_out, 29000, -29000);

        PID_Control_Smis(GM6020_Yaw.MchanicalAngle, Temp.Yaw, &GM6020_Yaw_PID, GM6020_Yaw.Speed);
        PID_Control(GM6020_Yaw.Speed, GM6020_Yaw_PID.pid_out, &GM6020_Yaw_SPID);
        limit(GM6020_Yaw_SPID.pid_out, 29000, -29000);

        Send_buff[0] = GM6020_Yaw_SPID.pid_out;
        Send_buff[1] = GM6020_Pitch_SPID.pid_out;

#if TEST == 0
        MotorSend(&hcan1, 0x1ff, Send_buff);
#else
        UNUSED(Send_buff);
#endif
        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}
