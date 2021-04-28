#include "PTZ_Init_Task.h"
#include "FreeRTOS_Task.h"
//P1970 2513 1042
//Y3476 4792 2100 
/*云台归中进程*/
TaskHandle_t PTZ_Init_Handler;
void PTZ_Init_task(void *pvParameters) {
    while(GM6020_Pitch.MchanicalAngle == 0 || GM6020_Yaw.MchanicalAngle == 0)
        vTaskDelay(10);//等待设备就位

    int16_t Send_buff[4] = {0, 0, 0, 0};

    Ramp_Typedef PTZ_Init = {.RampTime = 70000};

    float PitchRampInit = PTZ_Pitch_median - GM6020_Pitch.MchanicalAngle;
    float YawRampInit = PTZ_Yaw_median - GM6020_Yaw.MchanicalAngle;

    portTickType xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        PID_Control_Smis(GM6020_Pitch.MchanicalAngle,
                                PTZ_Pitch_median - (PitchRampInit * (1.0f - Slope(&PTZ_Init))),
                                &GM6020_Pitch_PID, GM6020_Pitch.Speed);

        PID_Control(GM6020_Pitch.Speed, GM6020_Pitch_PID.pid_out, &GM6020_Pitch_SPID);
        limit(GM6020_Pitch_SPID.pid_out, 29000, -29000);

        PID_Control_Smis(GM6020_Yaw.MchanicalAngle,
                                PTZ_Yaw_median - (YawRampInit * (1.0f - Slope(&PTZ_Init))),
                                &GM6020_Yaw_PID, GM6020_Yaw.Speed);

        PID_Control(GM6020_Yaw.Speed, GM6020_Yaw_PID.pid_out, &GM6020_Yaw_SPID);
        limit(GM6020_Yaw_SPID.pid_out, 29000, -29000);

        Send_buff[0] = GM6020_Yaw_SPID.pid_out;
        Send_buff[1] = GM6020_Pitch_SPID.pid_out;

#if TEST == 0
        MotorSend(&hcan1, 0x1ff, Send_buff);
#else
        UNUSED(Send_buff);
#endif
//        if (Slope(&PTZ_Init) == 1.0f && Robot_Status.RS_Ready == STATUS_TURN_ON)
//        {
//			vTaskDelete(NULL);
//        }
        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}
