#include "PTZ_Init_Task.h"
#include "Remote_Task.h"
#include "PTZ_Runtime_Task.h"
#include "Fire_Mechanism_Task.h"

TaskHandle_t PTZ_Init_Handler;
void PTZ_Init_task(void *pvParameters) {
    /* vTaskDelayUntil延时初始化 */
    portTickType xLastWakeTime = xTaskGetTickCount();
    
    Ramp_Typedef PTZ_Init = {.RampTime = 70000};

	while (GM6020_Pitch.MchanicalAngle == 0 || GM6020_Yaw.MchanicalAngle == 0)
        vTaskDelay(10);
	
	int16_t Send_buff[4] = { 0 };
	
	int16_t PTZ_Pitch_angle = QuickCentering(GM6020_Pitch.MchanicalAngle, PTZ_Pitch_median);
	int16_t PTZ_Angle_Yaw = QuickCentering(GM6020_Yaw.MchanicalAngle, PTZ_Yaw_median);
	
	int16_t PitchRampInit = PTZ_Pitch_median - PTZ_Pitch_angle;
	int16_t YawRampInit = PTZ_Yaw_median - PTZ_Angle_Yaw;	
	
	for( ;; ) {
		PID_Control_Smis(QuickCentering(GM6020_Pitch.MchanicalAngle, PTZ_Pitch_median), 
						PTZ_Pitch_median - (PitchRampInit * (1.0f - Slope(&PTZ_Init))), 
						&GM6020_Pitch_PID, GM6020_Pitch.Speed);
        
		PID_Control_Smis(QuickCentering(GM6020_Yaw.MchanicalAngle, PTZ_Yaw_median), 
						PTZ_Yaw_median - (YawRampInit * (1.0f - Slope(&PTZ_Init))), 
						&GM6020_Yaw_PID, GM6020_Yaw.Speed);
        
        PID_Control(GM6020_Pitch.Speed, GM6020_Pitch_PID.pid_out, &GM6020_Pitch_SPID);
        limit(GM6020_Pitch_SPID.pid_out, 29000, -29000);
        
        PID_Control(GM6020_Yaw.Speed, GM6020_Yaw_PID.pid_out, &GM6020_Yaw_SPID);
        limit(GM6020_Yaw_SPID.pid_out, 29000, -29000);
        
        Send_buff[0] = GM6020_Pitch_SPID.pid_out;
        Send_buff[1] = GM6020_Yaw_SPID.pid_out;
#if TEST == 0
        MotorSend(&hcan1, 0x1ff, Send_buff);
#else 
        UNUSED(Send_buff);
#endif

        if(Slope(&PTZ_Init) == 1.0f) {
            xTaskCreate((TaskFunction_t)PTZ_Runtime_task,
                (const char *)"PTZ_Runtime_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)2,
                (TaskHandle_t *)&PTZ_Runtime_Handler);
                
            xTaskCreate((TaskFunction_t)Remote_task,
                (const char *)"Remote_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)2,
                (TaskHandle_t *)&Remote_task_Handler);
            xTaskCreate((TaskFunction_t)Fire_Mechanism_Task,
                (const char *)"Fire_Mechanism_Task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)2,
                (TaskHandle_t *)&Fire_Mechanism_task_Handler);
            Robot_Status.RS_Ready = STATUS_TURN_ON;
            vTaskDelete(NULL);
        }
        vTaskDelayUntil(&xLastWakeTime, 2);
	}
}
