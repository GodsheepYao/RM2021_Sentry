#include "PTZ_Init_Task.h"

TaskHandle_t PTZ_Init_Handler;
void PTZ_Init_task(void *pvParameters) {
	
	while (GM6020_Pitch.MchanicalAngle == 0 || GM6020_Yaw.MchanicalAngle == 0)
        vTaskDelay(10);
	
	int16_t Send_buff[4] = { 0 };
	
	int16_t PTZ_Pitch_angle = GM6020_Pitch.MchanicalAngle;
	if(PTZ_Pitch_angle > PTZ_Pitch_median + 4095) PTZ_Pitch_angle -= 8191;
	float PTZ_Angle_Yaw = QuickCentering(GM6020_Yaw.MchanicalAngle, PTZ_Yaw_median);
	
	Ramp_Typedef PTZ_Init = {.RampTime = 70000};
	
	float PitchRampInit = PTZ_Pitch_median - PTZ_Pitch_angle;
	float YawRampInit = PTZ_Yaw_median - PTZ_Angle_Yaw;	
	
	for( ;; ) {
		PID_Control_Smis(PTZ_Pitch_angle, 
						PTZ_Pitch_median - (PitchRampInit * (1.0f - Slope(&PTZ_Init))), 
						&GM6020_Pitch_PID, GM6020_Pitch.Speed);
		PID_Control_Smis(QuickCentering(GM6020_Yaw.MchanicalAngle, PTZ_Yaw_median), 
						PTZ_Yaw_median - (YawRampInit * (1.0f - Slope(&PTZ_Init))), 
						&GM6020_Yaw_PID, GM6020_Yaw.Speed);
	}
}
