#include "Chassis_Init_Task.h"
#include "FreeRTOS_Task.h"
#include "Remote_Task.h"
#include "Chassis_Fire_Task.h"
#include "PTZ_Runtime_Task.h"
#include "Power_Protection_Task.h"

TaskHandle_t Chassis_Init_Handler;
int16_t Encoder_post = 13;
void Chassis_Init_task(void *pvParameters) {
    int16_t Send_buff[4] = {0, 0, 0, 0};
    float SpeedExp = 2500;
    uint8_t Direct = 0;
    uint16_t time_count = 0;
    
    PID_Smis Chassis_PID = {.Kp = 12, .Ki = 0, .Kd = 0, .limit = 1000};
    
    portTickType xLastWakeTime = xTaskGetTickCount();
    for( ;; ) {
        if(Direct == 0) {
            if(HAL_GPIO_ReadPin(Optical_Fiber2_GPIO_Port, Optical_Fiber2_Pin)) 
                SpeedExp = 2500;
            else {
                SpeedExp = -100;
                Encoder_offsef = Encoder_Locat + Encoder_post;
                time_count++;
                if(time_count == 50) {
                    Direct = 1;
                    time_count = 0;
                }
            }
        }
        
        if(Direct == 1) {
            if(HAL_GPIO_ReadPin(Optical_Fiber1_GPIO_Port, Optical_Fiber1_Pin)) 
                SpeedExp = -2500;
            else {
                SpeedExp = 100;
                Encoder_Max = Encoder_Locat - Encoder_offsef - Encoder_post;
                time_count++;
                if(time_count == 50) {
                    Direct = 2;
                    time_count = 0;
                }
            }
        }
        
        if(Direct == 2) {
            Chassis_PID.Kp = 100;
            Chassis_PID.Ki = 1;
            Chassis_PID.Kd = 10;
            PID_Control_Smis(Encoder_Locat - Encoder_offsef, Encoder_Max / 2.0f , &Chassis_PID, Encoder_Speed);
            limit(Chassis_PID.pid_out, 3000, -3000);
            SpeedExp = -Chassis_PID.pid_out;
            if(__fabs((Encoder_Locat - Encoder_offsef) - (Encoder_Max / 2.0f)) < 1) {
               Direct = 3;
            }
        
        }
        if(Direct == 3) {
            Send_buff[0] = 0;
            MotorSend(&hcan1, 0x200, Send_buff);
#if Down_Remote == 0
            xTaskCreate((TaskFunction_t)Remote_task,
                (const char *)"Remote_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)1,
                (TaskHandle_t *)&Remote_task_Handler);
#endif
            xTaskCreate((TaskFunction_t)Power_Protection_task,
                (const char *)"Power_Protection_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)1,
                (TaskHandle_t *)&Power_Protection_Handler);
            Robot_Status.RS_Ready = STATUS_TURN_ON;
            vTaskDelete(NULL);
        }
        
        PID_Control(ChassisMotor.Speed, SpeedExp, &ChassisMotor_SPID);
        limit(ChassisMotor_SPID.pid_out, 16384, -16384);
        Send_buff[0] = ChassisMotor_SPID.pid_out;
        
#if TEST == 0        
        MotorSend(&hcan1, 0x200, Send_buff);
#endif
        
        vTaskDelayUntil(&xLastWakeTime, 2);
    }

}
