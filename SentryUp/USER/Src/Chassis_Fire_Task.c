#include "Chassis_Fire_Task.h"
#include "FreeRTOS_Task.h"
#include "PC_Task.h"

TaskHandle_t Chassis_Fire_Handler;
void Chassis_Fire_task(void *pvParameters) {
    int16_t Send_buff[4] = { 0 };
    int16_t FrictionwheelSpeed = 0;
    int16_t  ChassisSpeed = 0;
    
//    PID Chassis_PID = {.Kp = 12, .Ki = 0, .Kd = 0, .limit = 1000};

    portTickType xLastWakeTime = xTaskGetTickCount();
    
    for( ;; ) {
        ChassisSpeed = Robot_Status.RS_Auto ? Chasssis_Auto_Speed : ChassisSpeedExp;
        
        if(Encoder_Max - (Encoder_Locat - Encoder_offsef) < 20) {
            if(ChassisSpeed < 0)
                ChassisSpeed *= (Encoder_Max - (Encoder_Locat - Encoder_offsef)) / 20;
        }

        if(Encoder_Locat - Encoder_offsef < 20) {
            if(ChassisSpeed > 0)
                ChassisSpeed *= (Encoder_Locat - Encoder_offsef) / 20;
        }
        
        if(Robot_Status.RS_Loaded && Frictionwheel1.temp < 80 && Frictionwheel2.temp < 80)
            FrictionwheelSpeed = FrictionwheelSpeedExp;
        else
            FrictionwheelSpeed = 0;
        
        PID_Control(ChassisMotor.Speed, ChassisSpeed, &ChassisMotor_SPID);
        limit(ChassisMotor_SPID.pid_out, 29000, -29000);
        
        PID_Control(Frictionwheel1.Speed, -FrictionwheelSpeed, &Frictionwheel1_SPID);
        limit(Frictionwheel1_SPID.pid_out, 29000, -29000);
        PID_Control(Frictionwheel2.Speed, FrictionwheelSpeed, &Frictionwheel2_SPID);
        limit(Frictionwheel2_SPID.pid_out, 29000, -29000);
       
        int16_t bili = refree_power_limit_bili() * 100;
#if Chassis_Active        
        Send_buff[0] = ChassisMotor_SPID.pid_out * bili / 100;
#endif
        Send_buff[2] = Frictionwheel1_SPID.pid_out;
        Send_buff[3] = Frictionwheel2_SPID.pid_out;
        
        if(!HAL_GPIO_ReadPin(Optical_Fiber1_GPIO_Port, Optical_Fiber1_Pin))
            HAL_GPIO_WritePin(Fiber1_GPIO_Port, Fiber1_Pin, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(Fiber1_GPIO_Port, Fiber1_Pin, GPIO_PIN_SET);
        
        if(!HAL_GPIO_ReadPin(Optical_Fiber2_GPIO_Port, Optical_Fiber2_Pin))
            HAL_GPIO_WritePin(Fiber2_GPIO_Port, Fiber2_Pin, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(Fiber2_GPIO_Port, Fiber2_Pin, GPIO_PIN_SET);
#if TEST == 0
        MotorSend(&hcan1, 0x200, Send_buff);
#else
        UNUSED(Send_buff);
#endif
        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}
