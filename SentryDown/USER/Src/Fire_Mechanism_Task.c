#include "Fire_Mechanism_Task.h"


/*发射机构进程*/
TaskHandle_t Fire_Mechanism_task_Handler;
void Fire_Mechanism_Task(void *pvParameters){
    int16_t Send_buff[4] = { 0 };
    int16_t FrictionwheelSpeed = 0;
    
    portTickType xLastWakeTime = xTaskGetTickCount();
    
    for( ;; ) {
        if(Robot_Status.RS_Loaded)
            FrictionwheelSpeed = FrictionwheelSpeedExp;
        else
            FrictionwheelSpeed = 0;
        
        PID_Control(Frictionwheel1.Speed, -FrictionwheelSpeed, &Frictionwheel1_SPID);
        limit(Frictionwheel1_SPID.pid_out, 29000, -29000);
        
        PID_Control(Frictionwheel2.Speed, FrictionwheelSpeed, &Frictionwheel2_SPID);
        limit(Frictionwheel2_SPID.pid_out, 29000, -29000);
        
        Send_buff[2] = Frictionwheel1_SPID.pid_out;
        Send_buff[3] = Frictionwheel2_SPID.pid_out;
        
#if TEST == 0
        MotorSend(&hcan1,0x200,Send_buff);
#else
        UNUSED(Send_buff);
#endif
        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}

