#include "FreeRTOS_Task.h"

//Pitch轴、Yaw轴电机数据
GM6020_TypeDef GM6020_Pitch,GM6020_Yaw;

//拨弹电机数据
M2006_TypeDef Pluck1;

//摩擦轮电机数据
RM3510_TypeDef Frictionwheel1,Frictionwheel2;

//Pitch轴角度、速度PID
PID_Smis GM6020_Pitch_PID = {.Kp = 0,.Ki = 0,.Kd = 0,.limit = 5000};
PID GM6020_Pitch_SPID = {.Kp = 0,.Ki = 0,.Kd = 0};

//Yaw轴角度、速度PID
PID_Smis GM6020_Yaw_PID = {.Kp = 0,.Ki = 0,.Kd = 0,.limit = 5000};
PID GM6020_Yaw_SPID = {.Kp = 0,.Ki = 0,.Kd = 0};

//拨弹电机速度PID
PID Pluck1_SPID = {.Kp = 0,.Ki = 0,.Kd = 0,.limit = 5000};

//摩擦轮速度PID
PID Frictionwheel1_SPID = {.Kp = 0,.Ki = 0,.Kd = 0,.limit = 1000};
PID Frictionwheel2_SPID = {.Kp = 0,.Ki = 0,.Kd = 0,.limit = 1000};


/*初始任务*/
void StartTask(void)
{   
	CanFilter_Init(&hcan1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_TIM_Base_Start(&htim2);
	
	xTaskCreate((TaskFunction_t)PTZ_Init_task,
                (const char *)"PTZ_Init_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)1,
                (TaskHandle_t *)&PTZ_Init_Handler);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1) {
    uint32_t can_id = CAN_Receive_DataFrame(&hcan1, CAN1_buff);
    switch (can_id) {
        case 0x203:
            RM3510_Receive(&Frictionwheel1, CAN1_buff);
            break;
        case 0x204:
            RM3510_Receive(&Frictionwheel2, CAN1_buff);
            break;
        case 0x205:
            GM6020_Receive(&GM6020_Pitch, CAN1_buff);
            break;
        case 0x206:
            GM6020_Receive(&GM6020_Yaw, CAN1_buff);
            break;
        case 0x207:
            M2006_Receive(&Pluck1, CAN1_buff);
            break;
    }
  }
}

uint32_t Get_TimerTick()
{
    return TIM2->CNT;
}

