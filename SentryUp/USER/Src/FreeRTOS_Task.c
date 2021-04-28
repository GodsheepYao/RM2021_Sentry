#include "FreeRTOS_Task.h"
#include "Remote_Task.h"
#include "Chassis_Init_Task.h"
#include "PTZ_Init_Task.h"
#include "PC_Task.h"

//Pitch轴、Yaw轴电机数据
GM6020_TypeDef GM6020_Pitch, GM6020_Yaw;

//拨弹电机数据
M2006_TypeDef Pluck1, Pluck2;

//底盘电机数据
RM3508_TypeDef ChassisMotor;

//摩擦轮电机数据
RM3510_TypeDef Frictionwheel1, Frictionwheel2;

//云台期望角度结构体
PTZAngle_Ref_t PTZAngle_Ref = {.Pitch = PTZ_Pitch_median, .Yaw = PTZ_Yaw_median};

//底盘速度期望
int16_t ChassisSpeedExp = 0;

//拨弹速度期望
int16_t PluckSpeedExp = -2000;

//摩擦轮速度期望
int16_t FrictionwheelSpeedExp = 5000;

//Pitch轴角度、速度PID
PID_Smis GM6020_Pitch_PID = {.Kp = 15, .Ki = 0.1, .Kd = -25, .limit = 5000};
PID GM6020_Pitch_SPID = {.Kp = 10, .Ki = 0, .Kd = 3};

//Yaw轴角度、速度PID
PID_Smis GM6020_Yaw_PID = {.Kp = 5,.Ki = 0,.Kd = 0,.limit = 5000};
PID GM6020_Yaw_SPID = {.Kp = 5,.Ki = 0,.Kd = 0};

//拨弹电机速度PID
PID Pluck1_SPID = {.Kp = 13, .Ki = 0.5, .Kd = 1, .limit = 5000};
PID Pluck2_SPID = {.Kp = 13, .Ki = 0.5, .Kd = 1, .limit = 5000};

//摩擦轮电机速度PID
PID Frictionwheel1_SPID = {.Kp = 0,.Ki = 0,.Kd = 0,.limit = 1000};
PID Frictionwheel2_SPID = {.Kp = 0,.Ki = 0,.Kd = 0,.limit = 1000};

//机器人状态标志位
Robot_Status_t Robot_Status;

/*初始任务*/
void StartTask(void) {
#if Down_Remote == 0
    /*创建二值信号量*/
    Remote_Semaphore = xSemaphoreCreateBinary();
#endif
    CanFilter_Init(&hcan1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    CanFilter_Init(&hcan2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);  
    
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, usart1_dma_buff, 30);
    
    HAL_TIM_Base_Init(&htim2);
    
    xTaskCreate((TaskFunction_t)Chassis_Init_task,
                (const char *)"Chassis_Init_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)2,
                (TaskHandle_t *)&Chassis_Init_Handler);
    xTaskCreate((TaskFunction_t)PTZ_Init_task,
                (const char *)"PTZ_Init_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)2,
                (TaskHandle_t *)&PTZ_Init_Handler);
    xTaskCreate((TaskFunction_t)PC_task,
                (const char *)"PC_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)3,
                (TaskHandle_t *)&PC_task_Handler);
    
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN1) {
    uint32_t can_id = CAN_Receive_DataFrame(&hcan1, CAN1_buff);
      switch(can_id) {
          case 0x201:
              RM3508_Receive(&ChassisMotor, CAN1_buff);
              break;
          case 0x205:
                GM6020_Receive(&GM6020_Yaw, CAN1_buff);
                break;
          case 0x206:
                GM6020_Receive(&GM6020_Pitch, CAN1_buff);
                break;
      }
  }
  
}

uint32_t Get_TimerTick()
{
    return TIM2->CNT;
}
