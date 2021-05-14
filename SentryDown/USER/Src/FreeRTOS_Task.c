#include "FreeRTOS_Task.h"
#include "PTZ_Init_Task.h"
#include "PTZ_Runtime_Task.h"
#include "Fire_Mechanism_Task.h"
#include "PC_Task.h"
#include "Remote_Task.h"
#include "Power_Protection_Task.h"


//Pitch轴、Yaw轴电机数据
GM6020_TypeDef GM6020_Pitch,GM6020_Yaw;

//拨弹电机数据
M2006_TypeDef Pluck1;

//摩擦轮电机数据
RM3508_TypeDef Frictionwheel1, Frictionwheel2;

//拨弹速度期望
int16_t PluckSpeedExp = -5000;

//摩擦轮速度期望
int16_t FrictionwheelSpeedExp = -7000;

//云台期望
PTZAngle_Ref_t PTZAngle_Ref = {.Pitch = PTZ_Pitch_median,.Yaw = PTZ_Yaw_median};

//Pitch轴角度、速度PID
PID_Smis GM6020_Pitch_PID = {.Kp = 8,.Ki = 0.05,.Kd = -30,.limit = 2000};
PID GM6020_Pitch_SPID = {.Kp = 15,.Ki = 0,.Kd = 3};

//Yaw轴角度、速度PID
PID_Smis GM6020_Yaw_PID = {.Kp = 10,.Ki = 0,.Kd = -45,.limit = 2000};
PID GM6020_Yaw_SPID = {.Kp = 10,.Ki = 0,.Kd = 2};

//拨弹电机速度PID
PID Pluck1_SPID = {.Kp = 13,.Ki = 0.5,.Kd = 1,.limit = 5000};

//摩擦轮速度PID
PID Frictionwheel1_SPID = {.Kp = 10,.Ki = 0.2,.Kd = 1,.limit = 1000};
PID Frictionwheel2_SPID = {.Kp = 10,.Ki = 0.2,.Kd = 1,.limit = 1000};

//机器人状态标志位
Robot_Status_t Robot_Status;

//上下板通信数据
UpBoard_Data_t UpBoard_Data;

//下云台出弹计数
int16_t Pill_Out = 0;

//所有电机看门狗
WatchDog_TypeDef Yaw_Dog, Pitch_Dog, Friction1_Dog, Friction2_Dog, Pluck_Dog;

/*初始任务*/
void StartTask(void) {
    Robot_Status.RS_Dead = 1;
#if Down_Remote == 1
    /*创建二值信号量*/
    Remote_Semaphore = xSemaphoreCreateBinary();
#endif
    
    WatchDog_Init(&Yaw_Dog, 10);
    WatchDog_Init(&Pitch_Dog, 10);
    WatchDog_Init(&Friction1_Dog, 10);
    WatchDog_Init(&Friction2_Dog, 10);
    WatchDog_Init(&Pluck_Dog, 10);
    
	CanFilter_Init(&hcan1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    CanFilter_Init(&hcan2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);  
    
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, usart1_dma_buff, 30);

	HAL_TIM_Base_Start(&htim2);
    
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
                
    xTaskCreate((TaskFunction_t)PC_task,
                (const char *)"PC_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)3,
                (TaskHandle_t *)&PC_task_Handler);
                
    xTaskCreate((TaskFunction_t)Power_Protection_task,
                (const char *)"Power_Protection_task",
                (uint16_t)256,
                (void *)NULL,
                (UBaseType_t)1,
                (TaskHandle_t *)&Power_Protection_Handler);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN1) {
    uint32_t can_id = CAN_Receive_DataFrame(&hcan1, CAN1_buff);
    switch (can_id) {
        case 0x203:
            RM3508_Receive(&Frictionwheel1, CAN1_buff);
            Feed_Dog(&Friction1_Dog);
            break;
        case 0x204:
            RM3508_Receive(&Frictionwheel2, CAN1_buff);
            Feed_Dog(&Friction2_Dog);
            break;
        case 0x205:
            GM6020_Receive(&GM6020_Pitch, CAN1_buff);
            Feed_Dog(&Pitch_Dog);
            break;
        case 0x206:
            GM6020_Receive(&GM6020_Yaw, CAN1_buff);
            Feed_Dog(&Yaw_Dog);
            break;
        case 0x207:
            M2006_Receive(&Pluck1, CAN1_buff);
            Feed_Dog(&Pluck_Dog);
            break;
    }
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN2)
    {
        uint32_t can_id = CAN_Receive_DataFrame(&hcan2, CAN2_buff);
        switch (can_id)
        {
            case 0x100:
                memcpy(&UpBoard_Data, CAN2_buff,sizeof(UpBoard_Data_t));
                Robot_Status.RS_Auto = UpBoard_Data.Status.RS_Auto;
                Robot_Status.RS_Downctl = UpBoard_Data.Status.RS_Downctl;
                Robot_Status.RS_Ready = UpBoard_Data.Status.RS_Ready;
                break;
            case 0x0ff:
                CAN_Remote_Rx(CAN2_buff);
                break;
        }
    }
}

uint32_t Get_TimerTick() {
    return TIM2->CNT;
}

