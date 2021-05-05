#ifndef _PC_task_H
#define _PC_task_H

#include "RMLibHead.h"
#include "VCOMCOMM.h"

#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "FreeRTOS_Task.h"

/**
 * 机器人反馈信息结构体
 */
typedef struct {
    int16_t Yaw_MchanicalAngle;       //!<@brief Yaw轴机械角度
    int16_t Pitch_MchanicalAngle;     //!<@brief Pitch轴机械角度
    int16_t Yaw_AngularVelocity;      //!<@brief Yaw轴角速度
    int16_t Pitch_AngularVelocity;    //!<@brief Pitch轴角速度
    int16_t Location;                 //!<@brief 车体位置(mm, 左零点)
    int16_t Speed;                    //!<@brief 车体速度(mm/s)
    Robot_Status_t Status;            //!<@brief 机器人当前状态标志位
} Robot_Info_t;

/**
 * PC控制机器人的结构体
 */
typedef struct {
    int16_t Yaw_Angle;          //!<@brief Yaw轴机械角度
    int16_t Pitch_Angle;        //!<@brief Pitch轴机械角度
    Robot_Status_t Status;       //!<@brief 机器人控制标志位
    int16_t Speed;               //!<@brief 车体速度(mm/s)
} Control_Info_t;

extern uint8_t PTZ_First_Flag;

extern TaskHandle_t PC_task_Handler;
void PC_task(void *pvParameters);

#endif
