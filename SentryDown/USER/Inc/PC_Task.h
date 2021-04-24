#ifndef _PC_task_H
#define _PC_task_H

#include "RMLibHead.h"
#include "VCOMCOMM.h"

#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "FreeRTOS_Task.h"

typedef struct __PACKED_USB_STRUCT {
    int32_t        Yaw_MchanicalAngle;       //Yaw轴机械角度
    int16_t        Yaw_AngularVelocity;      //Yaw轴角速度
    int32_t        Pitch_MchanicalAngle;     //Pitch轴机械角度
    int16_t        Pitch_AngularVelocity;    //Pitch轴角速度
    int16_t        Location;                 //车体位置(mm)
    int16_t        Speed;                    //车体速度(m/s)
    Robot_Status_t Status;
}USB_PACK_t;                                

extern TaskHandle_t PC_task_Handler;
void PC_task(void *pvParameters);

#endif
