#ifndef _PC_task_H
#define _PC_task_H

#include "RMLibHead.h"
#include "VCOMCOMM.h"

#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "FreeRTOS_Task.h"

typedef struct __PACKED_USB_STRUCT {
    int32_t        Yaw_MchanicalAngle;       //Yaw���е�Ƕ�
    int16_t        Yaw_AngularVelocity;      //Yaw����ٶ�
    int32_t        Pitch_MchanicalAngle;     //Pitch���е�Ƕ�
    int16_t        Pitch_AngularVelocity;    //Pitch����ٶ�
    int16_t        Location;                 //����λ��(mm)
    int16_t        Speed;                    //�����ٶ�(m/s)
    Robot_Status_t Status;
}USB_PACK_t;                     

typedef struct __PACKED_UpBoard_STRUCT {
    int16_t Encoder_Locat;
    int16_t Speed;
    Robot_Status_t Status;
}UpBoard_Data_t;

extern TaskHandle_t PC_task_Handler;
void PC_task(void *pvParameters);

#endif
