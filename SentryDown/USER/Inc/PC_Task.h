#ifndef _PC_task_H
#define _PC_task_H

#include "RMLibHead.h"
#include "VCOMCOMM.h"

#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "FreeRTOS_Task.h"

typedef struct __PACKED_USB_STRUCT {
    uint16_t       Yaw_MchanicalAngle;       //Yaw轴机械角度
    uint16_t       Pitch_MchanicalAngle;     //Pitch轴机械角度
    int16_t        Location;                 //车体位置(mm)
    int16_t        Speed;                    //车体速度(m/s)
    Robot_Status_t Status;
}USB_PACK_t;    

extern uint8_t PTZ_First_Flag;

extern TaskHandle_t PC_task_Handler;
void PC_task(void *pvParameters);

#endif
