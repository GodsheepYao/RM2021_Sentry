#include "PC_Task.h"
#include "FreeRTOS_Task.h"

TaskHandle_t PC_task_Handler;
void PC_task(void *pvParameters){
    USB_PACK_t USB_PACK;
    UpBoard_Data_t UpBoard_Data;
//    float last_Encoder_Locat = 15.7f * (float)(Encoder_Rand * 1600.0f + TIM1->CNT) / 1600.0f;

    portTickType xLastWakeTime = xTaskGetTickCount();
    for( ;; ) {
//        UpBoard_Data.Encoder_Locat = Encoder_Locat * 10.0f;
//        UpBoard_Data.Speed = Encoder_Speed * 100.0f;
        UpBoard_Data.Status = Robot_Status;
        
        if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
            USB_PACK.Yaw_MchanicalAngle    = GM6020_Yaw.MchanicalAngle;         
            USB_PACK.Yaw_AngularVelocity   = GM6020_Yaw.Speed;                     	
            USB_PACK.Pitch_MchanicalAngle  = GM6020_Pitch.MchanicalAngle;     	
            USB_PACK.Pitch_AngularVelocity = GM6020_Pitch.Speed;                   	
//            USB_PACK.Location              = UpBoard_Data.Encoder_Locat;                             //≥µÃÂŒª÷√(mm)
//            USB_PACK.Speed                 = UpBoard_Data.Speed;										   
            USB_PACK.Status = UpBoard_Data.Status;
            
//            portENTER_CRITICAL();    
//            VCOMM_Transmit(0x01, 0x01, (uint8_t*)&USB_PACK, sizeof(USB_PACK_t));
//            portEXIT_CRITICAL();
        }
        
        CAN_Send_StdDataFrame(&hcan2, 0x100, (uint8_t*)&UpBoard_Data);
//        last_Encoder_Locat = Encoder_Locat;
        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}
