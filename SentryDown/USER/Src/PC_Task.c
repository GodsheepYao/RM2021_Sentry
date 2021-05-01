#include "PC_Task.h"

uint8_t test[15] = "helloworld\r\n";

/*PC通信任务*/
TaskHandle_t PC_task_Handler;
void PC_task(void *pvParameters){
    USB_PACK_t USB_PACK;
    
    portTickType xLastWakeTime = xTaskGetTickCount();
    for( ;; ) {
        if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
            USB_PACK.Yaw_MchanicalAngle    = GM6020_Yaw.MchanicalAngle;                          	
            USB_PACK.Pitch_MchanicalAngle  = GM6020_Pitch.MchanicalAngle;     	                 											   
            USB_PACK.Status = Robot_Status;
        
            portENTER_CRITICAL();    
            VCOMM_Transmit(0x01, 0x01, (uint8_t*)&USB_PACK, sizeof(USB_PACK_t));
//            VCOMM_Transmit(0x01, 0x01, (uint8_t*)&test, sizeof(test));
            portEXIT_CRITICAL();
        }
        
        if(Robot_Status.RS_Downctl)
            HAL_GPIO_WritePin(DownMode_LED_GPIO_Port, DownMode_LED_Pin, GPIO_PIN_RESET);   
        else 
            HAL_GPIO_WritePin(DownMode_LED_GPIO_Port, DownMode_LED_Pin, GPIO_PIN_SET);
        
        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}

void VCOMM_CallBack(uint8_t fun_code, uint16_t id, uint8_t *data, uint8_t len) {
    if(Robot_Status.RS_Auto && Robot_Status.RS_Ready) {
        if(fun_code == 0x01 && id == 0x01 && len == sizeof(USB_PACK_t)) {
            USB_PACK_t USB_Receive_PACK = *(USB_PACK_t*) data;
            
            PTZAngle_Ref.Pitch = (USB_Receive_PACK.Pitch_MchanicalAngle + PTZ_Pitch_median) % 8192;
            limit(PTZAngle_Ref.Pitch, PTZ_Pitch_MAX, PTZ_Pitch_MIN);           
            PTZAngle_Ref.Yaw = (USB_Receive_PACK.Yaw_MchanicalAngle + PTZ_Yaw_median) % 8192;
        }
    }
}


