#ifndef __POWER_PROTECTION_TASK_H
#define __POWER_PROTECTION_TASK_H

#include "RMLibHead.h"
#include "WatchDog.h"

#define Clear(p) memset(&(p), 0, sizeof(p))
#define Dog_HaveStatus(RS, status) (((RS)&(status)) == (status))
#define Dog_SetStatus(RS, status) ((RS) |= (status))
#define Dog_ClearStatus(RS, status) ((RS) &= ~(status))
#define Dog_NotStatus(RS, status) ((RS) ^= ~(status))

typedef enum {
    YawDog_Flag       = 0x01,
    PitchDog_Flag     = 0x02,
    PluckDog_Flag     = 0x04,
    Friction1Dog_Flag = 0x08,
    Friction2Dog_Flag = 0x10,
    ALL_Flag          = 0x1f,
} WatchDog_Flag_t;

extern SemaphoreHandle_t Power_Semaphore;

extern TaskHandle_t Power_Protection_Handler;
void Power_Protection_task(void *pvParameters);
void PowerProtectionProcess(void);

#endif
