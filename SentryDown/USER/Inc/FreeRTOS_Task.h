#ifndef _FREERTOS_TASK_H_
#define _FREERTOS_TASK_H_
#include "tim.h"

#include "PID.h"
#include "CANDrive.h"
#include "motor.h"
#include "config.h"

#include "PTZ_Init_Task.h"

//Pitch轴、Yaw轴电机数据
extern GM6020_TypeDef GM6020_Pitch,GM6020_Yaw;

//拨弹电机数据
extern M2006_TypeDef Pluck1;

//摩擦轮电机数据
extern RM3510_TypeDef Frictionwheel1,Frictionwheel2;

//Pitch轴角度、速度PID
extern PID_Smis GM6020_Pitch_PID;
extern PID GM6020_Pitch_SPID;

//Yaw轴角度、速度PID
extern PID_Smis GM6020_Yaw_PID;
extern PID GM6020_Yaw_SPID;

//拨弹电机速度PID
extern PID Pluck1_SPID;

//摩擦轮速度PID
extern PID Frictionwheel1_SPID;
extern PID Frictionwheel2_SPID;

void StartTask(void);

#endif
