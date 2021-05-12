#ifndef _FREERTOS_TASK_H_
#define _FREERTOS_TASK_H_
#include "tim.h"
#include "usart.h"

#include "PID.h"
#include "CANDrive.h"
#include "motor.h"
#include "config.h"
#include "Chassis.h"
#include "WatchDog.h"
#include "RMQueue.h"
#include "Referee_offcial.h"

#define STATUS_TURN_ON 1      //!<@brief 状态开启
#define STATUS_TURN_OFF 0     //!<@brief 状态关闭

#define Queue_Deep 7	
#define Queue_Width 32	

typedef struct {
    uint32_t RS_Ready     :1; //!<@brief 初始化完成
    uint32_t RS_Dead      :1; //!<@brief 战场死亡标志位
    uint32_t RS_Kill      :1; //!<@brief 杀死标志位
    uint32_t RS_Fire      :1; //!<@brief 开火
    uint32_t RS_Loaded    :1; //!<@brief 上膛(摩擦轮启动)
    uint32_t RS_Downctl   :1; //!<@brief 下云台操控模式
    uint32_t RS_Auto      :1; //!<@brief 自瞄模式
} Robot_Status_t;

typedef struct __PACKED_UpBoard_STRUCT {
    int16_t Radiofreq_Limit;
    Robot_Status_t Status;
}UpBoard_Data_t;

void StartTask(void);

extern GM6020_TypeDef GM6020_Pitch,GM6020_Yaw;          //!<@brief Pitch轴、Yaw轴电机数据

extern M2006_TypeDef Pluck1, Pluck2;                    //!<@brief 拨弹电机数据

extern RM3508_TypeDef Frictionwheel1,Frictionwheel2;    //!<@brief 摩擦轮电机数据

extern RM3508_TypeDef ChassisMotor;                     //!<@brief 底盘电机数据

extern int16_t ChassisSpeedExp;             //!<@brief 底盘电机期望

extern PTZAngle_Ref_t PTZAngle_Ref;         //!<@brief 云台角度期望

extern int16_t FrictionwheelSpeedExp;       //!<@brief 摩擦轮速度期望

extern int16_t PluckSpeedExp;               //!<@brief 拨弹速度期望

extern uint8_t Pluck_Select;                //!<@brief 拨弹盘选择

extern int16_t Pill_SupplyDown, Pill_Out;   //!<@brief 下供弹计数

extern PID ChassisMotor_SPID;       //!<@brief 底盘速度PID

extern PID_Smis GM6020_Pitch_PID;   //!<@brief Pitch轴角度PID
extern PID GM6020_Pitch_SPID;       //!<@brief Pitch轴速度PID

extern PID_Smis GM6020_Yaw_PID;     //!<@brief Yaw轴角度PID
extern PID GM6020_Yaw_SPID;         //!<@brief Yaw轴速度PID

extern PID Pluck1_SPID, Pluck2_SPID;//!<@brief 拨弹电机速度PID

extern PID Frictionwheel1_SPID;     //!<@brief 摩擦轮1速度PID
extern PID Frictionwheel2_SPID;     //!<@brief 摩擦轮2速度PID

//编码器数据
extern int16_t Encoder_Rand;
extern float Encoder_Speed;
extern float Encoder_Locat;
extern float Encoder_offsef;
extern float Encoder_Max;

extern Robot_Status_t Robot_Status; //!<@brief 机器人全局标志位

extern WatchDog_TypeDef Chassis_Dog, Yaw_Dog, Pitch_Dog, 
       Friction1_Dog, Friction2_Dog, Pluck1_Dog, Pluck2_Dog; //!<@brief 所有下云台电机看门狗

extern WatchDog_TypeDef Remote_Dog; //!<@brief 遥控器看门狗

extern RMQueue_Handle Referee_Queue;//!<@brief 裁判系统队列

#endif
