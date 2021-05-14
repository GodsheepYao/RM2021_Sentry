#ifndef _CONFIG_H_
#define _CONFIG_H_

#define TEST 0
#define Down_Remote 0
#define Chassis_Active 0

#ifdef __HIGH
	#define PTZ_Yaw_median 3400
	#define PTZ_Yaw_MAX (PTZ_Yaw_median + 8191/360*60)
	#define PTZ_Yaw_MIN (PTZ_Yaw_median - 8191/360*60)

	#define PTZ_Pitch_median 6080
	#define PTZ_Pitch_MAX 6400
	#define PTZ_Pitch_MIN 5400

	#define Sensitivity_Chassis 10
	#define Sensitivity_Yaw -0.07f
	#define Sensitivity_Pitch 0.07f
	
#endif

#ifdef __LOW

    #define PTZ_Yaw_median 1304

    #define PTZ_Pitch_median 4078
    #define PTZ_Pitch_MAX 4619
    #define PTZ_Pitch_MIN 2700

    #define Sensitivity_Yaw 0.1f
    #define Sensitivity_Pitch 0.02f
	
#endif

#endif
