#ifndef _CONFIG_H_
#define _CONFIG_H_

#ifdef __HIGH
	#define PTZ_Yaw_median 3455
	#define PTZ_Yaw_MAX (PTZ_Yaw_median + 8191/360*60)
	#define PTZ_Yaw_MIN (PTZ_Yaw_median - 8191/360*60)

	#define PTZ_Pitch_median 6151
	#define PTZ_Pitch_MAX 6681
	#define PTZ_Pitch_MIN 5473

	#define Sensitivity_Chassis 8
	#define Sensitivity_Yaw 0.07f
	#define Sensitivity_Pitch -0.07f
	
#endif

#ifdef __LOW

    #define PTZ_Yaw_median 7726

    #define PTZ_Pitch_median 6706
    #define PTZ_Pitch_MAX 7100
    #define PTZ_Pitch_MIN 5212

    #define Sensitivity_Yaw 0.02f
    #define Sensitivity_Pitch 0.02f
	
#endif

#endif
