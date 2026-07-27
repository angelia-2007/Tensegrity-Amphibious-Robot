#ifndef _FILTER_H
#define _FILTER_H
#include "main.h"

//°ÍÌØÎÖË¹ÂË²¨Æ÷
typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;
extern Butter_BufferData Accel_X_Butter_Buf;
extern Butter_BufferData Accel_Y_Butter_Buf;
extern Butter_BufferData Accel_Z_Butter_Buf;

typedef struct
{
    uint8_t available;	
    float k_1[3];
    float k_2[3];

    float xv_1[3];
    float xv_2[3];
    float out;
}Butter_LP_float;
extern Butter_LP_float RPY_Acc_Filter[3];
extern Butter_LP_float POS_Acc_Filter[3];
extern Butter_LP_float World_Acc_Filter[3];

extern Butter_LP_float Tof_Filter[3];
extern Butter_LP_float Flow_vel_Filter[3];

extern Butter_LP_float Flow_302_Filter1[2];
extern Butter_LP_float Flow_302_Filter2[2];
extern Butter_LP_float Flow_wuming_Filter1[2];

extern Butter_LP_float BMI_POS_Acc_Filter1[3];
extern Butter_LP_float BMI_POS_Acc_Filter2[3];
extern Butter_LP_float BMI_POS_Acc_Filter3[3];
extern Butter_LP_float BMI_RPY_Acc_Filter[3];
extern Butter_LP_float BMI_RPY_Gyro_Filter[3];
extern Butter_LP_float BMI_Flow_Gyro_Filter[3];

extern Butter_LP_float BMI_POS_Acc_Filter1_00[3];
extern Butter_LP_float BMI_POS_Acc_Filter2_00[3];
extern Butter_LP_float BMI_POS_Acc_Filter3_00[3];
extern Butter_LP_float BMI_RPY_Acc_Filter_00[3];
extern Butter_LP_float BMI_RPY_Gyro_Filter_00[3];
extern Butter_LP_float BMI_Flow_Gyro_Filter_00[3];

void Butter_init( Butter_LP_float* filter , float sample_freq , float cutoff_freq );
float Butter_run( Butter_LP_float* filter , float newdata );

//Î¢·ÖÆ÷
typedef struct
{
    float X1,LX1;
    float X2,LX2;
}TD_BufferData;
extern TD_BufferData Baro_TD;
extern TD_BufferData Gyro_TD[3];

void Tracking_Differentiator(float in , float *out , float Ts, float Td , TD_BufferData *TD);

//¿¨¶ûÂüÂË²¨Æ÷
double Baro_Speed_Kalman_Filter(double ImuDeltaSpeed,double BaroSpeed,double dt);
double Baro_Alt_Kalman_Filter(double Speed,double BaroAlt , double dt);

double Speed_X_Filter(double ImuSpeed,double FlowSpeed , double dt);
double Speed_Y_Filter(double ImuSpeed,double FlowSpeed , double dt);
double POS_X_Filter(double Speed,double Int , double dt);
double POS_Y_Filter(double Speed,double Int , double dt);

#endif

