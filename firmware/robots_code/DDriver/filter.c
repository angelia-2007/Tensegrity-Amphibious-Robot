#include "filter.h"

Butter_BufferData Accel_X_Butter_Buf;
Butter_BufferData Accel_Y_Butter_Buf;
Butter_BufferData Accel_Z_Butter_Buf;

Butter_LP_float RPY_Acc_Filter[3];
Butter_LP_float POS_Acc_Filter[3];
Butter_LP_float World_Acc_Filter[3];

Butter_LP_float Tof_Filter[3];
Butter_LP_float Flow_vel_Filter[3];

Butter_LP_float Flow_302_Filter1[2];
Butter_LP_float Flow_302_Filter2[2];
Butter_LP_float Flow_wuming_Filter1[2];

Butter_LP_float BMI_POS_Acc_Filter1[3];
Butter_LP_float BMI_POS_Acc_Filter2[3];
Butter_LP_float BMI_POS_Acc_Filter3[3];
Butter_LP_float BMI_RPY_Acc_Filter[3];
Butter_LP_float BMI_RPY_Gyro_Filter[3];
Butter_LP_float BMI_Flow_Gyro_Filter[3];


Butter_LP_float BMI_POS_Acc_Filter1_00[3];
Butter_LP_float BMI_POS_Acc_Filter2_00[3];
Butter_LP_float BMI_POS_Acc_Filter3_00[3];
Butter_LP_float BMI_RPY_Acc_Filter_00[3];
Butter_LP_float BMI_RPY_Gyro_Filter_00[3];
Butter_LP_float BMI_Flow_Gyro_Filter_00[3];


TD_BufferData Baro_TD;
TD_BufferData Gyro_TD[3];

//¿¨¶ûÂüÂË²¨Æ÷
double Baro_Speed_Kalman_Filter(double ImuDeltaSpeed,double BaroSpeed,double dt)
{     
    double Q=0.008f; 
    double R=1.0f; 
    

    static double Speed=0.0f;
    static double Q_bias=0.0, Speed_err=0;
    static double PCt_0=0, PCt_1=0, E=0;
    static double K_0=0, K_1=0, t_0=0.0f, t_1=-0.0f; 
    double Pdot[4] ={0,0,0,0}; 
    static double PP[2][2] = { { 0.0f, 0.0f},{ 0.0f, 0.0f } };
//    
//    if(RC.UnLock==0)Q=0.5f;
    
    Speed += (ImuDeltaSpeed + Q_bias); 
    
    Pdot[0]=Q - PP[0][1] - PP[1][0]; 
    Pdot[1] = -PP[1][1]; 
    Pdot[2] = -PP[1][1]; 
    Pdot[3] = 0; 

    PP[0][0] += Pdot[0] * dt; 
    PP[0][1] += Pdot[1] * dt; 
    PP[1][0] += Pdot[2] * dt; 
    PP[1][1] += Pdot[3] * dt; 

    PCt_0 = PP[0][0];
    PCt_1 = PP[1][0]; 
    E = R + PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E; 

    Speed_err = BaroSpeed - Speed; 
    Speed += K_0 * Speed_err; 
    Q_bias += K_1 * Speed_err; 
    
    t_0 = PCt_0; 
    t_1 = PP[0][1];
    PP[0][0] -= K_0 * t_0; 
    PP[0][1] -= K_0 * t_1; 
    PP[1][0] -= K_1 * t_0; 
    PP[1][1] -= K_1 * t_1; 
    
    return Speed; 
} 


double Baro_Alt_Kalman_Filter(double Speed,double BaroAlt , double dt)
{     
    double Q=0.02f; 
    double R=1.0f; 
   
    static double Alt=0.0f;
    static double Q_bias=0.0, Alt_err=0;
    static double PCt_0=0, PCt_1=0, E=0;
    static double K_0=0, K_1=0, t_0=0.0f, t_1=0.0f; 
    double Pdot[4] ={0,0,0,0}; 
    static double PP[2][2] = { { 0.0f, 0.0f},{ 0.0f, 0.0f } };
    
//    if(RC.UnLock==0)Q=0.5f;
    
    Alt += (Speed - Q_bias)*dt; 
    
    Pdot[0]=Q - PP[0][1] - PP[1][0]; 
    Pdot[1] = -PP[1][1]; 
    Pdot[2] = -PP[1][1]; 
    Pdot[3] = 0; 

    PP[0][0] += Pdot[0] * dt; 
    PP[0][1] += Pdot[1] * dt; 
    PP[1][0] += Pdot[2] * dt; 
    PP[1][1] += Pdot[3] * dt; 

    PCt_0 = PP[0][0];
    PCt_1 = PP[1][0]; 
    E = R + PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E; 

    Alt_err = BaroAlt - Alt;
    
    Alt +=  K_0 * Alt_err; 
    Q_bias += K_1 * Alt_err; 

    t_0 = PCt_0; 
    t_1 = PP[0][1];
    PP[0][0] -= K_0 * t_0; 
    PP[0][1] -= K_0 * t_1; 
    PP[1][0] -= K_1 * t_0; 
    PP[1][1] -= K_1 * t_1; 
    
    return Alt; 
} 


double Speed_X_Filter(double ImuSpeed,double FlowSpeed , double dt)
{     
    double Q=0.005f;
    double R=1.0f; 
   
    static double Speed=0.0f;
    static double Q_bias=0.0, Speed_err=0;
    static double PCt_0=0, PCt_1=0, E=0;
    static double K_0=0, K_1=0, t_0=0.0f, t_1=0.0f; 
    double Pdot[4] ={0,0,0,0}; 
    static double PP[2][2] = { { 0.0f, 0.0f},{ 0.0f, 0.0f } };

    
    Speed += (ImuSpeed - Q_bias); 
    
    Pdot[0]=Q - PP[0][1] - PP[1][0]; 
    Pdot[1] = -PP[1][1]; 
    Pdot[2] = -PP[1][1]; 
    Pdot[3] = 0; 

    PP[0][0] += Pdot[0] * dt; 
    PP[0][1] += Pdot[1] * dt; 
    PP[1][0] += Pdot[2] * dt; 
    PP[1][1] += Pdot[3] * dt; 

    PCt_0 = PP[0][0];
    PCt_1 = PP[1][0]; 
    E = R + PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E; 

    Speed_err = FlowSpeed - Speed; 
    Speed += K_0 * Speed_err; 
    Q_bias += K_1 * Speed_err; 

    t_0 = PCt_0; 
    t_1 = PP[0][1];
    PP[0][0] -= K_0 * t_0; 
    PP[0][1] -= K_0 * t_1; 
    PP[1][0] -= K_1 * t_0; 
    PP[1][1] -= K_1 * t_1; 
    
    return Speed; 
} 

double Speed_Y_Filter(double ImuSpeed,double FlowSpeed , double dt)
{     
    double Q=0.005f;
    double R=1.0f; 
   
    static double Speed=0.0f;
    static double Q_bias=0.0, Speed_err=0;
    static double PCt_0=0, PCt_1=0, E=0;
    static double K_0=0, K_1=0, t_0=0.0f, t_1=0.0f; 
    double Pdot[4] ={0,0,0,0}; 
    static double PP[2][2] = { { 0.0f, 0.0f},{ 0.0f, 0.0f } };

    
    Speed += (ImuSpeed - Q_bias); 
    
    Pdot[0]=Q - PP[0][1] - PP[1][0]; 
    Pdot[1] = -PP[1][1]; 
    Pdot[2] = -PP[1][1]; 
    Pdot[3] = 0; 

    PP[0][0] += Pdot[0] * dt; 
    PP[0][1] += Pdot[1] * dt; 
    PP[1][0] += Pdot[2] * dt; 
    PP[1][1] += Pdot[3] * dt; 

    PCt_0 = PP[0][0];
    PCt_1 = PP[1][0]; 
    E = R + PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E; 

    Speed_err = FlowSpeed - Speed; 
    Speed += K_0 * Speed_err; 
    Q_bias += K_1 * Speed_err; 

    t_0 = PCt_0; 
    t_1 = PP[0][1];
    PP[0][0] -= K_0 * t_0; 
    PP[0][1] -= K_0 * t_1; 
    PP[1][0] -= K_1 * t_0; 
    PP[1][1] -= K_1 * t_1; 
    
    return Speed; 
} 


double POS_X_Filter(double Speed,double Int , double dt)
{     
    double Q=0.005f;
    double R=1.0f; 
   
    static double POS=0.0f;
    static double Q_bias=0.0, POS_err=0;
    static double PCt_0=0, PCt_1=0, E=0;
    static double K_0=0, K_1=0, t_0=0.0f, t_1=0.0f; 
    double Pdot[4] ={0,0,0,0}; 
    static double PP[2][2] = { { 0.0f, 0.0f},{ 0.0f, 0.0f } };

    
    POS += (Speed - Q_bias)*dt; 
    
    Pdot[0]=Q - PP[0][1] - PP[1][0]; 
    Pdot[1] = -PP[1][1]; 
    Pdot[2] = -PP[1][1]; 
    Pdot[3] = 0; 

    PP[0][0] += Pdot[0] * dt; 
    PP[0][1] += Pdot[1] * dt; 
    PP[1][0] += Pdot[2] * dt; 
    PP[1][1] += Pdot[3] * dt; 

    PCt_0 = PP[0][0];
    PCt_1 = PP[1][0]; 
    E = R + PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E; 

    POS_err = Int - POS; 
    POS += K_0 * POS_err; 
    Q_bias += K_1 * POS_err; 

    t_0 = PCt_0; 
    t_1 = PP[0][1];
    PP[0][0] -= K_0 * t_0; 
    PP[0][1] -= K_0 * t_1; 
    PP[1][0] -= K_1 * t_0; 
    PP[1][1] -= K_1 * t_1; 
    
    return POS; 
} 


double POS_Y_Filter(double Speed,double Int , double dt)
{     
    float Q=0.005f;
    float R=1.0f; 
   
    static float POS=0.0f;
    static float Q_bias=0.0, POS_err=0;
    static float PCt_0=0, PCt_1=0, E=0;
    static float K_0=0, K_1=0, t_0=0.0f, t_1=0.0f; 
    float Pdot[4] ={0,0,0,0}; 
    static float PP[2][2] = { { 0.0f, 0.0f},{ 0.0f, 0.0f } };

    
    POS += (Speed - Q_bias)*dt; 
    
    Pdot[0]=Q - PP[0][1] - PP[1][0]; 
    Pdot[1] = -PP[1][1]; 
    Pdot[2] = -PP[1][1]; 
    Pdot[3] = 0; 

    PP[0][0] += Pdot[0] * dt; 
    PP[0][1] += Pdot[1] * dt; 
    PP[1][0] += Pdot[2] * dt; 
    PP[1][1] += Pdot[3] * dt; 

    PCt_0 = PP[0][0];
    PCt_1 = PP[1][0]; 
    E = R + PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E; 

    POS_err = Int - POS; 
    POS += K_0 * POS_err; 
    Q_bias += K_1 * POS_err; 

    t_0 = PCt_0; 
    t_1 = PP[0][1];
    PP[0][0] -= K_0 * t_0; 
    PP[0][1] -= K_0 * t_1; 
    PP[1][0] -= K_1 * t_0; 
    PP[1][1] -= K_1 * t_1; 
    
    return POS; 
} 




//Î¢·ÖÆ÷
void Tracking_Differentiator(float in , float *out , float Ts, float Td , TD_BufferData *TD)
{   
    TD->X1 = TD->LX1 + Ts * TD->LX2;
    TD->X2 = TD->LX2 - Ts * ( ( TD->LX1 - in ) / ( Td * Td ) + 2.0f/ Td * TD->LX2 );
    TD->LX1 = TD->X1;
    TD->LX2 = TD->X2;
    *out = TD->X2;

}


//°ÍÌØÎÖË¹ÂË²¨Æ÷
uint8_t BUT_IIR_calc_freq_f( float* k , const float sample_freq, const float cutoff_freq , const float cp )
{	
	const float Pi = 3.1415926535897932384626433832795f;
	if ( (cutoff_freq <= 0.0001f) || (sample_freq <= 2.1f * cutoff_freq) ) {
			// no filtering
		return 0;
	}
	float cos_PI_cp = cosf( Pi * cp );
	
	float fr = sample_freq / cutoff_freq;
	float ohm = tanf(Pi/fr);
	float ohm2 = ohm*ohm;
	float c = 1.0f+2.0f*cos_PI_cp*ohm + ohm2;
	float inv_c = 1.0f / c;
	k[0] = ohm2 * inv_c;
	k[1] = 2.0f*(ohm2-1.0f) * inv_c;
	k[2] = (1.0f-2.0f*cos_PI_cp*ohm+ohm2) * inv_c;
	
	return 1;
}

uint8_t Butter_setCutoffFrequency( Butter_LP_float* filter , float sample_freq , float cutoff_freq )
{
    filter->available = BUT_IIR_calc_freq_f( filter->k_1 , sample_freq , cutoff_freq , 1.0f / 8 );
    BUT_IIR_calc_freq_f( filter->k_2 , sample_freq , cutoff_freq , 3.0f / 8 );
    return filter->available;
}

void Butter_init( Butter_LP_float* filter , float sample_freq , float cutoff_freq )
{
    filter->xv_1[0] = filter->xv_1[1] = filter->xv_1[2] = 0;
    filter->xv_2[0] = filter->xv_2[1] = filter->xv_2[2] = 0;
    filter->out = 0;
    
    Butter_setCutoffFrequency( filter , sample_freq , cutoff_freq );
}


float Butter_run( Butter_LP_float* filter , float newdata )
{
    if( filter->available )
    {
        filter->xv_1[2] = filter->xv_1[1]; filter->xv_1[1] = filter->xv_1[0]; 
        filter->xv_1[0] = newdata * filter->k_1[0] - filter->k_1[1] * filter->xv_1[1] - filter->k_1[2] * filter->xv_1[2];
        float out1 = filter->xv_1[0] + 2.0f * filter->xv_1[1] + filter->xv_1[2];
        
        filter->xv_2[2] = filter->xv_2[1]; filter->xv_2[1] = filter->xv_2[0]; 
        filter->xv_2[0] = out1 * filter->k_2[0] - filter->k_2[1] * filter->xv_2[1] - filter->k_2[2] * filter->xv_2[2];
        filter->out = filter->xv_2[0] + 2.0f * filter->xv_2[1] + filter->xv_2[2];
    }
    else
        filter->out = newdata;
    return filter->out;
}

