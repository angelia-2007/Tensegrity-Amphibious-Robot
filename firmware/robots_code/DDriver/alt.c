#include "alt.h"
ALT_INFO Alt={500,0,0,0,0,0};


void Alt_Fus(void)
{
    static uint32_t last_run_time=0;
    float dt=0,d_dis;
    int8_t i=0;
    
    //计算间隔时间
    dt=(HAL_GetTick()-last_run_time)/1000.0f;
    last_run_time=HAL_GetTick();
    
    #define MaxAccelBias 0.01f
    static float Baro_Buf[20];
    static float SpeedErr=0,AltErr=0,IMUbias=0;
    static uint8_t BaroDatErr=0;

    //判断VL53L1X数据是否有效，主要是太阳光下波动太大
    static float VL_Buf[20];
   // static uint8_t tof_data_err=0;
//    float Average=0,Quality=0,deltas=0;
	
  //读取气压计原始数据
    Baro.Org_Alt=SPL06_Get_Altitude(); 

    //气压原始数据缓存
    for(i=19;i>0;i--)
    {
        Baro_Buf[i]=Baro_Buf[i-1];
    }
    Baro_Buf[0]=Baro.Org_Alt;

    //气压数据错误
    if(fabs(Baro_Buf[0]-Baro_Buf[5])>0.1f)
    {
        BaroDatErr=15;
    }
    else
    {
        if(BaroDatErr>0)BaroDatErr--;
    }  
		Alt.baro_alt_use+=(Baro_Buf[0]-Baro_Buf[1])*0.001f;
		
//    //TOF数据有效判断
//    if(SYS.VL53LX_ERR==0)
//    {
//        //缓存20个历史值，计算均值
//        for(i=19;i>0;i--)
//        {
//            VL_Buf[i]=VL_Buf[i-1];
//        }

//				VL_Buf[0]=vl53l1x.Distance*0.01f*fabs(IMU_Data.CosX*IMU_Data.CosY);		
//				d_dis = VL_Buf[0] - VL_Buf[1];
//        //方差太大，数据跳变,或者数据完全一模一样   || deltas<0.001f
////        if(vl53l1x.RangeStatus>=2 || fabs(VL_Buf[0]-VL_Buf[10])>0.3f )
////        {
////            tof_data_err=5;
////        }  
////        else
////        {
////            if(tof_data_err>0)tof_data_err--;
////        }

//    }
		
//		if(SYS.VL53LX_ERR==0 && tof_data_err==0)
//    {
//        Flow.tof_err=0;
//        
//        Alt.Ground_Alt+=(Alt.Fus_Alt-VL_Buf[0]-Alt.Ground_Alt)*0.1f;
//				Alt.tof_alt_use+=VL_Buf[0]-VL_Buf[1];
//				Alt.alt_use = Alt.tof_alt_use;
//			  Baro.ground_press=Baro.press;
//    }
//    else
//    {
//				
//        Alt.alt_use+=Baro_Buf[0]-Baro_Buf[1];
//        Flow.tof_err=1;
//    }
    
	if(fabs(d_dis)<0.1)
	{
		Alt.alt_use+=d_dis;
	}
	Alt.alt_use+=(Alt.baro_alt_use - Alt.alt_use) * 0.005f;		
	Alt.Ground_Alt+=(Alt.Fus_Alt-VL_Buf[0]-Alt.Ground_Alt)*0.1f;		
	
	
	Alt.alt_use = 	Alt.baro_alt_use;
    //速度滤波
    Tracking_Differentiator(Alt.alt_use , &Alt.alt_speed , 0.01f , 0.1f , &Baro_TD);
    Alt.Fus_Speed+=(imu_bmi088_data.POS_Acc_NoG_w[2] + IMUbias)*9.8f*dt;
    if(BaroDatErr==0)
    {
        SpeedErr=Alt.alt_speed-Alt.Fus_Speed;
        Alt.Fus_Speed+=SpeedErr*0.005f;
        if(fabs(Alt.alt_speed)<0.3f)IMUbias+=SpeedErr*0.0001f;
        IMUbias=IMUbias>MaxAccelBias?MaxAccelBias:(IMUbias<-MaxAccelBias?-MaxAccelBias:IMUbias);
    }
    
    //高度滤波
    Alt.Fus_Alt+=Alt.Fus_Speed*dt;
    AltErr=Alt.alt_use-Alt.Fus_Alt;
    Alt.Fus_Alt+=AltErr*0.02f;
    
    //起飞前快速跟随
    if(RC.Fly_flag==0)
    {
        Alt.Fus_Speed+=SpeedErr*0.02f;
        IMUbias=0;
        Alt.Fus_Alt+=AltErr*0.02f;
    }

    Alt.Flow_alt=Alt.Fus_Alt-Alt.Ground_Alt;
    
}



