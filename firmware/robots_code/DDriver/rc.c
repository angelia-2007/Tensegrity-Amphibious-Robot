#include "rc.h"
#ifdef USE_CRSF
RC_INFO RC={0,0,0,0,1,0,0,200,0,   0,0,0,0,0,0,0     ,0};
#else
RC_INFO RC={0,0,0,0,1,0,0,200,0,   0,0,0,0,0,0,0};
#endif
uint8_t OutDoor;
uint8_t JZ=0;
uint8_t mode_ground=0;
void FrSky_INFO_Handle(void)
{
	static uint16_t t1=0,t2=0;

	//ROL倾角最大值
	RC.ROL=((float)SBUS_Ch[0]-1024.0f)/671.0f*Rol_MAX;
	if(fabs(RC.ROL)<Rol_MAX*0.1f)RC.ROL=0;

	//PIT倾角最大值
	RC.PIT=((float)SBUS_Ch[2]-1024.0f)/671.0f*Pit_MAX;
	if(fabs(RC.PIT)<Pit_MAX*0.1f)RC.PIT=0;

	//YAW角速度最大值
	RC.YAW=((float)SBUS_Ch[3]-1024.0f)/671.0f*YAW_MAX;
	if(fabs(RC.YAW)<YAW_MAX*0.3f)RC.YAW=0;
	if(RC.YAW>YAW_MAX*0.3f)RC.YAW=RC.YAW-YAW_MAX*0.3f;
	if(RC.YAW<(-YAW_MAX*0.3f))RC.YAW=RC.YAW+YAW_MAX*0.3f;

	//定高最大垂直速度
	RC.THR=((float)SBUS_Ch[1]-1024.0f)/671.0f*Thr_MAX;
	if(fabs(RC.THR)<Thr_MAX*0.1f)RC.THR=0;

	//姿态模式最大PWM油门
	RC.THR_PWM=((float)SBUS_Ch[1]-353.0f)/1342.0f*Pwm_MAX;

	//SE:模式选择【5】
	//SF锁【6】
    if(SBUS_Ch[5]<500)
    {
		RC.AutoThr=0;	//进入手动模式
		RC.AutoPoint =0;
    }else    
    if(SBUS_Ch[5]>500 && SBUS_Ch[5]<1200)
    {
            RC.AutoThr=1;//进入定高模式
						RC.AutoPoint = 0;
    }else
	if(SBUS_Ch[5]>1200)
	{
					RC.AutoThr=1;//进入定高模式
					RC.AutoPoint = 1;//进入定点模式
	}
    
		//油门校准JZ：0:->不校准；1->高油门；2->低油门
//    if((SBUS_Ch[6]<200&&RC.UnLock==0)||(SBUS_Ch[7]<200&&RC.UnLock==0))
//		{
//		 JZ=0;
//		}
//		if(SBUS_Ch[6]>900&& SBUS_Ch[6]<1100&&RC.UnLock==0&&SBUS_Ch[7]>1800)
//		{
//		  JZ=1;
//			
//		}
//		if(SBUS_Ch[6]>1800&& RC.UnLock==0&&SBUS_Ch[7]>1800)
//		{
//		  JZ=2;
//			if(!(SBUS_Ch[6]>1800))
//				JZ=0;
//		}	
//		
		if(SBUS_Ch[6]>900) 
		{
			RC.Fly_flag=0;
			// RC.AutoUP=0;
			RC.AutoFly=0;    
			RC.UnLock=0;
			RC.AutoDOWN=0;
		}
	

//    //按键？？？,一键起飞
//    if(SBUS_Ch[5]>900&& SBUS_Ch[5]<1100&& RC.UnLock==1 && RC.Fly_flag==0 )
//    {
//        RC.Fly_flag=1;
//       // RC.AutoUP=1;
//        RC.AutoFly=1;
//        //RC.Lock_Alt=Alt.Ground_Alt+0.4f;
//    }		
		
//    if(SBUS_Ch[4]=='A') OutDoor=1;
//		if(SBUS_Ch[4]=='B') OutDoor=0;
    //下外八解锁
    if(RC.ROL<-Rol_MAX*0.5f && RC.PIT>Pit_MAX*0.5f && RC.YAW>YAW_MAX*0.5f && RC.THR<-Thr_MAX*0.5f)
    {
        t1++;
			
		if(t1>20)
        {
            if(!RC.UnLock)
            {
                RC.Fly_flag=0;//解锁不会马上起飞，进入准备状态
                RC.UnLock=1;
				RC.UlockTime=HAL_GetTick();		  	
			}
        }
    }
    else
    {
        t1=0;
    }
    
    //上外八较零
    if(RC.UnLock==0 && RC.ROL<-Rol_MAX*0.5f && RC.PIT<-Pit_MAX*0.5f && RC.YAW>YAW_MAX*0.5f && RC.THR>Thr_MAX*0.5f)
    {
        if(JZ_BMI088 == false)
        {
            t2++;
            if(t2>50)
            {
				JZ_BMI088 = true;
            }
            
        }
    }
    else
    {
        t2=0;
    }
    
}
