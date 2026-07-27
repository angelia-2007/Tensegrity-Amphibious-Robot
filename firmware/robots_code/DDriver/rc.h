#ifndef _RC__H
#define _RC__H
#include "main.h"
#include "global.h"
//非定点模式
#define Pwm_MAX     1200.0f   //PWM   < 700
#define Thr_MAX     1.3f     //Spd   < 1.5m/s
#define Rol_MAX     20.0f    //Roll  < 20°
#define Pit_MAX     20.0f    //Pitch < 20°
#define YAW_MAX     110.0f   //Rota  < 100°/s



typedef struct
{
    uint8_t AutoFly;
    uint8_t AutoUP;
    uint8_t AutoDOWN;
		uint8_t AutoPoint;
    uint8_t AutoThr;
    uint8_t UnLock;
	  uint8_t InOut;
    uint8_t Fly_flag;
    uint16_t Miss;	
	uint32_t UlockTime;
	uint8_t MODE;
	bool SbusIsLink;
	
    
    float YAW;
    float YAW_Angle;
    float PIT;
    float ROL;
    float Lock_Alt;
    float THR;
    float THR_PWM;
		#ifdef USE_CRSF
		uint16_t link_quality;
		
		#endif
}RC_INFO;

extern RC_INFO RC;

extern uint8_t OutDoor;
extern uint8_t JZ;
extern uint8_t mode_ground;
void FrSky_INFO_Handle(void);

#endif

