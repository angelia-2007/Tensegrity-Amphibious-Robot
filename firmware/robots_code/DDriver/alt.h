#ifndef _HIGH_H
#define _HIGH_H
#include "main.h"

typedef struct 
{ 
	uint16_t Hold_Thr;
	float alt_use;
	float tof_alt_use;
	float baro_alt_use;

	float alt_speed;

	float Fus_Speed;
	float Fus_Alt;  
	float Ground_Alt;
	float Flow_alt;

	float ctr_pwm;
}ALT_INFO;
extern ALT_INFO Alt;
void Alt_Fus(void);


#endif

