#ifndef __MOTOR_H_
#define __MOTOR_H_
#include "main.h"
#define Motor_Value_Limited 	500


extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim3;
void Motor_Output(int16_t motor1 , int16_t motor2 , int16_t motor3 , int16_t motor4,int16_t motor5);


int16_t Limit_Value_Sover12(int16_t value);
int16_t Limit_Value_Sover1(int16_t value);
int16_t Limit_Value_Sover2(int16_t value);
int16_t Limit_Value_Sover3(int16_t value);
int16_t Moto_Limit_Value(int16_t value);

#endif
