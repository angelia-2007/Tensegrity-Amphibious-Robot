#include "motor.h"

//舵机1中心点1831
//舵机2中心点1993
void Motor_Output(int16_t motor1 , int16_t motor2 , int16_t motor3 , int16_t motor4,int16_t motor5)
{
	int16_t Lmotor1,Lmotor2,Lmotor3,Lmotor4,Lmotor5;
	Lmotor1 = Moto_Limit_Value(motor1);
	Lmotor2 = Moto_Limit_Value(motor2);
	Lmotor3 = Limit_Value_Sover1(motor3);
	Lmotor4 = Limit_Value_Sover2(motor4);
	Lmotor5 = Limit_Value_Sover3(motor5);			//位置
	
	
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Lmotor1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Lmotor2);
//	WriteSpe(1,Lmotor3,255);
//	WriteSpe(2,Lmotor4,255);
//	WritePosEx(3,Lmotor5,1000,255);
}

int16_t Limit_Value_Sover12(int16_t value)
{
	//1050-1500
	int16_t limited_value;
	limited_value = value > 5000 ? 5000 : value ;
	limited_value = value < -5000 ? -5000 : value ;
	return limited_value;
}

int16_t Limit_Value_Sover1(int16_t value)
{
	//1050-1500
	int16_t limited_value;
	limited_value = value > 2500 ? 2500 : value ;
	limited_value = value < 1500 ? 1500 : value ;
	return limited_value;
}

int16_t Limit_Value_Sover2(int16_t value)
{
	//1050-1500
	int16_t limited_value;
	limited_value = value > 2500 ? 2500 : value ;
	limited_value = value < 1500 ? 1500 : value ;
	return limited_value;
}

int16_t Limit_Value_Sover3(int16_t value)
{
	//1050-1500
	int16_t limited_value;
	limited_value = value > 3200 ? 3200 : value ;
	limited_value = value < 2100 ? 2100 : value ;
	return limited_value;
}

int16_t Moto_Limit_Value(int16_t value)
{
	//1050-1500
	int16_t limited_value;
	limited_value = value > 1900 ? 1900 : value ;
	limited_value = value < 1200 ? 1200 : value ;
	return limited_value;
}


