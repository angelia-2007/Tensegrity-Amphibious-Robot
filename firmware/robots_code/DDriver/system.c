#include "main.h"
#include "system.h"


void system_init(void)
{
	Soft_IIC_Init();
	SPL06_Init();
	imu_bmi088_init();
	Modules_Init();
	
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
//	HAL_Delay(1000);
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	HAL_Delay(1000);
	//HAL_Delay(1000);

//		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	Butter_init( &RPY_Acc_Filter[0] , 500 , 0.5 );
	Butter_init( &RPY_Acc_Filter[1] , 500 , 0.5 );
	Butter_init( &RPY_Acc_Filter[2] , 500 , 0.5 );
	Butter_init( &POS_Acc_Filter[0] , 500 , 50 );
	Butter_init( &POS_Acc_Filter[1] , 500 , 50 );
	Butter_init( &POS_Acc_Filter[2] , 500 , 50 );
	Butter_init( &World_Acc_Filter[0] , 500 , 50 );
	Butter_init( &World_Acc_Filter[1] , 500 , 50 );
	Butter_init( &World_Acc_Filter[2] , 500 , 50 );

	for(uint8_t i = 0; i < 3; i++)
	{
		Butter_init( &BMI_POS_Acc_Filter1[i] , 500 , 50 );
		Butter_init( &BMI_POS_Acc_Filter2[i] , 50 , 5 );
		Butter_init( &BMI_POS_Acc_Filter3[i] , 50 , 5 );
		Butter_init( &BMI_RPY_Acc_Filter[i] , 500 , 50 );
		Butter_init( &BMI_RPY_Gyro_Filter[i] , 500 , 150 );
		Butter_init( &BMI_Flow_Gyro_Filter[i] , 50 , 10 );
		Butter_init( &Tof_Filter[i] , 50 , 10 );
		Butter_init( &Flow_vel_Filter[i] , 50 , 20 );
	}
	
		for(uint8_t i = 0; i < 3; i++)
	{
		Butter_init( &BMI_POS_Acc_Filter1_00[i] , 500 , 50 );
		Butter_init( &BMI_POS_Acc_Filter2_00[i] , 50 , 5 );
		Butter_init( &BMI_POS_Acc_Filter3_00[i] , 50 , 5 );
		Butter_init( &BMI_RPY_Acc_Filter_00[i] , 500 , 50 );
		Butter_init( &BMI_RPY_Gyro_Filter_00[i] , 500 , 150 );
		Butter_init( &BMI_Flow_Gyro_Filter_00[i] , 50 , 10 );
	}
	
	for(uint8_t i = 0; i < 2; i++)
	{
		Butter_init( &Flow_302_Filter1[i] , 50 , 8 );
		Butter_init( &Flow_302_Filter2[i] , 50 , 10 );
		Butter_init( &Flow_wuming_Filter1[i] , 50 , 20 );
	}
}

