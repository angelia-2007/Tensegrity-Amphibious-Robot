#include "control.h"
#include "imu_bmi088.h"
#define XPYP 1
#define XPYM 2
#define XMYP 3
#define XMYM 4
#define YPZP 5
#define YPZM 6
#define YMZP 7
#define YMZM 8
#define ZPXP 9
#define ZPXM 10
#define ZMXP 11
#define ZMXM 12



uint8_t tense_action[8][3]=
{
	 {XMYM,YMZM,ZMXM},	 //000
	{XMYM,YMZP,ZPXM},		//001
	{XMYP,YPZM,ZMXM},  //010
	{XMYP,YPZP,ZPXM},          //011
	{XPYM,YMZM,ZMXP},//100
	{XPYM,YMZP,ZPXP},//101
	{XPYP,YPZM,ZMXP},//110
	{XPYP,YPZP,ZPXP}//111
};
uint8_t currentstate=0;


float gravity[8][3]=
{
	4.9f,3.4f,7.3f,//000
	-1.1f,8.5f,-0.8f,//001
	9.7f,-3.0f,-0.48f,//010
	3.7f,1.8f,-8.5f,//011
	-3.3f,-4.1f,8.6f,//100
	
	-9.2f,0.9f,0.4f,//101
	1.5f,-10.0f,0.8f,//110
	-4.4f,-5.5f,-7.2f
};
float result[8];

volatile uint8_t debug_tense_id = 0;
volatile uint8_t debug_tense_action = 0;

void tense_move(uint8_t id,uint8_t action){
	debug_tense_id = id;
    debug_tense_action = action;
	switch (id){
		case 1:
			if(action)
			{
				HAL_GPIO_WritePin(O1_GPIO_Port,O1_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(O1_GPIO_Port,O1_Pin,GPIO_PIN_RESET);
			}
		break;
				case 2:
			if(action)
			{
				HAL_GPIO_WritePin(O2_GPIO_Port,O2_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(O2_GPIO_Port,O2_Pin,GPIO_PIN_RESET);
			}
		break;
					case 3:
			if(action)
			{
				HAL_GPIO_WritePin(O3_GPIO_Port,O3_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(O3_GPIO_Port,O3_Pin,GPIO_PIN_RESET);
			}
		break;
					case 4:
			if(action)
			{
				HAL_GPIO_WritePin(O4_GPIO_Port,O4_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(O4_GPIO_Port,O4_Pin,GPIO_PIN_RESET);
			}
		break;
					case 5:
			if(action)
			{
				HAL_GPIO_WritePin(O5_GPIO_Port,O5_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(O5_GPIO_Port,O5_Pin,GPIO_PIN_RESET);
			}
		break;
					case 6:
			if(action)
			{
				HAL_GPIO_WritePin(O6_GPIO_Port,O6_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(O6_GPIO_Port,O6_Pin,GPIO_PIN_RESET);
			}
		break;
					case 7:
			if(action)
			{
				HAL_GPIO_WritePin(O7_GPIO_Port,O7_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(O7_GPIO_Port,O7_Pin,GPIO_PIN_RESET);
			}
		break;
					case 8:
			if(action)
			{
				HAL_GPIO_WritePin(O8_GPIO_Port,O8_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(O8_GPIO_Port,O8_Pin,GPIO_PIN_RESET);
			}
		break;
					case 9:
			if(action)
			{
				HAL_GPIO_WritePin(O9_GPIO_Port,O9_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(O9_GPIO_Port,O9_Pin,GPIO_PIN_RESET);
			}
		break;
					case 10:
			if(action)
			{
				HAL_GPIO_WritePin(O10_GPIO_Port,O10_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(O10_GPIO_Port,O10_Pin,GPIO_PIN_RESET);
			}
		break;
					case 11:
			if(action)
			{
				HAL_GPIO_WritePin(O11_GPIO_Port,O11_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(O11_GPIO_Port,O11_Pin,GPIO_PIN_RESET);
			}
		break;
					case 12:
			if(action)
			{
				HAL_GPIO_WritePin(O12_GPIO_Port,O12_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(O12_GPIO_Port,O12_Pin,GPIO_PIN_RESET);
			}
		break;
		
		
		
		}
		
		
		
		
		
	}
	uint8_t action1,action2;
	
	
	uint8_t tense_route(uint8_t axis)
	{
		uint8_t temp=currentstate;
		switch(axis){
			case 1://Z axiz
				action1=tense_action[temp][2];
				action2=tense_action[temp][1];
			return 1;
			break;
					case 2://Y axiz
				action1=tense_action[temp][1];
				action2=tense_action[temp][0];
			return 1;
			break;
			case 4://X axiz
				action1=tense_action[temp][0];
				action2=tense_action[temp][2];
			return 1;
			break;					
		}
		
		return 0;
		
		
	}
void state_est(void)
{
	for(int i=0;i<8;i++)
	{
		result[i]=imu_bmi088_data.Acc[0]*gravity[i][0]+imu_bmi088_data.Acc[1]*gravity[i][1]+imu_bmi088_data.Acc[2]*gravity[i][2];
	}
	float maxvalue=0;
	int max;
		for(int i=0;i<8;i++)
	{
		if(result[i]>maxvalue)
		{
			maxvalue=result[i];
			max=i;
		}
	}
	currentstate=max;
}
	
	
int softcontrol=0;
uint8_t waiting_for_input=1;
int loop_count=0;
int wait_count=0;
int toggle_axis=1;

#define ac1 4
#define ac2 12
#define ac3 8
#define ac4 9
#define ac5 6
#define ac6 1
#define ac7 1
#define ac8 1
#define ac9 1
#define ac10 1
#define ac11 1
#define ac12 1
#define on_interval 100
#define off_interval 1000
int route_task_count=0;
int route_cmp_count=0;
extern RC_INFO RC;
int running;
void tense_update(void)//500HZ
{
	//state_est();
	if(running)
	{
		RC.SbusIsLink=1;
		SBUS_Ch[4]=1600;
	}
if(RC.SbusIsLink){
	if(SBUS_Ch[4]>1500)
	{
		running=1;
	route_task_count++;
	if(route_task_count==off_interval)
	{
		 route_cmp_count=route_task_count;
		 tense_move(ac1,1);
	}else if(route_task_count==on_interval+off_interval)
	{
		 route_cmp_count=route_task_count;
		 tense_move(ac1,0);
	}else if(route_task_count==1*on_interval+2*off_interval)
	{
		route_cmp_count=route_task_count;
		 tense_move(ac2,1);
	}else if(route_task_count==2*on_interval+2*off_interval)
	{
		route_cmp_count=route_task_count;
		 tense_move(ac2,0);
	}else if(route_task_count==2*on_interval+3*off_interval)
	{
		route_cmp_count=route_task_count;
		 tense_move(ac3,1);
	}else if(route_task_count==3*on_interval+3*off_interval)
	{
		route_cmp_count=route_task_count;
		 tense_move(ac3,0);
	}else if(route_task_count==3*on_interval+4*off_interval)
	{
		route_cmp_count=route_task_count;
		 tense_move(ac4,1);
	}else if(route_task_count==4*on_interval+4*off_interval)
	{
		route_cmp_count=route_task_count;
		 tense_move(ac4,0);
	}else if(route_task_count==4*on_interval+5*off_interval)
	{
		route_cmp_count=route_task_count;
		 tense_move(ac5,1);
	}else if(route_task_count==5*on_interval+5*off_interval)
	{
		route_cmp_count=route_task_count;
		 tense_move(ac5,0);
	}else if(route_task_count==6*on_interval+6*off_interval)
	{
		route_cmp_count=route_task_count;
		 //tense_move(ac6,1);
		running=0;
	}
	/*else if(route_task_count==route_cmp_count+on_interval)
	{
		route_cmp_count=route_task_count;
		 tense_move(ac6,0);
	}else if(route_task_count==route_cmp_count+off_interval)
	{
		route_cmp_count=route_task_count;
		 tense_move(ac7,1);
	}else if(route_task_count==route_cmp_count+on_interval)
	{
		route_cmp_count=route_task_count;
		 tense_move(ac7,0);
	}

*/


	}else{

	route_task_count=0;
	if(SBUS_Ch[0]>1500)
		{
			tense_move(1,1);
		}else{
			tense_move(1,0);
		}
	if(SBUS_Ch[0]<500)
		{
			tense_move(2,1);
		}else{
			tense_move(2,0);
		}	
		if(SBUS_Ch[1]>1500)
		{
			tense_move(3,1);
		}else{
			tense_move(3,0);
		}
	if(SBUS_Ch[1]<500)
		{
			tense_move(4,1);
		}else{
			tense_move(4,0);
		}
	if(SBUS_Ch[2]>1500)
		{
			tense_move(5,1);
		}else{
			tense_move(5,0);
		}
	if(SBUS_Ch[2]<500)
		{
			tense_move(6,1);
		}else{
			tense_move(6,0);
		}
	if(SBUS_Ch[3]>1500)
		{
			tense_move(7,1);
		}else{
			tense_move(7,0);
		}
	if(SBUS_Ch[3]<500)
		{
			tense_move(8,1);
		}else{
			tense_move(8,0);
		}
		if(SBUS_Ch[5]>1500)
		{
			tense_move(9,1);
		}else{
			tense_move(9,0);
		}
		if(SBUS_Ch[6]>1500)
		{
			tense_move(10,1);
		}else{
			tense_move(10,0);
		}
				if(SBUS_Ch[7]>1500)
		{
			tense_move(10,1);
		}else{
			tense_move(10,0);
		}
				if(SBUS_Ch[8]>1500)
		{
			tense_move(11,1);
		}else{
			tense_move(11,0);
		}
				if(SBUS_Ch[9]>1500)
		{
			tense_move(12,1);
		}else{
			tense_move(12,0);
		}

			
			
			
			
	
	}
}else{

tense_move(1,0);
	tense_move(2,0);
	tense_move(3,0);
	tense_move(4,0);
	tense_move(5,0);
	tense_move(6,0);
	tense_move(7,0);
	tense_move(8,0);
	tense_move(9,0);
	tense_move(10,0);
	tense_move(11,0);
	tense_move(12,0);


}	
	
	
	
	/* 单独删除这个就可以
	if(waiting_for_input)
	{
		if(SBUS_Ch[4]<1500)
		{
		wait_count++;
			if(wait_count>300)
			{
			wait_count=0;
			loop_count=0;
			waiting_for_input=0;
			if(tense_route(toggle_axis))
		{
			loop_count=0;
			waiting_for_input=0;
			switch(toggle_axis)
			{
				case 1:toggle_axis=2;
				break;
				case 2: toggle_axis=4;
				break;
				case 4 :toggle_axis=1;
				break;
				
				
			}
			return;
		}		
			return;
	}
			
	}
	
		switch (softcontrol){

			case 1:
				softcontrol=0;
				if(tense_route(5))
		{
			loop_count=0;
			waiting_for_input=0;
			return;
		}
			
			break;
			
			case 2:
						softcontrol=0;
				if(tense_route(8))
		{
			loop_count=0;
			waiting_for_input=0;
			return;
		}		
			
			
			break;
			case 3:
								softcontrol=0;
				if(tense_route(9))
		{
			loop_count=0;
			waiting_for_input=0;
			return;
		}
		break;

		}			
			if(SBUS_Ch[0]>1500)
	{
		if(tense_route(4))
		{
			loop_count=0;
			waiting_for_input=0;
			return;
		}
		
		
	}
	
				if(SBUS_Ch[1]>1500)
	{
		if(tense_route(2))
		{
			loop_count=0;
			waiting_for_input=0;
			return;
		}
		
		
	}
				if(SBUS_Ch[3]>1500)
	{
		if(tense_route(1))
		{
			loop_count=0;
			waiting_for_input=0;
			return;
		}
	}
		
		
	
	
}else{
	loop_count++;
	if(loop_count==10)
	{
		 tense_move(action1,1);
	}else if(loop_count==510)
	{
		 tense_move(action1,0);

	}
	else if(loop_count==1000)
	{
		 tense_move(action2,1);
		
	}else if(loop_count==1500)
	{
		 tense_move(action2,0);
	}else if(loop_count==2000){
		waiting_for_input=1;
	}
	
}
	
	

	
	
	
	
	
	
	
	
	if(SBUS_Ch[4]>1500)
	{
		HAL_GPIO_WritePin(O1_GPIO_Port,O1_Pin,GPIO_PIN_SET);
		
	}else{
		HAL_GPIO_WritePin(O1_GPIO_Port,O1_Pin,GPIO_PIN_RESET);
	}
	
		if(SBUS_Ch[5]>1500)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
		
	}else{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	}
	*/
	
}


