#include "led.h"

uint8_t LED_STATUS=0;

void LED_Run(void)
{
    static int16_t LED_PWM=0;
    static uint8_t Dir=0;
    
    if(RC.UnLock==0)//Î´½âËø×´Ì¬
    {
        if(RC.SbusIsLink ==false)//Ã»ÓÐÒ£¿ØÐÅºÅ
        {
            LED_STATUS=2;//ÂýºôÎüµÆ
        }
        else//ÓÐÒ£¿ØÐÅºÅ
        {
            LED_STATUS=3;//¿ìºôÎüµÆ
        }
        if(JZ_BMI088 == 1)
        {
            LED_STATUS=4;
        }
    }
    else//½âËø×´Ì¬
    {
        if(RC.SbusIsLink ==false)//Ã»ÓÐÒ£¿ØÐÅºÅ
        {
            LED_STATUS=0;//¹ØµÆ
        }
        else//ÓÐÒ£¿ØÐÅºÅ
        {
            LED_STATUS=1;//³£ÁÁ
        }
    }
   
//    if(!Link_State)
//			LED_STATUS=1;
		//LED_STATUS=LED_STATUS_test;
    LED_STATUS=2;//ÂýºôÎüµÆ
    //if(!KEY)LED_STATUS=0;
    switch(LED_STATUS)
    {
        case 0://³£Ãð
            LED_PWM=0;
        break;
        
        case 1://³£ÁÁ
            LED_PWM=1000;
        break;
        
        case 2://ÂýºôÎüµÆ
            if(Dir==0)
            {
                LED_PWM+=10;
                if(LED_PWM>=1000)
                {
                    LED_PWM=1000;
                    Dir=1;
                }
            }
            else
            {
                LED_PWM-=10;
                if(LED_PWM<=0)
                {
                    LED_PWM=0;
                    Dir=0;
                }
            }
        break;
            
        case 3://¿ìºôÎüµÆ
            if(Dir==0)
            {
                LED_PWM+=100;
                if(LED_PWM>=1000)
                {
                    LED_PWM=1000;
                    Dir=1;
                }
            }
            else
            {
                LED_PWM-=100;
                if(LED_PWM<=0)
                {
                    LED_PWM=0;
                    Dir=0;
                }
            }
        break;
            
        case 4://³¬¿ìºôÎüµÆ
            if(Dir==0)
            {
                LED_PWM+=300;
                if(LED_PWM>=1000)
                {
                    LED_PWM=1000;
                    Dir=1;
                }
            }
            else
            {
                LED_PWM-=300;
                if(LED_PWM<=0)
                {
                    LED_PWM=0;
                    Dir=0;
                }
            }
        break;
          
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, LED_PWM);
}



