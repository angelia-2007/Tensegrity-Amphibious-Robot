#include "QMC5883L.h"
#if 1//IIC
void HAL_Delay_us3(uint16_t us)
{
    for(uint16_t j=0;j<us;j++)
		{
			for(uint16_t i=0;i<7;i++)
			{  
					__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();//__NOP();//__NOP();
			}  
		}
}
void Soft_SDA_Output(void)
{
    GPIO_InitTypeDef SOFT_IIC_GPIO_STRUCT;
    SOFT_IIC_GPIO_STRUCT.Mode = GPIO_MODE_OUTPUT_PP;
	SOFT_IIC_GPIO_STRUCT.Pull = GPIO_PULLUP;
    SOFT_IIC_GPIO_STRUCT.Pin = IIC_SDA_PIN2;
    SOFT_IIC_GPIO_STRUCT.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(IIC_SDA_PORT2, &SOFT_IIC_GPIO_STRUCT);
}

/**
 * @brief SDA引脚设置输入模式
 * @param  无
 * @return 无
 */
void Soft_SDA_Input(void)
{
    GPIO_InitTypeDef SOFT_IIC_GPIO_STRUCT;
    SOFT_IIC_GPIO_STRUCT.Mode = GPIO_MODE_INPUT;
	  SOFT_IIC_GPIO_STRUCT.Pull = GPIO_PULLUP;
    SOFT_IIC_GPIO_STRUCT.Pin = IIC_SDA_PIN2;
    SOFT_IIC_GPIO_STRUCT.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(IIC_SDA_PORT2, &SOFT_IIC_GPIO_STRUCT);
}


void Soft_SCL_Output(void)
{
    GPIO_InitTypeDef SOFT_IIC_GPIO_STRUCT;
    SOFT_IIC_GPIO_STRUCT.Mode = GPIO_MODE_OUTPUT_PP;
	SOFT_IIC_GPIO_STRUCT.Pull = GPIO_PULLUP;
    SOFT_IIC_GPIO_STRUCT.Pin = IIC_SCL_PIN2;
    SOFT_IIC_GPIO_STRUCT.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(IIC_SCL_PORT2, &SOFT_IIC_GPIO_STRUCT);
}

/**
 * @brief SDA引脚设置输入模式
 * @param  无
 * @return 无
 */
void Soft_SCL_Input(void)
{
    GPIO_InitTypeDef SOFT_IIC_GPIO_STRUCT;
    SOFT_IIC_GPIO_STRUCT.Mode = GPIO_MODE_INPUT;
	  SOFT_IIC_GPIO_STRUCT.Pull = GPIO_PULLUP;
    SOFT_IIC_GPIO_STRUCT.Pin = IIC_SCL_PIN2;
    SOFT_IIC_GPIO_STRUCT.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(IIC_SCL_PORT2, &SOFT_IIC_GPIO_STRUCT);
}

void IIC_Init(void)
{
	IIC_SCL_OUT;
	IIC_SDA_OUT;
}
void IIC_Start(void)//产生IIC起始信号
{
	IIC_SDA_OUT;     //sda线输出
	IIC_SDA_SET;//IIC_SDA=1;	  	  
	IIC_SCL_SET;//IIC_SCL=1;
	HAL_Delay_us3(5);
	IIC_SDA_RESET;//IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	HAL_Delay_us3(5);
	IIC_SCL_RESET;//IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}
void IIC_Stop(void)//产生IIC停止信号
{
	IIC_SDA_OUT;//sda线输出
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_RESET;//IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	HAL_Delay_us3(5);
	IIC_SCL_SET;//IIC_SCL=1; 
	HAL_Delay_us3(5);
	IIC_SDA_SET;//IIC_SDA=1;//发送I2C总线结束信号				   	
}
void IIC_Ack(void)//产生ACK应答
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_RESET;//IIC_SDA=0;
	HAL_Delay_us3(5);
	IIC_SCL_SET;//IIC_SCL=1;
	HAL_Delay_us3(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}   
void IIC_NAck(void)//不产生ACK应答	
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_SET;//IIC_SDA=1;
	HAL_Delay_us3(5);
	IIC_SCL_SET;//IIC_SCL=1;
	HAL_Delay_us3(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}	
uint8_t IIC_Wait_Ack(void)//等待应答信号到来:1,接收应答失败;0,接收应答成功
{
	uint8_t ucErrTime=0;
	IIC_SDA_IN;      //SDA设置为输入     
	IIC_SCL_SET;//IIC_SCL=1;
	HAL_Delay_us3(5);	 
	while(IIC_SDA_State)//检测SDA是否仍为高电平
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_RESET;//IIC_SCL=0;
	return 0;  
} 
void IIC_Send_Byte(uint8_t txd)//IIC发送一个字节; 先发送高位
{                        
	uint8_t t;   
	IIC_SDA_OUT; 	    
	IIC_SCL_RESET;//IIC_SCL=0;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
			//IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_SET;//IIC_SDA=1;
		else
			IIC_SDA_RESET;//IIC_SDA=0;
		
		txd<<=1; 	  
		HAL_Delay_us3(5);   //对TEA5767这三个延时都是必须的
		IIC_SCL_SET;//IIC_SCL=1;
		HAL_Delay_us3(5); 
		IIC_SCL_RESET;//IIC_SCL=0;	
		HAL_Delay_us3(5);
	}	 
} 
uint8_t IIC_Read_Byte(unsigned char ack)//读一个字节，可加是否应答位,1加ack，0不加ack 从高位开始读
{
	unsigned char i,receive=0;
	IIC_SDA_IN;//SDA设置为输入
	for(i=0;i<8;i++ )
	{
		IIC_SCL_RESET;// IIC_SCL=0; 
		HAL_Delay_us3(5);
		IIC_SCL_SET;//IIC_SCL=1;
		receive<<=1;
		if(IIC_SDA_State)
			receive++;   
		HAL_Delay_us3(5); 
	}					 
		if (ack)
			IIC_Ack(); //发送ACK
		else
			IIC_NAck();//发送nACK   
		return receive;
}

uint8_t IIC_Write_Byte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t data)//直接写一个字节
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //发送写命令
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//发送地址	  
	ret |= IIC_Wait_Ack();		
	IIC_Send_Byte(data);     //发送字节							   
	ret |= IIC_Wait_Ack(); 

	IIC_Stop();
	HAL_Delay_us3(10);
	return ret;
}
uint8_t IIC_ReadMulByte(uint8_t DrvAddr,uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)//读字节
{  	    																 
	uint8_t ret=0;
	
	IIC_Start();  
	IIC_Send_Byte(DrvAddr);	   //发送写命令
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);//发送地址	    
	ret |= IIC_Wait_Ack();	    
	
	IIC_Start();
	IIC_Send_Byte(DrvAddr+1);           //进入接收模式			   
	ret |= IIC_Wait_Ack();
	while(NumToRead)
	{
		if(NumToRead==1)
		{
			*pBuffer=IIC_Read_Byte(0);	
		}
		else
		{
			*pBuffer=IIC_Read_Byte(1);
		}
		pBuffer++;
		NumToRead--;
	}
	IIC_Stop();//产生一个停止条件	
	return ret;	
}
uint8_t IIC_WriteMulByte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)//可一次写多个字节
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //发送写命令
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//发送地址	  
	ret |= IIC_Wait_Ack();		

	while(NumToWrite--)
	{ 										  		   
		IIC_Send_Byte(*pBuffer);     //发送字节							   
		ret |= IIC_Wait_Ack(); 
		pBuffer++;
	}
	IIC_Stop();
	HAL_Delay_us3(10);
	return ret;
}
#endif

/******************************************************地磁******************************************************/
uint8_t F_TASK_QMC5883L=0;
float_xyz Mag_angle;
int16_t Mag_temperature;
#if USING_QMC5883L
void QMC5883L_Init(void)
{
	uint8_t Config=0;
	
	IIC_Write_Byte(QMC5883L_ADDRESS,0x0B,	0x01);
	IIC_Write_Byte(QMC5883L_ADDRESS,0x20,	0x40);
	IIC_Write_Byte(QMC5883L_ADDRESS,0x21,	0x01);
	Config =	OVER_SAMPLE_RATIO_512\
						|FULL_SCALE_2G\
						|OUTPUT_DATA_RATE_10HZ\
						|MODE_CONTROL_CONTINUOUS;
	/****OSR=512,RNG=+/-2G,ODR=200Hz,MODE= continuous*******/
	IIC_Write_Byte(QMC5883L_ADDRESS,0x09,	Config);
	
	HAL_Delay(2);
}
short HMC_data_real[3];
uint8_t HMC_first=1;
short HMC_x_max,HMC_y_max,HMC_x_min,HMC_y_min,HMC_x_zero,HMC_y_zero,HMC_realx,HMC_realy;
float atan_xy,HMC_k;
float angle_HMC;
uint16_t run_flag;
void QMC5883L_GetAngle(float_xyz* data)//芯片第一脚在左上角，以下方做为指示，0为南，90为西，180为北，270为东
{
	uint8_t Mag[6],temp;
	i16_xyz Mag_data;
	IIC_ReadMulByte(QMC5883L_ADDRESS,0x06,&temp,1);

	IIC_ReadMulByte(QMC5883L_ADDRESS,0x00,Mag,6);
	Mag_data.x= (int16_t)(Mag[1] << 8 | Mag[0]);// First byte is LSB, second byte is MSB
	Mag_data.y= (int16_t)(Mag[3] << 8 | Mag[2]);
	Mag_data.z= (int16_t)(Mag[5] << 8 | Mag[4]);
	
	data->x=Mag_data.x;
	data->y=Mag_data.y;
	data->z=Mag_data.z;
	HMC_data_real[0] = Mag_data.x;
	HMC_data_real[1] = Mag_data.y;
	HMC_data_real[2] = Mag_data.z;
	
	if(HMC_first)
	{
		HMC_x_max=HMC_data_real[0];
		HMC_x_min=HMC_data_real[0];
		HMC_y_max=HMC_data_real[2];
		HMC_y_min=HMC_data_real[2];
		HMC_first=0;
	}
	if(run_flag == 0)
	{
	if(HMC_data_real[0]>HMC_x_max)HMC_x_max=HMC_data_real[0];
	if(HMC_data_real[0]<HMC_x_min)HMC_x_min=HMC_data_real[0];
	if(HMC_data_real[2]>HMC_y_max)HMC_y_max=HMC_data_real[2];
	if(HMC_data_real[2]<HMC_y_min)HMC_y_min=HMC_data_real[2];
}		
	HMC_x_zero=(HMC_x_max+HMC_x_min)/2;
	HMC_realx=HMC_data_real[0]-HMC_x_zero;
	HMC_y_zero=(HMC_y_max+HMC_y_min)/2;
	HMC_realy=HMC_data_real[2]-HMC_y_zero;
	HMC_k=(float)(HMC_x_max-HMC_x_min)/(float)(HMC_y_max-HMC_y_min);
	
//	  atan_xy=(float)HMC_data_real[2]/(float)HMC_data_real[0];
//	  angle_HMC=atan(atan_xy)*360/3.141592653589793;
	  angle_HMC=atan2((float)(HMC_realx),HMC_k*(float)(HMC_realy))*180/3.141592653589793;
		
//	mcu_dp_value_update(DPID_DIRECTION,(uint32_t)Mag_angle.x);

	
}
void TASK_QMC5883L(void)
{
	QMC5883L_GetAngle(&Mag_angle);
}
#endif
/******************************************************地磁******************************************************/
void Modules_Init(void)
{
	IIC_Init();
	QMC5883L_Init();
}
void SwitchIO_Init(void)
{
//	LED_4_OUT;	
}
void IO_Init(void)
{
	Modules_Init();
	SwitchIO_Init();
}
