#include "I2C.h"
/**************************************************************************
Function: IIC pin initialization
Input   : none
Output  : none
函数功能：IIC引脚初始化
入口参数：无
返回  值：无
**************************************************************************/
void I2C_GPIOInit(void)
{
	
//	GPIO_InitTypeDef  GPIO_InitStructure;
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

//	IIC_SCL=1;
//	IIC_SDA=1;

}
void HAL_Delay_us1(uint16_t us)
{
    for(uint16_t j=0;j<us;j++)
		{
			for(uint16_t i=0;i<7;i++)
			{  
					__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();//__NOP();//__NOP();
			}  
		}
}


/**
 * @brief SDA引脚设置输出模式
 * @param  无
 * @return 无
 */
static void Soft_IIC_Output1(void)
{
    GPIO_InitTypeDef SOFT_IIC_GPIO_STRUCT;
    SOFT_IIC_GPIO_STRUCT.Mode = GPIO_MODE_OUTPUT_PP;
	SOFT_IIC_GPIO_STRUCT.Pull = GPIO_PULLUP;
    SOFT_IIC_GPIO_STRUCT.Pin = IIC_SDA_PIN1;
    SOFT_IIC_GPIO_STRUCT.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(IIC_SDA_PORT1, &SOFT_IIC_GPIO_STRUCT);
}

/**
 * @brief SDA引脚设置输入模式
 * @param  无
 * @return 无
 */
static void Soft_IIC_Input1(void)
{
    GPIO_InitTypeDef SOFT_IIC_GPIO_STRUCT;
    SOFT_IIC_GPIO_STRUCT.Mode = GPIO_MODE_INPUT;
	  SOFT_IIC_GPIO_STRUCT.Pull = GPIO_PULLUP;
    SOFT_IIC_GPIO_STRUCT.Pin = IIC_SDA_PIN1;
    SOFT_IIC_GPIO_STRUCT.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(IIC_SDA_PORT1, &SOFT_IIC_GPIO_STRUCT);
}

/**************************************************************************
Function: Simulate IIC start signal
Input   : none
Output  : none
函数功能：模拟IIC起始信号
入口参数：无
返回  值：无
**************************************************************************/
void I2C_Start(void)
{
	
    Soft_IIC_Output1();
//    IIC_SCL_L();
    IIC_SDA_H1();
    IIC_SCL_H1();
    HAL_Delay_us1(IIC_DELAY_TIME);
    IIC_SDA_L1();
    HAL_Delay_us1(IIC_DELAY_TIME);
    IIC_SCL_L1();
	HAL_Delay_us1(IIC_DELAY_TIME);

}

/**************************************************************************
Function: Analog IIC end signal
Input   : none
Output  : none
函数功能：模拟IIC结束信号
入口参数：无
返回  值：无
**************************************************************************/
void I2C_Stop(void)
{
	
	Soft_IIC_Output1();
	IIC_SCL_L1();
	HAL_Delay_us1(IIC_DELAY_TIME);
	IIC_SDA_L1();
	HAL_Delay_us1(IIC_DELAY_TIME);
	IIC_SCL_H1();
	HAL_Delay_us1(IIC_DELAY_TIME);
	IIC_SDA_H1();
	HAL_Delay_us1(IIC_DELAY_TIME);
}

/**************************************************************************
Function: IIC response
Input   : none
Output  : none
函数功能：IIC应答
入口参数：无
返回  值：无
**************************************************************************/
void I2C_Ack(void)
{
	
	IIC_SCL_L1();
	HAL_Delay_us1(IIC_DELAY_TIME);
	Soft_IIC_Output1();
	IIC_SDA_L1();
	HAL_Delay_us1(IIC_DELAY_TIME);
	IIC_SCL_H1();
	HAL_Delay_us1(IIC_DELAY_TIME);
	IIC_SCL_L1();
	HAL_Delay_us1(IIC_DELAY_TIME);
	
}


/**************************************************************************
Function: IIC don't reply
Input   : none
Output  : none
函数功能：IIC不应答
入口参数：无
返回  值：无
**************************************************************************/ 
void I2C_NAck(void)
{
	IIC_SCL_L1();
	HAL_Delay_us1(IIC_DELAY_TIME);
	Soft_IIC_Output1();
	IIC_SDA_H1();
	HAL_Delay_us1(IIC_DELAY_TIME);
	IIC_SCL_H1();
	HAL_Delay_us1(IIC_DELAY_TIME);
	IIC_SCL_L1();
	HAL_Delay_us1(IIC_DELAY_TIME);
	
}



bool I2C_WaiteForAck(void)
{
	
	uint8_t wait;
    Soft_IIC_Input1();
    IIC_SDA_H1();
	HAL_Delay_us1(IIC_DELAY_TIME);
    IIC_SCL_H1();
    HAL_Delay_us1(IIC_DELAY_TIME);
    while (HAL_GPIO_ReadPin(IIC_SDA_PORT1, IIC_SDA_PIN1))
    {
        wait++;
        if (wait > 100)
        {
            I2C_Stop();
            return 1;
        }
    }
    IIC_SCL_L1();
    return 0;
}



bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data)
{
    uint8_t Dat;
    
    Dat =I2C_ReadOneByte(DevAddr, RegAddr);
    Dat = (Data != 0) ? (Dat | (1 << BitNum)) : (Dat & ~(1 << BitNum));
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}




bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data)
{

    uint8_t Dat, Mask;
    
	Dat = I2C_ReadOneByte(DevAddr, RegAddr);
    Mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
    Data <<= (8 - Length);
    Data >>= (7 - BitStart);
    Dat &= Mask;
    Dat |= Data;
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}

/**************************************************************************
Function: IIC sends a bit
Input   : none
Output  : none
函数功能：IIC发送一个位
入口参数：无
返回  值：无
**************************************************************************/
void I2C_WriteByte(uint8_t Data)
{
	    uint8_t i;
    Soft_IIC_Output1();

    for (i = 0; i < 8; i ++)
    {
			
			IIC_SCL_L1();
					HAL_Delay_us1(IIC_DELAY_TIME);
			if(Data & 0x80)
				IIC_SDA_H1();
			else
				IIC_SDA_L1();
			
			HAL_Delay_us1(IIC_DELAY_TIME);
			Data <<= 1;
			IIC_SCL_H1();
			HAL_Delay_us1(IIC_DELAY_TIME);
			
    }
	IIC_SCL_L1();
}


uint8_t I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_WriteByte(Data);
	I2C_WaiteForAck();
	I2C_Stop();
	return 1;
}


bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;

	if(0 == Num || NULL == pBuff)
	{
		return false;
	}
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	
	for(i = 0; i < Num; i ++)
	{
		I2C_WriteByte(*(pBuff + i));
		I2C_WaiteForAck();
	}
	I2C_Stop();

	return true;
}

/**************************************************************************
Function: IIC reads a bit
Input   : none
Output  : none
函数功能：IIC读取一个位
入口参数：无
返回  值：无
**************************************************************************/
uint8_t I2C_ReadByte(uint8_t Ack)
{
	uint8_t data = 0, i;
//	  IIC_SDA_H1();
    Soft_IIC_Input1();
//    HAL_Delay_us1(IIC_DELAY_TIME);
    for (i = 0; i < 8; i ++)
    {
			data <<= 1;
			IIC_SCL_L1();
			HAL_Delay_us1(IIC_DELAY_TIME);
			IIC_SCL_H1();
			HAL_Delay_us1(IIC_DELAY_TIME);
      if (HAL_GPIO_ReadPin(IIC_SDA_PORT1, IIC_SDA_PIN1) == 1)
				data |= 0x01;
//	  else
//		  data &= ~0x01;
//      HAL_Delay_us1(IIC_DELAY_TIME);
    }
		IIC_SCL_L1();
	if(I2C_ACK == Ack)
		I2C_Ack();
	else
		I2C_NAck();
//	if(I2C_ACK == 1)
//		I2C_Ack();
//	else
//		I2C_NAck();
    return data;
}




uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t TempVal = 0;
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
	I2C_WaiteForAck();
	TempVal = I2C_ReadByte(I2C_NACK);
	I2C_Stop();
	
	return TempVal;
}

bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;

	if(0 == Num || NULL == pBuff)
	{
		return false;
	}
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
//	I2C_WriteByte(DevAddr + 1);
	I2C_WaiteForAck();

	for(i = 0; i < Num; i ++)
	{
		if((Num - 1) == i)
		{
			*(pBuff + i) = I2C_ReadByte(I2C_NACK);
		}
		else
		{
			*(pBuff + i) = I2C_ReadByte(I2C_ACK);
		}
	}

	I2C_Stop();
	
	return true;
}

uint8_t HMC5883_SB_Read(uint8_t Slave_Address, uint8_t Register_Address) 
{
    static uint8_t Res_Data = 0;
    I2C_Start(); 
	I2C_WriteByte(Slave_Address);//0X3C	
	I2C_WaiteForAck();		//等待应答 
    I2C_WriteByte(Register_Address);	//寄存器地址
    I2C_WaiteForAck();		//等待应答
    I2C_Start();
	I2C_WriteByte(Slave_Address + 1);//0X3D	
    I2C_WaiteForAck();		//等待应答 
	Res_Data=I2C_ReadByte(0);//读取数据,发送nACK 
    I2C_Stop();			//产生一个停止条件 
	return Res_Data;		
}


///**************************************************************************
//Function: IIC continuous reading data
//Input   : dev：Target device IIC address；reg:Register address；
//					length：Number of bytes；*data:The pointer where the read data will be stored
//Output  : count：Number of bytes read out-1
//函数功能：IIC连续读数据
//入口参数：dev：目标设备IIC地址；reg:寄存器地址；length：字节数；
//					*data:读出的数据将要存放的指针
//返回  值：count：读出来的字节数量-1
//**************************************************************************/ 
//uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data){
//    uint8_t count = 0;
//	
//	IIC_Start();
//	IIC_Send_Byte(dev);	   //发送写命令
//	IIC_Wait_Ack();
//	IIC_Send_Byte(reg);   //发送地址
//  IIC_Wait_Ack();	  
//	IIC_Start();
//	IIC_Send_Byte(dev+1);  //进入接收模式	
//	IIC_Wait_Ack();
//	
//    for(count=0;count<length;count++){
//		 
//		 if(count!=length-1)   data[count]=IIC_Read_Byte(1);  //带ACK的读数据
//		 else                  data[count]=IIC_Read_Byte(0);  //最后一个字节NACK
//	}
//    IIC_Stop();//产生一个停止条件
//    return count;
//}

//*****************************磁力计*********************************************/ 

void HMC_INIT(void)
{
	 uint8_t data1[1] = {0x78};
	 uint8_t data2[1] = {0x40};
	 uint8_t data3[1] = {0x00};
//	I2C_WriteOneByte(0x3c,0x00,0x78);	
//	I2C_WriteOneByte(0x3c,0x01,0x40);	
//	I2C_WriteOneByte(0x3c,0x02,0x00);
	
	stm32_soft_i2c_write(GPIOC,0xD,0x00,data1,1);
	stm32_soft_i2c_write(GPIOC,0xD,0x01,data2,1);
	stm32_soft_i2c_write(GPIOC,0xD,0x02,data3,1);
	
}
//uint8_t HMC_data[6] = {0,0,0,0,0,0},HMC_first=1;
//short HMC_data_real[3];
//short HMC_x_max,HMC_y_max,HMC_x_min,HMC_y_min,HMC_x_zero,HMC_y_zero,HMC_realx,HMC_realy;
//float atan_xy,HMC_k;
//float angle_HMC;
//uint16_t run_flag;
// short Recive_Data[6] ; 
//void HMC_READ(void)
//{
//		
//	for(int i=0; i<6; i++)
//    {
//        Recive_Data[i] = HMC5883_SB_Read(0x3c, i+3) ;  //get data
//    }
//	
////	  I2C_ReadBuff(0x3c, 0x03, 6, HMC_data);
//	stm32_soft_i2c_read(GPIOC,0xD, 0x03,HMC_data,6);
//		HMC_data_real[0]=(HMC_data[0]<<8)+HMC_data[1];   		  //读取X轴cili
//		HMC_data_real[1]=(HMC_data[2]<<8)+HMC_data[3];  ;    //读取Y轴
//		HMC_data_real[2]=(HMC_data[4]<<8)+HMC_data[5];  ;    //读取Z轴
//	if(HMC_first)
//	{
//		HMC_x_max=HMC_data_real[0];
//		HMC_x_min=HMC_data_real[0];
//		HMC_y_max=HMC_data_real[2];
//		HMC_y_min=HMC_data_real[2];
//		HMC_first=0;
//	}
//	if(run_flag == 0)
//	{
//	if(HMC_data_real[0]>HMC_x_max)HMC_x_max=HMC_data_real[0];
//	if(HMC_data_real[0]<HMC_x_min)HMC_x_min=HMC_data_real[0];
//	if(HMC_data_real[2]>HMC_y_max)HMC_y_max=HMC_data_real[2];
//	if(HMC_data_real[2]<HMC_y_min)HMC_y_min=HMC_data_real[2];
//}		
//	HMC_x_zero=(HMC_x_max+HMC_x_min)/2;
//	HMC_realx=HMC_data_real[0]-HMC_x_zero;
//	HMC_y_zero=(HMC_y_max+HMC_y_min)/2;
//	HMC_realy=HMC_data_real[2]-HMC_y_zero;
//	HMC_k=(float)(HMC_x_max-HMC_x_min)/(float)(HMC_y_max-HMC_y_min);
//	
////	  atan_xy=(float)HMC_data_real[2]/(float)HMC_data_real[0];
////	  angle_HMC=atan(atan_xy)*360/3.141592653589793;
//	  angle_HMC=atan2((float)(HMC_realx),HMC_k*(float)(HMC_realy))*180/3.141592653589793;
//		
//}


//模拟iic连续读取数据
int8_t soft_i2c_read(GPIO_TypeDef* GPIOx,uint16_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	
	//发送起始位
	I2C_Start();
	//发送器件地址+写命令
	I2C_WriteByte((dev_addr<<1)|0);
	if(I2C_WaiteForAck()) //等待应答
	{
		Soft_IIC_Stop();
		return 1;
	}
	//发送写（起始）寄存器地址
	I2C_WriteByte(reg_addr);
	I2C_WaiteForAck();//等待应答

	//发送重复起始位
	I2C_Start();
	//发送器件地址+读命令
	I2C_WriteByte((dev_addr<<1)|1);
	I2C_WaiteForAck();//等待应答
	//读取数据
	while (len)
	{
		if (len == 1)
			*data = I2C_ReadByte(I2C_NACK); // 读数据,发送nACK
		else
			*data = Soft_IIC_Recv_Byte(I2C_ACK); // 读数据,发送ACK
		len--;
		data++;
	}
	I2C_Stop();//产生一个停止条件
 
	return 0;
}
//模拟iic连续写数据
int8_t soft_i2c_write(GPIO_TypeDef* GPIOx,uint16_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	I2C_Start();
  I2C_WriteByte((dev_addr<<1)|0); //发送器件地址+写命令
	if(I2C_WaiteForAck()) //等待应答
	{
		I2C_Stop();
		return 1;
	}
	
	I2C_WriteByte(reg_addr);//发送写（起始）寄存器地址
	if(I2C_WaiteForAck()) //等待应答
	{
		I2C_Stop();
		return 1;
	}
	
	for (uint8_t i = 0; i < len; i++)
	{
		I2C_WriteByte(data[i]); // 发送数据
		I2C_WaiteForAck();
	}

	I2C_Stop();//产生一个停止条件
	return 0;
	
}




