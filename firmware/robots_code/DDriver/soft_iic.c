#include "soft_iic.h"

//SDA->PB14
//SCL->PB15

//void HAL_Delay_us(uint16_t us)
//{
//	uint16_t differ=0xffff-us-5;                    //设定定时器计数器起始值
//	__HAL_TIM_SET_COUNTER(&htim6,differ);
//	HAL_TIM_Base_Start(&htim6);                     //启动定时器
//	while(differ<0xffff-5)                             //补偿，判断
//	{
//		differ=__HAL_TIM_GET_COUNTER(&htim6);             //查询计数器的计数值
//	}
//	HAL_TIM_Base_Stop(&htim6);
//}
void HAL_Delay_us(uint16_t us)
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
 * @brief IIC初始化
 * @param  无
 * @return 无
 */
void Soft_IIC_Init(void)
{
    IIC_SDA_H();
    IIC_SCL_H();
}

/**
 * @brief SDA引脚设置输出模式
 * @param  无
 * @return 无
 */
static void Soft_IIC_Output(void)
{
    GPIO_InitTypeDef SOFT_IIC_GPIO_STRUCT;
    SOFT_IIC_GPIO_STRUCT.Mode = GPIO_MODE_OUTPUT_PP;
	  SOFT_IIC_GPIO_STRUCT.Pull = GPIO_PULLUP;
    SOFT_IIC_GPIO_STRUCT.Pin = IIC_SDA_PIN;
    SOFT_IIC_GPIO_STRUCT.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(IIC_SDA_PORT, &SOFT_IIC_GPIO_STRUCT);
}

/**
 * @brief SDA引脚设置输入模式
 * @param  无
 * @return 无
 */
static void Soft_IIC_Input(void)
{
    GPIO_InitTypeDef SOFT_IIC_GPIO_STRUCT;
    SOFT_IIC_GPIO_STRUCT.Mode = GPIO_MODE_INPUT;
	  SOFT_IIC_GPIO_STRUCT.Pull = GPIO_PULLUP;
    SOFT_IIC_GPIO_STRUCT.Pin = IIC_SDA_PIN;
    SOFT_IIC_GPIO_STRUCT.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(IIC_SDA_PORT, &SOFT_IIC_GPIO_STRUCT);
}

/**
 * @brief IIC起始信号
 * @param  无
 * @return 无
 */
void Soft_IIC_Start(void)
{
    Soft_IIC_Output();
//    IIC_SCL_L();
    IIC_SDA_H();
    IIC_SCL_H();
    HAL_Delay_us(IIC_DELAY_TIME);
    IIC_SDA_L();
    HAL_Delay_us(IIC_DELAY_TIME);
    IIC_SCL_L();
	  HAL_Delay_us(IIC_DELAY_TIME);
}

/**
 * @brief IIC停止信号
 * @param  无
 * @return 无
 */
void Soft_IIC_Stop(void)
{
    Soft_IIC_Output();
    IIC_SCL_L();
	  HAL_Delay_us(IIC_DELAY_TIME);
    IIC_SDA_L();
	  HAL_Delay_us(IIC_DELAY_TIME);
    IIC_SCL_H();
    HAL_Delay_us(IIC_DELAY_TIME);
    IIC_SDA_H();
    HAL_Delay_us(IIC_DELAY_TIME);
}

/**
 * @brief IIC应答信号
 * @param  无
 * @return 无
 */
void Soft_IIC_ACK(void)
{
    IIC_SCL_L();
		HAL_Delay_us(IIC_DELAY_TIME);
	  Soft_IIC_Output();
    IIC_SDA_L();
    HAL_Delay_us(IIC_DELAY_TIME);
    IIC_SCL_H();
    HAL_Delay_us(IIC_DELAY_TIME);
    IIC_SCL_L();
		HAL_Delay_us(IIC_DELAY_TIME);
}

/**
 * @brief IIC无应答信号
 * @param  无
 * @return 无
 */
void Soft_IIC_NACK(void)
{
	  IIC_SCL_L();
		HAL_Delay_us(IIC_DELAY_TIME);
    Soft_IIC_Output();
    IIC_SDA_H();
    HAL_Delay_us(IIC_DELAY_TIME);
    IIC_SCL_H();
    HAL_Delay_us(IIC_DELAY_TIME);
	  IIC_SCL_L();
		HAL_Delay_us(IIC_DELAY_TIME);
}

/**
 * @brief IIC等待应答信号
 * @param  无
 * @return 
 */
uint8_t Soft_IIC_Wait_ACK(void)
{
    uint8_t wait;
    Soft_IIC_Input();
    IIC_SDA_H();
		HAL_Delay_us(IIC_DELAY_TIME);
    IIC_SCL_H();
    HAL_Delay_us(IIC_DELAY_TIME);
    while (HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN))
    {
        wait++;
        if (wait > 100)
        {
            Soft_IIC_Stop();
            return 1;
        }
    }
    IIC_SCL_L();
    return 0;
}


/**
 * @brief IIC写入单个数据
 * @param  无
 * @return 应答信号, 0无应答 1有应答
 */
void Soft_IIC_Write_Byte(uint8_t Byte)
{
    uint8_t i;
    Soft_IIC_Output();
    for (i = 0; i < 8; i ++)
    {
			IIC_SCL_L();
			HAL_Delay_us(IIC_DELAY_TIME);
			if(Byte & 0x80)
				IIC_SDA_H();
			else
				IIC_SDA_L();
			Byte <<= 1;
			HAL_Delay_us(IIC_DELAY_TIME);
			IIC_SCL_H();
			HAL_Delay_us(IIC_DELAY_TIME);
    }
		IIC_SCL_L();
//    return (Soft_IIC_Wait_ACK());
}

/**
 * @brief IIC读一个数据
 * @param  ACK:应答 NACK:不应答
 * @return 返回读到的数据
 */
uint8_t Soft_IIC_Recv_Byte(uint8_t ack)
{
    uint8_t data = 0, i;
	  IIC_SDA_H();
    Soft_IIC_Input();
//    HAL_Delay_us(IIC_DELAY_TIME);
    for (i = 0; i < 8; i ++)
    {
			data <<= 1;
			IIC_SCL_L();
			HAL_Delay_us(IIC_DELAY_TIME);
			IIC_SCL_H();
			HAL_Delay_us(IIC_DELAY_TIME);
      if (HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN) == 1)
				data |= 0x01;
//      HAL_Delay_us(IIC_DELAY_TIME);
    }
		IIC_SCL_L();
    if (!ack)
    {
        Soft_IIC_NACK();
    }
    else
    {
        Soft_IIC_ACK();
    }
    return data;
}
//模拟iic连续读取数据
int8_t stm32_soft_i2c_read(GPIO_TypeDef* GPIOx,uint16_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	
	//发送起始位
	Soft_IIC_Start();
	//发送器件地址+写命令
	Soft_IIC_Write_Byte((dev_addr<<1)|0);
	if(Soft_IIC_Wait_ACK()) //等待应答
	{
		Soft_IIC_Stop();
		return 1;
	}
	//发送写（起始）寄存器地址
	Soft_IIC_Write_Byte(reg_addr);
	Soft_IIC_Wait_ACK();//等待应答

	//发送重复起始位
	Soft_IIC_Start();
	//发送器件地址+读命令
	Soft_IIC_Write_Byte((dev_addr<<1)|1);
	Soft_IIC_Wait_ACK();//等待应答
	//读取数据
	while (len)
	{
		if (len == 1)
			*data = Soft_IIC_Recv_Byte(0); // 读数据,发送nACK
		else
			*data = Soft_IIC_Recv_Byte(1); // 读数据,发送ACK
		len--;
		data++;
	}
	Soft_IIC_Stop();//产生一个停止条件
 
	return 0;
}
//模拟iic连续写数据
int8_t stm32_soft_i2c_write(GPIO_TypeDef* GPIOx,uint16_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	Soft_IIC_Start();
  Soft_IIC_Write_Byte((dev_addr<<1)|0); //发送器件地址+写命令
	if(Soft_IIC_Wait_ACK()) //等待应答
	{
		Soft_IIC_Stop();
		return 1;
	}
	
	Soft_IIC_Write_Byte(reg_addr);//发送写（起始）寄存器地址
	if(Soft_IIC_Wait_ACK()) //等待应答
	{
		Soft_IIC_Stop();
		return 1;
	}
	
	for (uint8_t i = 0; i < len; i++)
	{
		Soft_IIC_Write_Byte(data[i]); // 发送数据
		Soft_IIC_Wait_ACK();
	}
//	if(Soft_IIC_Wait_ACK()) //等待应答
//	{
//		Soft_IIC_Stop();
//		return 1;
//	}
	Soft_IIC_Stop();//产生一个停止条件
	return 0;
	
}





