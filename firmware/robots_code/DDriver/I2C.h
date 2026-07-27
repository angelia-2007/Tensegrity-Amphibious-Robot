#ifndef _I2C_H_
#define _I2C_H_

#include "main.h"
#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"

#define IIC_SCL_PIN1 GPIO_PIN_0
#define IIC_SCL_PORT1 GPIOC

#define IIC_SDA_PIN1 GPIO_PIN_1
#define IIC_SDA_PORT1 GPIOC

#define IIC_SCL_H1()     HAL_GPIO_WritePin(IIC_SCL_PORT1,IIC_SCL_PIN1,GPIO_PIN_SET)
#define IIC_SCL_L1()     HAL_GPIO_WritePin(IIC_SCL_PORT1,IIC_SCL_PIN1,GPIO_PIN_RESET)
#define IIC_SDA_H1()     HAL_GPIO_WritePin(IIC_SDA_PORT1,IIC_SDA_PIN1,GPIO_PIN_SET)
#define IIC_SDA_L1()     HAL_GPIO_WritePin(IIC_SDA_PORT1,IIC_SDA_PIN1,GPIO_PIN_RESET)


#ifndef I2C_Direction_Transmitter
	#define  I2C_Direction_Transmitter      ((uint8_t)0x00)
#endif

#ifndef I2C_Direction_Receiver
	#define  I2C_Direction_Receiver         ((uint8_t)0x01)
#endif

enum
{
	I2C_NACK,
	I2C_ACK
};

void I2C_SDAMode(uint8_t Mode);
void I2C_Start(void);
void I2C_Stop(void);
bool I2C_WaiteForAck(void);
void I2C_Ack(void);
void I2C_NAck(void);
bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data);
bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data);
void I2C_WriteByte(uint8_t Data);
uint8_t I2C_ReadByte(uint8_t Ack);
uint8_t I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data);
uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr);
bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);
bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);
void HMC_INIT(void);
void HMC_READ(void);
void I2C_GPIOInit(void);

int8_t soft_i2c_read(GPIO_TypeDef* GPIOx,uint16_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t soft_i2c_write(GPIO_TypeDef* GPIOx,uint16_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

#endif

