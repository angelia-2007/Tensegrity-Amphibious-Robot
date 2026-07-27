#ifndef __SOFT_IIC
#define __SOFT_IIC
#include "main.h"
void HAL_Delay_us(uint16_t us);

#define IIC_DELAY_TIME 3

//SDA->PB14
//SCL->PB15
#define IIC_SCL_PIN GPIO_PIN_0
#define IIC_SCL_PORT GPIOC

#define IIC_SDA_PIN GPIO_PIN_1
#define IIC_SDA_PORT GPIOC

#define IIC_SCL_H()     HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN,GPIO_PIN_SET)
#define IIC_SCL_L()     HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN,GPIO_PIN_RESET)
#define IIC_SDA_H()     HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN,GPIO_PIN_SET)
#define IIC_SDA_L()     HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN,GPIO_PIN_RESET)


void Soft_IIC_Init(void);
void Soft_IIC_Start(void);
void Soft_IIC_Stop(void);
void Soft_IIC_ACK(void);
void Soft_IIC_NACK(void);
uint8_t Soft_IIC_Wait_ACK(void);
void Soft_IIC_Write_Byte(uint8_t Byte);
uint8_t Soft_IIC_Recv_Byte(uint8_t ack);


int8_t stm32_soft_i2c_read(GPIO_TypeDef* GPIOx,uint16_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t stm32_soft_i2c_write(GPIO_TypeDef* GPIOx,uint16_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

#endif




