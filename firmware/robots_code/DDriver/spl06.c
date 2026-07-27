#include "spl06.h"

static int16_t _C0,_C1,_C01,_C11,_C20,_C21,_C30;
static int32_t _C00,_C10;
static float _kT,_kP;


Baro_Parameter Baro;

uint8_t SPL06_Init(void)
{
    uint8_t t;
	uint8_t coef[18];
	uint8_t ID;
	
//    GPIO_InitTypeDef GPIO_InitStruct;
//	
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
//    
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(GPIOA,&GPIO_InitStruct);
    
	if(SPL06_Write_Reg(SP06_RESET,0x89))
	{
		return 1;
	}
	
	
	ID = SPL06_Read_Reg(SP06_ID);
	
	if(ID != 0x10)
	{
        return 1;
	}
	
	
	HAL_Delay(200);		//复位后系数准备好需要至少40ms
	SPL06_Read_Buffer(SP06_COEF,coef,18);
	
    _C0 	= ((int16_t)coef[0]<<4 ) + ((coef[1]&0xF0)>>4);
	_C0 = (_C0&0x0800)?(0xF000|_C0):_C0;
	_C1 	= ((int16_t)(coef[1]&0x0F)<<8 ) + coef[2];
	_C1 = (_C1&0x0800)?(0xF000|_C1):_C1;
	

	_C00 	= ((int32_t)coef[3]<<12 ) + ((uint32_t)coef[4]<<4 ) + (coef[5]>>4);
	_C10   	= ((int32_t)(coef[5]&0x0F)<<16 ) + ((uint32_t)coef[6]<<8 ) + coef[7];
	_C00 = (_C00&0x080000)?(0xFFF00000|_C00):_C00;
	_C10 = (_C10&0x080000)?(0xFFF00000|_C10):_C10;
	
	
	_C01   	= ((int16_t)coef[8]<<8 ) + coef[9];
	
	_C11   	= ((int16_t)coef[10]<<8 ) + coef[11];
	_C11 = (_C11&0x0800)?(0xF000|_C11):_C11;
	_C20   	= ((int16_t)coef[12]<<8 ) + coef[13];
	_C20 = (_C20&0x0800)?(0xF000|_C20):_C20;
	_C21   	= ((int16_t)coef[14]<<8 ) + coef[15];
	_C21 = (_C21&0x0800)?(0xF000|_C21):_C21;
	_C30   	= ((int16_t)coef[16]<<8 ) + coef[17];
	_C30 = (_C30&0x0800)?(0xF000|_C30):_C30;

	
	SPL06_Config_Pressure(PM_RATE_128,PM_PRC_16);
	SPL06_Config_Temperature(PM_RATE_8,TMP_PRC_8);
	
	SPL06_Start(MEAS_CTRL_ContinuousPressTemp);	//启动连续的气压温度测量
	HAL_Delay(20);

    //获取地面数据温度和气压
    t = 0;
    while(t < 10)
    {
        SPL06_Update();
        Baro.ground_press = Baro.press;
        Baro.ground_temp = Baro.temp;
        HAL_Delay(20);
        t ++;
    }
    
    
    for(t=0;t<20;t++)
    {
        
        SPL06_Update();
        Baro.ground_press = 0.8f*Baro.ground_press + 0.2f*Baro.press;
        Baro.ground_temp = 0.8f*Baro.ground_temp + 0.2f*Baro.temp;
        HAL_Delay(20);	
        
    }
    
	return 0;
}

void SPL06_Get_Ground_Parameter(void)
{
        SPL06_Update();
        Baro.ground_press = Baro.press;
        Baro.ground_temp = Baro.temp;
}

uint8_t SPL06_Write_Reg(uint8_t reg,uint8_t val)
{
	CS_SPL_L;
	CS_GYRO;
	CS_ACCEL;
//	HAL_SPI_Transmit(&hspi1, &reg, 1, 50);
//	HAL_SPI_Transmit(&hspi1, &val, 1, 50);
	uint8_t regg[1] = {reg};
	uint8_t vall[1] = {val};
	regg[0] &= 0x7f;

	HAL_SPI_Transmit(&hspi1, regg, 1, 50);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
	HAL_SPI_Transmit(&hspi1, vall, 1, 50);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);

	CS_SPL_H;
	return 0;
}
uint8_t SPL06_Read_Reg(uint8_t reg)
{
	uint8_t res[1];
//	uint8_t res1;
	uint8_t regg[1] = {reg};
	CS_SPL_L;
	CS_GYRO;
	CS_ACCEL;

	regg[0] |= 0x80;
	HAL_SPI_Transmit(&hspi1, regg, 1, 50);
	HAL_SPI_Receive(&hspi1, res, 1, 50);
	CS_SPL_H;
//	res1 = &res;
	return res[0];
}

uint8_t SPL06_Read_Buffer(uint8_t reg,uint8_t* buffer,uint8_t len)
{
	CS_SPL_L;CS_GYRO;
	CS_ACCEL;
	uint8_t regg[1] = {reg};
//	SPI1_Read_Reg_Buffer(reg,buffer,len);
//	uint8_t *p = buffer;
	
	regg[0] |= 0x80;
	HAL_SPI_Transmit(&hspi1, regg, 1, 50);
	HAL_SPI_Receive(&hspi1, buffer, len, 50);
	
//	for(int i=0;i<len;i++)
//	{
//		HAL_SPI_Receive(&hspi1, res, 1, 50);
//		*p++= 
//	}
	
	CS_SPL_H;
	return 0;
}







void SPL06_Config_Temperature(uint8_t rate,uint8_t oversampling)
{
	switch(oversampling)
	{
		case TMP_PRC_1:
			_kT = 524288;
			break;
		case TMP_PRC_2:
			_kT = 1572864;
			break;
		case TMP_PRC_4:
			_kT = 3670016;
			break;
		case TMP_PRC_8:
			_kT = 7864320;
			break;
		case TMP_PRC_16:
			_kT = 253952;
			break;
		case TMP_PRC_32:
			_kT = 516096;
			break;
		case TMP_PRC_64:
			_kT = 1040384;
			break;
		case TMP_PRC_128:
			_kT = 2088960;
			break;
	}
	_kT = 1.0f/_kT;
	
	SPL06_Write_Reg(SP06_TMP_CFG,rate|oversampling|0x80);	//温度每秒128次测量一次
	if(oversampling > TMP_PRC_8)
	{
		uint8_t temp = SPL06_Read_Reg(SP06_CFG_REG);
		SPL06_Write_Reg(SP06_CFG_REG,temp|SPL06_CFG_T_SHIFT);
	}
	
}

void SPL06_Config_Pressure(uint8_t rate,uint8_t oversampling)
{
	switch(oversampling)
	{
		case PM_PRC_1:
			_kP = 524288;
			break;
		case PM_PRC_2:
			_kP = 1572864;
			break;
		case PM_PRC_4:
			_kP = 3670016;
			break;
		case PM_PRC_8:
			_kP = 7864320;
			break;
		case PM_PRC_16:
			_kP = 253952;
			break;
		case PM_PRC_32:
			_kP = 516096;
			break;
		case PM_PRC_64:
			_kP = 1040384;
			break;
		case PM_PRC_128:
			_kP = 2088960;
			break;
	}
	_kP = 1.0f/_kP;
	SPL06_Write_Reg(SP06_PSR_CFG,rate|oversampling);
	if(oversampling > PM_PRC_8)
	{
		uint8_t temp = SPL06_Read_Reg(SP06_CFG_REG);
		SPL06_Write_Reg(SP06_CFG_REG,temp|SPL06_CFG_P_SHIFT);
	}
	
}




void SPL06_Start(uint8_t mode)
{
	SPL06_Write_Reg(SP06_MEAS_CFG, mode);
}


int32_t SPL06_Get_Pressure_Adc(void)
{
	uint8_t buf[3];
	int32_t adc;
	SPL06_Read_Buffer(SP06_PSR_B2,buf,3);
	adc = (int32_t)(buf[0]<<16) + (buf[1]<<8) + buf[2];
	adc = (adc&0x800000)?(0xFF000000|adc):adc;
	return adc;
}

int32_t SPL06_Get_Temperature_Adc(void)
{
	uint8_t buf[3];
	int32_t adc;
	SPL06_Read_Buffer(SP06_TMP_B2,buf,3);
	adc = (int32_t)(buf[0]<<16) + (buf[1]<<8) + buf[2];
	adc = (adc&0x800000)?(0xFF000000|adc):adc;
	return adc;
}





void SPL06_Update_Pressure(void)
{
	float Traw_src, Praw_src;
	float qua2, qua3;

	Traw_src = _kT * Baro.Original_temp;
	Praw_src = _kP * Baro.Original_press;

	//计算温度
	Baro.temp = 0.5f*_C0 + Traw_src * _C1;

	//计算气压
	qua2 = _C10 + Praw_src * (_C20 + Praw_src* _C30);
	qua3 = Traw_src * Praw_src * (_C11 + Praw_src * _C21);
	
    Baro.press = _C00 + Praw_src * qua2 + Traw_src * _C01 + qua3;
}


void SPL06_Update(void)
{
	Baro.Original_temp = SPL06_Get_Temperature_Adc();
	Baro.Original_press = SPL06_Get_Pressure_Adc();
	SPL06_Update_Pressure();
}


float SPL06_Get_Altitude(void)
{
//    float alt_3,height;
//    SPL06_Update(); 
//    alt_3 = ( 101000 - Baro.press ) / 1000.0f;
//    height = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * ( 101000 - Baro.press ) * 100.0f ;
//    
//    if(Baro.check==0)
//    {
//        Baro.check=1;
//        Baro.offset_Alt=height;
//    }
//    
//    Baro.baro_alt=height-Baro.offset_Alt;
//    return Baro.baro_alt;
//    
    
	float ret;
	float scaling = Baro.press / Baro.ground_press;
	float temp = Baro.ground_temp + 273.15f;
    SPL06_Update();
	// This is an exact calculation that is within +-2.5m of the standard atmosphere tables
	// in the troposphere (up to 11,000 m amsl).
	ret = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)));
	return ret;

}

