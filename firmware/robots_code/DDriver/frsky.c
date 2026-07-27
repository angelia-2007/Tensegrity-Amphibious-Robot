#include "frsky.h"
uint16_t SBUS_Ch[16];  
uint16_t signal;
void Sbus_data_Handle(void)
{
	int index = 0;
	static int lost_count = 0;
	for(int i = 0;i<26;i++)
	{
		if(SBUS_buff[i] == 0X0F && SBUS_buff[i+24] == 0X00)
		{
			index = i;
			break;
		}	
	}
//	SBUS_Ch[0] = ((int16_t)SBUS_buff[ 0] >> 0 | ((int16_t)SBUS_buff[ 1] << 8 )) & 0x07FF;
//	SBUS_Ch[2] = ((int16_t)SBUS_buff[ 1] >> 3 | ((int16_t)SBUS_buff[ 2] << 5 )) & 0x07FF;
//	SBUS_Ch[1] = ((int16_t)SBUS_buff[ 2] >> 6 | ((int16_t)SBUS_buff[ 3] << 2 ) | (int16_t)SBUS_buff[ 4] << 10 ) & 0x07FF;
//	SBUS_Ch[3] = ((int16_t)SBUS_buff[ 4] >> 1 | ((int16_t)SBUS_buff[ 5] << 7 )) & 0x07FF;
//	SBUS_Ch[4] = ((int16_t)SBUS_buff[ 5] >> 4 | ((int16_t)SBUS_buff[ 6] << 4 )) & 0x07FF;
//	SBUS_Ch[5] = ((int16_t)SBUS_buff[ 6] >> 7 | ((int16_t)SBUS_buff[ 7] << 1 ) | (int16_t)SBUS_buff[8] << 9 ) & 0x07FF;
//	SBUS_Ch[6] = ((int16_t)SBUS_buff[ 8] >> 2 | ((int16_t)SBUS_buff[9] << 6 )) & 0x07FF;
//	SBUS_Ch[7] = ((int16_t)SBUS_buff[9] >> 5 | ((int16_t)SBUS_buff[10] << 3 )) & 0x07FF;
//	SBUS_Ch[8] = ((int16_t)SBUS_buff[11] << 0 | ((int16_t)SBUS_buff[12] << 8 )) & 0x07FF;
//	SBUS_Ch[9] = ((int16_t)SBUS_buff[12] >> 3 | ((int16_t)SBUS_buff[13] << 5 )) & 0x07FF;
//	SBUS_Ch[10]= ((int16_t)SBUS_buff[13] >> 6 | ((int16_t)SBUS_buff[14] << 2 ) | (int16_t)SBUS_buff[15] << 10 ) & 0x07FF;
//	SBUS_Ch[11] = ((int16_t)SBUS_buff[15] >> 1 | ((int16_t)SBUS_buff[16] << 7 )) & 0x07FF;
//	SBUS_Ch[12] = ((int16_t)SBUS_buff[16] >> 4 | ((int16_t)SBUS_buff[17] << 4 )) & 0x07FF;
//	SBUS_Ch[13] = ((int16_t)SBUS_buff[17] >> 7 | ((int16_t)SBUS_buff[18] << 1 ) | (int16_t)SBUS_buff[19] << 9 ) & 0x07FF;
//	SBUS_Ch[14] = ((int16_t)SBUS_buff[19] >> 2 | ((int16_t)SBUS_buff[20] << 6 )) & 0x07FF;
//	SBUS_Ch[15] = ((int16_t)SBUS_buff[20] >> 5 | ((int16_t)SBUS_buff[21] << 3 )) & 0x07FF;
//	signal=SBUS_Ch[15];
	
	SBUS_Ch[0] = ((int16_t)SBUS_buff[ index + 1] >> 0 | ((int16_t)SBUS_buff[index +  2] << 8 )) & 0x07FF;
	SBUS_Ch[2] = ((int16_t)SBUS_buff[ index + 2] >> 3 | ((int16_t)SBUS_buff[index +  3] << 5 )) & 0x07FF;
	SBUS_Ch[1] = ((int16_t)SBUS_buff[ index + 3] >> 6 | ((int16_t)SBUS_buff[index +  4] << 2 ) | (int16_t)SBUS_buff[index +  5] << 10 ) & 0x07FF;
	SBUS_Ch[3] = ((int16_t)SBUS_buff[ index + 5] >> 1 | ((int16_t)SBUS_buff[index +  6] << 7 )) & 0x07FF;
	SBUS_Ch[4] = ((int16_t)SBUS_buff[ index + 6] >> 4 | ((int16_t)SBUS_buff[index +  7] << 4 )) & 0x07FF;
	SBUS_Ch[5] = ((int16_t)SBUS_buff[ index + 7] >> 7 | ((int16_t)SBUS_buff[index +  8] << 1 ) | (int16_t)SBUS_buff[index + 9] << 9 ) & 0x07FF;
	SBUS_Ch[6] = ((int16_t)SBUS_buff[ index + 9] >> 2 | ((int16_t)SBUS_buff[index + 10] << 6 )) & 0x07FF;
	SBUS_Ch[7] = ((int16_t)SBUS_buff[index + 10] >> 5 | ((int16_t)SBUS_buff[index + 11] << 3 )) & 0x07FF;
	SBUS_Ch[8] = ((int16_t)SBUS_buff[index + 12] << 0 | ((int16_t)SBUS_buff[index + 13] << 8 )) & 0x07FF;
	SBUS_Ch[9] = ((int16_t)SBUS_buff[index + 13] >> 3 | ((int16_t)SBUS_buff[index + 14] << 5 )) & 0x07FF;
	SBUS_Ch[10]= ((int16_t)SBUS_buff[index + 14] >> 6 | ((int16_t)SBUS_buff[index + 15] << 2 ) | (int16_t)SBUS_buff[index + 16] << 10 ) & 0x07FF;
	SBUS_Ch[11] = ((int16_t)SBUS_buff[index + 16] >> 1 | ((int16_t)SBUS_buff[index + 17] << 7 )) & 0x07FF;
	SBUS_Ch[12] = ((int16_t)SBUS_buff[index + 17] >> 4 | ((int16_t)SBUS_buff[index + 18] << 4 )) & 0x07FF;
	SBUS_Ch[13] = ((int16_t)SBUS_buff[index + 18] >> 7 | ((int16_t)SBUS_buff[index + 19] << 1 ) | (int16_t)SBUS_buff[index + 20] << 9 ) & 0x07FF;
	SBUS_Ch[14] = ((int16_t)SBUS_buff[index + 20] >> 2 | ((int16_t)SBUS_buff[index + 21] << 6 )) & 0x07FF;
	SBUS_Ch[15] = ((int16_t)SBUS_buff[index + 21] >> 5 | ((int16_t)SBUS_buff[index + 22] << 3 )) & 0x07FF;
	signal=SBUS_Ch[index + 15];
	if(SBUS_buff[index + 23] ==0X0C )
	{
		if(lost_count++ > 30) lost_count = 30;
	}else
	{
		lost_count = 0;
	}
	if(lost_count >25)		RC.SbusIsLink = 0;
	else 		RC.SbusIsLink = 1;

}
void Sbus_Data_Count(uint8_t *buf)
{
	
	SBUS_Ch[ 0] = ((int16_t)buf[ 2] >> 0 | ((int16_t)buf[ 3] << 8 )) & 0x07FF;
	SBUS_Ch[ 1] = ((int16_t)buf[ 3] >> 3 | ((int16_t)buf[ 4] << 5 )) & 0x07FF;
	SBUS_Ch[ 2] = ((int16_t)buf[ 4] >> 6 | ((int16_t)buf[ 5] << 2 )  | (int16_t)buf[ 6] << 10 ) & 0x07FF;
	SBUS_Ch[ 3] = ((int16_t)buf[ 6] >> 1 | ((int16_t)buf[ 7] << 7 )) & 0x07FF;
	SBUS_Ch[ 4] = ((int16_t)buf[ 7] >> 4 | ((int16_t)buf[ 8] << 4 )) & 0x07FF;
	SBUS_Ch[ 5] = ((int16_t)buf[ 8] >> 7 | ((int16_t)buf[ 9] << 1 )  | (int16_t)buf[10] <<  9 ) & 0x07FF;
	SBUS_Ch[ 6] = ((int16_t)buf[10] >> 2 | ((int16_t)buf[11] << 6 )) & 0x07FF;
	SBUS_Ch[ 7] = ((int16_t)buf[11] >> 5 | ((int16_t)buf[12] << 3 )) & 0x07FF;
	
	SBUS_Ch[ 8] = ((int16_t)buf[13] << 0 | ((int16_t)buf[14] << 8 )) & 0x07FF;
	SBUS_Ch[ 9] = ((int16_t)buf[14] >> 3 | ((int16_t)buf[15] << 5 )) & 0x07FF;
	SBUS_Ch[10] = ((int16_t)buf[15] >> 6 | ((int16_t)buf[16] << 2 )  | (int16_t)buf[17] << 10 ) & 0x07FF;
	SBUS_Ch[11] = ((int16_t)buf[17] >> 1 | ((int16_t)buf[18] << 7 )) & 0x07FF;
	SBUS_Ch[12] = ((int16_t)buf[18] >> 4 | ((int16_t)buf[19] << 4 )) & 0x07FF;
	SBUS_Ch[13] = ((int16_t)buf[19] >> 7 | ((int16_t)buf[20] << 1 )  | (int16_t)buf[21] <<  9 ) & 0x07FF;
	SBUS_Ch[14] = ((int16_t)buf[21] >> 2 | ((int16_t)buf[22] << 6 )) & 0x07FF;
	SBUS_Ch[15] = ((int16_t)buf[22] >> 5 | ((int16_t)buf[23] << 3 )) & 0x07FF;

}
void Rssi_decode(uint8_t *buf)
{
	RC.link_quality=buf[5];
	
	if(RC.link_quality<10)
	{
	RC.SbusIsLink = 0;
	}else{
		RC.SbusIsLink = 1;
	}
}
void RC_Data_Handle(void)
{
	RC.SbusIsLink = 1;
	if(SBUS_buff[2]==0x16)
	{
	Sbus_Data_Count(SBUS_buff+1);
	}else if(SBUS_buff[2]==0x14)
	{
		
		Rssi_decode(SBUS_buff);
	}
	

	
	
}
