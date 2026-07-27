#ifndef __IMU_BMI088
#define __IMU_BMI088
#include "main.h"
#include "bmi08x.h"
#include "bmi088.h"
#include "bmi08x_defs.h"



#define bmi088_Acc_Scale (1.0f)
extern bool JZ_BMI088;
extern float rMat_b2w[3][3];/*本体到世界旋转矩阵*/
extern float rMat_w2b[3][3];/*世界到本体旋转矩阵*/

typedef struct
{
    float q0;//q0;
    float q1;//q1;
    float q2;//q2;
    float q3;//q3;
} Quaterniond;

typedef struct
{
	  Quaterniond Q;

	float kp;
	float ki;

	float ax,ay,az;
	float gx,gy,gz;
	
	float Temp;
	float Acc[3];//单位m/s^2
	float Gyro[3];//单位rad/s

	float Acc_NoG_b[3];//消除重力加速度的本体加速度
	float Acc_NoG_w[3];//消除重力加速度的世界加速度

	float POS_Acc[3];//滤波后数据
	float POS_Acc_NoG_b[3];//消除重力加速度的本体加速度
	float POS_Acc_NoG_w[3];//消除重力加速度的世界加速度

	float RPY_Acc[3];
	float RPY_Gyro[3];
	float Flow_Gyro[3];

	float Acc_Scale;
	float offset_Gyro[3];
	float offset_Acc[3];
	bool gyro_offset_Check;
	bool acc_offset_Check;
	bool acc_scale_Check;

    float rol;
    float pit;
    float yaw;
} _imu_st ;
extern _imu_st imu_bmi088_data_00;
extern _imu_st imu_bmi088_data;
extern _imu_st vision_bmi088_data_First;
extern _imu_st vision_bmi088_data_Second;


#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* 计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE				0.01f	/* 陀螺仪零偏方差阈值 */
#define ACC_VARIANCE_BASE				0.001f	  /* 加速度计零偏方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES  		1000		/* 加速计采样个数 */
typedef struct
{
	bool       isBiasValueFound;
	bool       isBufferFilled;
	bool		jzSuccess;
	float      buffer[3][SENSORS_NBR_OF_BIAS_SAMPLES];
	uint8_t 	jzProcess;
}Offset_st;


extern Offset_st	gyroOffsetRunning_00;
//extern Offset_st	accOffsetRunning_00;

extern Offset_st	gyroOffsetRunning;
extern Offset_st	accOffsetRunning;

bool processGyroBias00(Offset_st* offsetbuf,float gx, float gy, float gz, float *OffsetOut);
bool processGyroBias(Offset_st* offsetbuf,float gx, float gy, float gz, float *OffsetOut);
bool processAccBias(float ax, float ay, float az, float *OffsetOut);
bool processAccScale(float ax, float ay, float az);


int8_t stm32_spi_write(GPIO_TypeDef* GPIOx,uint16_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t stm32_spi_read(GPIO_TypeDef* GPIOx, uint16_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t imu_bmi088_init(void);
int8_t imu_bmi088_init_00(void);
void process_bmi088_data_00(void);
void process_bmi088_data(void);
void IMU_Vision_Mahony(_imu_st *imu, Quaterniond *q_up, float dt);
void IMU_Flow_First_Mahony(_imu_st *imu, Quaterniond *q_up, float dt);
void IMU_Update_Mahony(_imu_st *imu, float dt);
void Vector_b2w(float vec_in[], float vec_out[]);
void Vector_w2b(float vec_in[], float vec_out[]);
void imuComputeRotationMatrix(Quaterniond *q);
Quaterniond quat_e(Quaterniond *qd, Quaterniond *q);


float applyDeadbandf(float value, float deadband);
bool ISQQQQ(Quaterniond* qq);
bool ISQQQQ1(float Q1,float Q2,float Q3,float Q4);

#endif

