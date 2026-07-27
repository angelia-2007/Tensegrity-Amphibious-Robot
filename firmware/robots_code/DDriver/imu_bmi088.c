#include "imu_bmi088.h"
#include "soft_iic.h"

bool USE_ACC = false;
bool JZ_BMI088 = false;
bool FIRST_TAG = 0;	//第一次赋值0，定位彻底丢失也会置零

#define GG 9.8f  //重力加速度
#define DEG2RAD		0.017453293f	/* 度转弧度 π/180 */
#define RAD2DEG		57.29578f		/* 弧度转度 180/π */

#define bmi088_offset_Gyro_x_00 (0.00297783874f)
#define bmi088_offset_Gyro_y_00 (0.00479990849f)
#define bmi088_offset_Gyro_z_00 (0.00228656898f)

//#define bmi088_offset_Gyro_x_00 (0.00670692252f)
//#define bmi088_offset_Gyro_y_00 (-0.0006691537f)
//#define bmi088_offset_Gyro_z_00 (0.00205414633f)

#define bmi088_offset_Gyro_x (0.00965602882f)
#define bmi088_offset_Gyro_y (0.00557700405f)
#define bmi088_offset_Gyro_z (0.00139555754f)

#define bmi088_offset_Acc_x (-0.0142251849f)
#define bmi088_offset_Acc_y (0.12636441f)
#define bmi088_offset_Acc_z (0.00530577f)

#define CS1_Pin GPIO_PIN_12
#define CS1_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_13
#define CS2_GPIO_Port GPIOB

float rMat_b2w[3][3];/*本体到世界旋转矩阵*/
float rMat_w2b[3][3];/*世界到本体旋转矩阵*/

//Offset_st	gyroOffsetRunning_00;
//Offset_st	accOffsetRunning_00;

Offset_st	gyroOffsetRunning;
Offset_st	gyroOffsetRunning_00;

Offset_st	accOffsetRunning;

int8_t rslt;
int8_t rslt_00;
uint8_t data = 0;
int32_t bmi08x_sensor_temp;
struct bmi08x_sensor_data user_accel_bmi088;
struct bmi08x_sensor_data user_gyro_bmi088;

struct bmi08x_sensor_data user_accel_bmi088_00;
struct bmi08x_sensor_data user_gyro_bmi088_00;

float accRange;
float gyroRange;
//不与上层融合数据，仅地面调试使用
_imu_st imu_bmi088_data_00 =  {.Q = {1,0,0,0},//初始四元数
								.kp=0.5,.ki=0.001,//融合系数（与上层视觉或者重力）
								.gyro_offset_Check=false,.acc_offset_Check=false,.acc_scale_Check=false,//是否自己校准
								.offset_Gyro[0] = bmi088_offset_Gyro_x_00,
								.offset_Gyro[1] = bmi088_offset_Gyro_y_00, 
								.offset_Gyro[2] = bmi088_offset_Gyro_z_00,	
								.Acc_Scale = bmi088_Acc_Scale,														
                           };

_imu_st imu_bmi088_data =  {.Q = {1,0,0,0},//初始四元数
								.kp=0.5,.ki=0.001,//融合系数（与上层视觉或者重力）
								.gyro_offset_Check=false,.acc_offset_Check=false,.acc_scale_Check=false,//是否自己校准
								.offset_Gyro[0] = bmi088_offset_Gyro_x,
								.offset_Gyro[1] = bmi088_offset_Gyro_y, 
								.offset_Gyro[2] = bmi088_offset_Gyro_z,	
								.offset_Acc[0] = bmi088_offset_Acc_x,
								.offset_Acc[1] = bmi088_offset_Acc_y,
								.offset_Acc[2] = bmi088_offset_Acc_z,
								.Acc_Scale = bmi088_Acc_Scale,														
                           };
//与上层融合数据
_imu_st vision_bmi088_data_First =  {.Q = {1,0,0,0},//初始四元数
								.kp=0.5,.ki=0.001,//融合系数（与上层视觉或者重力）
								.gyro_offset_Check=false,.acc_offset_Check=false,.acc_scale_Check=false,//是否自己校准
								.Acc_Scale = bmi088_Acc_Scale,														
                           };


struct bmi08x_dev dev = {
        .accel_id = CS1_Pin,
        .gyro_id = CS2_Pin,
        .intf = BMI08X_SPI_INTF,  
        .read = &stm32_spi_read,//user_spi_read,  
        .write = &stm32_spi_write,//user_spi_write,  
        .delay_ms = &HAL_Delay//user_delay_milli_sec
};


bool ISQQQQ(Quaterniond* qq)
{
	float temp ;
	temp = qq->q0 * qq->q0 + qq->q1 * qq->q1 + qq->q2 * qq->q2 + qq->q3 * qq->q3;
	
	if(fabs(temp-1.0f)>0.01f)
	{
		return false;
	}else return true;
}

bool ISQQQQ1(float Q1,float Q2,float Q3,float Q4)
{
	float temp ;
	temp = Q1 * Q1 + Q2 * Q2 + Q3 * Q3 + Q4 * Q4;
	
	if(fabs(temp-1.0f)>0.01f)
	{
		return false;
	}else return true;
}


//dev->read
//dev->write
//初始化bmi088并配置参数
int8_t imu_bmi088_init(void)
{
	HAL_GPIO_WritePin(GPIOE, CS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, CS2_Pin, GPIO_PIN_SET);
	rslt = bmi088_init(&dev);//初始化
	if(rslt != BMI08X_OK)
	{
		return 1;//错误
	}
	/* Perform soft reset */
	rslt = bmi08a_soft_reset(&dev);
	if (rslt != BMI08X_OK)
	{
		return 1;//错误
	}
	/* Read the accel power mode */
	rslt = bmi08a_get_power_mode(&dev);
	/* Read the accel sensor config parameters (odr,bw,range) */
	rslt = bmi08a_get_meas_conf(&dev);
	/* Initialize the device instance as per the initialization example */

  /* Assign the desired configurations */
	dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
	dev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;//最大读取频率
	dev.accel_cfg.range = BMI088_ACCEL_RANGE_6G;//量程
	dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
	
	rslt = bmi08a_set_power_mode(&dev);
	
	/* Wait for 10ms to switch between the power modes - delay taken care inside the function */
	rslt = bmi08a_set_meas_conf(&dev);
		
  /* Configuring the gyro	 */
	dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
	
	rslt = bmi08g_set_power_mode(&dev);
	/* Wait for 30ms to switch between the power modes - delay taken care inside the function */
	
	/* Assign the desired configurations */
	dev.gyro_cfg.odr = BMI08X_GYRO_BW_116_ODR_1000_HZ;
	dev.gyro_cfg.range = BMI08X_GYRO_RANGE_1000_DPS;//量程
	dev.gyro_cfg.bw = BMI08X_GYRO_BW_116_ODR_1000_HZ;
	
	rslt = bmi08g_set_meas_conf(&dev);
	
	switch(dev.accel_cfg.range)
	{
		case BMI088_ACCEL_RANGE_3G: accRange = 3.0f; break;
		case BMI088_ACCEL_RANGE_6G: accRange = 6.0f; break;
    case BMI088_ACCEL_RANGE_12G: accRange = 12.0f; break;
    case BMI088_ACCEL_RANGE_24G: accRange = 24.0f; break;
    default: accRange = 6.0f;		
	}
	switch(dev.gyro_cfg.range)
	{
		case BMI08X_GYRO_RANGE_2000_DPS: gyroRange = 2000.0f; break;
		case BMI08X_GYRO_RANGE_1000_DPS: gyroRange = 1000.0f; break;
    case BMI08X_GYRO_RANGE_500_DPS: gyroRange = 500.0f; break;
    case BMI08X_GYRO_RANGE_250_DPS: gyroRange = 250.0f; break;
		case BMI08X_GYRO_RANGE_125_DPS: gyroRange = 125.0f; break;
    default: gyroRange = 1000.0f;		
	}
	
	if (rslt != BMI08X_OK)
	{
		return 1;//错误
	}
	
	return 0;//完成初始化
}
//处理bmi088传感器数据并转换单位
void process_bmi088_data(void)
{
		float gx,gy,gz,ax,ay,az;
		static uint32_t last_jz_success_time = 0;
		uint32_t current_time;
		/* Read the sensor data into the sensor data instance */
		rslt = bmi08a_get_data(&user_accel_bmi088, &dev);
		/* Read the sensor data into the sensor data instance */
		rslt = bmi08g_get_data(&user_gyro_bmi088, &dev);
	
//		rslt = bmi08a_get_sensor_temperature(&dev,&bmi08x_sensor_temp);
//	  imu_bmi088_data.Temp = bmi08x_sensor_temp/1000.0f;

	  //单位转换
		ax = user_accel_bmi088.x *accRange/32768.0f;//单位g
		ay = user_accel_bmi088.y *accRange/32768.0f;
		az = user_accel_bmi088.z *accRange/32768.0f;//Range = 6g;
		gx = user_gyro_bmi088.x *gyroRange*DEG2RAD/32768.0f;//单位rad/s
		gy = user_gyro_bmi088.y *gyroRange*DEG2RAD/32768.0f;//Range = 1000°/s
		gz = user_gyro_bmi088.z *gyroRange*DEG2RAD/32768.0f;
	
	
		current_time = HAL_GetTick();
		if(JZ_BMI088 && current_time-last_jz_success_time>2000 )//需要校准陀螺仪零飘就设置为true,会触发一次校准
		{
			gyroOffsetRunning.jzSuccess = processGyroBias(&gyroOffsetRunning,gx,gy,gz,imu_bmi088_data.offset_Gyro);
			accOffsetRunning.jzSuccess = processAccBias(ax,ay,az,imu_bmi088_data.offset_Acc);
			//			imu_bmi088_data.gyro_offset_Check = gyroOffsetRunning.jzSuccess;		
			if(gyroOffsetRunning.jzSuccess )
			{
				last_jz_success_time = HAL_GetTick();	  	
			}
		}else
		{
			gyroOffsetRunning.isBiasValueFound = false;
			gyroOffsetRunning.isBufferFilled = false;
		}
	
  	//校正加单位转换
		imu_bmi088_data.Acc[0] = (ax - imu_bmi088_data.offset_Acc[0]) * GG / imu_bmi088_data.Acc_Scale;//单位m/s^2
		imu_bmi088_data.Acc[1] = (ay - imu_bmi088_data.offset_Acc[1]) * GG / imu_bmi088_data.Acc_Scale;
		imu_bmi088_data.Acc[2] = (az - imu_bmi088_data.offset_Acc[2]) * GG / imu_bmi088_data.Acc_Scale;//Range = 6g;
		imu_bmi088_data.Gyro[0] = gx - imu_bmi088_data.offset_Gyro[0];//单位rad/s
		imu_bmi088_data.Gyro[1] = gy - imu_bmi088_data.offset_Gyro[1];//Range = 1000°/s
		imu_bmi088_data.Gyro[2] = gz - imu_bmi088_data.offset_Gyro[2];
	
		for(uint8_t i = 0; i < 3; i++)
		{
			//姿态使用的滤波
			imu_bmi088_data.RPY_Acc[i] = Butter_run( &BMI_RPY_Acc_Filter[i] , imu_bmi088_data.Acc[i] );
			imu_bmi088_data.RPY_Gyro[i] = Butter_run( &BMI_RPY_Gyro_Filter[i] , imu_bmi088_data.Gyro[i] );
			//位置融合使用的滤波
			imu_bmi088_data.POS_Acc[i] = Butter_run( &BMI_POS_Acc_Filter1[i] , imu_bmi088_data.Acc[i] );
			imu_bmi088_data.POS_Acc_NoG_b[i] = Butter_run( &BMI_POS_Acc_Filter2[i] , imu_bmi088_data.Acc_NoG_b[i] );
			imu_bmi088_data.POS_Acc_NoG_w[i] = Butter_run( &BMI_POS_Acc_Filter3[i] , imu_bmi088_data.Acc_NoG_w[i] );
		}
		
//		imu_bmi088_data.Earth_Accel[0] = imu_bmi088_data.POS_Acc[2]*sinf(imu_bmi088_data.rol * DEG2RAD)+ imu_bmi088_data.POS_Acc[0]*cosf(imu_bmi088_data.rol * DEG2RAD);
//		imu_bmi088_data.Earth_Accel[1] = imu_bmi088_data.POS_Acc[2]*sinf(imu_bmi088_data.pit * DEG2RAD)- imu_bmi088_data.POS_Acc[1]*cosf(imu_bmi088_data.pit * DEG2RAD);
//		imu_bmi088_data.Earth_Accel[2] = imu_bmi088_data.POS_Accel_Z*fabs(imu_bmi088_data.CosX*imu_bmi088_data.CosY) + imu_bmi088_data.POS_Accel_Y*imu_bmi088_data.SinY- imu_bmi088_data.POS_Accel_X*imu_bmi088_data.SinX- 1.0f;      
}

void IMU_Vision_Mahony(_imu_st *imu, Quaterniond *q_up, float dt)
{
//    float normalise;
//    float ex, ey, ez;//
//    float q0s, q1s, q2s, q3s;/*  */
//    static float R11,R21;/* (1,1),(2,1) */
//    static float vecxZ, vecyZ, veczZ;/* z(0,0,1)' */
//	static float exInt, eyInt, ezInt;
//    float half_T = 0.5f * dt;

//	static bool follow = false;
//	if((ISQQQQ(&imu->Q) == false || (isnan(imu->Q.q0)==1)) \
//		&& ISQQQQ(q_up) == true  && (isnan(q_up->q0)==0))
//	{
//		imu->Q.q0 = q_up->q0;
//		imu->Q.q1 = q_up->q1;
//		imu->Q.q2 = q_up->q2;
//		imu->Q.q3 = q_up->q3;
//	}
//	
//	if(ISQQQQ(q_up) == true && ISQQQQ(&imu->Q) == true && (isnan(q_up->q0)==0) && (isnan(imu->Q.q0) == 0))
//	{
//		follow = true;
//	}else
//	{follow = false;}
//	

//		float q0Last = imu->Q.q0;
//		float q1Last = imu->Q.q1;
//		float q2Last = imu->Q.q2;
//		float q3Last = imu->Q.q3;
//		float delta_theta[3];/* xyz */

//		if(follow&&!GET_SYSTEM_FLAG(COMMUNICATION_TO_NX_LOST_FAST) && comm_data.receive_fast_bag.ORBSLAM_Flag != 0)		//上层数据正常
//		{		
////		if(GET_SYSTEM_FLAG(APRILTAG_STATE))
////		{
////			imu->kp = 0.1f*2.0f;
////		}else
////		{
////			imu->kp = 0.1f;
////		}
//		if(comm_data.receive_fast_bag.ORBSLAM_Flag == 1)
//		{
//			imu->kp = 1.0f;
//			imu->ki = 0.0005f;
//		}else if(comm_data.receive_fast_bag.ORBSLAM_Flag == 2)
//		{
//			imu->kp = 0.1f ;
//			imu->ki = 0.0005f;
//		}else if(comm_data.receive_fast_bag.ORBSLAM_Flag == 3)
//		{
//			imu->kp = 0.5f ;
//			imu->ki = 0.001f;			
//		}else if(comm_data.receive_fast_bag.ORBSLAM_Flag == 4)
//		{
//			imu->kp = 0.7f;
//			imu->ki = 0.001f;
//		}

//		Quaterniond q_e;
//		q_e = quat_e(q_up,&imu->Q);//计算四元数误差

//		if(q_e.q0 <= 0){
//			ex = q_e.q1;
//			ey = q_e.q2;
//			ez = q_e.q3;
//		}else{
//			ex = -q_e.q1;
//			ey = -q_e.q2;
//			ez = -q_e.q3;
//		}

//        /* , */
//        exInt += imu->ki * ex * dt ;
//        eyInt += imu->ki * ey * dt ;
//        ezInt += imu->ki * ez * dt ;

//        /* PI, */
//        imu->RPY_Gyro[0] += imu->kp * ex + exInt;
//        imu->RPY_Gyro[1] += imu->kp * ey + eyInt;
//        imu->RPY_Gyro[2] += imu->kp * ez + ezInt;

//		delta_theta[0] = imu->RPY_Gyro[0]*half_T;
//		delta_theta[1] = imu->RPY_Gyro[1]*half_T;
//		delta_theta[2] = imu->RPY_Gyro[2]*half_T;
////		delta_theta_s = delta_theta[0]*delta_theta[0] + delta_theta[1]*delta_theta[1] + delta_theta[2]*delta_theta[2];
//    }else if(!follow || GET_SYSTEM_FLAG(COMMUNICATION_TO_NX_LOST_FAST) || comm_data.receive_fast_bag.ORBSLAM_Flag == 0)
//	{
//		delta_theta[0] = imu->Gyro[0]*half_T;
//		delta_theta[1] = imu->Gyro[1]*half_T;
//		delta_theta[2] = imu->Gyro[2]*half_T;
////		delta_theta_s = delta_theta[0]*delta_theta[0] + delta_theta[1]*delta_theta[1] + delta_theta[2]*delta_theta[2];
//	}

//    /* TkTk+1, */

//    /* , */
//    /* Q(Tk+1)=(I+0.5*delta_theta)Q(Tk) */
// 	imu->Q.q0 += -q1Last * delta_theta[0] - q2Last * delta_theta[1] - q3Last * delta_theta[2];
// 	imu->Q.q1 +=  q0Last * delta_theta[0] + q2Last * delta_theta[2] - q3Last * delta_theta[1];
// 	imu->Q.q2 +=  q0Last * delta_theta[1] - q1Last * delta_theta[2] + q3Last * delta_theta[0];
// 	imu->Q.q3 +=  q0Last * delta_theta[2] + q1Last * delta_theta[1] - q2Last * delta_theta[0];

//    /*  */
//    /* Q(Tk+1)=((1-0.125*delta_theta_s)I+0.5*delta_theta)Q(Tk) */
////    imu->Q.q0 = q0Last*(1-delta_theta_s) - q1Last * delta_theta[0] - q2Last * delta_theta[1] - q3Last * delta_theta[2];
////    imu->Q.q1 = q1Last*(1-delta_theta_s) + q0Last * delta_theta[0] + q2Last * delta_theta[2] - q3Last * delta_theta[1];
////    imu->Q.q2 = q2Last*(1-delta_theta_s) + q0Last * delta_theta[1] - q1Last * delta_theta[2] + q3Last * delta_theta[0];
////    imu->Q.q3 = q3Last*(1-delta_theta_s) + q0Last * delta_theta[2] + q1Last * delta_theta[1] - q2Last * delta_theta[0];

//	float vall = ((imu->Q.q0 * imu->Q.q0 + imu->Q.q1 * imu->Q.q1 + imu->Q.q2 * imu->Q.q2 + imu->Q.q3 * imu->Q.q3));
//	if ( vall >= 0.0f )
//	{
//		normalise = sqrt(vall);
//	}
//	
//	if(normalise != 0)
//	{
//		imu->Q.q0 /= normalise;
//		imu->Q.q1 /= normalise;
//		imu->Q.q2 /= normalise;
//		imu->Q.q3 /= normalise;
//	}
//	
//    /* ***********************观察数据使用****************************************************** */
//    q0s = imu->Q.q0 * imu->Q.q0;
//    q1s = imu->Q.q1 * imu->Q.q1;
//    q2s = imu->Q.q2 * imu->Q.q2;
//    q3s = imu->Q.q3 * imu->Q.q3;

//    R11 = q0s + q1s - q2s - q3s;/* (1,1) */
//    R21 = 2 * (imu->Q.q1 * imu->Q.q2 + imu->Q.q0 * imu->Q.q3);/* (2,1) */

//    /* z(0,0,1) */
//    vecxZ = 2 * (imu->Q.q1 * imu->Q.q3 - imu->Q.q0 * imu->Q.q2);/* (3,1) */
//    vecyZ = 2 * (imu->Q.q0 * imu->Q.q1 + imu->Q.q2 * imu->Q.q3);/* (3,2) */
//    veczZ = q0s - q1s - q2s + q3s;	/* (3,3) */

//    if (vecxZ>1) vecxZ=1;
//    if (vecxZ<-1) vecxZ=-1;

//    /* roll pitch yaw  */
//    imu->pit = -asinf(vecxZ) * 57.30f;
//    imu->rol = atan2f(vecyZ, veczZ) * 57.30f;
//    imu->yaw = atan2f(R21, R11) * 57.30f;
		/* ***********************观察数据使用****************************************************** */
}
const float fast_atan_table[257] = 
{
	0.000000e+00, 3.921549e-03, 7.842976e-03, 1.176416e-02,
	1.568499e-02, 1.960533e-02, 2.352507e-02, 2.744409e-02,
	3.136226e-02, 3.527947e-02, 3.919560e-02, 4.311053e-02,
	4.702413e-02, 5.093629e-02, 5.484690e-02, 5.875582e-02,
	6.266295e-02, 6.656816e-02, 7.047134e-02, 7.437238e-02,
	7.827114e-02, 8.216752e-02, 8.606141e-02, 8.995267e-02,
	9.384121e-02, 9.772691e-02, 1.016096e-01, 1.054893e-01,
	1.093658e-01, 1.132390e-01, 1.171087e-01, 1.209750e-01,
	1.248376e-01, 1.286965e-01, 1.325515e-01, 1.364026e-01,
	1.402496e-01, 1.440924e-01, 1.479310e-01, 1.517652e-01,
	1.555948e-01, 1.594199e-01, 1.632403e-01, 1.670559e-01,
	1.708665e-01, 1.746722e-01, 1.784728e-01, 1.822681e-01,
	1.860582e-01, 1.898428e-01, 1.936220e-01, 1.973956e-01,
	2.011634e-01, 2.049255e-01, 2.086818e-01, 2.124320e-01,
	2.161762e-01, 2.199143e-01, 2.236461e-01, 2.273716e-01,
	2.310907e-01, 2.348033e-01, 2.385093e-01, 2.422086e-01,
	2.459012e-01, 2.495869e-01, 2.532658e-01, 2.569376e-01,
	2.606024e-01, 2.642600e-01, 2.679104e-01, 2.715535e-01,
	2.751892e-01, 2.788175e-01, 2.824383e-01, 2.860514e-01,
	2.896569e-01, 2.932547e-01, 2.968447e-01, 3.004268e-01,
	3.040009e-01, 3.075671e-01, 3.111252e-01, 3.146752e-01,
	3.182170e-01, 3.217506e-01, 3.252758e-01, 3.287927e-01,
	3.323012e-01, 3.358012e-01, 3.392926e-01, 3.427755e-01,
	3.462497e-01, 3.497153e-01, 3.531721e-01, 3.566201e-01,
	3.600593e-01, 3.634896e-01, 3.669110e-01, 3.703234e-01,
	3.737268e-01, 3.771211e-01, 3.805064e-01, 3.838825e-01,
	3.872494e-01, 3.906070e-01, 3.939555e-01, 3.972946e-01,
	4.006244e-01, 4.039448e-01, 4.072558e-01, 4.105574e-01,
	4.138496e-01, 4.171322e-01, 4.204054e-01, 4.236689e-01,
	4.269229e-01, 4.301673e-01, 4.334021e-01, 4.366272e-01,
	4.398426e-01, 4.430483e-01, 4.462443e-01, 4.494306e-01,
	4.526070e-01, 4.557738e-01, 4.589307e-01, 4.620778e-01,
	4.652150e-01, 4.683424e-01, 4.714600e-01, 4.745676e-01,
	4.776654e-01, 4.807532e-01, 4.838312e-01, 4.868992e-01,
	4.899573e-01, 4.930055e-01, 4.960437e-01, 4.990719e-01,
	5.020902e-01, 5.050985e-01, 5.080968e-01, 5.110852e-01,
	5.140636e-01, 5.170320e-01, 5.199904e-01, 5.229388e-01,
	5.258772e-01, 5.288056e-01, 5.317241e-01, 5.346325e-01,
	5.375310e-01, 5.404195e-01, 5.432980e-01, 5.461666e-01,
	5.490251e-01, 5.518738e-01, 5.547124e-01, 5.575411e-01,
	5.603599e-01, 5.631687e-01, 5.659676e-01, 5.687566e-01,
	5.715357e-01, 5.743048e-01, 5.770641e-01, 5.798135e-01,
	5.825531e-01, 5.852828e-01, 5.880026e-01, 5.907126e-01,
	5.934128e-01, 5.961032e-01, 5.987839e-01, 6.014547e-01,
	6.041158e-01, 6.067672e-01, 6.094088e-01, 6.120407e-01,
	6.146630e-01, 6.172755e-01, 6.198784e-01, 6.224717e-01,
	6.250554e-01, 6.276294e-01, 6.301939e-01, 6.327488e-01,
	6.352942e-01, 6.378301e-01, 6.403565e-01, 6.428734e-01,
	6.453808e-01, 6.478788e-01, 6.503674e-01, 6.528466e-01,
	6.553165e-01, 6.577770e-01, 6.602282e-01, 6.626701e-01,
	6.651027e-01, 6.675261e-01, 6.699402e-01, 6.723452e-01,
	6.747409e-01, 6.771276e-01, 6.795051e-01, 6.818735e-01,
	6.842328e-01, 6.865831e-01, 6.889244e-01, 6.912567e-01,
	6.935800e-01, 6.958943e-01, 6.981998e-01, 7.004964e-01,
	7.027841e-01, 7.050630e-01, 7.073330e-01, 7.095943e-01,
	7.118469e-01, 7.140907e-01, 7.163258e-01, 7.185523e-01,
	7.207701e-01, 7.229794e-01, 7.251800e-01, 7.273721e-01,
	7.295557e-01, 7.317307e-01, 7.338974e-01, 7.360555e-01,
	7.382053e-01, 7.403467e-01, 7.424797e-01, 7.446045e-01,
	7.467209e-01, 7.488291e-01, 7.509291e-01, 7.530208e-01,
	7.551044e-01, 7.571798e-01, 7.592472e-01, 7.613064e-01,
	7.633576e-01, 7.654008e-01, 7.674360e-01, 7.694633e-01,
	7.714826e-01, 7.734940e-01, 7.754975e-01, 7.774932e-01,
	7.794811e-01, 7.814612e-01, 7.834335e-01, 7.853983e-01,
	7.853983e-01
};


float my_abs(float f)
{
	if (f >= 0.0f)
	{
		return f;
	}

	return -f;
}

float se_atan2(float y, float x) 
{
	float x_abs, y_abs, z;
	float alpha, angle, base_angle;
	int index;

	/* don't divide by zero! */
	if ((y == 0.0f) || (x == 0.0f))//if ((y == 0.0f) && (x == 0.0f))
		angle = 0.0f;
	else 
	{
		/* normalize to +/- 45 degree range */
		y_abs = my_abs(y);
		x_abs = my_abs(x);
		//z = (y_abs < x_abs ? y_abs / x_abs : x_abs / y_abs);
		if (y_abs < x_abs)
			z = y_abs / x_abs;
		else
			z = x_abs / y_abs;
		/* when ratio approaches the table resolution, the angle is */
		/*      best approximated with the argument itself...       */
		if (z < 0.003921569f)
			base_angle = z;
		else 
		{
			/* find index and interpolation value */
			alpha = z * (float) 256 - .5f;
			index = (int) alpha;
			alpha -= (float) index;
			/* determine base angle based on quadrant and */
			/* add or subtract table value from base angle based on quadrant */
			base_angle = fast_atan_table[index];
			base_angle += (fast_atan_table[index + 1] - fast_atan_table[index]) * alpha;
		}

		if (x_abs > y_abs) 
		{        /* -45 -> 45 or 135 -> 225 */
			if (x >= 0.0f) 
			{           /* -45 -> 45 */
				if (y >= 0.0f)
					angle = base_angle;   /* 0 -> 45, angle OK */
				else
					angle = -base_angle;  /* -45 -> 0, angle = -angle */
			} 
			else
			{                  /* 135 -> 180 or 180 -> -135 */
				angle = 3.14159265358979323846;

				if (y >= 0.0f)
					angle -= base_angle;  /* 135 -> 180, angle = 180 - angle */
				else
					angle = base_angle - angle;   /* 180 -> -135, angle = angle - 180 */
			}
		} 
		else 
		{                    /* 45 -> 135 or -135 -> -45 */
			if (y >= 0.0f) 
			{           /* 45 -> 135 */
				angle = 1.57079632679489661923;

				if (x >= 0.0f)
					angle -= base_angle;  /* 45 -> 90, angle = 90 - angle */
				else
					angle += base_angle;  /* 90 -> 135, angle = 90 + angle */
			} 
			else
			{                  /* -135 -> -45 */
				angle = -1.57079632679489661923;

				if (x >= 0.0f)
					angle += base_angle;  /* -90 -> -45, angle = -90 + angle */
				else
					angle -= base_angle;  /* -135 -> -90, angle = -90 - angle */
			}
		}
	}
	return (angle);
}

float se_sqrt(float dat)
{   
    long i;
	float x, y;
	x = dat * 0.5F;
	y = dat;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );
	y = * ( float * ) &i;
	y = y * ( 1.5f - ( x * y * y ) );
	y = y * ( 1.5f - ( x * y * y ) );
	
	return y*dat;
}


double mx(double rad)
{   
	double sine;
	if (rad < 0)
		sine = rad*(1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine*(-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f *( sine - 1) + 1);
	return sine;
}


//double se_sin(double rad)
//{
//	s8 flag_ = 1;

//	if (rad >= 3.14159265f)
//	{
//		rad -= 3.14159265f;
//		flag_ = -1;
//	}

//	return mx(rad) * flag_;
//}


//float se_cos(double rad)
//{
//	s8 _flag = 1;
//	rad += 3.14159265f/2.0;

//	if (rad >= 3.14159265f)
//	{
//		_flag = -1;
//		rad -= 3.14159265f;
//	}

//	return se_sin(rad)*_flag;
//}


//跟随First IMU
void IMU_Flow_First_Mahony(_imu_st *imu, Quaterniond *q_up, float dt)
{
    float normalise;
    float ex, ey, ez;//
	static float exInt, eyInt, ezInt;
    float half_T = 0.5f * dt;
	static bool follow = false;
    float q0s, q1s, q2s, q3s;/*  */
    static float R11,R21;/* (1,1),(2,1) */
    static float vecxZ, vecyZ, veczZ;/* z(0,0,1)' */
	if((ISQQQQ(&imu->Q) == false || (isnan(imu->Q.q0)==1))&& ISQQQQ(q_up) == true  && (isnan(q_up->q0)==0))
	{
		imu->Q.q0 = q_up->q0;
		imu->Q.q1 = q_up->q1;
		imu->Q.q2 = q_up->q2;
		imu->Q.q3 = q_up->q3;
	}
	
	
	if(ISQQQQ(q_up) == true && ISQQQQ(&imu->Q) == true && (isnan(q_up->q0)==0) && (isnan(imu->Q.q0) == 0))
	{
		follow = true;
	}else
	{follow = false;}
	
	if(follow)
	{
		
    float q0Last = imu->Q.q0;
    float q1Last = imu->Q.q1;
    float q2Last = imu->Q.q2;
    float q3Last = imu->Q.q3;
    float delta_theta[3];/* xyz */
//    float delta_theta_s;/* xyz */
	
		Quaterniond q_e;
		q_e = quat_e(q_up,&imu->Q);//计算四元数误差

		if(q_e.q0 <= 0){
			ex = q_e.q1;
			ey = q_e.q2;
			ez = q_e.q3;
		}else{
			ex = -q_e.q1;
			ey = -q_e.q2;
			ez = -q_e.q3;
		}

        /* , */
        exInt += imu->ki * ex * dt ;
        eyInt += imu->ki * ey * dt ;
        ezInt += imu->ki * ez * dt ;

        /* PI, */
        imu->RPY_Gyro[0] += imu->kp * ex + exInt;
        imu->RPY_Gyro[1] += imu->kp * ey + eyInt;
        imu->RPY_Gyro[2] += imu->kp * ez + ezInt;

		delta_theta[0] = imu->RPY_Gyro[0]*half_T;
		delta_theta[1] = imu->RPY_Gyro[1]*half_T;
		delta_theta[2] = imu->RPY_Gyro[2]*half_T;
//		delta_theta_s = delta_theta[0]*delta_theta[0] + delta_theta[1]*delta_theta[1] + delta_theta[2]*delta_theta[2];
    /* TkTk+1, */

    /* , */
    /* Q(Tk+1)=(I+0.5*delta_theta)Q(Tk) */
 	imu->Q.q0 += -q1Last * delta_theta[0] - q2Last * delta_theta[1] - q3Last * delta_theta[2];
 	imu->Q.q1 +=  q0Last * delta_theta[0] + q2Last * delta_theta[2] - q3Last * delta_theta[1];
 	imu->Q.q2 +=  q0Last * delta_theta[1] - q1Last * delta_theta[2] + q3Last * delta_theta[0];
 	imu->Q.q3 +=  q0Last * delta_theta[2] + q1Last * delta_theta[1] - q2Last * delta_theta[0];

//    /*  */
//    /* Q(Tk+1)=((1-0.125*delta_theta_s)I+0.5*delta_theta)Q(Tk) */
//    imu->Q.q0 = q0Last*(1-delta_theta_s) - q1Last * delta_theta[0] - q2Last * delta_theta[1] - q3Last * delta_theta[2];
//    imu->Q.q1 = q1Last*(1-delta_theta_s) + q0Last * delta_theta[0] + q2Last * delta_theta[2] - q3Last * delta_theta[1];
//    imu->Q.q2 = q2Last*(1-delta_theta_s) + q0Last * delta_theta[1] - q1Last * delta_theta[2] + q3Last * delta_theta[0];
//    imu->Q.q3 = q3Last*(1-delta_theta_s) + q0Last * delta_theta[2] + q1Last * delta_theta[1] - q2Last * delta_theta[0];

    /*  */
	arm_sqrt_f32((imu->Q.q0 * imu->Q.q0 + imu->Q.q1 * imu->Q.q1 + imu->Q.q2 * imu->Q.q2 + imu->Q.q3 * imu->Q.q3),&normalise);
//	normalise = ((imu->Q.q0 * imu->Q.q0 + imu->Q.q1 * imu->Q.q1 + imu->Q.q2 * imu->Q.q2 + imu->Q.q3 * imu->Q.q3));
//	if(normalise == 0) normalise = 0.1;
	if(isnan(normalise)) normalise = 1;
    imu->Q.q0 /= normalise;
    imu->Q.q1 /= normalise;
    imu->Q.q2 /= normalise;
    imu->Q.q3 /= normalise;
	
//	imuComputeRotationMatrix(&imu->Q);	  
	
	
//	
//	imu->rol=-se_atan2(2*imu->Q.q1*imu->Q.q3 - 2*imu->Q.q0*imu->Q.q2,se_sqrt(1-(2*imu->Q.q1*imu->Q.q3 - 2*imu->Q.q0*imu->Q.q2)*(2*imu->Q.q1*imu->Q.q3 - 2*imu->Q.q0*imu->Q.q2)))* 57.29578f; // 俯仰
//	imu->pit=se_atan2(2 * imu->Q.q2 * imu->Q.q3 + 2 * imu->Q.q0 * imu->Q.q1, -2 * imu->Q.q1 * imu->Q.q1 - 2 * imu->Q.q2 * imu->Q.q2 + 1)* 57.29578f; // 横滚
//	imu->yaw=se_atan2(2*imu->Q.q1*imu->Q.q2 + 2*imu->Q.q0*imu->Q.q3, -2*imu->Q.q2*imu->Q.q2-2*imu->Q.q3*imu->Q.q3 + 1)* 57.2957795f; // 横滚
	
	
	  /* ***********************观察数据使用****************************************************** */
    q0s = imu->Q.q0 * imu->Q.q0;
    q1s = imu->Q.q1 * imu->Q.q1;
    q2s = imu->Q.q2 * imu->Q.q2;
    q3s = imu->Q.q3 * imu->Q.q3;

    R11 = q0s + q1s - q2s - q3s;/* (1,1) */
    R21 = 2 * (imu->Q.q1 * imu->Q.q2 + imu->Q.q0 * imu->Q.q3);/* (2,1) */

    /* z(0,0,1) */
    vecxZ = 2 * (imu->Q.q1 * imu->Q.q3 - imu->Q.q0 * imu->Q.q2);/* (3,1) */
    vecyZ = 2 * (imu->Q.q0 * imu->Q.q1 + imu->Q.q2 * imu->Q.q3);/* (3,2) */
    veczZ = q0s - q1s - q2s + q3s;	/* (3,3) */

    if (vecxZ>1) vecxZ=1;
    if (vecxZ<-1) vecxZ=-1;

    /* roll pitch yaw  */
    imu->pit = -asinf(vecxZ) * 57.30f;
    imu->rol = atan2f(vecyZ, veczZ) * 57.30f;
    imu->yaw = atan2f(R21, R11) * 57.30f;
		/* ***********************观察数据使用****************************************************** */
	}
}

//陀螺仪自身融合姿态
void IMU_Update_Mahony(_imu_st *imu, float dt)
{
    float normalise;
    float nor_acc[3] = {0};
    float ex, ey, ez;//
    float q0s, q1s, q2s, q3s;/*  */
    static float R11,R21;/* (1,1),(2,1) */
    static float vecxZ, vecyZ, veczZ;/* z(0,0,1)' */
		static float exInt, eyInt, ezInt;
    float half_T = 0.5f * dt;

    float q0Last = imu->Q.q0;
    float q1Last = imu->Q.q1;
    float q2Last = imu->Q.q2;
    float q3Last = imu->Q.q3;
    float delta_theta[3];/* xyz */
    float delta_theta_s;/* xyz */

    /* 0 */
    if((imu->RPY_Acc[0] != 0.0f) || (imu->RPY_Acc[1] != 0.0f) || (imu->RPY_Acc[2] != 0.0f))
    {
        nor_acc[0] = imu->RPY_Acc[0];
        nor_acc[1] = imu->RPY_Acc[1];
        nor_acc[2] = imu->RPY_Acc[2];

        /*  */
		arm_sqrt_f32((nor_acc[0] * nor_acc[0] + nor_acc[1] * nor_acc[1] + nor_acc[2] * nor_acc[2]),&normalise);
        nor_acc[0] /= normalise;
        nor_acc[1] /= normalise;
        nor_acc[2] /= normalise;

        /* , */
        /* |a x b| = |a|*|b|*sin(theta);|a|=|b|=1,thetasin(theta)theta, */
        ex = (nor_acc[1] * veczZ - nor_acc[2] * vecyZ);
        ey = (nor_acc[2] * vecxZ - nor_acc[0] * veczZ);
        ez = (nor_acc[0] * vecyZ - nor_acc[1] * vecxZ);

        /* , */
        exInt += imu->ki * ex * dt ;
        eyInt += imu->ki * ey * dt ;
        ezInt += imu->ki * ez * dt ;

        /* PI, */
        imu->RPY_Gyro[0] += imu->kp * ex + exInt;
        imu->RPY_Gyro[1] += imu->kp * ey + eyInt;
        imu->RPY_Gyro[2] += imu->kp * ez + ezInt;
    }

    /* TkTk+1, */
    delta_theta[0] = imu->RPY_Gyro[0]*half_T;
    delta_theta[1] = imu->RPY_Gyro[1]*half_T;
    delta_theta[2] = imu->RPY_Gyro[2]*half_T;
    delta_theta_s = delta_theta[0]*delta_theta[0] + delta_theta[1]*delta_theta[1] + delta_theta[2]*delta_theta[2];
    /* , */
    /* Q(Tk+1)=(I+0.5*delta_theta)Q(Tk) */
// 	q0 += -q1Last * delta_theta[0] - q2Last * delta_theta[1] - q3Last * delta_theta[2];
// 	q1 +=  q0Last * delta_theta[0] + q2Last * delta_theta[2] - q3Last * delta_theta[1];
// 	q2 +=  q0Last * delta_theta[1] - q1Last * delta_theta[2] + q3Last * delta_theta[0];
// 	q3 +=  q0Last * delta_theta[2] + q1Last * delta_theta[1] - q2Last * delta_theta[0];

    /*  */
    /* Q(Tk+1)=((1-0.125*delta_theta_s)I+0.5*delta_theta)Q(Tk) */
    imu->Q.q0 = q0Last*(1-delta_theta_s) - q1Last * delta_theta[0] - q2Last * delta_theta[1] - q3Last * delta_theta[2];
    imu->Q.q1 = q1Last*(1-delta_theta_s) + q0Last * delta_theta[0] + q2Last * delta_theta[2] - q3Last * delta_theta[1];
    imu->Q.q2 = q2Last*(1-delta_theta_s) + q0Last * delta_theta[1] - q1Last * delta_theta[2] + q3Last * delta_theta[0];
    imu->Q.q3 = q3Last*(1-delta_theta_s) + q0Last * delta_theta[2] + q1Last * delta_theta[1] - q2Last * delta_theta[0];

    /*  */
		arm_sqrt_f32((imu->Q.q0 * imu->Q.q0 + imu->Q.q1 * imu->Q.q1 + imu->Q.q2 * imu->Q.q2 + imu->Q.q3 * imu->Q.q3),&normalise);
    imu->Q.q0 /= normalise;
    imu->Q.q1 /= normalise;
    imu->Q.q2 /= normalise;
    imu->Q.q3 /= normalise;
		
	/*计算旋转矩阵*/
	imuComputeRotationMatrix(&imu->Q);	
	/*本体加速度转换到世界坐标系*/
	Vector_b2w(imu->Acc, imu->Acc_NoG_w);
	/*消除重力加速度的影响*/
	imu->Acc_NoG_w[2] -= (GG);//不知道为什么他偶尔会有一帧抽动的数据，转换后再滤波为POS_Acc_NoG_w使用
	/*再转换回本体坐标系*/
	Vector_w2b(imu->Acc_NoG_w, imu->Acc_NoG_b);
		
    /* ***********************观察数据使用****************************************************** */
    q0s = imu->Q.q0 * imu->Q.q0;
    q1s = imu->Q.q1 * imu->Q.q1;
    q2s = imu->Q.q2 * imu->Q.q2;
    q3s = imu->Q.q3 * imu->Q.q3;

    R11 = q0s + q1s - q2s - q3s;/* (1,1) */
    R21 = 2 * (imu->Q.q1 * imu->Q.q2 + imu->Q.q0 * imu->Q.q3);/* (2,1) */

    /* z(0,0,1) */
    vecxZ = 2 * (imu->Q.q1 * imu->Q.q3 - imu->Q.q0 * imu->Q.q2);/* (3,1) */
    vecyZ = 2 * (imu->Q.q0 * imu->Q.q1 + imu->Q.q2 * imu->Q.q3);/* (3,2) */
    veczZ = q0s - q1s - q2s + q3s;	/* (3,3) */

    if (vecxZ>1) vecxZ=1;
    if (vecxZ<-1) vecxZ=-1;

    /* roll pitch yaw  */
    imu->pit = -asinf(vecxZ) * 57.30f;
    imu->rol = atan2f(vecyZ, veczZ) * 57.30f;
    imu->yaw = atan2f(R21, R11) * 57.30f;
		/* ***********************观察数据使用****************************************************** */
}



/*将矢量从本体转换到世界坐标系*/
void Vector_b2w(float vec_in[], float vec_out[])
{
	vec_out[0] = vec_in[0]* rMat_b2w[0][0] + vec_in[1] * rMat_b2w[0][1] + vec_in[2] * rMat_b2w[0][2];
  vec_out[1] = vec_in[0]* rMat_b2w[1][0] + vec_in[1] * rMat_b2w[1][1] + vec_in[2] * rMat_b2w[1][2];
	vec_out[2] = vec_in[0]* rMat_b2w[2][0] + vec_in[1] * rMat_b2w[2][1] + vec_in[2] * rMat_b2w[2][2];
}
/*将矢量从世界转换到本体坐标系*/
void Vector_w2b(float vec_in[], float vec_out[])
{
	vec_out[0] = vec_in[0]* rMat_w2b[0][0] + vec_in[1] * rMat_w2b[0][1] + vec_in[2] * rMat_w2b[0][2];
	vec_out[1] = vec_in[0]* rMat_w2b[1][0] + vec_in[1] * rMat_w2b[1][1] + vec_in[2] * rMat_w2b[1][2];
	vec_out[2] = vec_in[0]* rMat_w2b[2][0] + vec_in[1] * rMat_w2b[2][1] + vec_in[2] * rMat_w2b[2][2];
}
/*计算旋转矩阵*/
void imuComputeRotationMatrix(Quaterniond *q)
{
    float q1q1 = q->q1 * q->q1;
    float q2q2 = q->q2 * q->q2;
    float q3q3 = q->q3 * q->q3;

    float q0q1 = q->q0 * q->q1;
    float q0q2 = q->q0 * q->q2;
    float q0q3 = q->q0 * q->q3;
    float q1q2 = q->q1 * q->q2;
    float q1q3 = q->q1 * q->q3;
    float q2q3 = q->q2 * q->q3;
	  /*计算本体到世界旋转矩阵*/
    rMat_b2w[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat_b2w[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat_b2w[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat_b2w[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat_b2w[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat_b2w[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat_b2w[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat_b2w[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat_b2w[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
		
		/*计算世界到本体旋转矩阵*/
		/*1.四元数求逆*/
//    q1q1 = (-q->q1) * (-q->q1);
//    q2q2 = (-q->q2) * (-q->q2);
//    q3q3 = (-q->q3) * (-q->q3);

//    q0q1 = q->q0 * (-q->q1);
//    q0q2 = q->q0 * (-q->q2);
//    q0q3 = q->q0 * (-q->q3);
//    q1q2 = (-q->q1) * (-q->q2);
//    q1q3 = (-q->q1) * (-q->q3);
//    q2q3 = (-q->q2) * (-q->q3);
		/*2.算矩阵*/
		rMat_w2b[0][0] = rMat_b2w[0][0];
    rMat_w2b[0][1] = 2.0f * (q1q2 + q0q3);
    rMat_w2b[0][2] = 2.0f * (q1q3 - q0q2);

    rMat_w2b[1][0] = 2.0f * (q1q2 - q0q3);
    rMat_w2b[1][1] = rMat_b2w[1][1];
    rMat_w2b[1][2] = 2.0f * (q2q3 + q0q1);

    rMat_w2b[2][0] = 2.0f * (q1q3 + q0q2);
    rMat_w2b[2][1] = 2.0f * (q2q3 - q0q1);
    rMat_w2b[2][2] = rMat_b2w[2][2];
}


Quaterniond quat_e(Quaterniond *qd, Quaterniond *q)
{
  Quaterniond q_e;
	q_e.q0 = q->q0*qd->q0 + q->q1*qd->q1 + q->q2*qd->q2 + q->q3*qd->q3;
	q_e.q1 = q->q1*qd->q0 - q->q0*qd->q1 + q->q2*qd->q3 - q->q3*qd->q2;
	q_e.q2 = q->q2*qd->q0 - q->q0*qd->q2 - q->q1*qd->q3 + q->q3*qd->q1;
	q_e.q3 = q->q1*qd->q2 - q->q0*qd->q3 - q->q2*qd->q1 + q->q3*qd->q0;
	
	return q_e;
}
/**
 * 计算陀螺仪零飘
 */
bool processGyroBias00(Offset_st* offsetbuf,float gx, float gy, float gz, float *OffsetOut)
{
/**
 * 往循环缓冲区添加一个新值，缓冲区满后，替换旧的的值
 */
	
	static uint16_t i;
	
	offsetbuf->buffer[0][i] = gx;
	offsetbuf->buffer[1][i] = gy;
	offsetbuf->buffer[2][i] = gz;
	i++;
	offsetbuf->jzProcess = 1;
//	SState.JZ_Process_State = 1;		//正在校准
	
	if (i >= SENSORS_NBR_OF_BIAS_SAMPLES)
	{
		i = 0;
		offsetbuf->isBufferFilled = true;		//每次满1024后开始计算方差
	}
	
	if (offsetbuf->isBufferFilled)
	{
		static float mean[3] = {0};
		static float variance[3] = {0};
		arm_mean_f32(offsetbuf->buffer[0],SENSORS_NBR_OF_BIAS_SAMPLES,&mean[0]);
		arm_mean_f32(offsetbuf->buffer[1],SENSORS_NBR_OF_BIAS_SAMPLES,&mean[1]);
		arm_mean_f32(offsetbuf->buffer[2],SENSORS_NBR_OF_BIAS_SAMPLES,&mean[2]);
		arm_var_f32(offsetbuf->buffer[0],SENSORS_NBR_OF_BIAS_SAMPLES,&variance[0]);
		arm_var_f32(offsetbuf->buffer[1],SENSORS_NBR_OF_BIAS_SAMPLES,&variance[1]);
		arm_var_f32(offsetbuf->buffer[2],SENSORS_NBR_OF_BIAS_SAMPLES,&variance[2]);
		
		if (variance[0] < GYRO_VARIANCE_BASE && variance[1] < GYRO_VARIANCE_BASE && variance[2] < GYRO_VARIANCE_BASE)
		{
			OffsetOut[0] = mean[0];
			OffsetOut[1] = mean[1];
			OffsetOut[2] = mean[2];
			offsetbuf->isBiasValueFound= true;
			offsetbuf->jzProcess = 2;				//校准完成
		}else
		{
			offsetbuf->isBufferFilled=false;
			offsetbuf->jzProcess = 1;				//未校准
			
		}
	}

	return offsetbuf->isBiasValueFound;
}


bool processGyroBias(Offset_st* offsetbuf,float gx, float gy, float gz, float *OffsetOut)
{
/**
 * 往循环缓冲区添加一个新值，缓冲区满后，替换旧的的值
 */
	
	static uint16_t i;
	
	offsetbuf->buffer[0][i] = gx;
	offsetbuf->buffer[1][i] = gy;
	offsetbuf->buffer[2][i] = gz;
	i++;
	offsetbuf->jzProcess = 1;
//	SState.JZ_Process_State = 1;		//正在校准
	
	if (i >= SENSORS_NBR_OF_BIAS_SAMPLES)
	{
		i = 0;
		offsetbuf->isBufferFilled = true;		//每次满1024后开始计算方差
	}
	
	if (offsetbuf->isBufferFilled)
	{
		static float mean[3] = {0};
		static float variance[3] = {0};
		arm_mean_f32(offsetbuf->buffer[0],SENSORS_NBR_OF_BIAS_SAMPLES,&mean[0]);
		arm_mean_f32(offsetbuf->buffer[1],SENSORS_NBR_OF_BIAS_SAMPLES,&mean[1]);
		arm_mean_f32(offsetbuf->buffer[2],SENSORS_NBR_OF_BIAS_SAMPLES,&mean[2]);
		arm_var_f32(offsetbuf->buffer[0],SENSORS_NBR_OF_BIAS_SAMPLES,&variance[0]);
		arm_var_f32(offsetbuf->buffer[1],SENSORS_NBR_OF_BIAS_SAMPLES,&variance[1]);
		arm_var_f32(offsetbuf->buffer[2],SENSORS_NBR_OF_BIAS_SAMPLES,&variance[2]);
		
		if (variance[0] < GYRO_VARIANCE_BASE && variance[1] < GYRO_VARIANCE_BASE && variance[2] < GYRO_VARIANCE_BASE)
		{
			OffsetOut[0] = mean[0];
			OffsetOut[1] = mean[1];
			OffsetOut[2] = mean[2];
			offsetbuf->isBiasValueFound= true;
			offsetbuf->jzProcess = 2;				//校准完成
		}else
		{
			offsetbuf->isBufferFilled=false;
			offsetbuf->jzProcess = 1;				//未校准
		}
	}

	return offsetbuf->isBiasValueFound;
}


/**
 * 计算加速度计零飘
 */
bool processAccBias(float ax, float ay, float az, float *OffsetOut)
{
/**
 * 往循环缓冲区添加一个新值，缓冲区满后，替换旧的的值
 */
	
	static uint16_t i;
	accOffsetRunning.buffer[0][i] = ax;
	accOffsetRunning.buffer[1][i] = ay;
	accOffsetRunning.buffer[2][i] = az;
	i++;

	if (i >= SENSORS_NBR_OF_BIAS_SAMPLES)
	{
		i = 0;
		accOffsetRunning.isBufferFilled = true;
	}
	
	if (accOffsetRunning.isBufferFilled)
	{
		static float mean[3] = {0};
		static float variance[3] = {0};
		arm_mean_f32(accOffsetRunning.buffer[0],SENSORS_NBR_OF_BIAS_SAMPLES,&mean[0]);
		arm_mean_f32(accOffsetRunning.buffer[1],SENSORS_NBR_OF_BIAS_SAMPLES,&mean[1]);
		arm_mean_f32(accOffsetRunning.buffer[2],SENSORS_NBR_OF_BIAS_SAMPLES,&mean[2]);
		arm_var_f32(accOffsetRunning.buffer[0],SENSORS_NBR_OF_BIAS_SAMPLES,&variance[0]);
		arm_var_f32(accOffsetRunning.buffer[1],SENSORS_NBR_OF_BIAS_SAMPLES,&variance[1]);
		arm_var_f32(accOffsetRunning.buffer[2],SENSORS_NBR_OF_BIAS_SAMPLES,&variance[2]);
		
		if (variance[0] < ACC_VARIANCE_BASE && variance[0] < ACC_VARIANCE_BASE && variance[0] < ACC_VARIANCE_BASE)
		{
			OffsetOut[0] = mean[0];
			OffsetOut[1] = mean[1];
			OffsetOut[2] = mean[2];
		  accOffsetRunning.isBiasValueFound= true;

		}else
			accOffsetRunning.isBufferFilled=false;
		}

	return accOffsetRunning.isBiasValueFound;
}
/**
 * 根据样本计算重力加速度缩放因子
 */
bool processAccScale(float ax, float ay, float az)
{
	bool accBiasFound = false;
	static uint32_t accScaleSumCount = 0;
	static float accScaleSum = 0;
	static float accScale = 0;

	if (!accBiasFound)
	{
		arm_sqrt_f32((ax*ax+ay*ay+az*az),&accScale);
		accScaleSum += accScale;
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			imu_bmi088_data.Acc_Scale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accScaleSumCount = 0;
			accScaleSum = 0;
			accScale = 0;
			accBiasFound = true;
		}
	}

	return accBiasFound;
}
float applyDeadbandf(float value, float deadband)
{
    if (fabsf(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}



int8_t stm32_spi_read(GPIO_TypeDef* GPIOx, uint16_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	reg_addr |= 0x80;
	HAL_GPIO_WritePin(GPIOx, cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 50);
	HAL_SPI_Receive(&hspi1, data, len, 50);
	HAL_GPIO_WritePin(GPIOx, cs_pin, GPIO_PIN_SET);
	
//	reg_addr |= 0x80;
//	HAL_GPIO_WritePin(GPIOx, cs_pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit_DMA(&hspi2, &reg_addr, 1);
//	while((HAL_DMA_GetState(&hdma_spi2_tx) != HAL_DMA_STATE_READY));
//	
//	HAL_SPI_Receive_DMA(&hspi2, data, len);
//	while((HAL_DMA_GetState(&hdma_spi2_rx) != HAL_DMA_STATE_READY));
//	
//	HAL_GPIO_WritePin(GPIOx, cs_pin, GPIO_PIN_SET);
	return 0;
}


int8_t stm32_spi_write(GPIO_TypeDef* GPIOx,uint16_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	reg_addr &= 0x7f;
	HAL_GPIO_WritePin(GPIOx, cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 50);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
	HAL_SPI_Transmit(&hspi1, data, len, 50);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
	HAL_GPIO_WritePin(GPIOx, cs_pin, GPIO_PIN_SET);
	
//	reg_addr &= 0x7f;
//	HAL_GPIO_WritePin(GPIOx, cs_pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit_DMA(&hspi2, &reg_addr, 1);
//	while((HAL_DMA_GetState(&hdma_spi2_tx) != HAL_DMA_STATE_READY));

//	HAL_SPI_Transmit_DMA(&hspi2, data, len);
//	while((HAL_DMA_GetState(&hdma_spi2_tx) != HAL_DMA_STATE_READY));

//	HAL_GPIO_WritePin(GPIOx, cs_pin, GPIO_PIN_SET);
	return 0;
}
















//---------------------------------机体BMI088，用00表示-------------------------------

struct bmi08x_dev dev_00 = {
        .accel_id = BMI08X_ACCEL_I2C_ADDR_PRIMARY,
        .gyro_id = BMI08X_GYRO_I2C_ADDR_PRIMARY,
        .intf = BMI08X_I2C_INTF,  
        .read = &stm32_soft_i2c_read,//user_iic_read,  
        .write = &stm32_soft_i2c_write,//user_iic_write,  
        .delay_ms = &HAL_Delay//user_delay_milli_sec
};

//初始化bmi088并配置参数
int8_t imu_bmi088_init_00(void)
{
	rslt_00 = bmi088_init(&dev_00);//初始化
	if(rslt_00 != BMI08X_OK)
	{
		return 1;//错误
	}
	/* Perform soft reset */
	rslt_00 = bmi08a_soft_reset(&dev_00);
	if (rslt_00 != BMI08X_OK)
	{
		return 1;//错误
	}
	/* Read the accel power mode */
	rslt_00 = bmi08a_get_power_mode(&dev_00);
	/* Read the accel sensor config parameters (odr,bw,range) */
	rslt_00 = bmi08a_get_meas_conf(&dev_00);
	/* Initialize the device instance as per the initialization example */

  /* Assign the desired configurations */
	dev_00.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
	dev_00.accel_cfg.odr = BMI08X_ACCEL_ODR_800_HZ;//最大读取频率
	dev_00.accel_cfg.range = BMI088_ACCEL_RANGE_6G;//量程
	dev_00.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
	
	rslt_00 = bmi08a_set_power_mode(&dev_00);
	
	/* Wait for 10ms to switch between the power modes - delay taken care inside the function */
	rslt_00 = bmi08a_set_meas_conf(&dev_00);
		
  /* Configuring the gyro	 */
	dev_00.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
	
	rslt_00 = bmi08g_set_power_mode(&dev_00);
	/* Wait for 30ms to switch between the power modes - delay taken care inside the function */
	
	/* Assign the desired configurations */
	dev_00.gyro_cfg.odr = BMI08X_GYRO_BW_116_ODR_1000_HZ;
	dev_00.gyro_cfg.range = BMI08X_GYRO_RANGE_1000_DPS;//量程
	dev_00.gyro_cfg.bw = BMI08X_GYRO_BW_116_ODR_1000_HZ;
	
	rslt_00 = bmi08g_set_meas_conf(&dev_00);
	
	switch(dev_00.accel_cfg.range)
	{
		case BMI088_ACCEL_RANGE_3G: accRange = 3.0f; break;
		case BMI088_ACCEL_RANGE_6G: accRange = 6.0f; break;
		case BMI088_ACCEL_RANGE_12G: accRange = 12.0f; break;
		case BMI088_ACCEL_RANGE_24G: accRange = 24.0f; break;
		default: accRange = 6.0f;		
	}
	switch(dev_00.gyro_cfg.range)
	{
		case BMI08X_GYRO_RANGE_2000_DPS: gyroRange = 2000.0f; break;
		case BMI08X_GYRO_RANGE_1000_DPS: gyroRange = 1000.0f; break;
		case BMI08X_GYRO_RANGE_500_DPS: gyroRange = 500.0f; break;
		case BMI08X_GYRO_RANGE_250_DPS: gyroRange = 250.0f; break;
		case BMI08X_GYRO_RANGE_125_DPS: gyroRange = 125.0f; break;
		default: gyroRange = 1000.0f;		
	}
	
	if (rslt_00 != BMI08X_OK)
	{
		return 1;//错误
	}
	
	return 0;//完成初始化
}



//处理bmi088传感器数据并转换单位
void process_bmi088_data_00(void)
{
	  float gx,gy,gz,ax,ay,az;
	uint32_t current_time;
	static uint32_t last_jz_success_time = 0;
		/* Read the sensor data into the sensor data instance */
		rslt = bmi08a_get_data(&user_accel_bmi088_00, &dev_00);
		/* Read the sensor data into the sensor data instance */
		rslt = bmi08g_get_data(&user_gyro_bmi088_00, &dev_00);
	
//		rslt = bmi08a_get_sensor_temperature(&dev,&bmi08x_sensor_temp);
//	  imu_bmi088_data.Temp = bmi08x_sensor_temp/1000.0f;

	  //单位转换
		ax = user_accel_bmi088_00.x *accRange/32768.0f;//单位g
		ay = user_accel_bmi088_00.y *accRange/32768.0f;
		az = user_accel_bmi088_00.z *accRange/32768.0f;//Range = 6g;
		gx = user_gyro_bmi088_00.x *gyroRange*DEG2RAD/32768.0f;//单位rad/s
		gy = user_gyro_bmi088_00.y *gyroRange*DEG2RAD/32768.0f;//Range = 1000°/s
		gz = user_gyro_bmi088_00.z *gyroRange*DEG2RAD/32768.0f;
	
		current_time = HAL_GetTick();
		if(0&& current_time-last_jz_success_time>2000 )//需要校准陀螺仪零飘就设置为true,会触发一次校准
		{
			gyroOffsetRunning_00.jzSuccess = processGyroBias00(&gyroOffsetRunning_00,gx,gy,gz,imu_bmi088_data_00.offset_Gyro);
//			imu_bmi088_data_00.gyro_offset_Check = gyroOffsetRunning_00.jzSuccess;
			if(gyroOffsetRunning_00.jzSuccess )
			{
				last_jz_success_time = HAL_GetTick();	  	
			}
		}else
		{
			gyroOffsetRunning_00.isBiasValueFound = false;
			gyroOffsetRunning_00.isBufferFilled = false;
		}

	
  	//校正加单位转换
		imu_bmi088_data_00.Acc[0] = ax * GG / imu_bmi088_data_00.Acc_Scale;//单位m/s^2
		imu_bmi088_data_00.Acc[1] = ay * GG / imu_bmi088_data_00.Acc_Scale;
		imu_bmi088_data_00.Acc[2] = az * GG / imu_bmi088_data_00.Acc_Scale;//Range = 6g;
		imu_bmi088_data_00.Gyro[0] = gx - imu_bmi088_data_00.offset_Gyro[0];//单位rad/s
		imu_bmi088_data_00.Gyro[1] = gy - imu_bmi088_data_00.offset_Gyro[1];//Range = 1000°/s
		imu_bmi088_data_00.Gyro[2] = gz - imu_bmi088_data_00.offset_Gyro[2];
	
		for(uint8_t i = 0; i < 3; i++)
		{
			//姿态使用的滤波
			imu_bmi088_data_00.RPY_Acc[i] = Butter_run( &BMI_RPY_Acc_Filter_00[i] , imu_bmi088_data_00.Acc[i] );
			imu_bmi088_data_00.RPY_Gyro[i] = Butter_run( &BMI_RPY_Gyro_Filter_00[i] , imu_bmi088_data_00.Gyro[i] );
			//位置融合使用的滤波
			imu_bmi088_data_00.POS_Acc[i] = Butter_run( &BMI_POS_Acc_Filter1_00[i] , imu_bmi088_data_00.Acc[i] );
			imu_bmi088_data_00.POS_Acc_NoG_b[i] = Butter_run( &BMI_POS_Acc_Filter2_00[i] , imu_bmi088_data_00.Acc_NoG_b[i] );
			imu_bmi088_data_00.POS_Acc_NoG_w[i] = Butter_run( &BMI_POS_Acc_Filter3_00[i] , imu_bmi088_data_00.Acc_NoG_w[i] );
		}
		//赋值，调试用
//		for(uint8_t j = 0; j < 3; j++)
//		{
//			vision_bmi088_data.Acc[j] = imu_bmi088_data.Acc[j];
//			vision_bmi088_data.Gyro[j] = imu_bmi088_data.Gyro[j];
//			vision_bmi088_data.POS_Acc[j] = imu_bmi088_data.POS_Acc[j];
//			vision_bmi088_data.RPY_Acc[j] = imu_bmi088_data.RPY_Acc[j];
//			vision_bmi088_data.RPY_Gyro[j] = imu_bmi088_data.RPY_Gyro[j];
//		}
}
