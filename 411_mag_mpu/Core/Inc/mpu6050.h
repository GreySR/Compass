#include "arm_math.h"

#define WHO_AM_I_VALUE 0x68
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_DEV_ADD_AD0_LOW 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B


// Заготовка
// Kalman structure
typedef struct
{
	float64_t Q_angle;
	float64_t Q_bias;
	float64_t R_measure;
	float64_t angle;
	float64_t bias;
	float64_t P[2][2];
} Kalman_t;

void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
float64_t Kalman_getAngle(Kalman_t *Kalman, float64_t newAngle, float64_t newRate, float64_t dt);


// MPU6050 structure
typedef struct
{
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	float64_t Ax;
	float64_t Ay;
	float64_t Az;

	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;
	float64_t Gx;
	float64_t Gy;
	float64_t Gz;

	float64_t KalmanAngleX;
	float64_t KalmanAngleY;
} MPU6050_t;





