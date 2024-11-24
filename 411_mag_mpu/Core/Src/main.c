/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mag.h"
#include "mpu6050.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CALIBRATION_TIMEOUT 5000 //timeout in milliseconds
#define RAD_TO_DEG (180.0 / PI)//DEG_PER_RAD (180.0 / PI)
#define DEG_TO_RAD (PI / 180.0)
#define MAG_DECLINATION_E 10.8 // Magnetic declination Orel 52.97 36.06 10.48
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t magData, delay = 13;
uint8_t isAC = 0, isSB = 0;
int16_t X_RAW, Y_RAW, Z_RAW;

int16_t X_offset;
int16_t Y_offset;
int16_t Z_offset;
uint8_t XY_Offset[6];

float32_t heading  = 0.0f;
float32_t alpha = 0.98;
float64_t dt = 0.02;

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float32_t Ax, Ay, Az, Gx, Gy, Gz;
float32_t fPhi, fThe, fPsi;
float32_t OldfPhi = 1., OldfThe = 1.;
float32_t fSinPhi, fSinThe, fSinPsi;
float32_t fCosPhi, fCosThe, fCosPsi;
float64_t roll, pitch, K_roll, K_pitch;
float64_t theta_x, theta_y;

uint32_t timer = 0;
Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

/*
float32_t A[3][3] = {{ 1.098399, 0.031869, -0.082378},
										 { 0.031869, 1.135835,  0.007728},
										 {-0.082378, 0.007728,  1.155759}};
float32_t b[3] = {1067.947654, -574.889729, -1243.537693};
*/

float32_t A[3][3] = {{ 1.098399, -0.031869, -0.082378},
										 { -0.031869, 1.135835,  -0.007728},
										 {-0.082378, -0.007728,  1.155759}};
float32_t b[3] = {1067.947654, 574.889729, -1243.537693};

/*
float32_t A[3][3] = {{ 0.113592,  -0.003187, -0.000773},
										 { -0.003187,  0.109848,  -0.008238},
										 {-0.000773, -0.008238,  0.115585}};
float32_t b[3] = {574.889729, 1067.947654, -1243.537693};
*/

float32_t Mag_cal[3] = {0., 0., 0.};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void I2C_Reset(void);

//MAG3110
void MAG3110_readMag(void);
void MAG3110_readHeading(void);

//void MAG3110_setOffset(void);
void MAG3110_setOffset(int16_t, int16_t, int16_t);
void MAG3110_getOffset(void);
//void MAG3110_setRawMode(void);
void MAG3110_setRawMode(uint8_t);
void MAG3110_setNormalMode(void);
void MAG3110_setActiveMode(void);
void MAG3110_setStandbyMode(void);
uint8_t MAG3110_isActive(void);
uint8_t MAG3110_isStandby(void);
void MAG3110_setDR_OS(uint8_t const DROS);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  I2C_Reset();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init();
  MAG3110_setActiveMode();
  MAG3110_getOffset();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char tx_buffer[70];
  while (1)
  {
  	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  	MPU6050_Read_Gyro();
  	MPU6050_Read_Accel();

  	// Заготовка
		/*
  	roll =  atan( Ay / sqrt(Ax * Ax + Az * Az)) * RAD_TO_DEG;
  	pitch = atan(-Ax / sqrt(Ay * Ay + Az * Az)) * RAD_TO_DEG;
  	K_roll  = Kalman_getAngle(&KalmanX,  roll, Gx * DEG_TO_RAD, dt);
  	K_pitch = Kalman_getAngle(&KalmanY, pitch, Gy * DEG_TO_RAD, dt);
  	*/

  	// Заготовка
  	roll =  atan2( Ay, sqrt(Ax * Ax + Az * Az)); //* RAD_TO_DEG;
  	pitch = atan2(-Ax, sqrt(Ay * Ay + Az * Az)); //* RAD_TO_DEG;
  	K_roll  = Kalman_getAngle(&KalmanX,  roll, Gx * DEG_TO_RAD, dt);
  	K_pitch = Kalman_getAngle(&KalmanY, pitch, Gy * DEG_TO_RAD, dt);


  	//MAG3110_readMag();
		MAG3110_readHeading();


  	//sprintf(tx_buffer, "$%d %d %d %f %f %f;", X_RAW, Y_RAW, Z_RAW, Ax, Ay, Az); // SerialPlotPlotter

		sprintf(tx_buffer, "%06.2f", heading);
  	HAL_UART_Transmit(&huart1, tx_buffer, strlen(tx_buffer), 1000);

  	HAL_Delay(delay);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void I2C_Reset(void) {
	// Solution from HAL_BUSY
	// https://istarik.ru/blog/stm32/123.html
	__I2C1_CLK_ENABLE();
	HAL_Delay(100);
	__I2C1_FORCE_RESET();
	HAL_Delay(100);
	__I2C1_RELEASE_RESET();
	HAL_Delay(100);

	__I2C2_CLK_ENABLE();
	HAL_Delay(100);
	__I2C2_FORCE_RESET();
	HAL_Delay(100);
	__I2C2_RELEASE_RESET();
	HAL_Delay(100);
}

void MAG3110_readMag(void) {
	uint8_t Out_XYZ[6];
	HAL_I2C_Mem_Read(&hi2c1, MAG3110_DEV_ADDR_R, OUT_X_MSB, 1, Out_XYZ, 6, 1000);
	X_RAW = (int16_t)(Out_XYZ[0] << 8 | Out_XYZ[1]);
	Y_RAW = (int16_t)(Out_XYZ[2] << 8 | Out_XYZ[3]);
	Z_RAW = (int16_t)(Out_XYZ[4] << 8 | Out_XYZ[5]);
	int16_t XYZ_RAW[3] = {X_RAW, -Y_RAW, Z_RAW};

	for(int i = 0; i < 3; ++i) {
		Mag_cal[i] = 0.;
		for(int j = 0; j < 3; ++j) {
			Mag_cal[i] += A[i][j] * (XYZ_RAW[j] - b[j]);
		}
	}
}

void MAG3110_readHeading(void) {
	MAG3110_readMag();
	float32_t xf = Mag_cal[0];
	float32_t yf = Mag_cal[1];
	float32_t zf = Mag_cal[2];

	//-------------------------------------------------------------------------------

	// fPhi == Roll
	// fThe == Pitch
	// fPsi == Yaw/Heading

	fPhi = atan2(Ay , Az);
	fPhi = 0.7 * OldfPhi + 0.3 * fPhi;
	OldfPhi = fPhi;
	fSinPhi = sin(fPhi);
	fCosPhi = cos(fPhi);


	fThe = atan(-Ax / (Ay * fSinPhi + Az * fCosPhi));
	fThe = 0.7 * OldfThe + 0.3 * fThe;
	OldfThe = fThe;
	fSinThe = sin(fThe);
	fCosThe = cos(fThe);

	fPsi = atan2((zf * fSinPhi - yf * fCosPhi), (xf * fCosThe + yf * fSinThe * fSinPhi + zf * fSinThe * fCosPhi));


	//-------------------------------------------------------------------------------

	// Заготовка
	theta_x = K_roll;
	theta_y = K_pitch;
  //xf = xf * cos(theta_y) + yf * sin(theta_x) * sin(theta_y) + Mag_cal[2] * cos(theta_x) * sin(theta_y);
  //yf = yf * cos(theta_x) - Mag_cal[2] * sin(theta_x);

	// Calculate the heading
	//heading  = (atan2(yf, xf) * RAD_TO_DEG);
	heading = fPsi * RAD_TO_DEG;
	heading = (heading >= 0) ? heading : heading + 360;
}

void MAG3110_setOffset(int16_t x_offset, int16_t y_offset, int16_t z_offset) {
	x_offset = 0;
	x_offset = x_offset << 1;
	magData = (uint8_t)(x_offset >> 8);
	HAL_I2C_Mem_Write(&hi2c1, MAG3110_DEV_ADDR_W, OFF_X_MSB, 1, &magData, 1, 1000);
	magData = (uint8_t)(x_offset & 0xFF);
	HAL_I2C_Mem_Write(&hi2c1, MAG3110_DEV_ADDR_W, OFF_X_LSB, 1, &magData, 1, 1000);

	y_offset = 0;
	y_offset = y_offset << 1;
	magData = (uint8_t)(y_offset >> 8);
	HAL_I2C_Mem_Write(&hi2c1, MAG3110_DEV_ADDR_W, OFF_Y_MSB, 1, &magData, 1, 1000);
	magData = (uint8_t)(y_offset & 0xFF);
	HAL_I2C_Mem_Write(&hi2c1, MAG3110_DEV_ADDR_W, OFF_Y_LSB, 1, &magData, 1, 1000);

	z_offset = 0;
	z_offset = z_offset << 1;
	magData = (uint8_t)(z_offset >> 8);
	HAL_I2C_Mem_Write(&hi2c1, MAG3110_DEV_ADDR_W, OFF_Z_MSB, 1, &magData, 1, 1000);
	magData = (uint8_t)(z_offset & 0xFF);
	HAL_I2C_Mem_Write(&hi2c1, MAG3110_DEV_ADDR_W, OFF_Z_LSB, 1, &magData, 1, 1000);
}

void MAG3110_getOffset(void) {
	HAL_I2C_Mem_Read(&hi2c1, MAG3110_DEV_ADDR_R, OFF_X_MSB, 1, XY_Offset, 6, 1000);
	X_offset = (int16_t)(XY_Offset[0] << 8 | XY_Offset[1]);
	Y_offset = (int16_t)(XY_Offset[2] << 8 | XY_Offset[3]);
	Z_offset = (int16_t)(XY_Offset[4] << 8 | XY_Offset[4]);
}

void MAG3110_setRawMode(uint8_t tf) {
	if(tf == 1) {
		magData = MAG3110_AUTO_MRST_EN | MAG3110_RAW_MODE;
		HAL_I2C_Mem_Write(&hi2c1, MAG3110_DEV_ADDR_W, CTRL_REG2, 1, &magData, 1, 1000);
		HAL_Delay(50);
	} else {
		magData = MAG3110_AUTO_MRST_EN | 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MAG3110_DEV_ADDR_W, CTRL_REG2, 1, &magData, 1, 1000);
		HAL_Delay(50);
	}
}
// дублирование двух функций - в raw-false и normal
void MAG3110_setNormalMode(void) {
	magData = MAG3110_AUTO_MRST_EN | MAG3110_NORMAL_MODE;
	HAL_I2C_Mem_Write(&hi2c1, MAG3110_DEV_ADDR_W, CTRL_REG2, 1, &magData, 1, 1000);
	HAL_Delay(50);
}

void MAG3110_setActiveMode(void) {
	magData = 0;
	HAL_I2C_Mem_Read(&hi2c1, MAG3110_DEV_ADDR_R, CTRL_REG1, 1, &magData, 1, 1000);
	magData |= (1 << 0); // MAG3110_ACTIVE_MODE
	HAL_I2C_Mem_Write(&hi2c1, MAG3110_DEV_ADDR_W, CTRL_REG1, 1, &magData, 1, 1000);
	isAC = 1;
	isSB = 0;
	HAL_Delay(50);
}

void MAG3110_setStandbyMode(void) {
	magData = 0;
	HAL_I2C_Mem_Read(&hi2c1, MAG3110_DEV_ADDR_R, CTRL_REG1, 1, &magData, 1, 1000);
	magData &= ~(1 << 0); // MAG3110_STANDBY_MODE
	HAL_I2C_Mem_Write(&hi2c1, MAG3110_DEV_ADDR_W, CTRL_REG1, 1, &magData, 1, 1000);
	isAC = 0;
	isSB = 1;
	HAL_Delay(50);
}

uint8_t MAG3110_isActive(void) {
	return isAC;
}

uint8_t MAG3110_isStandby(void) {
	return isSB;
}

void MAG3110_setDR_OS(uint8_t const DROS) {
	uint8_t status = MAG3110_isActive();
	if(status) {
		MAG3110_setStandbyMode();
	}
	magData = 0;
	HAL_I2C_Mem_Read(&hi2c1, MAG3110_DEV_ADDR_R, CTRL_REG1, 1, &magData, 1, 1000);
	magData &= 0x07; // оставляем три младших бита
	magData |= DROS;
	HAL_I2C_Mem_Write(&hi2c1, MAG3110_DEV_ADDR_W, CTRL_REG1, 1, &magData, 1, 1000);
	if(status) {
		MAG3110_setActiveMode();
	}
}

void MPU6050_Init(void) {
	uint8_t Data;
	Data = 0;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_DEV_ADD_AD0_LOW, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

	Data = 0x07;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_DEV_ADD_AD0_LOW, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

	Data = 0x10; //16
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_DEV_ADD_AD0_LOW, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

	Data = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_DEV_ADD_AD0_LOW, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
}

void MPU6050_Read_Accel(void) {
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_DEV_ADD_AD0_LOW, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	uint16_t mult = 1;

	Ax =  mult * Accel_X_RAW / 4096.0;
	Ay =  -mult * Accel_Y_RAW / 4096.0;
	Az =  mult * Accel_Z_RAW / 4096.0;
}

void MPU6050_Read_Gyro(void) {
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_DEV_ADD_AD0_LOW, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Gx =  Gyro_X_RAW/131.0;
	Gy =  Gyro_Y_RAW/131.0;
	Gz = -Gyro_Z_RAW/131.0;
}

float64_t Kalman_getAngle(Kalman_t *Kalman, float64_t newAngle, float64_t newRate, float64_t dt) {
	float64_t rate = newRate - Kalman->bias;
	Kalman->angle += dt * rate;

	Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Kalman->Q_bias * dt;

	float64_t S = Kalman->P[0][0] + Kalman->R_measure;
	float64_t K[2];
	K[0] = Kalman->P[0][0] / S;
	K[1] = Kalman->P[1][0] / S;

	float64_t y = newAngle - Kalman->angle;
	Kalman->angle += K[0] * y;
	Kalman->bias += K[1] * y;

	float64_t P00_temp = Kalman->P[0][0];
	float64_t P01_temp = Kalman->P[0][1];

	Kalman->P[0][0] -= K[0] * P00_temp;
	Kalman->P[0][1] -= K[0] * P01_temp;
	Kalman->P[1][0] -= K[1] * P00_temp;
	Kalman->P[1][1] -= K[1] * P01_temp;

	return Kalman->angle;
};


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
