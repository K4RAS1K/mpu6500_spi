#ifndef MAIN_MPU6500_LIB_H_
#define MAIN_MPU6500_LIB_H_

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_err.h"
#include <stdint.h>
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include <string.h>
#include "esp_log.h"
#include "spi_driver.h"
#include <math.h>
#include "MadgwickAHRS/MadgwickAHRS.h"
//#include "MahonyAHRS/MahonyAHRS.h"
#include "MovingAverage/MovingAverage.h"
#include "HighPassFilter/HighPassFilter.h"

#define ACCEL_16G
#define GYRO_2000_DPS

#ifdef ACCEL_2G
	#define ACCEL			0xE0
	#define ACCEL_sens		(1.0f/16384.0f) * G2MSS
#endif
#ifdef ACCEL_4G
	#define ACCEL			0xE8
	#define ACCEL_sens		(1.0f/8192.0f) * G2MSS
#endif
#ifdef ACCEL_8G
	#define ACCEL			0xF0
	#define ACCEL_sens		(1.0f/4096.0f) * G2MSS
#endif
#ifdef ACCEL_16G
	#define ACCEL			0xF8
	#define ACCEL_sens		(1.0f/2048.0f) * G2MSS
#endif
#ifdef GYRO_250_DPS
	#define GYRO			0xE0
	#define GYRO_sens		(1.0f/131.0f) * D2R
#endif
#ifdef GYRO_500_DPS
	#define GYRO			0xE8
	#define GYRO_sens		1.0f/65.5f) * D2R
#endif
#ifdef GYRO_1000_DPS
	#define GYRO			0xF0
	#define GYRO_sens		(1.0f/32.8f) * D2R
#endif
#ifdef GYRO_2000_DPS
	#define GYRO			0xF8
	#define GYRO_sens		(1.0f/16.4f) * D2R
#endif

#define CALIBRATION_SAMPLES 100

#define G2MSS 9.81

#define WHO_AM_I    		0x75
#define PWR_MGMT_1  		0x6B
#define SMPLRT_DIV			0x68
#define SIGNAL_PATH_RESET   0x68
#define USER_CTRL  			0x6A
#define SIGNAL_PATH_RESET	0x68

#define PWR1_DEVICE_RESET   0x80
#define SIG_COND_RST   		0x01

#define ACCEL_CONFIG 		0x1C

#define ACCEL_XOUT_H     	0x3B

#define GYRO_CONFIG			0x1B

#define GYRO_XOUT_H 		0x43

#define CONFIG              0x1A

#define USER_CTRL 			0x6A
#define DMP_ENABLE 			0x80

#define CONFIG           	0x1A

void mpu6500_data(void);

void mpu_reset_user_ctrl(spi_device_handle_t handle);

void mpu_reset_signal_path(spi_device_handle_t handle);

void mpu_reset(spi_device_handle_t handle);

void mpu_init(spi_device_handle_t handle);

void mpu_accel_init(spi_device_handle_t handle);

void mpu_gyro_init(spi_device_handle_t handle);

void mpu_whoami(spi_device_handle_t handle);

void mpu_smplrt_div(spi_device_handle_t handle, uint8_t smplrt_div);

float* mpu_read_accel(spi_device_handle_t handle,float *accel_data);

float* mpu_read_gyro(spi_device_handle_t handle,float *gyro_data);

void mpu_config(spi_device_handle_t handle);

void gyro_create_offset(spi_device_handle_t handle);

void accel_create_offset(spi_device_handle_t handle);

void gyro_add_offset(float *x,float *y,float *z);

void accel_add_offset(float *x,float *y,float *z);

struct MotionData* calculate_position(struct MotionData* accel_data, float *acceleration);

void mpu_set_bandwidth(spi_device_handle_t handle);

void MA_Init(void);

void MA_Update(float *accel_data,float *gyro_data);

void HP_Init(void);

void HP_Update(float *accel_data);

#endif
