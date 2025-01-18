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

#define CALIBRATION_SAMPLES 1000

#define WHO_AM_I    		0x75
#define PWR_MGMT_1  		0x6B
#define SMPLRT_DIV			0x68
#define SIGNAL_PATH_RESET   0x68
#define USER_CTRL  			0x6A
#define SIGNAL_PATH_RESET	0x68

#define PWR1_DEVICE_RESET   0x80
#define SIG_COND_RST   		0x01

#define ACCEL_CONFIG 		0x1C
#define ACCEL_2G			0xE0
#define ACCEL_2G_sens		2
#define ACCEL_4G			0xE8
#define ACCEL_4G_sens		4
#define ACCEL_8G			0xF0
#define ACCEL_8G_sens		8
#define ACCEL_16G			0xF8
#define ACCEL_16G_sens		16
#define ACCEL_XOUT_H     	0x3B

#define GYRO_CONFIG			0x1B
#define GYRO_250_DPS		0xE0
#define GYRO_250_DPS_sens	250
#define GYRO_500_DPS		0xE8
#define GYRO_500_DPS_sens	500
#define GYRO_1000_DPS		0xF0
#define GYRO_1000_DPS_sens	1000
#define GYRO_2000_DPS		0xF8
#define GYRO_2000_DPS_sens	2000
#define GYRO_XOUT_H 		0x43

#define CONFIG              0x1A

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

#endif
