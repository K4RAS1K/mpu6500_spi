#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_err.h"
#include <stdint.h>
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include <string.h>
#include "esp_log.h"
#include "KalmanFilter.c"

#define PIN_NUM_CLK         14
#define PIN_NUM_MISO        12
#define PIN_NUM_MOSI        13 
#define HOST 				HSPI_HOST

#define PIN_NUM_CS          15

#define SPI_MODE 			3

#define SPI_CLOCK_SPEED_HZ (500*1000) 

#define CALIBRATION_SAMPLES 100

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

#define SPIBUS_READ     	(0x80)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    	(0x7F)  /*!< addr & SPIBUS_WRITE */

void spi_init(spi_device_handle_t *handle) {
	//SPI_BEGIN
	spi_bus_config_t config;
    memset(&config, 0, sizeof(spi_bus_config_t));
    config.mosi_io_num = PIN_NUM_MOSI;
    config.miso_io_num = PIN_NUM_MISO;
    config.sclk_io_num = PIN_NUM_CLK;
    config.quadwp_io_num = -1;  // -1 not used
    config.quadhd_io_num = -1;  // -1 not used
    config.max_transfer_sz = 4092;
    spi_bus_initialize(HOST, &config, 0);  // 0 DMA not used
    //ADD_DEVISE
    spi_device_interface_config_t dev_config;
    memset(&dev_config, 0, sizeof(spi_device_interface_config_t));
    dev_config.command_bits = 0;
    dev_config.address_bits = 8;
    dev_config.dummy_bits = 0;
    dev_config.mode = SPI_MODE;
    dev_config.duty_cycle_pos = 128;  // default 128 = 50%/50% duty
    dev_config.cs_ena_pretrans = 0;  // 0 not used
    dev_config.cs_ena_posttrans = 0;  // 0 not used
    dev_config.clock_speed_hz = SPI_CLOCK_SPEED_HZ;
    dev_config.spics_io_num = PIN_NUM_CS;
    dev_config.flags = 0;  // 0 not used
    dev_config.queue_size = 1;
    dev_config.pre_cb = NULL;
    dev_config.post_cb = NULL;
    spi_bus_add_device(HOST, &dev_config, handle);
}
//переписать нормально 

void mpu_read_bytes(spi_device_handle_t handle, uint8_t regAddr, uint8_t length, uint8_t *data) {
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr | SPIBUS_READ;
    transaction.length = length * 8;
    transaction.rxlength = length * 8;
    transaction.user = NULL;
    transaction.tx_buffer = NULL;
    transaction.rx_buffer = data;
    spi_device_transmit(handle, &transaction);   
}

void mpu_write_byte(spi_device_handle_t handle, uint8_t regAddr, uint8_t data){
	uint8_t length = 1;
	uint8_t buffer[16];
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = data;
	spi_transaction_t transaction;
    transaction.flags = 0;
    //transaction.cmd = data;
    transaction.addr = regAddr & SPIBUS_WRITE;
    transaction.length = length * 8;
    transaction.rxlength = 0;
    transaction.user = NULL;
    transaction.tx_buffer = buffer;
    transaction.rx_buffer = NULL;
    spi_device_transmit(handle, &transaction);
}

void mpu_write_bytes(spi_device_handle_t handle, uint8_t regAddr,uint8_t length, uint8_t *data){
	memset(data, 0, sizeof(data));
	spi_transaction_t transaction;
    transaction.flags = 0;
    //transaction.cmd = data;
    transaction.addr = regAddr & SPIBUS_WRITE;
    transaction.length = length * 8;
    transaction.rxlength = 0;
    transaction.user = NULL;
    transaction.tx_buffer = data;
    transaction.rx_buffer = NULL;
    spi_device_transmit(handle, &transaction);
}

void mpu_reset_user_ctrl(spi_device_handle_t handle) {
	mpu_write_byte(handle,USER_CTRL,SIG_COND_RST);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	ESP_LOGI("mpu_reset_user_ctrl","user ctrl reset");
}

void mpu_reset_signal_path(spi_device_handle_t handle) {
	mpu_write_byte(handle,SIGNAL_PATH_RESET,0x07);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	ESP_LOGI("mpu_reset_signal_path","signal path reset");
}

void mpu_reset(spi_device_handle_t handle) {
	mpu_write_byte(handle,PWR_MGMT_1,PWR1_DEVICE_RESET);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	mpu_reset_user_ctrl(handle);
	ESP_LOGI("mpu_reset","mpu reset");
}

void mpu_init(spi_device_handle_t handle) {
	uint8_t data[16]; 
    memset(data, 0, sizeof(data));
	mpu_write_byte(handle,PWR_MGMT_1, PWR1_DEVICE_RESET);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	mpu_write_byte(handle,PWR_MGMT_1, 0x00);
	uint8_t buffer[16];
	memset(buffer, 0, sizeof(buffer));
	mpu_read_bytes(handle, PWR_MGMT_1, 1, buffer);
	ESP_LOGI("mpu_init","mpu: %x",buffer[0]);
}

void mpu_accel_init(spi_device_handle_t handle,int16_t *accel_offsets) {
	uint8_t buffer[16];
	memset(buffer, 0, sizeof(buffer));
	mpu_write_byte(handle, ACCEL_CONFIG, ACCEL_2G);
	mpu_read_bytes(handle, ACCEL_CONFIG, 1, buffer);
	ESP_LOGI("mpu_accel_init","mpu sensivety: %x",buffer[0]);
	int32_t x_sum = 0, y_sum = 0, z_sum = 0;
    int16_t x = 0, y = 0, z = 0;
	for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        uint8_t data[6];
        mpu_read_bytes(handle, ACCEL_XOUT_H, 6, data);

        x = (data[0] << 8) | data[1];
        y = (data[2] << 8) | data[3];
        z = (data[4] << 8) | data[5];

        x_sum += x;
        y_sum += y;
        z_sum += z;

        vTaskDelay(pdMS_TO_TICKS(10)); // Wait 10ms between samples
    }
    accel_offsets[0] = -(x_sum / CALIBRATION_SAMPLES);
    ESP_LOGI("mpu_accel_init","accel_offsets_x: %d",accel_offsets[0]);	
    accel_offsets[1] = -(y_sum / CALIBRATION_SAMPLES);
    ESP_LOGI("mpu_accel_init","accel_offsets_y: %d",accel_offsets[1]);	
    accel_offsets[2] = -(z_sum / CALIBRATION_SAMPLES);
    ESP_LOGI("mpu_accel_init","accel_offsets_z: %d",accel_offsets[2]);	
};

void mpu_gyro_init(spi_device_handle_t handle,int16_t *gyro_offsets) {
	uint8_t buffer[16];
	memset(buffer, 0, sizeof(buffer));
	mpu_write_byte(handle, GYRO_CONFIG, GYRO_250_DPS);
	mpu_read_bytes(handle, GYRO_CONFIG, 1, buffer);
	ESP_LOGI("mpu_accel_init","mpu sensivety: %x",buffer[0]);
	int32_t x_sum = 0, y_sum = 0, z_sum = 0;
    int16_t x = 0, y = 0, z = 0;
	for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        uint8_t data[6];
        mpu_read_bytes(handle, ACCEL_XOUT_H, 6, data);

        x = (data[0] << 8) | data[1];
        y = (data[2] << 8) | data[3];
        z = (data[4] << 8) | data[5];

        x_sum += x;
        y_sum += y;
        z_sum += z;

        vTaskDelay(pdMS_TO_TICKS(10)); // Wait 10ms between samples
    }
    gyro_offsets[0] = -(x_sum / CALIBRATION_SAMPLES);
    ESP_LOGI("mpu_accel_init","accel_offsets_x: %d",gyro_offsets[0]);	
    gyro_offsets[1] = -(y_sum / CALIBRATION_SAMPLES);
    ESP_LOGI("mpu_accel_init","accel_offsets_y: %d",gyro_offsets[1]);	
    gyro_offsets[2] = -(z_sum / CALIBRATION_SAMPLES);
    ESP_LOGI("mpu_accel_init","accel_offsets_z: %d",gyro_offsets[2]);	
};

void mpu_whoami(spi_device_handle_t handle) {
	uint8_t data[16]; 
    memset(data, 0, sizeof(data));
    uint8_t length = 1;
	
	mpu_read_bytes(handle,WHO_AM_I,length,data);
	
	for(int i =0;i<length;i++){
		ESP_LOGI("mpu_whoami","data: %x",data[i]);	
	}
}

void mpu_smplrt_div(spi_device_handle_t handle, uint8_t smplrt_div, uint8_t *data) {
	memset(data, 0, sizeof(data));
	mpu_write_byte(handle,SMPLRT_DIV, smplrt_div);
	mpu_read_bytes(handle, SMPLRT_DIV, 1,  data);
	for(int i =0;i<1;i++){
		ESP_LOGI("mpu_smplrt_div","data: %x",data[i]);	
	}
}

void mpu_read_accel(spi_device_handle_t handle,uint8_t *data,int16_t *accel_offsets) {
	memset(data, 0, sizeof(data));
	mpu_read_bytes(handle, ACCEL_XOUT_H, 6,  data);
	/*for(int i =0;i<6;i++){
		ESP_LOGI("mpu_read_accel","data: %x",data[i]);	
	}*/
    float x = data[0] << 8 | data[1];
    float y = data[2] << 8 | data[3];
    float z = data[4] << 8 | data[5];
    
    x += accel_offsets[0];
    y += accel_offsets[1];
    z += accel_offsets[2];
    
    x = x / 32768 * ACCEL_2G_sens;
    y = y / 32768 * ACCEL_2G_sens;
    z = z / 32768 * ACCEL_2G_sens;
    
    ESP_LOGI("mpu_read_accel","x: %.2f",x);	
    ESP_LOGI("mpu_read_accel","y: %.2f",y);	
    ESP_LOGI("mpu_read_accel","z: %.2f",z);	
}

void mpu_read_gyro(spi_device_handle_t handle,uint8_t *data,int16_t * gyro_offsets) {
	memset(data, 0, sizeof(data));
	mpu_read_bytes(handle, GYRO_XOUT_H, 6,  data);
	/*for(int i =0;i<6;i++){
		ESP_LOGI("mpu_read_accel","data: %x",data[i]);	
	}*/
    float x = data[0] << 8 | data[1];
    float y = data[2] << 8 | data[3];
    float z = data[4] << 8 | data[5];
    
    x += gyro_offsets[0];
    y += gyro_offsets[1];
    z += gyro_offsets[2];
    
    x = x / 32768 * GYRO_250_DPS_sens;
    y = y / 32768 * GYRO_250_DPS_sens;
    z = z / 32768 * GYRO_250_DPS_sens;
    
    ESP_LOGI("mpu_read_gyro","x: %.2f",x);	
    ESP_LOGI("mpu_read_gyro","y: %.2f",y);	
    ESP_LOGI("mpu_read_gyro","z: %.2f",z);	
}

void mpu_config(spi_device_handle_t handle,uint8_t *data) {
	memset(data, 0, sizeof(data));
	mpu_write_byte(handle,CONFIG, 0x03);
	mpu_read_bytes(handle, CONFIG, 1,  data);
	for(int i =0;i<1;i++){
		ESP_LOGI("mpu_config","data: %x",data[i]);	
	}
}

void mpu6500_data(void) {
	uint8_t data[16]; 
	int16_t accel_offsets[3]={0,0,0};
	int16_t gyro_offsets[3]={0,0,0}; 
    memset(data, 0, sizeof(data));
	spi_device_handle_t handle;
	spi_init(&handle);
	mpu_whoami(handle);
	mpu_init(handle);
	mpu_config(handle,data);
	mpu_accel_init(handle,accel_offsets);
	mpu_gyro_init(handle,gyro_offsets);
	while(1) {
		mpu_read_accel(handle,data,accel_offsets);
		mpu_read_gyro(handle,data,gyro_offsets);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}