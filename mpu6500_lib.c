#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_err.h"
#include <stdint.h>
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include <string.h>
#include "esp_log.h"

#define PIN_NUM_CLK         14
#define PIN_NUM_MISO        12
#define PIN_NUM_MOSI        13 
#define HOST 				HSPI_HOST

#define PIN_NUM_CS          15

#define SPI_MODE 			3

#define SPI_CLOCK_SPEED_HZ (500*1000) 

#define MPU6500_WHO_AM_I    0x75
#define MPU6500_PWR_MGMT_1  0x6B
#define MPU6500_GYRO_CONFIG 0x1B
#define MPU6500_ACCEL_CONFIG 0x1C

#define ACCEL_CONFIG2                 0x1D
#define ACONFIG2_FIFO_SIZE_BIT        7  // [7:6]
#define ACONFIG2_FIFO_SIZE_LENGTH     2
#define ACONFIG2_ACCEL_FCHOICE_B_BIT  3
#define ACONFIG2_A_DLPF_CFG_BIT       2  // [2:0]
#define ACONFIG2_A_DLPF_CFG_LENGTH    3
#define FIFO_SIZE_1K 				  1

#define ACCEL_CONFIG        	0x1C
#define ACONFIG_XA_ST_BIT   	7
#define ACONFIG_YA_ST_BIT   	6
#define ACONFIG_ZA_ST_BIT   	5
#define ACONFIG_FS_SEL_BIT  	4  // [4:3]
#define ACONFIG_FS_SEL_LENGT	2
#define ACONFIG_HPF_BIT     	2  // [2:0]
#define ACONFIG_HPF_LENGTH  	3

#define ACONFIG_FS_SEL_LENGTH 	2
#define ACCEL_FS_4G 			1

#define SPIBUS_READ     (0x80)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    (0x7F)  /*!< addr & SPIBUS_WRITE */

uint8_t buffer[16];  

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

void mpu_read_bytes(spi_device_handle_t handle, uint8_t regAddr, uint8_t length, int16_t *data) {
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

void mpu_write_bytes(spi_device_handle_t handle, uint8_t regAddr, uint8_t length, uint8_t data){
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

void mpu_init(spi_device_handle_t handle) {
	int16_t data[16]; 
    memset(data, 0, sizeof(data));
	mpu_write_bytes(handle,0x6B, 1, 0x80);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	mpu_write_bytes(handle,0x6B, 1, 0x00);
}

void mpu_whoami(spi_device_handle_t handle) {
	int16_t data[16]; 
    memset(data, 0, sizeof(data));
    uint8_t length = 1;
	
	mpu_read_bytes(handle,MPU6500_WHO_AM_I,length,data);
	
	for(int i =0;i<length;i++){
		ESP_LOGI("mpu_whoami","data: %x",data[i]);	
	}
}

void mpu_smplrt_div(spi_device_handle_t handle, uint8_t smplrt_div, int16_t *data) {
	memset(data, 0, sizeof(data));
	mpu_write_bytes(handle,0x19, 1, smplrt_div);
	mpu_read_bytes(handle, 0x19, 1,  data);
	for(int i =0;i<1;i++){
		ESP_LOGI("mpu_smplrt_div","data: %x",data[i]);	
	}
}

void mpu_read_accel(spi_device_handle_t handle) {
	int16_t data[16]; 
    memset(data, 0, sizeof(data));
    uint8_t length = 16;
	
	mpu_read_bytes(handle,0x3B,length,data);
	
	for(int i =0;i<length;i++){
		ESP_LOGI("mpu_read_accel","data: %x",data[i]);	
	}	
	int16_t x = (data[0] << 8) | data[1];
	int16_t y = (data[2] << 8) | data[3];
	int16_t z = (data[4] << 8) | data[5];
	ESP_LOGI("mpu_read_accel","x: %d",x);	
	ESP_LOGI("mpu_read_accel","y: %d",y);	
	ESP_LOGI("mpu_read_accel","z: %d",z);	
}

void mpu6500_data(void) {
	int16_t data[16]; 
    memset(data, 0, sizeof(data));
	spi_device_handle_t handle;
	spi_init(&handle);
	mpu_whoami(handle);
	mpu_init(handle);
	while(1) {

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}