#ifndef MAIN_SPI_DRIVER_H_
#define MAIN_SPI_DRIVER_H_

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_err.h"
#include <stdint.h>
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include <string.h>
#include "esp_log.h"


#define SPIBUS_READ     	(0x80)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    	(0x7F)  /*!< addr & SPIBUS_WRITE */

#define PIN_NUM_CLK         14
#define PIN_NUM_MISO        12
#define PIN_NUM_MOSI        13 
#define HOST 				HSPI_HOST

#define PIN_NUM_CS          15

#define SPI_MODE 			3

#define SPI_CLOCK_SPEED_HZ (500*1000)


void spi_init(spi_device_handle_t *handle);

void mpu_read_bytes(spi_device_handle_t handle, uint8_t regAddr, uint8_t length, uint8_t *data);

void mpu_write_byte(spi_device_handle_t handle, uint8_t regAddr, uint8_t data);

void mpu_write_bytes(spi_device_handle_t handle, uint8_t regAddr,uint8_t length, uint8_t *data);


#endif