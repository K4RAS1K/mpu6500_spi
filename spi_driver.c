#include "spi_driver.h"

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

void mpu_write_byte(spi_device_handle_t handle, uint8_t regAddr, uint8_t data) {
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

void mpu_write_bytes(spi_device_handle_t handle, uint8_t regAddr,uint8_t length, uint8_t *data) {
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
