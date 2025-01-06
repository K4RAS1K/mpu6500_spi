#include "mpu6500_lib.h"

int16_t accel_offsets[3]={0,0,0};
int16_t gyro_offsets[3]={0,0,0}; 

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

void mpu_accel_init(spi_device_handle_t handle) {
	uint8_t buffer[16];
	memset(buffer, 0, sizeof(buffer));
	mpu_write_byte(handle, ACCEL_CONFIG, ACCEL_2G);
	mpu_read_bytes(handle, ACCEL_CONFIG, 1, buffer);
	ESP_LOGI("mpu_accel_init","mpu sensivety: %x",buffer[0]);
	accel_create_offset(handle);
};

void mpu_gyro_init(spi_device_handle_t handle) {
	uint8_t buffer[16];
	memset(buffer, 0, sizeof(buffer));
	mpu_write_byte(handle, GYRO_CONFIG, GYRO_250_DPS);
	mpu_read_bytes(handle, GYRO_CONFIG, 1, buffer);
	ESP_LOGI("mpu_accel_init","mpu sensivety: %x",buffer[0]);
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

void mpu_read_accel(spi_device_handle_t handle,uint8_t *data) {
	
	memset(data, 0, sizeof(data));
	mpu_read_bytes(handle, ACCEL_XOUT_H, 6,  data);

    int16_t x = (int16_t)(data[0] << 8 | data[1]);
    int16_t y = (int16_t)(data[2] << 8 | data[3]);
    int16_t z = (int16_t)(data[4] << 8 | data[5]);
    
    x += accel_offsets[0];
    y += accel_offsets[1];
    z += accel_offsets[2];

    float accel_x = x / 32768.0 * ACCEL_2G_sens;
    float accel_y = y / 32768.0 * ACCEL_2G_sens;
    float accel_z = z / 32768.0 * ACCEL_2G_sens;

    ESP_LOGI("mpu_read_gyro", "x: %.3f, y: %.3f, z: %.3f\r\n", accel_x, accel_y, accel_z);
}

void mpu_read_gyro(spi_device_handle_t handle,uint8_t *data) {
	memset(data, 0, sizeof(data));
	mpu_read_bytes(handle, GYRO_XOUT_H, 6,  data);

    float x = data[0] << 8 | data[1];
    float y = data[2] << 8 | data[3];
    float z = data[4] << 8 | data[5];
    
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

void gyro_create_offset(spi_device_handle_t handle) {
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

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    gyro_offsets[0] = -(x_sum / CALIBRATION_SAMPLES);
    ESP_LOGI("mpu_accel_init","accel_offsets_x: %d",gyro_offsets[0]);	
    gyro_offsets[1] = -(y_sum / CALIBRATION_SAMPLES);
    ESP_LOGI("mpu_accel_init","accel_offsets_y: %d",gyro_offsets[1]);	
    gyro_offsets[2] = -(z_sum / CALIBRATION_SAMPLES);
    ESP_LOGI("mpu_accel_init","accel_offsets_z: %d",gyro_offsets[2]);	
}

void accel_create_offset(spi_device_handle_t handle) {
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

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    accel_offsets[0] = -(x_sum / CALIBRATION_SAMPLES);
    ESP_LOGI("mpu_accel_init","accel_offsets_x: %d",accel_offsets[0]);	
    accel_offsets[1] = -(y_sum / CALIBRATION_SAMPLES);
    ESP_LOGI("mpu_accel_init","accel_offsets_y: %d",accel_offsets[1]);	
    accel_offsets[2] = -(z_sum / CALIBRATION_SAMPLES);
    ESP_LOGI("mpu_accel_init","accel_offsets_z: %d",accel_offsets[2]);	
}

void gyro_add_offset(float *x,float *y,float *z) {
	x += gyro_offsets[0];
    y += gyro_offsets[1];
    z += gyro_offsets[2];	
}

void accel_add_offset(float *x,float *y,float *z) {
    x += accel_offsets[0];
    y += accel_offsets[1];
    z += accel_offsets[2];
}

void mpu6500_data(void) {
	uint8_t data[16]; 
    memset(data, 0, sizeof(data));
	spi_device_handle_t handle;
	spi_init(&handle);
	mpu_whoami(handle);
	mpu_init(handle);
	mpu_config(handle,data);
	mpu_accel_init(handle);
	mpu_gyro_init(handle);
	while(1) {
		mpu_read_accel(handle,data);
		//mpu_read_gyro(handle,data);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}