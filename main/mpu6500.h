#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO           GPIO_NUM_22     
#define I2C_MASTER_SDA_IO           GPIO_NUM_21     
#define I2C_MASTER_NUM              0       
#define I2C_MASTER_FREQ_HZ          400000  
#define I2C_MASTER_TX_BUF_DISABLE   0      
#define I2C_MASTER_RX_BUF_DISABLE   0       
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6500_SENSOR_ADDR         0x68    
#define MPU6500_PWR_MGMT_1          0x6B    
#define MPU6500_ACCEL_XOUT_H        0x3B   

#define GRAVITY_EARTH               9.80665f
#define ACCEL_SCALE_FACTOR          16384.0f

esp_err_t i2c_master_init(void);
esp_err_t mpu6500_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t mpu6500_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len);