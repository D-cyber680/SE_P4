#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/i2c.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include "string.h"
#include "stdlib.h"

#include "mpu6050/mpu6050.h"

static const char *TAG = "SE_P4: ";

#define I2C_MASTER_SCL_IO 22        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t device_register_write_byte(
    uint8_t register_address,
    uint8_t size,
    uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (mpu6050_device_address << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, register_address, 1);
    i2c_master_write(cmd, data, size - 1, 0);
    i2c_master_write_byte(cmd, data[size - 1], 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}
static esp_err_t device_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t ret;
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        reg_addr,
        len,
        data);

    return ESP_OK;
}
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    mpu6050_init();
    mpu6050_set_rate(200);                 // configuramos el sampling rate
    mpu6050_set_dlpf_mode(5);              // modo 5: low pass filter
    mpu6050_set_full_scale_gyro_range(1);  // scale 1 en giroscopio
    mpu6050_set_full_scale_accel_range(1); // scale 1 en acelerometro

    while (mpu6050_test_connection())
    {
        ESP_LOGI(TAG, "WHO_AM_I = 0x%x \n", mpu6050_get_device_id());
        ESP_LOGI(TAG, "GYRO_XOUT_H = 0x%x \n", mpu6050_get_rotation_x());
        ESP_LOGI(TAG, "GYRO_YOUT_H = 0x%x \n", mpu6050_get_rotation_y());
        ESP_LOGI(TAG, "GYRO_ZOUT_H = 0x%x \n", mpu6050_get_rotation_z());
        ESP_LOGI(TAG, "ACCEL_XOUT_H = 0x%x \n", mpu6050_get_acceleration_x());
        ESP_LOGI(TAG, "ACCEL_YOUT_H = 0x%x \n", mpu6050_get_acceleration_y());
        ESP_LOGI(TAG, "ACCEL_ZOUT_H = 0x%x \n", mpu6050_get_acceleration_z());

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
