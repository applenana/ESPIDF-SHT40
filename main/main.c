#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO           5         // SCL引脚
#define I2C_MASTER_SDA_IO           4         // SDA引脚
#define I2C_MASTER_NUM              I2C_NUM_0  // I2C端口号
#define I2C_MASTER_FREQ_HZ          100000     // I2C时钟频率
#define I2C_MASTER_TX_BUF_DISABLE   0          // 禁用TX缓冲区
#define I2C_MASTER_RX_BUF_DISABLE   0          // 禁用RX缓冲区
#define SHT40_SENSOR_ADDR           0x44       // SHT40默认I2C地址
#define SHT40_MEASURE_CMD           0xFD       // 测量命令

static const char *TAG = "SHT40";

esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t sht40_read_measurement(float *temperature, float *humidity) {
    uint8_t data[6];
    uint8_t cmd = SHT40_MEASURE_CMD;

    // 发送测量命令
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, SHT40_SENSOR_ADDR, &cmd, 1, pdMS_TO_TICKS(100)));

    // 延时等待测量完成
    vTaskDelay(pdMS_TO_TICKS(10));

    // 读取传感器数据
    ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_MASTER_NUM, SHT40_SENSOR_ADDR, data, 6, pdMS_TO_TICKS(100)));

    // 数据解码
    uint16_t raw_temp = (data[0] << 8) | data[1];
    uint16_t raw_humi = (data[3] << 8) | data[4];

    // 转换为实际值
    *temperature = -45 + 175 * ((float)raw_temp / 65535.0f);
    *humidity = 100 * ((float)raw_humi / 65535.0f);

    return ESP_OK;
}

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    float temperature, humidity;

    while (1) {
        if (sht40_read_measurement(&temperature, &humidity) == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);
        } else {
            ESP_LOGE(TAG, "Failed to read from SHT40");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒读取一次
    }
}
