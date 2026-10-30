#include "pca9685.h"
#include "boards/bread-compact-wifi-lcd/config.h" // 引入你之前修改的引脚配置
#include <esp_log.h>
#include <math.h>

static const char *TAG = "PCA9685";

// PCA9685 寄存器地址
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06

Pca9685::Pca9685() : i2c_bus_(nullptr), pca9685_dev_(nullptr) {}

Pca9685::~Pca9685() {
    if (pca9685_dev_) i2c_master_bus_rm_device(pca9685_dev_);
    if (i2c_bus_) i2c_del_master_bus(i2c_bus_);
}

esp_err_t Pca9685::Init() {
    // 1. 初始化 I2C 总线
    i2c_master_bus_config_t i2c_mst_config = {};
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = I2C_NUM_0; // 使用 I2C0
    i2c_mst_config.scl_io_num = PCA9685_I2C_SCL_PIN;
    i2c_mst_config.sda_io_num = PCA9685_I2C_SDA_PIN;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_));

    // 2. 挂载 PCA9685 设备
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = PCA9685_I2C_ADDR;
    dev_cfg.scl_speed_hz = PCA9685_I2C_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &pca9685_dev_));

    // 3. 复位 PCA9685
    WriteReg(PCA9685_MODE1, 0x00); // 正常模式
    vTaskDelay(pdMS_TO_TICKS(10));

    // 默认频率 50Hz (适合舵机)
    return SetFreq(50);
}

esp_err_t Pca9685::WriteReg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(pca9685_dev_, buf, 2, 100);
}

esp_err_t Pca9685::ReadReg(uint8_t reg, uint8_t *data) {
    return i2c_master_transmit_receive(pca9685_dev_, &reg, 1, data, 1, 100);
}

esp_err_t Pca9685::SetFreq(float freq) {
    if (freq < 1.0f) freq = 1.0f;
    if (freq > 1500.0f) freq = 1500.0f;

    // 计算 prescale 值
    float prescaleval = 25000000.0f;
    prescaleval /= 4096.0f;
    prescaleval /= freq;
    prescaleval -= 1.0f;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5f);

    // 必须处于睡眠模式才能写 prescale
    uint8_t oldmode;
    ReadReg(PCA9685_MODE1, &oldmode);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // SLEEP bit
    WriteReg(PCA9685_MODE1, newmode);
    
    WriteReg(PCA9685_PRESCALE, prescale);
    WriteReg(PCA9685_MODE1, oldmode);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // 重启 (RESTART bit)
    WriteReg(PCA9685_MODE1, oldmode | 0x80);
    return ESP_OK;
}

esp_err_t Pca9685::SetPwm(uint8_t channel, uint16_t on_time, uint16_t off_time) {
    if (channel > 15) return ESP_ERR_INVALID_ARG;
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    
    uint8_t buf[5];
    buf[0] = reg;
    buf[1] = on_time & 0xFF;
    buf[2] = on_time >> 8;
    buf[3] = off_time & 0xFF;
    buf[4] = off_time >> 8;
    
    return i2c_master_transmit(pca9685_dev_, buf, 5, 100);
}

esp_err_t Pca9685::SetAllPwm(uint16_t on_time, uint16_t off_time) {
    uint8_t buf[5];
    buf[0] = PCA9685_LED0_ON_L;
    buf[1] = on_time & 0xFF;
    buf[2] = on_time >> 8;
    buf[3] = off_time & 0xFF;
    buf[4] = off_time >> 8;
    
    return i2c_master_transmit(pca9685_dev_, buf, 5, 100);
}

esp_err_t Pca9685::SetServoAngle(uint8_t channel, float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // 舵机脉宽计算: 0度 -> 0.5ms (102 ticks @ 50Hz), 180度 -> 2.5ms (512 ticks @ 50Hz)
    // 这是一个近似公式，具体取决于你的舵机型号
    uint16_t pulse = (uint16_t)(102 + (angle / 180.0f) * 410);
    
    // 由于 50Hz 周期是 4096，off_time = pulse，on_time = 0
    return SetPwm(channel, 0, pulse);
}
