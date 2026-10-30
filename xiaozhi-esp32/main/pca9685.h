#ifndef _PCA9685_H_
#define _PCA9685_H_

#include <driver/i2c_master.h>
#include <esp_err.h>
#include <cstdint>

// PCA9685 默认 I2C 地址 (如果不确定，可以用 I2C 扫描工具查看，通常是 0x40)
#define PCA9685_I2C_ADDR 0x40

class Pca9685 {
public:
    Pca9685();
    ~Pca9685();

    // 初始化 I2C 和 PCA9685
    esp_err_t Init();
    
    // 设置 PWM 频率 (舵机通常设置为 50Hz)
    esp_err_t SetFreq(float freq);
    
    // 设置单个通道的 PWM 脉宽 (0-15 通道)
    // on_time 和 off_time 是 0-4095 的值，代表一个周期内的高低电平时间
    esp_err_t SetPwm(uint8_t channel, uint16_t on_time, uint16_t off_time);
    
    // 设置舵机角度 (0-180度)，自动计算脉宽
    esp_err_t SetServoAngle(uint8_t channel, float angle);
    
    // 设置所有通道的 PWM
    esp_err_t SetAllPwm(uint16_t on_time, uint16_t off_time);

private:
    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t pca9685_dev_;
    
    esp_err_t WriteReg(uint8_t reg, uint8_t data);
    esp_err_t ReadReg(uint8_t reg, uint8_t *data);
};

#endif // _PCA9685_H_
