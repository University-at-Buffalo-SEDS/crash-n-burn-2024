#ifndef BMI088_GYRO_HPP
#define BMI088_GYRO_HPP

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

class BMI088Gyro {
public:
    BMI088Gyro(uint8_t cs_pin);

    void setup();
    void step();
    void get(float* data);  

private:
    static constexpr uint8_t BMI088_GYR_CHIP_ID = 0x0F;
    static constexpr uint8_t BMI088_GYR_2000DPS_RANGE = 0x00;
    static constexpr float   BMI088_GYR_2000DPS_RES_LSB_DPS = 16.384f;
    static constexpr uint8_t BMI088_GYR_ODR_100Hz_BW_32Hz = 0x07;

    static constexpr uint8_t BMI088_GYR_REG_CHIP_ID = 0x00;
    static constexpr uint8_t BMI088_GYR_REG_DATA = 0x02;
    static constexpr uint8_t BMI088_GYR_REG_RANGE = 0x0F;
    static constexpr uint8_t BMI088_GYR_REG_BANDWIDTH = 0x10;
    static constexpr uint8_t BMI088_GYR_REG_SOFTRESET = 0x14;

    void spi_start();
    void spi_end();
    uint8_t read_reg(uint8_t reg);
    void write_reg(uint8_t reg, uint8_t data);
    void read_buf(uint8_t reg, uint8_t* data, uint8_t length);

    static const SPISettings spi_settings;

    float last_gyro[3];

    uint8_t cs_pin;
};

#endif