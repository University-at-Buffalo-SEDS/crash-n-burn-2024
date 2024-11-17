#ifndef BMI088_ACCEL_HPP
#define BMI088_ACCEL_HPP

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

class BMI088Accel {
public:
    BMI088Accel(uint8_t cs_pin);

    void setup();
    void step();
    void get(float* data);  

private:
    // BMI088 Accelerometer constants
    static constexpr uint8_t BMI088_ACC_CHIP_ID = 0x1E;
    static constexpr uint8_t BMI088_ACC_24G_RANGE = 0x03;
    static constexpr uint8_t BMI088_ACC_BWP_OSR4 = 0x80;
    static constexpr uint8_t BMI088_ACC_ODR_200Hz = 0x09;

    // BMI088 register addresses
    static constexpr uint8_t BMI088_ACC_REG_CHIP_ID = 0x00;
    static constexpr uint8_t BMI088_ACC_REG_DATA = 0x12;
    static constexpr uint8_t BMI088_ACC_REG_CONF = 0x40;
    static constexpr uint8_t BMI088_ACC_REG_RANGE = 0x41;
    static constexpr uint8_t BMI088_ACC_REG_PWR_CONF = 0x7C;
    static constexpr uint8_t BMI088_ACC_REG_PWR_CTRL = 0x7D;
    static constexpr uint8_t BMI088_ACC_REG_SOFTRESET = 0x7E;

    // SPI communication methods
    void spi_start();
    void spi_end();
    uint8_t read_reg(uint8_t reg);
    void write_reg(uint8_t reg, uint8_t data);
    void read_buf(uint8_t reg, uint8_t* data, uint8_t length);

    // SPI settings for BMI088
    static const SPISettings spi_settings;

    // Stores the last read accelerometer data
    float last_accel[3];

    // Device configuration settings
    uint8_t range_conf;
    uint8_t sampling_conf;

    // Chip select pin number
    uint8_t cs_pin;
};

#endif 
