#ifndef BMI088_GYRO_H
#define BMI088_GYRO_H

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <STM32FreeRTOS.h>  // Include FreeRTOS for synchronization primitives

class BMI088Gyro {
public:
    // Constructor that accepts the chip select pin number
    BMI088Gyro(uint8_t cs_pin);

    // Initializes the gyroscope device
    void setup();

    // Reads the latest gyroscope data
    void step();

    // Returns the last read gyroscope's XYZ angular velocity in degrees per second
    void get(float* data);  // Modified to copy data safely

    // Prints the gyroscope's data for debugging
    void print();

private:
    // BMI088 Gyroscope constants
    static constexpr uint8_t BMI088_GYR_CHIP_ID = 0x0F;
    static constexpr uint8_t BMI088_GYR_2000DPS_RANGE = 0x00;
    static constexpr float   BMI088_GYR_2000DPS_RES_LSB_DPS = 16.384f;
    static constexpr uint8_t BMI088_GYR_ODR_100Hz_BW_32Hz = 0x07;

    // BMI088 register addresses
    static constexpr uint8_t BMI088_GYR_REG_CHIP_ID = 0x00;
    static constexpr uint8_t BMI088_GYR_REG_DATA = 0x02;
    static constexpr uint8_t BMI088_GYR_REG_RANGE = 0x0F;
    static constexpr uint8_t BMI088_GYR_REG_BANDWIDTH = 0x10;
    static constexpr uint8_t BMI088_GYR_REG_SOFTRESET = 0x14;

    // SPI communication methods
    void spi_start();
    void spi_end();
    uint8_t read_reg(uint8_t reg);
    void write_reg(uint8_t reg, uint8_t data);
    void read_buf(uint8_t reg, uint8_t* data, uint8_t length);

    // SPI settings for BMI088
    static const SPISettings spi_settings;

    // Stores the last read gyroscope data
    float last_gyro[3];

    // Chip select pin number
    uint8_t cs_pin;

    // Mutex for synchronizing access to last_gyro
    SemaphoreHandle_t xLastGyroMutex;
};

#endif // BMI088_GYRO_H
