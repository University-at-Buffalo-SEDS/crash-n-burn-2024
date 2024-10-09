#ifndef BMI088_H
#define BMI088_H

#include <Arduino.h>
#include <SPI.h>

// Pin Definitions
#define BMI088_CS_PIN    PB11  // Chip Select Pin

// BMI088 Register Addresses
#define BMI088_ACC_REG_CHIP_ID          (0x00)
#define BMI088_ACC_REG_DATA             (0x12)
#define BMI088_ACC_REG_CONF             (0x40)
#define BMI088_ACC_REG_RANGE            (0x41)
#define BMI088_ACC_REG_PWR_CONF         (0x7C)
#define BMI088_ACC_REG_PWR_CTRL         (0x7D)
#define BMI088_ACC_REG_SOFTRESET        (0x7E)

// Configuration Constants
#define BMI088_ACC_24G_RANGE            (0x03)
#define BMI088_ACC_BWP_OSR4             (0x80)
#define BMI088_ACC_ODR_200Hz            (0x09)
#define BMI088_ACC_PWR_CTRL_ENABLE      (0x04)
#define BMI088_ACC_SOFTRESET_CMD        (0xB6)
#define BMI088_ACC_CHIP_ID              (0x1E)

// SPI Settings
#define BMI088_SPI_SPEED                10000000 // 10 MHz
#define BMI088_SPI_MODE                 SPI_MODE0
#define BMI088_SPI_ORDER                MSBFIRST

class BMI088 {
public:
    BMI088();
    bool begin();
    bool readAccelerometer(float* accelData);
    
private:
    SPISettings spiSettings;
    bool writeRegister(uint8_t reg, uint8_t data);
    bool readRegister(uint8_t reg, uint8_t& data);
    bool readRegisters(uint8_t reg, uint8_t* buffer, size_t length);
    bool resetDevice();
    bool configureDevice();
};

#endif // BMI088_H
