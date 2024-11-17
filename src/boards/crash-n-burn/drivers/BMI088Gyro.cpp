#include "BMI088Gyro.hpp"

// Initialize the SPI settings
const SPISettings BMI088Gyro::spi_settings(10000000, MSBFIRST, SPI_MODE0);

BMI088Gyro::BMI088Gyro(uint8_t cs_pin) : cs_pin(cs_pin) {
    // Initialize last_gyro to NAN
    last_gyro[0] = NAN;
    last_gyro[1] = NAN;
    last_gyro[2] = NAN;

    // Set the CS pin as output and deactivate the device by setting it HIGH
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
}

void BMI088Gyro::setup() {
    // Check if the gyroscope device is connected by reading the chip ID
    if (read_reg(BMI088_GYR_REG_CHIP_ID) != BMI088_GYR_CHIP_ID) {
        Serial.println(F("BMI088 Gyro not found!"));
    } else {
        Serial.println(F("BMI088 Gyro detected"));
    }

    // Perform a soft reset on the device
    write_reg(BMI088_GYR_REG_SOFTRESET, 0xB6);
    delay(50); // Wait for the reset to complete

    // Configure the gyroscope range and bandwidth
    write_reg(BMI088_GYR_REG_RANGE, BMI088_GYR_2000DPS_RANGE);
    write_reg(BMI088_GYR_REG_BANDWIDTH, BMI088_GYR_ODR_100Hz_BW_32Hz);
    delay(50);

    // Verify the configuration
    if (read_reg(BMI088_GYR_REG_RANGE) != BMI088_GYR_2000DPS_RANGE) {
        Serial.println(F("BMI088 Gyro incorrect range set!"));
    }

    if ((read_reg(BMI088_GYR_REG_BANDWIDTH) & ~0x80) != BMI088_GYR_ODR_100Hz_BW_32Hz) {
        Serial.println(F("BMI088 Gyro incorrect bandwidth set!"));
    }

    // Initial data reading
    step();
}

void BMI088Gyro::step() {
    uint8_t raw_data[6];
    read_buf(BMI088_GYR_REG_DATA, raw_data, sizeof(raw_data));

    // Convert raw data to 16-bit integers
    int16_t raw_x = ((int16_t)raw_data[1] << 8) | (int16_t)raw_data[0];
    int16_t raw_y = ((int16_t)raw_data[3] << 8) | (int16_t)raw_data[2];
    int16_t raw_z = ((int16_t)raw_data[5] << 8) | (int16_t)raw_data[4];

    // Convert from raw LSB to degrees per second
    last_gyro[0] = static_cast<float>(raw_x) / BMI088_GYR_2000DPS_RES_LSB_DPS;
    last_gyro[1] = static_cast<float>(raw_y) / BMI088_GYR_2000DPS_RES_LSB_DPS;
    last_gyro[2] = static_cast<float>(raw_z) / BMI088_GYR_2000DPS_RES_LSB_DPS;
}

void BMI088Gyro::get(float* data) {
    // Copy the last_gyro data into the provided array
    data[0] = last_gyro[0];
    data[1] = last_gyro[1];
    data[2] = last_gyro[2];
}

void BMI088Gyro::spi_start() {
    SPI.beginTransaction(spi_settings);
    digitalWrite(cs_pin, LOW);
}

void BMI088Gyro::spi_end() {
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();
}

uint8_t BMI088Gyro::read_reg(uint8_t reg) {
    spi_start();

    SPI.transfer(reg | 0x80);
    uint8_t res = SPI.transfer(0x00);

    spi_end();
    return res;
}

void BMI088Gyro::write_reg(uint8_t reg, uint8_t data) {
    spi_start();

    SPI.transfer(reg);
    SPI.transfer(data);

    spi_end();
}

void BMI088Gyro::read_buf(uint8_t reg, uint8_t* data, uint8_t length) {
    spi_start();

    SPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = SPI.transfer(0x00);
    }

    spi_end();
}
