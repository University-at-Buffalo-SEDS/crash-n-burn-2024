#include "BMI088Accel.hpp"

// Initialize the SPI settings
const SPISettings BMI088Accel::spi_settings(10000000, MSBFIRST, SPI_MODE0);

// Standard gravity in m/s^2
static constexpr float STANDARD_GRAVITY = 9.80665f;

BMI088Accel::BMI088Accel(uint8_t cs_pin) : cs_pin(cs_pin) {
    // Initialize last_accel to NAN
    last_accel[0] = NAN;
    last_accel[1] = NAN;
    last_accel[2] = NAN;

    // Initialize device configuration settings
    range_conf = BMI088_ACC_24G_RANGE;
    sampling_conf = BMI088_ACC_BWP_OSR4 | BMI088_ACC_ODR_200Hz;

    // Set the CS pin as output and deactivate the device by setting it HIGH
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
}

void BMI088Accel::setup() {
    // All delays are based on Section 4.6.1 of the datasheet
    // Requires a rising edge on CS to startup SPI on accel
    digitalWrite(cs_pin, LOW);
    delay(1);
    digitalWrite(cs_pin, HIGH);
    // Wait 50 milliseconds for CS high
    delay(50);

    // Perform a soft reset on the device
    write_reg(BMI088_ACC_REG_SOFTRESET, 0xB6);
    delay(50); // Wait for the reset to complete

    // Dummy read
    read_reg(BMI088_ACC_REG_CHIP_ID);

    // Check if the accelerometer device is connected by reading the chip ID
    if (read_reg(BMI088_ACC_REG_CHIP_ID) != BMI088_ACC_CHIP_ID) {
        Serial.println(F("BMI088 Accelerometer not found!"));
    } else {
        Serial.println(F("BMI088 Accelerometer detected"));
    }

    // Put the accelerometer into Active mode
    write_reg(BMI088_ACC_REG_PWR_CONF, 0x00);
    delay(50);
    // Turn on the accelerometer's sensor module
    write_reg(BMI088_ACC_REG_PWR_CTRL, 0x04);
    // Configure oversampling and output data rate
    write_reg(BMI088_ACC_REG_CONF, sampling_conf);
    // Set the accelerometer range
    write_reg(BMI088_ACC_REG_RANGE, range_conf);
    delay(50);

    // Verify the configuration
    if (read_reg(BMI088_ACC_REG_CONF) != sampling_conf) {
        Serial.println(F("BMI088 Accelerometer incorrect sampling rate set!"));
    }

    if ((read_reg(BMI088_ACC_REG_RANGE) & 0x03) != range_conf) {
        Serial.println(F("BMI088 Accelerometer incorrect range set!"));
    }

    // Test if accelerometer is powered on
    if (read_reg(BMI088_ACC_REG_PWR_CTRL) != 0x04) {
        Serial.println(F("BMI088 Accelerometer did not turn on!"));
    }

    // Initial data reading
    step();
}

void BMI088Accel::step() {
    uint8_t raw_data[6];
    read_buf(BMI088_ACC_REG_DATA, raw_data, sizeof(raw_data));

    // Convert raw data to 16-bit integers
    int16_t raw_x = ((int16_t)raw_data[1] << 8) | (int16_t)raw_data[0];
    int16_t raw_y = ((int16_t)raw_data[3] << 8) | (int16_t)raw_data[2];
    int16_t raw_z = ((int16_t)raw_data[5] << 8) | (int16_t)raw_data[4];

    // Use to go from LSB to G's
    float BMI088_MULTIPLIER = 1.0f / (1 << 15) * (1 << (range_conf + 1)) * 1.5f;

    // Convert from raw LSB to m/s^2
    last_accel[0] = static_cast<float>(raw_x) * BMI088_MULTIPLIER * STANDARD_GRAVITY;
    last_accel[1] = static_cast<float>(raw_y) * BMI088_MULTIPLIER * STANDARD_GRAVITY;
    last_accel[2] = static_cast<float>(raw_z) * BMI088_MULTIPLIER * STANDARD_GRAVITY;
}

void BMI088Accel::get(float* data) {
    // Copy the last_accel data into the provided array
    data[0] = last_accel[0];
    data[1] = last_accel[1];
    data[2] = last_accel[2];
}

void BMI088Accel::spi_start() {
    SPI.beginTransaction(spi_settings);
    digitalWrite(cs_pin, LOW);
}

void BMI088Accel::spi_end() {
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();
}

uint8_t BMI088Accel::read_reg(uint8_t reg) {
    spi_start();

    SPI.transfer(reg | 0x80);
    SPI.transfer(0x00); // Dummy byte as per datasheet
    uint8_t res = SPI.transfer(0x00);

    spi_end();
    return res;
}

void BMI088Accel::write_reg(uint8_t reg, uint8_t data) {
    spi_start();

    SPI.transfer(reg);
    SPI.transfer(data);

    spi_end();
}

void BMI088Accel::read_buf(uint8_t reg, uint8_t* data, uint8_t length) {
    spi_start();

    SPI.transfer(reg | 0x80);
    SPI.transfer(0x00); // Dummy byte as per datasheet
    for (uint8_t i = 0; i < length; i++) {
        data[i] = SPI.transfer(0x00);
    }

    spi_end();
}
