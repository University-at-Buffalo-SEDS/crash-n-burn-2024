#include "BMI088.h"

BMI088::BMI088() : spiSettings(BMI088_SPI_SPEED, BMI088_SPI_ORDER, BMI088_SPI_MODE) {}

bool BMI088::begin() {
    // Initialize CS pin
    pinMode(BMI088_CS_PIN, OUTPUT);
    digitalWrite(BMI088_CS_PIN, HIGH); // Deselect

    // Initialize SPI
    SPI.begin();

    // Reset Device
    if (!resetDevice()) {
        Serial.println(F("BMI088: Reset failed"));
        return false;
    }

    // Verify Chip ID
    uint8_t chip_id = 0;
    if (!readRegister(BMI088_ACC_REG_CHIP_ID, chip_id)) {
        Serial.println(F("BMI088: Failed to read Chip ID"));
        return false;
    }

    if (chip_id != BMI088_ACC_CHIP_ID) {
        Serial.print(F("BMI088: Unexpected Chip ID: 0x"));
        Serial.println(chip_id, HEX);
        return false;
    }
    Serial.println(F("BMI088: Chip ID verified"));

    // Configure Device
    if (!configureDevice()) {
        Serial.println(F("BMI088: Configuration failed"));
        return false;
    }

    Serial.println(F("BMI088: Initialization successful"));
    return true;
}

bool BMI088::resetDevice() {
    if (!writeRegister(BMI088_ACC_REG_SOFTRESET, BMI088_ACC_SOFTRESET_CMD)) {
        return false;
    }
    delay(100); // Wait for reset
    return true;
}

bool BMI088::configureDevice() {
    // Put Accel into Normal Mode
    if (!writeRegister(BMI088_ACC_REG_PWR_CONF, 0x00)) {
        return false;
    }
    delay(10);

    if (!writeRegister(BMI088_ACC_REG_PWR_CTRL, BMI088_ACC_PWR_CTRL_ENABLE)) {
        return false;
    }
    delay(10);

    // Set Configuration
    uint8_t sampling_conf = (BMI088_ACC_BWP_OSR4 | BMI088_ACC_ODR_200Hz);
    if (!writeRegister(BMI088_ACC_REG_CONF, sampling_conf)) {
        return false;
    }

    // Set Range
    if (!writeRegister(BMI088_ACC_REG_RANGE, BMI088_ACC_24G_RANGE)) {
        return false;
    }
    delay(10);

    // Verify Configuration
    uint8_t conf = 0;
    if (!readRegister(BMI088_ACC_REG_CONF, conf)) {
        return false;
    }
    if (conf != sampling_conf) {
        Serial.println(F("BMI088: Sampling configuration mismatch"));
        return false;
    }

    uint8_t range = 0;
    if (!readRegister(BMI088_ACC_REG_RANGE, range)) {
        return false;
    }
    if ((range & 0x03) != BMI088_ACC_24G_RANGE) {
        Serial.println(F("BMI088: Range configuration mismatch"));
        return false;
    }

    return true;
}

bool BMI088::writeRegister(uint8_t reg, uint8_t data) {
    digitalWrite(BMI088_CS_PIN, LOW);
    SPI.beginTransaction(spiSettings);
    SPI.transfer(reg & 0x7F); // Write operation
    SPI.transfer(data);
    SPI.endTransaction();
    digitalWrite(BMI088_CS_PIN, HIGH);
    return true;
}

bool BMI088::readRegister(uint8_t reg, uint8_t& data) {
    digitalWrite(BMI088_CS_PIN, LOW);
    SPI.beginTransaction(spiSettings);
    SPI.transfer(reg | 0x80); // Read operation
    data = SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(BMI088_CS_PIN, HIGH);
    return true;
}

bool BMI088::readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
    digitalWrite(BMI088_CS_PIN, LOW);
    SPI.beginTransaction(spiSettings);
    SPI.transfer(reg | 0x80); // Read with auto-increment
    for (size_t i = 0; i < length; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    SPI.endTransaction();
    digitalWrite(BMI088_CS_PIN, HIGH);
    return true;
}

bool BMI088::readAccelerometer(float* accelData) {
    uint8_t raw_data[6];
    if (!readRegisters(BMI088_ACC_REG_DATA, raw_data, 6)) {
        return false;
    }

    // Combine bytes into 16-bit signed integers
    int16_t raw_x = (int16_t)(raw_data[1] << 8 | raw_data[0]);
    int16_t raw_y = (int16_t)(raw_data[3] << 8 | raw_data[2]);
    int16_t raw_z = (int16_t)(raw_data[5] << 8 | raw_data[4]);

    // Convert raw data to g's
    // Assuming 24G range: sensitivity = 2000 mg/LSB (from datasheet)
    // Adjust based on actual sensitivity
    const float sensitivity = 2000.0f / 1000.0f; // g/LSB
    accelData[0] = raw_x / sensitivity;
    accelData[1] = raw_y / sensitivity;
    accelData[2] = raw_z / sensitivity;

    return true;
}
