#include "LTC2990.hpp"

LTC2990::LTC2990(uint8_t i2c_address) : i2c_address(i2c_address) {
    // Initialize voltages to NAN
    for (int i = 0; i < 4; i++) {
        last_voltages[i] = NAN;
    }
}

void LTC2990::setup() {
    int8_t ack;

    // Set to Single-Ended Mode
    ack = set_mode(V1_V2_V3_V4, VOLTAGE_MODE_MASK);
    if (ack != 0) {
        Serial.println(F("Failed to set Single-Ended Mode."));
        while (1);
    }

    // Enable all voltage channels
    ack = enable_all_voltages();
    if (ack != 0) {
        Serial.println(F("Failed to enable voltage channels."));
        while (1);
    }

    Serial.println(F("LTC2990 configured for Single-Ended Voltage Monitoring."));

    // Initial data reading
    step();
}

void LTC2990::step() {
    int8_t ack;
    int16_t adc_code;
    int8_t data_valid;

    // Trigger Conversion
    ack = trigger_conversion();
    if (ack != 0) {
        Serial.println(F("Failed to trigger conversion."));
        return;
    }

    delay(10); // Allow time for conversion

    // Read voltages V1 to V4
    uint8_t msb_registers[4] = {V1_MSB_REG, V2_MSB_REG, V3_MSB_REG, V4_MSB_REG};
    for (int i = 0; i < 4; i++) {
        ack = adc_read_new_data(msb_registers[i], &adc_code, &data_valid);
        if (ack != 0 || data_valid != 1) {
            Serial.print(F("Error reading V"));
            Serial.println(i + 1);
            last_voltages[i] = NAN;
            continue;
        }
        last_voltages[i] = code_to_single_ended_voltage(adc_code);
    }
}

void LTC2990::get(float* voltages) {
    // Copy the last_voltages data into the provided array
    for (int i = 0; i < 4; i++) {
        voltages[i] = last_voltages[i];
    }
}

// Private Methods

int8_t LTC2990::set_mode(uint8_t bits_to_set, uint8_t bits_to_clear) {
    uint8_t reg_data;
    int8_t ack;

    // Read current CONTROL_REG
    ack = read_register(CONTROL_REG, &reg_data);
    if (ack != 0) return ack;

    // Modify bits
    reg_data &= ~bits_to_clear;
    reg_data |= bits_to_set;

    // Write back CONTROL_REG
    ack = write_register(CONTROL_REG, reg_data);
    return ack;
}

int8_t LTC2990::enable_all_voltages() {
    return set_mode(ENABLE_ALL, TEMP_MEAS_MODE_MASK);
}

int8_t LTC2990::trigger_conversion() {
    return write_register(TRIGGER_REG, 0x00);
}

int8_t LTC2990::adc_read_new_data(uint8_t msb_register_address, int16_t* adc_code, int8_t* data_valid) {
    uint16_t timeout = TIMEOUT;
    int8_t ack;
    uint8_t status;
    uint8_t status_bit = (msb_register_address / 2) - 1;

    // Wait for new data
    while (timeout--) {
        ack = read_register(STATUS_REG, &status);
        if (ack != 0) return ack;

        if (((status >> status_bit) & 0x01) == 1) break;

        delay(1);
    }
    if (timeout == 0) return 1; // Timeout

    // Read ADC data
    uint8_t msb, lsb;
    ack = read_register(msb_register_address, &msb);
    if (ack != 0) return ack;

    ack = read_register(msb_register_address + 1, &lsb);
    if (ack != 0) return ack;

    uint16_t code = ((uint16_t)msb << 8) | lsb;
    *data_valid = (code >> 15) & 0x01;
    *adc_code = code & 0x7FFF;

    return (*data_valid == 1) ? 0 : 1;
}

float LTC2990::code_to_single_ended_voltage(int16_t adc_code) {
    float voltage;
    int16_t sign = 1;

    if (adc_code & 0x4000) { // Negative value
        adc_code = (adc_code ^ 0x7FFF) + 1; // Two's complement
        sign = -1;
    }

    adc_code &= 0x3FFF; // 14-bit value
    voltage = ((float)adc_code) * SINGLE_ENDED_LSB * sign;

    return voltage;
}

// I2C Communication Helpers

int8_t LTC2990::read_register(uint8_t reg_address, uint8_t* data) {
    Wire.beginTransmission(i2c_address);
    Wire.write(reg_address);
    if (Wire.endTransmission(false) != 0) return 1;

    Wire.requestFrom((int)i2c_address, 1);
    if (Wire.available()) {
        *data = Wire.read();
        return 0;
    }
    return 1;
}

int8_t LTC2990::write_register(uint8_t reg_address, uint8_t data) {
    Wire.beginTransmission(i2c_address);
    Wire.write(reg_address);
    Wire.write(data);
    return Wire.endTransmission();
}
