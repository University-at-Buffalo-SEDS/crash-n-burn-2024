#ifndef LTC2990_HPP
#define LTC2990_HPP

#include <Arduino.h>
#include <Wire.h>

class LTC2990 {
public:
    LTC2990(uint8_t i2c_address = 0x4C);

    void setup();
    void step();
    void get(float* voltages);

private:
    // I2C Address
    uint8_t i2c_address;

    // Conversion Constants
    static constexpr float SINGLE_ENDED_LSB = 5.0f / 16384.0f; // 5V / 2^14

    // Timeout for data validity in milliseconds
    static constexpr uint16_t TIMEOUT = 1000;

    // Internal buffer for voltage readings
    float last_voltages[4];

    // I2C Register Addresses
    static constexpr uint8_t STATUS_REG = 0x00;
    static constexpr uint8_t CONTROL_REG = 0x01;
    static constexpr uint8_t TRIGGER_REG = 0x02;
    static constexpr uint8_t V1_MSB_REG = 0x06;
    static constexpr uint8_t V1_LSB_REG = 0x07;
    static constexpr uint8_t V2_MSB_REG = 0x08;
    static constexpr uint8_t V2_LSB_REG = 0x09;
    static constexpr uint8_t V3_MSB_REG = 0x0A;
    static constexpr uint8_t V3_LSB_REG = 0x0B;
    static constexpr uint8_t V4_MSB_REG = 0x0C;
    static constexpr uint8_t V4_LSB_REG = 0x0D;

    // Control Register Settings
    static constexpr uint8_t VOLTAGE_MODE_MASK = 0x07;
    static constexpr uint8_t V1_V2_V3_V4 = 0x07;
    static constexpr uint8_t ENABLE_ALL = 0x18;
    static constexpr uint8_t TEMP_MEAS_MODE_MASK = 0x18;

    // Private Methods
    int8_t set_mode(uint8_t bits_to_set, uint8_t bits_to_clear);
    int8_t enable_all_voltages();
    int8_t trigger_conversion();
    int8_t adc_read_new_data(uint8_t msb_register_address, int16_t* adc_code, int8_t* data_valid);
    float code_to_single_ended_voltage(int16_t adc_code);

    // I2C Communication Helpers
    int8_t read_register(uint8_t reg_address, uint8_t* data);
    int8_t write_register(uint8_t reg_address, uint8_t data);
};

#endif 
