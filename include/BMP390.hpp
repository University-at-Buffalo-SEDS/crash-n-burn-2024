#ifndef BMP390_HPP
#define BMP390_HPP

#include <Arduino.h>
#include <SPI.h>
#include <cmath>

class BMP390 {
public:
    BMP390(uint8_t cs_pin);

    void setup();
    void step();
    float getAltitude();
    int16_t getTemperature();
    float getPressure();

private:
    // Constants
    static constexpr uint8_t BMP390_CHIP_ID = 0x60;
    static constexpr uint8_t ENABLE_PRESSURE = 0x01;
    static constexpr uint8_t ENABLE_TEMP = 0x02;
    static constexpr uint8_t ENABLE_SENSOR = 0x30;
    static constexpr uint8_t OSR_x2 = 0b001;
    static constexpr uint8_t OSR_x32 = 0b101;
    static constexpr uint8_t ODR_12p5_HZ = 0x04;
    static constexpr uint8_t SOFT_RESET = 0xB6;

    // Register addresses
    static constexpr uint8_t REG_CHIP_ID = 0x00;
    static constexpr uint8_t REG_DATA = 0x04;
    static constexpr uint8_t REG_PWR_CTRL = 0x1B;
    static constexpr uint8_t REG_OSR = 0x1C;
    static constexpr uint8_t REG_ODR = 0x1D;
    static constexpr uint8_t REG_CAL = 0x31;
    static constexpr uint8_t REG_CMD = 0x7E;

    // SPI settings
    static const SPISettings spi_settings;

    // Calibration data structures
    struct RawCalibData {
        uint16_t nvm_par_t1;
        uint16_t nvm_par_t2;
        int8_t   nvm_par_t3;
        int16_t  nvm_par_p1;
        int16_t  nvm_par_p2;
        int8_t   nvm_par_p3;
        int8_t   nvm_par_p4;
        uint16_t nvm_par_p5;
        uint16_t nvm_par_p6;
        int8_t   nvm_par_p7;
        int8_t   nvm_par_p8;
        int16_t  nvm_par_p9;
        int8_t   nvm_par_p10;
        int8_t   nvm_par_p11;
    } raw_calib_data;

    struct CalibData {
        float par_t1;
        float par_t2;
        float par_t3;
        float par_p1;
        float par_p2;
        float par_p3;
        float par_p4;
        float par_p5;
        float par_p6;
        float par_p7;
        float par_p8;
        float par_p9;
        float par_p10;
        float par_p11;
        float t_lin;
    } calib_data;

    // Private methods
    void loadCalibrationData(uint8_t *raw_data);
    float compensateTemperature(uint32_t uncomp_temp);
    float compensatePressure(uint32_t uncomp_press);
    void spiStart();
    void spiEnd();
    uint8_t readReg(uint8_t reg);
    void writeReg(uint8_t reg, uint8_t data);
    void readBuffer(uint8_t reg, uint8_t* data, uint8_t length);

    uint8_t cs_pin;
    
    // Variables to store the last values
    float last_altitude;
    float last_pressure;
    int16_t last_temperature;
};

#endif 
