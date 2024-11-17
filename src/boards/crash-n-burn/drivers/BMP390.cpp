#include "BMP390.hpp"

// SPI settings (10 MHz, MSB first, SPI Mode 0)
const SPISettings BMP390::spi_settings(10000000, MSBFIRST, SPI_MODE0);

BMP390::BMP390(uint8_t cs_pin) : cs_pin(cs_pin), last_altitude(NAN), last_pressure(NAN), last_temperature(INT16_MIN) {
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH); // Deselect the barometer
}

void BMP390::setup() {
    // Check if BMP390 is connected
    if (readReg(REG_CHIP_ID) != BMP390_CHIP_ID) {
        Serial.println(F("BMP390 not found!"));
        return;
    }
    Serial.println(F("BMP390 detected"));

    // Soft reset
    writeReg(REG_CMD, SOFT_RESET);
    delay(50);

    // Configure the barometer
    writeReg(REG_PWR_CTRL, ENABLE_PRESSURE | ENABLE_TEMP | ENABLE_SENSOR);
    writeReg(REG_OSR, (OSR_x2 << 3 | OSR_x32));
    writeReg(REG_ODR, ODR_12p5_HZ);
    delay(5);

    // Read and process calibration data
    uint8_t raw_calib[21];
    readBuffer(REG_CAL, raw_calib, sizeof(raw_calib));
    loadCalibrationData(raw_calib);
}

void BMP390::step() {
    uint8_t raw_data[6];
    readBuffer(REG_DATA, raw_data, sizeof(raw_data));

    uint32_t uncomp_press = ((uint32_t)raw_data[2] << 16) | ((uint32_t)raw_data[1] << 8) | raw_data[0];
    uint32_t uncomp_temp = ((uint32_t)raw_data[5] << 16) | ((uint32_t)raw_data[4] << 8) | raw_data[3];

    last_temperature = static_cast<int16_t>(compensateTemperature(uncomp_temp) * 100);
    last_pressure = compensatePressure(uncomp_press);

    // Calculate altitude using the pressure altitude formula
    last_altitude = 44330 * (1.0f - powf(last_pressure / 101325, 0.1903f));
}

float BMP390::getAltitude() {
    return last_altitude;
}

int16_t BMP390::getTemperature() {
    return last_temperature;
}

float BMP390::getPressure() {
    return last_pressure;
}

void BMP390::loadCalibrationData(uint8_t *raw_data) {
    raw_calib_data.nvm_par_t1 = (uint16_t)raw_data[1] << 8 | raw_data[0];
    calib_data.par_t1 = raw_calib_data.nvm_par_t1 / powf(2, -8);
    raw_calib_data.nvm_par_t2 = (uint16_t)raw_data[3] << 8 | raw_data[2];
    calib_data.par_t2 = raw_calib_data.nvm_par_t2 / powf(2, 30);
    raw_calib_data.nvm_par_t3 = raw_data[4];
    calib_data.par_t3 = raw_calib_data.nvm_par_t3 / powf(2, 48);
    raw_calib_data.nvm_par_p1 = (int16_t)raw_data[6] << 8 | raw_data[5];
    calib_data.par_p1 = (raw_calib_data.nvm_par_p1 - powf(2, 14)) / powf(2, 20);
    raw_calib_data.nvm_par_p2 = (int16_t)raw_data[8] << 8 | raw_data[7];
    calib_data.par_p2 = (raw_calib_data.nvm_par_p2 - powf(2, 14)) / powf(2, 29);
    raw_calib_data.nvm_par_p3 = raw_data[9];
    calib_data.par_p3 = raw_calib_data.nvm_par_p3 / powf(2, 32);
    raw_calib_data.nvm_par_p4 = raw_data[10];
    calib_data.par_p4 = raw_calib_data.nvm_par_p4 / powf(2, 37);
    raw_calib_data.nvm_par_p5 = (uint16_t)raw_data[12] << 8 | raw_data[11];
    calib_data.par_p5 = raw_calib_data.nvm_par_p5 / powf(2, -3);
    raw_calib_data.nvm_par_p6 = (uint16_t)raw_data[14] << 8 | raw_data[13];
    calib_data.par_p6 = raw_calib_data.nvm_par_p6 / powf(2, 6);
    raw_calib_data.nvm_par_p7 = raw_data[15];
    calib_data.par_p7 = raw_calib_data.nvm_par_p7 / powf(2, 8);
    raw_calib_data.nvm_par_p8 = raw_data[16];
    calib_data.par_p8 = raw_calib_data.nvm_par_p8 / powf(2, 15);
    raw_calib_data.nvm_par_p9 = (int16_t)raw_data[18] << 8 | raw_data[17];
    calib_data.par_p9 = raw_calib_data.nvm_par_p9 / powf(2, 48);
    raw_calib_data.nvm_par_p10 = raw_data[19];
    calib_data.par_p10 = raw_calib_data.nvm_par_p10 / powf(2, 48);
    raw_calib_data.nvm_par_p11 = raw_data[20];
    calib_data.par_p11 = raw_calib_data.nvm_par_p11 / powf(2, 65);
}

float BMP390::compensateTemperature(uint32_t uncomp_temp) {
    float partial_data1 = (float)(uncomp_temp - calib_data.par_t1);
    float partial_data2 = partial_data1 * calib_data.par_t2;
    calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data.par_t3;
    return calib_data.t_lin;
}

float BMP390::compensatePressure(uint32_t uncomp_press) {
    float partial_data1 = calib_data.par_p6 * calib_data.t_lin;
    float partial_data2 = calib_data.par_p7 * (calib_data.t_lin * calib_data.t_lin);
    float partial_data3 = calib_data.par_p8 * (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin);
    float partial_out1 = calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;
    float partial_out2 = uncomp_press * (calib_data.par_p1 + calib_data.par_p2 * calib_data.t_lin + calib_data.par_p3 * pow(calib_data.t_lin, 2) + calib_data.par_p4 * pow(calib_data.t_lin, 3));
    return partial_out1 + partial_out2 + pow(uncomp_press, 2) * (calib_data.par_p9 + calib_data.par_p10 * calib_data.t_lin) + pow(uncomp_press, 3) * calib_data.par_p11;
}

void BMP390::spiStart() {
    SPI.beginTransaction(spi_settings);
    digitalWrite(cs_pin, LOW);
}

void BMP390::spiEnd() {
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();
}

uint8_t BMP390::readReg(uint8_t reg) {
    spiStart();
    SPI.transfer(reg | 0x80);  
    SPI.transfer(0);           
    uint8_t res = SPI.transfer(0); 
    spiEnd();
    return res;
}


void BMP390::writeReg(uint8_t reg, uint8_t data) {
    spiStart();
    SPI.transfer(reg & ~0x80);
    SPI.transfer(data);
    spiEnd();
}

void BMP390::readBuffer(uint8_t reg, uint8_t* data, uint8_t length) {
    spiStart();
    SPI.transfer(reg | 0x80); 
    SPI.transfer(0);           
    for (uint8_t i = 0; i < length; i++) {
        data[i] = SPI.transfer(0); 
    }
    spiEnd();
}

