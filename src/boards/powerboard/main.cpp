#include <Arduino.h>
#include <Wire.h>
#include "config.h"

#define LTC2990_I2C_ADDRESS     0x98

#if defined (USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

#define LTC2990_STATUS_REG        0x00
#define LTC2990_CONTROL_REG       0x01
#define LTC2990_TRIGGER_REG       0x02
#define LTC2990_V1_MSB_REG        0x06
#define LTC2990_V1_LSB_REG        0x07
#define LTC2990_V2_MSB_REG        0x08
#define LTC2990_V2_LSB_REG        0x09
#define LTC2990_V3_MSB_REG        0x0A
#define LTC2990_V3_LSB_REG        0x0B
#define LTC2990_V4_MSB_REG        0x0C
#define LTC2990_V4_LSB_REG        0x0D
#define LTC2990_TINT_MSB_REG      0x04
#define LTC2990_TINT_LSB_REG      0x05

void ltc2990_init();
int16_t ltc2990_read_voltage(uint8_t msb_reg, uint8_t lsb_reg);
float ltc2990_read_internal_temperature();


void setup() {
    #if defined (USBCON) && defined(USBD_USE_CDC)
	    usb_serial.begin();
    #else
	    Serial.begin(9600);
    #endif
    while (!Serial) { ; } // Wait for Serial to initialize

    Wire.setSDA(LTC_SDA_PIN);
    Wire.setSCL(LTC_SCL_PIN);
    Wire.begin();
}

void loop() {
    Serial.println("Greetings from the digital world!");
}

int16_t ltc2990_read_voltage(uint8_t msb_reg, uint8_t lsb_reg) {
    Wire.beginTransmission(LTC2990_I2C_ADDRESS);
    Wire.write(msb_reg);
    Wire.endTransmission(false);
    Wire.requestFrom(LTC2990_I2C_ADDRESS, 1);
    uint8_t msb = Wire.read();

    Wire.beginTransmission(LTC2990_I2C_ADDRESS);
    Wire.write(lsb_reg);
    Wire.endTransmission(false);
    Wire.requestFrom(LTC2990_I2C_ADDRESS, 1);
    uint8_t lsb = Wire.read();

    int16_t result = ((int16_t)msb << 8) | lsb;
    result >>= 4;  // Only 12 bits used for voltage data
    return result;
}