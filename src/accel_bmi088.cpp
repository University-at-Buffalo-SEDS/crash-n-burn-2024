#include "accel.h"
#include <Arduino.h>
#include <SPI.h>

#define PIN_ACCEL_CHIPSELECT PB11


/*
 * Accel driver for BMI088
 * Datasheet for the BMI088 can be found at:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
 */

// Page 25
// BMI088 Accel values
#define BMI088_ACC_CHIP_ID              (0x1E)
#define BMI088_ACC_24G_RANGE            (0x03)
#define BMI088_ACC_BWP_OSR4             (0x80)
#define BMI088_ACC_ODR_200Hz            (0x09)

// BMI088 register addresses
#define BMI088_ACC_REG_CHIP_ID          (0x00)
#define BMI088_ACC_REG_DATA             (0x12)
#define BMI088_ACC_REG_CONF             (0x40)
#define BMI088_ACC_REG_RANGE            (0x41)
#define BMI088_ACC_REG_PWR_CONF         (0x7C)
#define BMI088_ACC_REG_PWR_CTRL         (0x7D)
#define BMI088_ACC_REG_SOFTRESET        (0x7E)

// SPI function prototypes
static void spi_start();
static void spi_end();
static u_int8_t red_register(u_int8_t register);
static void write_register(u_int8_t register, u_int8_t data);
static void read_buffer(u_int8_t register, u_int8_t *data, u_int8_t length);

// SPI settings for the BMI_088 (pg 46)
static const SPISettings spi_settings(10'000'000, MSBFIRST, SPI_MODE0);

// XYZ Acceleration values
static float accel[3] = {NAN, NAN, NAN};

