#include <Arduino.h>
#include <SPI.h>
#include <STM32FreeRTOS.h>

#include "BMI088Accel.hpp"
#include "BMI088Gyro.hpp"
#include "BMP390.hpp"           
#include "stm32pinouts.hpp"

#if defined(USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

// Create instances of the devices
BMI088Accel accel(ACCEL_CS_PIN);  // ACCEL_CS_PIN defined in config.h
BMI088Gyro gyro(GYRO_CS_PIN);     // GYRO_CS_PIN defined in config.h
BMP390 barometer(BARO_CS_PIN);    // BARO_CS_PIN should also be defined in config.h

// Shared data structures
float accelData[3];
float gyroData[3];
float baroData[3];  // Stores barometer altitude, pressure, and temperature

// Mutexes for thread safety
SemaphoreHandle_t xAccelDataMutex;
SemaphoreHandle_t xGyroDataMutex;
SemaphoreHandle_t xBaroDataMutex;  // New mutex for barometer data

// Task function declarations
void TaskReadSensors(void* pvParameters);
void TaskPrintSensors(void* pvParameters);

void setup() {
    // Initialize serial communication
    #if defined(USBCON) && defined(USBD_USE_CDC)
        usb_serial.begin();
    #else
        Serial.begin(9600);
    #endif

    // Wait for Serial to initialize
    while (!Serial) { ; } 

    // Initialize the SPI bus
    SPI.begin();

    // Setup the devices
    accel.setup();
    gyro.setup();
    barometer.setup();

    // Create mutexes for shared data
    xAccelDataMutex = xSemaphoreCreateMutex();
    xGyroDataMutex = xSemaphoreCreateMutex();
    xBaroDataMutex = xSemaphoreCreateMutex();

    if (xAccelDataMutex == NULL || xGyroDataMutex == NULL || xBaroDataMutex == NULL) {
        Serial.println(F("Error creating mutexes"));
        while (1);
    }

    // Create the tasks
    xTaskCreate(TaskReadSensors, "ReadSensors", 256, NULL, 2, NULL);
    xTaskCreate(TaskPrintSensors, "PrintSensors", 256, NULL, 1, NULL);

    // Start the scheduler
    vTaskStartScheduler();
}

void loop() {
    // Empty. Tasks are now scheduled by FreeRTOS.
}

// Task to read data from sensors
void TaskReadSensors(void* pvParameters) {
    (void) pvParameters;

    for (;;) {
        // Read data from accelerometer
        accel.step();

        // Copy data with mutex protection
        if (xSemaphoreTake(xAccelDataMutex, portMAX_DELAY) == pdTRUE) {
            accel.get(accelData);
            xSemaphoreGive(xAccelDataMutex);
        }

        // Read data from gyroscope
        gyro.step();

        // Copy data with mutex protection
        if (xSemaphoreTake(xGyroDataMutex, portMAX_DELAY) == pdTRUE) {
            gyro.get(gyroData);
            xSemaphoreGive(xGyroDataMutex);
        }

        // Read data from barometer
        barometer.step();

        // Copy barometer data with mutex protection
        if (xSemaphoreTake(xBaroDataMutex, portMAX_DELAY) == pdTRUE) {
            baroData[0] = barometer.getAltitude();
            baroData[1] = barometer.getPressure();
            baroData[2] = (float)barometer.getTemperature() / 100.0f;  // Convert to degrees Celsius
            xSemaphoreGive(xBaroDataMutex);
        }

        // Delay for 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task to print data from sensors
void TaskPrintSensors(void* pvParameters) {
    (void) pvParameters;

    for (;;) {
        // Print accelerometer data
        if (xSemaphoreTake(xAccelDataMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print(F("Accel: "));
            Serial.print(accelData[0]);
            Serial.print(F(", "));
            Serial.print(accelData[1]);
            Serial.print(F(", "));
            Serial.print(accelData[2]);
            Serial.print(F(" ("));
            Serial.print(sqrtf(accelData[0] * accelData[0] + accelData[1] * accelData[1] + accelData[2] * accelData[2]));
            Serial.println(F(") m/s^2"));
            xSemaphoreGive(xAccelDataMutex);
        }

        // Print gyroscope data
        if (xSemaphoreTake(xGyroDataMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print(F("Gyro: "));
            Serial.print(gyroData[0]);
            Serial.print(F(", "));
            Serial.print(gyroData[1]);
            Serial.print(F(", "));
            Serial.print(gyroData[2]);
            Serial.println(F(" degree/s"));
            xSemaphoreGive(xGyroDataMutex);
        }

        // Print barometer data
        if (xSemaphoreTake(xBaroDataMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print(F("Altitude: "));
            Serial.print(baroData[0]);
            Serial.println(F(" m"));

            Serial.print(F("Pressure: "));
            Serial.print(baroData[1]);
            Serial.println(F(" Pa"));

            Serial.print(F("Temp: "));
            Serial.print(baroData[2]);
            Serial.println(F(" °C"));
            xSemaphoreGive(xBaroDataMutex);
        }

        // Delay for 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}