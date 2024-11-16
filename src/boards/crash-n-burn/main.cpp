#include <Arduino.h>
#include <SPI.h>
#include <STM32FreeRTOS.h>

#include "BMI088Accel.h"
#include "BMI088Gyro.h"
#include "BMP390.h"           
#include "stm32pinouts.h"
#include "FlashMemory.h"

#if defined(USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

BMI088Accel accel(ACCEL_CS_PIN);  
BMI088Gyro gyro(GYRO_CS_PIN);    
BMP390 barometer(BARO_CS_PIN);    
FlashMemory flash(FLASH_CS_PIN);

float accelData[3];
float gyroData[3];
float baroData[3];  

SemaphoreHandle_t xAccelDataMutex;
SemaphoreHandle_t xGyroDataMutex;
SemaphoreHandle_t xBaroDataMutex;  

void TaskReadSensors(void* pvParameters);
void TaskPrintSensors(void* pvParameters);

void setup() {
    #if defined(USBCON) && defined(USBD_USE_CDC)
        usb_serial.begin();
    #else
        Serial.begin(9600);
    #endif

    while (!Serial) { ; } 

    SPI.begin();

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

// Flash testing code

/*
#include <Arduino.h>
#include "FlashMemory.h"
#include "BMI088Gyro.h"

// Flash and gyro setup
FlashMemory flash(FLASH_CS_PIN);
BMI088Gyro gyro(GYRO_CS_PIN);

// Define the size of the data to write and read
constexpr size_t NUM_READINGS = 1024; // Number of gyro readings
constexpr size_t DATA_SIZE = NUM_READINGS * 3 * sizeof(float); // Total size in bytes

// Buffer for gyro data
float gyroData[NUM_READINGS][3];
uint8_t writeBuffer[DATA_SIZE];
uint8_t readBuffer[DATA_SIZE];

void setup() {
    Serial.begin(115200);
    while (!Serial) { }

    // Initialize flash and gyro
    flash.setup();
    gyro.setup();

    // Collect gyroscope data
    Serial.println(F("Collecting gyroscope data..."));
    for (size_t i = 0; i < NUM_READINGS; i++) {
        gyro.step();
        gyro.get(gyroData[i]);
        delay(10); // Simulate a short delay between readings
    }

    // Pack the gyro data into the write buffer
    Serial.println(F("Packing data for writing..."));
    for (size_t i = 0; i < NUM_READINGS; i++) {
        memcpy(&writeBuffer[i * 3 * sizeof(float)], gyroData[i], 3 * sizeof(float));
    }

    // Write the data to flash
    Serial.println(F("Writing data to flash..."));
    for (size_t page = 0; page < (DATA_SIZE / FlashMemory::FLIGHT_FLASH_PAGE_SIZE); page++) {
        flash.write(page * FlashMemory::FLIGHT_FLASH_PAGE_SIZE, &writeBuffer[page * FlashMemory::FLIGHT_FLASH_PAGE_SIZE]);
    }

    // Read the data back from flash
    Serial.println(F("Reading data from flash..."));
    for (size_t page = 0; page < (DATA_SIZE / FlashMemory::FLIGHT_FLASH_PAGE_SIZE); page++) {
        flash.read(page * FlashMemory::FLIGHT_FLASH_PAGE_SIZE, &readBuffer[page * FlashMemory::FLIGHT_FLASH_PAGE_SIZE]);
    }

    // Unpack and display the data
    Serial.println(F("Displaying read data..."));
    for (size_t i = 0; i < NUM_READINGS; i++) {
        float readGyroData[3];
        memcpy(readGyroData, &readBuffer[i * 3 * sizeof(float)], 3 * sizeof(float));
        Serial.print("Rotation (X, Y, Z): ");
        Serial.print(readGyroData[0], 2);
        Serial.print(", ");
        Serial.print(readGyroData[1], 2);
        Serial.print(", ");
        Serial.println(readGyroData[2], 2);
    }
}

void loop() {
    // No additional logic in the loop
}
*/