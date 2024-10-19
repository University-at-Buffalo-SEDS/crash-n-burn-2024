#include <Arduino.h>
#include <SPI.h>
#include <STM32FreeRTOS.h> 

#include "BMI088Accel.h"
#include "config.h"

#if defined(USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

// Create an instance of the BMI088Accel class using the CS pin
BMI088Accel accel(PB11);

// Shared data array for accelerometer readings
float accelData[3];

// Mutex for synchronizing access to accelData
SemaphoreHandle_t xAccelDataMutex;

// Task function declarations
void TaskReadAccel(void *pvParameters);
void TaskPrintAccel(void *pvParameters);

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

    // Setup the accelerometer
    accel.setup();

    // Create a mutex for accessing accelData
    xAccelDataMutex = xSemaphoreCreateMutex();
    if (xAccelDataMutex == NULL) {
        Serial.println(F("Error creating mutex"));
        while (1);
    }

    // Create the tasks
    xTaskCreate(TaskReadAccel, "ReadAccel", 256, NULL, 2, NULL);
    xTaskCreate(TaskPrintAccel, "PrintAccel", 256, NULL, 1, NULL);

    // Start the scheduler
    vTaskStartScheduler();
}

void loop() {
    // Empty. Tasks are now scheduled by FreeRTOS.
}

// Task to read accelerometer data
void TaskReadAccel(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        // Read data from the accelerometer
        accel.step();

        // Acquire mutex to write to shared data
        if (xSemaphoreTake(xAccelDataMutex, portMAX_DELAY) == pdTRUE) {
            float *data = accel.get();
            accelData[0] = data[0];
            accelData[1] = data[1];
            accelData[2] = data[2];
            xSemaphoreGive(xAccelDataMutex); // Release mutex
        }

        // Delay for 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task to print accelerometer data
void TaskPrintAccel(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        // Acquire mutex to read from shared data
        if (xSemaphoreTake(xAccelDataMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print(F("Accel: "));
            Serial.print(accelData[0]);
            Serial.print(F(", "));
            Serial.print(accelData[1]);
            Serial.print(F(", "));
            Serial.print(accelData[2]);
            Serial.println(F(" m/s^2"));
            xSemaphoreGive(xAccelDataMutex); // Release mutex
        }

        // Delay for 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
