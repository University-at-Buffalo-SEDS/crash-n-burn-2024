#include <Arduino.h>
#include <SPI.h>
#include <STM32FreeRTOS.h>

#include "BMI088Accel.h"
#include "BMI088Gyro.h"
#include "config.h"

#if defined(USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

// Create instances of the devices
BMI088Accel accel(ACCEL_CS_PIN);  // ACCEL_CS_PIN defined in config.h
BMI088Gyro gyro(GYRO_CS_PIN);     // GYRO_CS_PIN defined in config.h

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
        // Read data from all devices
        accel.step();
        gyro.step();

        // Delay for 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task to print data from sensors
void TaskPrintSensors(void* pvParameters) {
    (void) pvParameters;

    for (;;) {
        // Print data from each device
        accel.print();
        gyro.print();

        // Delay for 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
