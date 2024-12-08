#include <Arduino.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>

#include "LTC2990.hpp"

#if !defined(HAL_CAN_MODULE_ENABLED)
#define HAL_CAN_MODULE_ENABLED
#endif

#if defined(USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

// Create an instance of the LTC2990 device
LTC2990 ltc2990;

// Shared data structure
float ltcVoltages[4];

// Mutex for thread safety
SemaphoreHandle_t xLTCDataMutex;

// Task function declarations
void TaskReadLTC(void* pvParameters);
void TaskPrintLTC(void* pvParameters);

void setup() {
    // Initialize serial communication
    #if defined(USBCON) && defined(USBD_USE_CDC)
        usb_serial.begin();
    #else
        Serial.begin(9600);
    #endif

    while (!Serial) { ; }

    Wire.begin();

    // Setup the device
    ltc2990.setup();

    // Create mutex for shared data
    xLTCDataMutex = xSemaphoreCreateMutex();

    if (xLTCDataMutex == NULL) {
        Serial.println(F("Error creating mutex"));
        while (1);
    }

    xTaskCreate(TaskReadLTC, "ReadLTC", 256, NULL, 2, NULL);
    xTaskCreate(TaskPrintLTC, "PrintLTC", 256, NULL, 1, NULL);

    vTaskStartScheduler();
}

void loop() {
    // Empty. Tasks are now scheduled by FreeRTOS.
}

// Task to read data from LTC2990
void TaskReadLTC(void* pvParameters) {
    (void) pvParameters;

    for (;;) {
        // Read data from LTC2990
        ltc2990.step();

        // Copy data with mutex protection
        if (xSemaphoreTake(xLTCDataMutex, portMAX_DELAY) == pdTRUE) {
            ltc2990.get(ltcVoltages);
            xSemaphoreGive(xLTCDataMutex);
        }

        // Delay for 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task to print data from LTC2990
void TaskPrintLTC(void* pvParameters) {
    (void) pvParameters;

    for (;;) {
        // Print LTC2990 data
        if (xSemaphoreTake(xLTCDataMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print(F("Voltages: "));
            for (int i = 0; i < 4; i++) {
                Serial.print(F("V"));
                Serial.print(i + 1);
                Serial.print(F(": "));
                Serial.print(ltcVoltages[i], 6);
                Serial.print(F(" V "));
            }
            Serial.println();
            xSemaphoreGive(xLTCDataMutex);
        }
    
        // Delay for 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
