#include <Arduino.h>
#include <SPI.h>
#include <STM32FreeRTOS.h> // Ensure FreeRTOS is installed
#include "BMI088.h"

// Create an instance of the BMI088 class
BMI088 bmi088;

// Mutex for protecting access to accelerometer data
SemaphoreHandle_t accelMutex;

// Shared accelerometer data
float accelData[3] = {0.0f, 0.0f, 0.0f};

// Task Handles
TaskHandle_t SensorTaskHandle = NULL;
TaskHandle_t SerialTaskHandle = NULL;

// Task Prototypes
void SensorTask(void *pvParameters);
void SerialTaskFunction(void *pvParameters);

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    while (!Serial) { ; } // Wait for Serial to initialize

    // Initialize BMI088
    if (!bmi088.begin()) {
        Serial.println(F("Failed to initialize BMI088"));
        // Handle initialization failure
        while (1) { ; }
    }

    // Create Mutex
    accelMutex = xSemaphoreCreateMutex();
    if (accelMutex == NULL) {
        Serial.println(F("Failed to create mutex"));
        while (1) { ; }
    }

    // Create Sensor Task
    xTaskCreate(
        SensorTask,         // Task function
        "Sensor Task",      // Task name
        256,                // Stack size (words)
        NULL,               // Task input parameter
        1,                  // Priority
        &SensorTaskHandle   // Task handle
    );

    // Create Serial Task
    xTaskCreate(
        SerialTaskFunction,
        "Serial Task",
        256,
        NULL,
        1,
        &SerialTaskHandle
    );

    // Start the scheduler
    vTaskStartScheduler();
}

void loop() {
    // Empty. Tasks are running in FreeRTOS.
}

// Sensor Task: Reads accelerometer data periodically
void SensorTask(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        float tempData[3];
        if (bmi088.readAccelerometer(tempData)) {
            // Acquire Mutex
            if (xSemaphoreTake(accelMutex, (TickType_t)10) == pdTRUE) {
                // Update shared data
                accelData[0] = tempData[0];
                accelData[1] = tempData[1];
                accelData[2] = tempData[2];
                // Release Mutex
                xSemaphoreGive(accelMutex);
            }
        } else {
            Serial.println(F("Failed to read accelerometer data"));
        }

        // Delay for 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Serial Task: Outputs accelerometer data over Serial
void SerialTaskFunction(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        float currentData[3];

        // Acquire Mutex
        if (xSemaphoreTake(accelMutex, (TickType_t)10) == pdTRUE) {
            // Copy data
            currentData[0] = accelData[0];
            currentData[1] = accelData[1];
            currentData[2] = accelData[2];
            // Release Mutex
            xSemaphoreGive(accelMutex);
        }

        // Output data
        Serial.print(F("Accel X: "));
        Serial.print(currentData[0], 3);
        Serial.print(F(" g, Y: "));
        Serial.print(currentData[1], 3);
        Serial.print(F(" g, Z: "));
        Serial.print(currentData[2], 3);
        Serial.println(F(" g"));

        // Calculate magnitude
        float magnitude = sqrt(pow(currentData[0], 2) + pow(currentData[1], 2) + pow(currentData[2], 2));
        Serial.print(F("Magnitude: "));
        Serial.print(magnitude, 3);
        Serial.println(F(" g"));

        // Delay for 500ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
