#include <Arduino.h>
#include <SPI.h>
#include <STM32FreeRTOS.h>

// Include sensor and logging headers
#include "BMI088Accel.hpp"
#include "BMI088Gyro.hpp"
#include "BMP390.hpp"
#include "stm32pinouts.hpp"
#include "log.hpp"

// Conditional inclusion of USBSerial
#if defined(USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

// Sensor objects
BMI088Accel accel(ACCEL_CS_PIN);
BMI088Gyro gyro(GYRO_CS_PIN);
BMP390 barometer(BARO_CS_PIN);

// Task handles
TaskHandle_t taskReadSensorsHandle;
TaskHandle_t taskLogStepHandle;
TaskHandle_t taskConditionMonitorHandle;

// Logging control variables
volatile bool dropDetected = false;
volatile bool loggingActive = false;
volatile uint32_t loggingStartTime = 0;

// Function prototypes
void TaskReadSensors(void* pvParameters);
void TaskLogStep(void* pvParameters);
void TaskConditionMonitor(void* pvParameters);

void setup() {
    // Initialize Serial or USB Serial based on configuration
    #if defined(USBCON) && defined(USBD_USE_CDC)
        usb_serial.begin();
        while (!usb_serial) { ; } // Wait for USB Serial to initialize
    #else
        Serial.begin(115200);
        while (!Serial) { ; } // Wait for Serial to initialize
    #endif

    // Initialize SPI bus
    SPI.begin();

    // Initialize sensors
    accel.setup();
    gyro.setup();
    barometer.setup();

    // Initialize logging system
    log_setup();

    // Create FreeRTOS tasks
    xTaskCreate(TaskReadSensors, "ReadSensors", 2048, NULL, 2, &taskReadSensorsHandle);
    xTaskCreate(TaskLogStep, "LogStep", 2048, NULL, 2, &taskLogStepHandle);
    xTaskCreate(TaskConditionMonitor, "ConditionMonitor", 2048, NULL, 3, &taskConditionMonitorHandle);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
}

void loop() {
    // Empty loop since FreeRTOS handles tasks
}

// Task to read sensors and add data to log buffer if logging is active
void TaskReadSensors(void* pvParameters) {
    (void) pvParameters;

    for (;;) {
        if (loggingActive) {
            uint32_t time_ms = millis();

            // Temporary buffers to hold sensor data
            float localAccelData[3];
            float localGyroData[3];
            float temperature;
            float pressure;

            // Read accelerometer data
            accel.step();
            accel.get(localAccelData);

            // Read gyroscope data
            gyro.step();
            gyro.get(localGyroData);

            // Read barometer data
            barometer.step();
            temperature = barometer.getTemperature() / 100.0f; // Convert to °C
            pressure = barometer.getPressure();

            // Create log message
            LogMessage logMsg(time_ms,
                              localGyroData[0], localGyroData[1], localGyroData[2],
                              localAccelData[0], localAccelData[1], localAccelData[2],
                              temperature, pressure);

            // Add to log buffer
            log_add(logMsg);
        }

        // Delay to set data collection rate (e.g., every 100 ms)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task to process the log buffer and write to flash
void TaskLogStep(void* pvParameters) {
    (void) pvParameters;

    for (;;) {
        log_step();

        // Delay between log steps (adjust as needed)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Task to monitor Z acceleration and control logging
void TaskConditionMonitor(void* pvParameters) {
    (void) pvParameters;

    const float accelerationThreshold = 0.2f; // Threshold in g's (adjust as needed)
    const uint32_t loggingDuration = 25000;    // Logging duration in milliseconds (25 seconds)

    for (;;) {
        // Read Z-axis acceleration
        float accelData[3];
        accel.step();
        accel.get(accelData);

        float zAcceleration = accelData[2];

        // Check if Z acceleration is close to 0 (free-fall)
        if (!dropDetected && fabs(zAcceleration) <= accelerationThreshold) {
            dropDetected = true;
            loggingActive = true;
            loggingStartTime = millis();

            log_start();
            
            #if defined(USBCON) && defined(USBD_USE_CDC)
                usb_serial.println("Drop detected! Logging started.");
            #else
                Serial.println("Drop detected! Logging started.");
            #endif
        }

        // Check if logging duration has elapsed
        if (loggingActive && (millis() - loggingStartTime >= loggingDuration)) {
            loggingActive = false;
            log_stop();

            #if defined(USBCON) && defined(USBD_USE_CDC)
                usb_serial.println("Logging duration elapsed. Logging stopped.");
            #else
                Serial.println("Logging duration elapsed. Logging stopped.");
            #endif

            // Optionally, print all logs after logging stops
            log_print_all();
        }

        // Delay between checks (adjust as needed)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
