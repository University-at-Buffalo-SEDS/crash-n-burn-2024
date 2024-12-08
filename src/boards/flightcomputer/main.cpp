#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>
#include <cmath>

// Include your sensor drivers
#include "BMI088Accel.hpp"
#include "BMI088Gyro.hpp"
#include "BMP390.hpp"

// Include configuration, Kalman filter, and related headers
#include "stm32pinouts.hpp"
#include "consts.hpp"
#include "kalman.hpp"
#include "avghistory.hpp"
// Assume that config.hpp defines channel_config, Channel, ChannelConfig, and other constants.

// If using USB for debugging
#if defined(USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

// Forward declarations of tasks
void TaskReadSensors(void* pvParameters);
void TaskPrintSensors(void* pvParameters);
void TaskDeployment(void* pvParameters);
void TaskChannel(void* pvParameters);

// Forward declarations of utility functions
void channel_fire(Channel chan);
uint32_t delta(uint32_t start, uint32_t end);

// Global data structures
BMI088Accel accel(ACCEL_CS_PIN);
BMI088Gyro gyro(GYRO_CS_PIN);
BMP390 barometer(BARO_CS_PIN);

float accelData[3];
float gyroData[3];
float baroData[3]; // altitude, pressure, temperature

// Mutexes for sensor data
SemaphoreHandle_t xAccelDataMutex;
SemaphoreHandle_t xGyroDataMutex;
SemaphoreHandle_t xBaroDataMutex;

struct ChannelConfig {
	int fire_pin;
};

enum class Channel {
	Drogue,
	Main,
	Count
};

constexpr std::array<ChannelConfig, (size_t)Channel::Count> channel_config = {
	ChannelConfig {PB5},
	ChannelConfig {PB6}
};
// Flight-related global variables
// Channel status
struct ChannelStatus {
    uint32_t fire_time;
    bool firing;
};
static std::array<ChannelStatus, channel_config.size()> channel_status;

// Mutex for channel status
SemaphoreHandle_t xChannelStatusMutex;

// Kalman Filter instance and mutex
#ifdef KALMAN_GAINS
static KalmanFilter kf(KALMAN_PERIOD / 1000.0f, {KALMAN_GAINS});
#else
static KalmanFilter kf(KALMAN_PERIOD / 1000.0f,
        ALTITUDE_SIGMA, ACCELERATION_SIGMA, MODEL_SIGMA);
#endif
SemaphoreHandle_t xKalmanMutex;

// Flight phase enumeration
enum class FlightPhase {
    Startup,
    Idle,
    Launched,
    DescendingWithDrogue,
    DescendingWithMain,
    Landed
};

// Additional global variables for flight logic
static bool launched = false;
static FlightPhase phase = FlightPhase::Startup;

// For estimating gravity and ground level
static AvgHistory<float, EST_HISTORY_SAMPLES, 3> gravity_est_state;
static AvgHistory<float, EST_HISTORY_SAMPLES, 3> ground_level_est_state;

static uint32_t land_time = 0;
static kfloat_t apogee = 0.0f;

// Utility function for time delta
uint32_t delta(uint32_t start, uint32_t end) {
    if (end >= start) return end - start;
    return (UINT32_MAX - start) + end + 1;
}

void setup() {
#if defined(USBCON) && defined(USBD_USE_CDC)
    usb_serial.begin();
#else
    Serial.begin(9600);
#endif

    while (!Serial) { ; }

    // Initialize SPI and I2C
    SPI.begin();
    Wire.begin();

    // Setup sensors
    accel.setup();
    gyro.setup();
    barometer.setup();

    // Create mutexes
    xAccelDataMutex = xSemaphoreCreateMutex();
    xGyroDataMutex = xSemaphoreCreateMutex();
    xBaroDataMutex = xSemaphoreCreateMutex();
    xChannelStatusMutex = xSemaphoreCreateMutex();
    xKalmanMutex = xSemaphoreCreateMutex();

    if (xAccelDataMutex == NULL || xGyroDataMutex == NULL || xBaroDataMutex == NULL ||
        xChannelStatusMutex == NULL || xKalmanMutex == NULL) {
        Serial.println(F("Error creating mutexes"));
        while (1);
    }

    // Initialize channel status
    if (xSemaphoreTake(xChannelStatusMutex, portMAX_DELAY) == pdTRUE) {
        for (auto &status : channel_status) {
            status.firing = false;
            status.fire_time = 0;
        }
        xSemaphoreGive(xChannelStatusMutex);
    }

    // Create tasks
    xTaskCreate(TaskReadSensors, "ReadSensors", 256, NULL, 2, NULL);
    xTaskCreate(TaskPrintSensors, "PrintSensors", 256, NULL, 1, NULL);
    xTaskCreate(TaskDeployment, "Deployment", 512, NULL, 2, NULL);
    xTaskCreate(TaskChannel, "Channel", 256, NULL, 2, NULL);

    // Start the scheduler
    vTaskStartScheduler();
}

void loop() {
    // FreeRTOS scheduler handles tasks
}

void TaskReadSensors(void* pvParameters) {
    (void) pvParameters;
    for (;;) {
        // Update accelerometer
        accel.step();
        if (xSemaphoreTake(xAccelDataMutex, portMAX_DELAY) == pdTRUE) {
            accel.get(accelData);
            xSemaphoreGive(xAccelDataMutex);
        }

        // Update gyroscope
        gyro.step();
        if (xSemaphoreTake(xGyroDataMutex, portMAX_DELAY) == pdTRUE) {
            gyro.get(gyroData);
            xSemaphoreGive(xGyroDataMutex);
        }

        // Update barometer
        barometer.step();
        if (xSemaphoreTake(xBaroDataMutex, portMAX_DELAY) == pdTRUE) {
            baroData[0] = barometer.getAltitude();
            baroData[1] = barometer.getPressure();
            baroData[2] = (float)barometer.getTemperature() / 100.0f;
            xSemaphoreGive(xBaroDataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void TaskPrintSensors(void* pvParameters) {
    (void) pvParameters;
    for (;;) {
        // Print accelerometer data
        if (xSemaphoreTake(xAccelDataMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print(F("Accel: "));
            Serial.print(accelData[0]); Serial.print(F(", "));
            Serial.print(accelData[1]); Serial.print(F(", "));
            Serial.print(accelData[2]); Serial.print(F(" ("));
            Serial.print(sqrtf(accelData[0]*accelData[0] + accelData[1]*accelData[1] + accelData[2]*accelData[2]));
            Serial.println(F(") m/s^2"));
            xSemaphoreGive(xAccelDataMutex);
        }

        // Print gyroscope data
        if (xSemaphoreTake(xGyroDataMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print(F("Gyro: "));
            Serial.print(gyroData[0]); Serial.print(F(", "));
            Serial.print(gyroData[1]); Serial.print(F(", "));
            Serial.print(gyroData[2]);
            Serial.println(F(" deg/s"));
            xSemaphoreGive(xGyroDataMutex);
        }

        // Print barometer data
        if (xSemaphoreTake(xBaroDataMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print(F("Altitude: "));
            Serial.print(baroData[0]); Serial.println(F(" m"));

            Serial.print(F("Pressure: "));
            Serial.print(baroData[1]); Serial.println(F(" Pa"));

            Serial.print(F("Temp: "));
            Serial.print(baroData[2]); Serial.println(F(" °C"));
            xSemaphoreGive(xBaroDataMutex);
        }

        // Print Kalman Filter estimates
        if (xSemaphoreTake(xKalmanMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print(F("Kalman [pos, rate, accel]: "));
            Serial.print(kf.getPosition()); Serial.print("m, ");
            Serial.print(kf.getVelocity()); Serial.print("m/s, ");
            Serial.print(kf.getAcceleration()); Serial.println("m/s^2");
            xSemaphoreGive(xKalmanMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void TaskDeployment(void* pvParameters) {
    (void) pvParameters;

    for (;;) {
        float currentAccel[3];
        float currentBaro[3];

        // Retrieve current sensor data
        if (xSemaphoreTake(xAccelDataMutex, portMAX_DELAY) == pdTRUE) {
            currentAccel[0] = accelData[0];
            currentAccel[1] = accelData[1];
            currentAccel[2] = accelData[2];
            xSemaphoreGive(xAccelDataMutex);
        }                                                           

        if (xSemaphoreTake(xBaroDataMutex, portMAX_DELAY) == pdTRUE) {
            currentBaro[0] = baroData[0]; // Altitude
            currentBaro[1] = baroData[1]; // Pressure
            currentBaro[2] = baroData[2]; // Temperature
            xSemaphoreGive(xBaroDataMutex);
        }

        float raw_alt = currentBaro[0];
        float accel_mag = sqrtf(currentAccel[0]*currentAccel[0] + currentAccel[1]*currentAccel[1] + currentAccel[2]*currentAccel[2]);

        // Handle phases
        if (phase < FlightPhase::Launched) {
            gravity_est_state.add(accel_mag);
            ground_level_est_state.add(raw_alt);
        }

        if (phase == FlightPhase::Startup) {
            if (!ground_level_est_state.full() || !gravity_est_state.full()) {
                vTaskDelay(pdMS_TO_TICKS(KALMAN_PERIOD));
                continue;
            }
            phase = FlightPhase::Idle;
        }

        accel_mag -= gravity_est_state.old_avg();
        float alt = raw_alt - ground_level_est_state.old_avg();

        // Check if any channel is firing
        bool any_channel_firing = false;
        if (xSemaphoreTake(xChannelStatusMutex, portMAX_DELAY) == pdTRUE) {
            for (const ChannelStatus &s : channel_status) {
                if (s.firing) {
                    any_channel_firing = true;
                    break;
                }
            }
            xSemaphoreGive(xChannelStatusMutex);
        }

        // Update Kalman Filter if no channel firing spike
        if (!any_channel_firing) {
            if (xSemaphoreTake(xKalmanMutex, portMAX_DELAY) == pdTRUE) {
                if (!kf.update(accel_mag, alt)) {
                    Serial.println(F("KalmanFilter: Update failed."));
                }
                xSemaphoreGive(xKalmanMutex);
            }
        }

        // Access Kalman state
        kfloat_t pos, vel, accel_est;
        if (xSemaphoreTake(xKalmanMutex, portMAX_DELAY) == pdTRUE) {
            pos = kf.getPosition();
            vel = kf.getVelocity();
            accel_est = kf.getAcceleration();
            xSemaphoreGive(xKalmanMutex);
        }

        // Flight logic
        if (phase == FlightPhase::Idle) {
            // Detect launch
            if (vel > LAUNCH_VELOCITY && accel_est > LAUNCH_ACCEL) {
                phase = FlightPhase::Launched;
                launched = true;
            }
        } else if (phase == FlightPhase::Launched) {
            // Detect apogee
            if (vel < 0) {
                apogee = pos;
                channel_fire(Channel::Drogue);

                phase = FlightPhase::DescendingWithDrogue;

                Serial.println(F("===================================== Apogee!"));
            }
        } else if (phase == FlightPhase::DescendingWithDrogue) {
            // Deploy main if conditions are met
            uint32_t now = millis();
            uint32_t drogue_fire_time = 0;
            if (xSemaphoreTake(xChannelStatusMutex, portMAX_DELAY) == pdTRUE) {
                drogue_fire_time = channel_status[(size_t)Channel::Drogue].fire_time;
                xSemaphoreGive(xChannelStatusMutex);
            }

            if ((pos < MAIN_DEPLOY_ALTITUDE
#ifdef FAILSAFE_VELOCITY
                 || vel < -FAILSAFE_VELOCITY
#endif
                ) && delta(drogue_fire_time, now) > 3000) {
                phase = FlightPhase::DescendingWithMain;
                channel_fire(Channel::Main);

                Serial.println(F("===================================== Deploy main!"));
            }
        } else if (phase == FlightPhase::DescendingWithMain) {
            if (pos < LANDED_ALT && fabs(vel) < LANDED_VELOCITY && fabs(accel_est) < LANDED_ACCEL) {
                if (land_time == 0) {
                    land_time = millis();
                    if (land_time == 0) land_time = 1;
                } else if (delta(land_time, millis()) > LANDED_TIME_MS) {
                    phase = FlightPhase::Landed;
                    Serial.println(F("===================================== Landed!"));
                }
            } else {
                land_time = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(KALMAN_PERIOD));
    }
}

void TaskChannel(void* pvParameters) {
    (void) pvParameters;
    for (;;) {
        uint32_t now = millis();
        // Manage channel firing timeouts
        if (xSemaphoreTake(xChannelStatusMutex, portMAX_DELAY) == pdTRUE) {
            for (size_t i = 0; i < channel_status.size(); ++i) {
                ChannelStatus &status = channel_status[i];
                const ChannelConfig &config = channel_config[i];
                if (status.firing && delta(status.fire_time, now) > CHANNEL_FIRE_TIME) {
                    status.firing = false;
                    digitalWrite(config.fire_pin, LOW);
                }
            }
            xSemaphoreGive(xChannelStatusMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Fire a specific channel (e.g., drogue or main)
void channel_fire(Channel chan) {
    if (xSemaphoreTake(xChannelStatusMutex, portMAX_DELAY) == pdTRUE) {
        ChannelStatus &status = channel_status[(size_t)chan];
        status.firing = true;
        status.fire_time = millis();
        digitalWrite(channel_config[(size_t)chan].fire_pin, HIGH);
        xSemaphoreGive(xChannelStatusMutex);
    }
}
