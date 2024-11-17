#include <cstddef>
#include <cstdint>

// Ensure the struct is packed without padding
struct __attribute__((packed)) LogMessage {
    uint32_t time_ms;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float temp;
    float pressure;
    uint8_t checksum;

    LogMessage() = default;

    LogMessage(uint32_t time_ms,
               float gyro_x, float gyro_y, float gyro_z,
               float accel_x, float accel_y, float accel_z,
               float temp, float pressure)
        : time_ms(time_ms),
          gyro_x(gyro_x), gyro_y(gyro_y), gyro_z(gyro_z),
          accel_x(accel_x), accel_y(accel_y), accel_z(accel_z),
          temp(temp), pressure(pressure), checksum(0)
    {
        checksum = calculate_checksum();
    }

    uint8_t calculate_checksum() const {
        const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
        uint8_t sum = 0;
        // Exclude checksum field from calculation
        for (size_t i = 0; i < sizeof(LogMessage) - sizeof(checksum); ++i) {
            sum ^= data[i];
        }
        return sum;
    }
};

// Logging control functions
void log_setup();
void log_start();
void log_stop();
void log_add(const LogMessage& data);
void log_step();
void log_print_all();
