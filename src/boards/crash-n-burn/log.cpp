#include "log.hpp"
#include "stm32pinouts.hpp"
#include "FlashMemory.hpp"
#include "RingBuffer.hpp"
#include <Arduino.h>
#include <EEPROM.h>
#include <cassert>

// Define EEPROM address for flight number storage
#define EEPROM_FLIGHT_ADDR 0

// Adjust buffer sizes as needed
#define LOG_BUF_SIZE 50 // Number of LogMessages in buffer
#define LOG_WRITE_BUF_SIZE (LOG_BUF_SIZE * sizeof(LogMessage))

// Flash memory and buffers
FlashMemory flash(FLASH_CS_PIN);
RingBuffer<LogMessage, LOG_BUF_SIZE> log_buf;
RingBuffer<uint8_t, LOG_WRITE_BUF_SIZE> write_buf;

// Logging state variables
static bool write_enabled = false;
static size_t current_page = 0;
static size_t written_pages = 0;
static uint8_t flight_num = 0;
static bool current_block_erased = false;

void log_setup() {
    flash.setup();

    // Read the last flight number from EEPROM
    flight_num = EEPROM.read(EEPROM_FLIGHT_ADDR) % FlashMemory::FLIGHT_FLASH_FLIGHTS;
    current_page = FlashMemory::FLIGHT_FLASH_FLIGHT_PAGES * flight_num;
    written_pages = 0;

    Serial.println(F("Log system initialized."));
}

void log_start() {
    if (!write_enabled) {
        write_enabled = true;
        current_page = FlashMemory::FLIGHT_FLASH_FLIGHT_PAGES * flight_num;
        written_pages = 0;
        current_block_erased = false;

        // Advance flight number for next time
        flight_num = (flight_num + 1) % FlashMemory::FLIGHT_FLASH_FLIGHTS;
        EEPROM.write(EEPROM_FLIGHT_ADDR, flight_num);

        Serial.println(F("Logging started."));
    }
}

void log_stop() {
    if (write_enabled) {
        write_enabled = false;
        Serial.println(F("Logging stopped."));
    }
}

void log_add(const LogMessage& data) {
    if (write_enabled) {
        if (!log_buf.push(data, true)) {
            Serial.println(F("Log buffer overflow!"));
        }
    }
}

void log_step() {
    if (!write_enabled || written_pages >= FlashMemory::FLIGHT_FLASH_FLIGHT_PAGES) {
        return;
    }

    // Move messages from log buffer to write buffer
    LogMessage temp;
    while (write_buf.available() >= sizeof(LogMessage)) {
        if (!log_buf.pop(&temp)) {
            break;
        }
        uint8_t* temp_bytes = reinterpret_cast<uint8_t*>(&temp);
        bool ok = write_buf.push(temp_bytes, sizeof(LogMessage), false);
        assert(ok);
    }

    // Write pages from write buffer to flash
    uint8_t page[FlashMemory::FLIGHT_FLASH_PAGE_SIZE];
    while (written_pages < FlashMemory::FLIGHT_FLASH_FLIGHT_PAGES && !flash.isBusy()) {
        // Erase new block if necessary
        if (!current_block_erased && (current_page % FlashMemory::FLIGHT_FLASH_PAGES_PER_BLOCK == 0)) {
            flash.erase(current_page * FlashMemory::FLIGHT_FLASH_PAGE_SIZE);
            current_block_erased = true;
            break; // Wait until next call after erase
        }
        current_block_erased = false;

        // If not enough data to fill a page, wait
        if (write_buf.used() < FlashMemory::FLIGHT_FLASH_PAGE_SIZE) {
            break;
        }

        // Pop data into page buffer
        if (!write_buf.pop(page, FlashMemory::FLIGHT_FLASH_PAGE_SIZE)) {
            break;
        }

        // Write page to flash
        flash.write(current_page * FlashMemory::FLIGHT_FLASH_PAGE_SIZE, page);
        current_page++;
        written_pages++;
    }
}


static void log_print_msg(const LogMessage& msg) {
    Serial.print(msg.time_ms); Serial.print(',');
    Serial.print(msg.gyro_x, 2); Serial.print(',');
    Serial.print(msg.gyro_y, 2); Serial.print(',');
    Serial.print(msg.gyro_z, 2); Serial.print(',');
    Serial.print(msg.accel_x, 2); Serial.print(',');
    Serial.print(msg.accel_y, 2); Serial.print(',');
    Serial.print(msg.accel_z, 2); Serial.print(',');
    Serial.print(msg.temp, 2); Serial.print(',');
    Serial.print(msg.pressure, 2);
    Serial.println();
}


static void log_print_flight(uint8_t flight) {
    uint8_t page[FlashMemory::FLIGHT_FLASH_PAGE_SIZE];
    RingBuffer<uint8_t, LOG_WRITE_BUF_SIZE> read_buf;
    size_t flight_start_page = flight * FlashMemory::FLIGHT_FLASH_FLIGHT_PAGES;

    for (size_t page_offset = 0; page_offset < FlashMemory::FLIGHT_FLASH_FLIGHT_PAGES; page_offset++) {
        size_t page_addr = (flight_start_page + page_offset) * FlashMemory::FLIGHT_FLASH_PAGE_SIZE;
        flash.read(page_addr, page);

        if (!read_buf.push(page, FlashMemory::FLIGHT_FLASH_PAGE_SIZE, false)) {
            Serial.println(F("Read buffer overflow."));
            break;
        }

        LogMessage msg;
        while (read_buf.pop(reinterpret_cast<uint8_t*>(&msg), sizeof(LogMessage))) {
            // Validate checksum
            if (msg.checksum != msg.calculate_checksum()) {
                Serial.println(F("Checksum mismatch. Stopping print."));
                return;
            }

            log_print_msg(msg);
        }
    }
}

void log_print_all() {
    if (write_enabled) {
        Serial.println(F("Cannot print logs while logging is active."));
        return;
    }

    Serial.println(F("Printing all flight logs..."));
    for (uint8_t flight = 0; flight < FlashMemory::FLIGHT_FLASH_FLIGHTS; flight++) {
        Serial.print(F("Flight "));
        Serial.println(flight);
        log_print_flight(flight);
        Serial.println(F("---"));
    }
}
