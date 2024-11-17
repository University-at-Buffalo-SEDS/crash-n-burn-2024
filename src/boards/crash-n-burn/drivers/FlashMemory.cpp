#include "FlashMemory.hpp"
#include <cassert>

const SPISettings FlashMemory::spi_settings(18000000, MSBFIRST, SPI_MODE0);

// Constructor
FlashMemory::FlashMemory(uint8_t cs_pin) : cs_pin(cs_pin) {
    // Initialize CS pin as OUTPUT and set HIGH (deselected)
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
}

// Setup method: Initialize SPI and verify device ID
void FlashMemory::setup() {
    // SPI is already initialized in main.cpp via SPI.begin()
    Serial.println(F("Initializing Flash..."));

    spi_begin();
    sendCommand(FlashInstruction::RELEASE_POWER_DOWN_DEVICE_ID);
    for (size_t i = 0; i < 3; ++i) {
        SPI.transfer(0x00); // Send dummy bytes
    }
    uint8_t device_id = SPI.transfer(0x00);
    spi_end();

    Serial.print(F("Flash Device ID: 0x"));
    Serial.println(device_id, HEX);

    if (device_id != DEVICE_ID) {
        Serial.println(F("Error: Incorrect Flash Device ID!"));
        abort(); // Or handle error appropriately
    } else {
        Serial.println(F("Flash detected."));
    }
}

// spi_begin: starts SPI transaction and selects the flash device
void FlashMemory::spi_begin() {
    SPI.beginTransaction(spi_settings);
    digitalWrite(cs_pin, LOW);
}

// spi_end: deselects the flash device and ends SPI transaction
void FlashMemory::spi_end() {
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();
}

// sendCommand: sends a flash instruction
void FlashMemory::sendCommand(FlashInstruction instruction) {
    SPI.transfer(static_cast<uint8_t>(instruction));
}

// flash_status_1: reads the status register 1
uint32_t FlashMemory::flash_status_1() {
    spi_begin();
    sendCommand(FlashInstruction::READ_STATUS_REGISTER_1);
    uint8_t status = SPI.transfer(0x00);
    spi_end();
    return status;
}

// flash_busy_internal: checks if flash is busy
bool FlashMemory::flash_busy_internal() {
    return (flash_status_1() & BUSY_MASK) != 0;
}

// isBusy: public method to check busy status
bool FlashMemory::isBusy() {
    return flash_busy_internal();
}

// write_enable: enables writing
void FlashMemory::write_enable() {
    assert(!flash_busy_internal());

    Serial.println(F("Enabling Write..."));
    spi_begin();
    sendCommand(FlashInstruction::WRITE_ENABLE);
    spi_end();

    uint32_t status = flash_status_1();
    if (!(status & WEL_MASK)) {
        Serial.println(F("Error: Write Enable Failed!"));
        abort();
    } else {
        Serial.println(F("Write Enabled."));
    }
}

// waitUntilNotBusy: waits until flash is not busy
void FlashMemory::waitUntilNotBusy() {
    Serial.println(F("Waiting for Flash to be ready..."));
    while (flash_busy_internal()) {
        delay(1); // Small delay to prevent tight loop
    }
    Serial.println(F("Flash is ready."));
}

// erase: erases a 32KB block at the specified page address
void FlashMemory::erase(size_t page_addr) {
    write_enable();

    Serial.print(F("Erasing block at address: 0x"));

    Serial.println(page_addr, HEX);

    spi_begin();
    sendCommand(FlashInstruction::BLOCK_ERASE_32KB);

    // Send 24-bit address
    SPI.transfer((page_addr >> 16) & 0xFF); // Address high byte
    SPI.transfer((page_addr >> 8) & 0xFF);  // Address middle byte
    SPI.transfer(page_addr & 0xFF);         // Address low byte
    spi_end();

    assert(flash_busy_internal());
    waitUntilNotBusy();

    Serial.println(F("Erase Complete."));
}

// write: writes a page to flash memory
void FlashMemory::write(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE]) {
    write_enable();

    Serial.print(F("Writing page at address: 0x"));
    Serial.println(page_addr, HEX);

    spi_begin();
    sendCommand(FlashInstruction::PAGE_PROGRAM);

    // Send 24-bit address
    SPI.transfer((page_addr >> 16) & 0xFF);
    SPI.transfer((page_addr >> 8) & 0xFF);
    SPI.transfer(page_addr & 0xFF);

    // Send page data
    for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; i++) {
        SPI.transfer(page[i]);
    }
    spi_end();

    assert(flash_busy_internal());
    waitUntilNotBusy();

#ifndef NDEBUG
    // Validate write
    uint8_t read_data[FLIGHT_FLASH_PAGE_SIZE];
    read(page_addr, read_data);
    for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; i++) {
        if (read_data[i] != page[i]) {
            Serial.print(F("Write verification failed at byte "));
            Serial.print(i);
            Serial.print(F(": wrote 0x"));
            Serial.print(page[i], HEX);
            Serial.print(F(", read 0x"));
            Serial.println(read_data[i], HEX);
            abort();
        }
    }
    Serial.println(F("Write verification passed."));
#endif
}

// read: reads a page from flash memory
void FlashMemory::read(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE]) {
    assert(!flash_busy_internal());

    Serial.print(F("Reading page at address: 0x"));
    Serial.println(page_addr, HEX);

    spi_begin();
    sendCommand(FlashInstruction::READ_DATA);

    // Send 24-bit address
    SPI.transfer((page_addr >> 16) & 0xFF);
    SPI.transfer((page_addr >> 8) & 0xFF);
    SPI.transfer(page_addr & 0xFF);

    // Read data
    for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; i++) {
        page[i] = SPI.transfer(0x00);
    }
    spi_end();

    Serial.println(F("Read Complete."));
}
