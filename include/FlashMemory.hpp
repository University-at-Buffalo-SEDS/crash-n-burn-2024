#ifndef FLASH_MEMORY_HPP
#define FLASH_MEMORY_HPP

#include <cstddef>
#include <cstdint>
#include <Arduino.h>
#include <SPI.h>

class FlashMemory {
public:
    FlashMemory(uint8_t cs_pin);

    static constexpr size_t FLIGHT_FLASH_PAGE_SIZE = 256;
    static constexpr size_t FLIGHT_FLASH_BLOCK_SIZE = 65536;
    static constexpr size_t FLIGHT_FLASH_PAGES_PER_BLOCK = FLIGHT_FLASH_BLOCK_SIZE / FLIGHT_FLASH_PAGE_SIZE;
    static constexpr size_t FLIGHT_FLASH_PAGE_COUNT = (1 << 22) / FLIGHT_FLASH_PAGE_SIZE;
    static constexpr size_t FLIGHT_FLASH_BLOCK_COUNT = FLIGHT_FLASH_PAGE_COUNT / FLIGHT_FLASH_PAGES_PER_BLOCK;
    static constexpr size_t FLIGHT_FLASH_FLIGHTS = 1;
    static constexpr size_t FLIGHT_FLASH_FLIGHT_PAGES = FLIGHT_FLASH_PAGES_PER_BLOCK * (FLIGHT_FLASH_BLOCK_COUNT / FLIGHT_FLASH_FLIGHTS);
    static constexpr size_t FLIGHT_FLASH_FLIGHT_SIZE = FLIGHT_FLASH_FLIGHT_PAGES * FLIGHT_FLASH_PAGE_SIZE;
    
    void setup();
    void erase(size_t page_addr);
    void write(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE]);
    void read(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE]);
    bool isBusy();

private:
    enum class FlashInstruction : uint8_t {
        PAGE_PROGRAM = 0x02,
        READ_DATA = 0x03,
        READ_STATUS_REGISTER_1 = 0x05,
        WRITE_ENABLE = 0x06,
        BLOCK_ERASE_32KB = 0x52,
        RELEASE_POWER_DOWN_DEVICE_ID = 0xAB,
    };

    static constexpr uint8_t DEVICE_ID = 0x15;
    static constexpr uint8_t BUSY_MASK = 0x01;
    static constexpr uint8_t WEL_MASK = 0x02;
    static const SPISettings spi_settings;

    uint8_t cs_pin;

    void spi_begin();
    void spi_end();
    void sendCommand(FlashInstruction instruction);
    uint32_t flash_status_1();
    bool flash_busy_internal();
    void write_enable();
    void waitUntilNotBusy();
};

#endif