#include <Arduino.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>

#define ARDUINO_NUCLEO_G474RE; //to make sure the stupid canfd library works
#include <ACANFD-STM32-programmable-ram-sections.h>

#include "LTC2990.h"


//why would you do this
// #if !defined(HAL_CAN_MODULE_ENABLED)
// #define HAL_CAN_MODULE_ENABLED
// #endif



#if defined(USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif
#include <ACANFD_STM32_Settings.h>

// Create an instance of the LTC2990 device
LTC2990 ltc2990;

// Shared data structure
float ltcVoltages[4];

// Mutex for thread safety
SemaphoreHandle_t xLTCDataMutex;

// Task function declarations
void TaskReadLTC(void* pvParameters);
void TaskPrintLTC(void* pvParameters);

const ACANFD_STM32::PinPort FDCAN2_TX_PIN_ARRAY_PB13 [1] {
    {PB_13, 9} //Tx pin: PB_13, alternate mapping AF9 maps to FDCAN2_TX
    //page 64 of stm32g491cet6 datasheet
};

const ACANFD_STM32::PinPort FDCAN2_RX_PIN_ARRAY_PB12 [1] {
    {PB_12, 9} //Tx pin: PB_13, alternate mapping AF9 maps to FDCAN2_TX
    //page 64 of stm32g491cet6 datasheet
};

ACANFD_STM32 fdcan2_PB12_13 {
    FDCAN2, //CAN Peripheral base address for fdcan2 controller
    SRAMCAN_BASE,
    FDCAN1_IT0_IRQn,
    FDCAN1_IT1_IRQn,
    FDCAN2_TX_PIN_ARRAY_PB13, //pin mapping array for tx
    1, //number of tx pins
    FDCAN2_RX_PIN_ARRAY_PB12, //pin mapping array for rx
    1 //number of rx pins
};

//do can bus setup with ACANFD_STM32 library
void CAN_BUS_INIT() {
    //arbitration data rate set to 500kb/s, data bit rate set to 1 x arbitration rate
    //constructing ACANFD_STM32_Settings enables the CAN clock and resets all CAN peripherals
    ACANFD_STM32_Settings settings (500 * 1000, DataBitRateFactor::x1); 

    //Normal operation mode
    settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD;

    //Tx Pin is configured in push/pull mode
    settings.mOpenCollectorOutput = false;

    //Rx pin pullup is disabled, CAN transciever has one already
    settings.mInputPullup = false;

    //Some output code from one of the examples
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Arbitration Phase segment 1: ") ;
    Serial.println (settings.mArbitrationPhaseSegment1) ;
    Serial.print ("Arbitration Phase segment 2: ") ;
    Serial.println (settings.mArbitrationPhaseSegment2) ;
    Serial.print ("Arbitration SJW: ") ;
    Serial.println (settings.mArbitrationSJW) ;
    Serial.print ("Actual Arbitration Bit Rate: ") ;
    Serial.print (settings.actualArbitrationBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Arbitration sample point: ") ;
    Serial.print (settings.arbitrationSamplePointFromBitStart ()) ;
    Serial.println ("%") ;
    Serial.print ("Exact Arbitration Bit Rate ? ") ;
    Serial.println (settings.exactArbitrationBitRate () ? "yes" : "no") ;
    Serial.print ("Data Phase segment 1: ") ;
    Serial.println (settings.mDataPhaseSegment1) ;
    Serial.print ("Data Phase segment 2: ") ;
    Serial.println (settings.mDataPhaseSegment2) ;
    Serial.print ("Data SJW: ") ;
    Serial.println (settings.mDataSJW) ;
    Serial.print ("Actual Data Bit Rate: ") ;
    Serial.print (settings.actualDataBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Data sample point: ") ;
    Serial.print (settings.dataSamplePointFromBitStart ()) ;
    Serial.println ("%") ;
    Serial.print ("Exact Data Bit Rate ? ") ;
    Serial.println (settings.exactDataBitRate () ? "yes" : "no") ;

    //Start CAN bus
    uint32_t err = fdcan1.beginFD(settings);


}

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
