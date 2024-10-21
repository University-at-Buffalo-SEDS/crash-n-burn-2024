#include <Arduino.h>

#if defined (USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

void setup() {
    #if defined (USBCON) && defined(USBD_USE_CDC)
	    usb_serial.begin();
    #else
	    Serial.begin(9600);
    #endif
    while (!Serial) { ; } // Wait for Serial to initialize
}

void loop() {
    Serial.println("Greetings from the digital world!");
}