# UB SEDS Avionics Firmware 2024-25
## Device Drivers 
This is not a comprehensive list, just the driver that we currently have on boards that need to be converted and tested with FreeRTOS
- [] LTC Driver
- [] Flash Diver
- [] BMP Driver
- [] GPS Driver
- [] APRS Driver
- [] Buzzer Driver
- [] Payload ADC Driver (if we get to this point D:)

## CAN Bus
1. First step will be getting a basic CAN Bus implementation working between two boards. This might look like telling a board to print something over serial
2. Then we work ourselves up to collecting data off sensors and printing that over serial on another board 
3. Then getting this all over FreeRTOS will the be the final step

## Flight Software
Lots of logic and understanding what the previous flight computer did. This also means understanding how to reimplement the Kalman filter.