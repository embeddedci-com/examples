# VL53L0X C library for STM32F103

This is a C port of [pololu's Arduino library](https://github.com/pololu/vl53l0x-arduino) for the ST VL53L0X time of flight sensor. It is targeted at the STM32F103 microcontroller but can be modified to be used with another µC.

For the I²C, UART and timing my own libraries are used. The following files are needed:
* usart1.h
* usart1.c
* init.h
* init.c
* i2c.h
* i2c.c

You can find them [here](https://github.com/MarcelMG/STM32F103C8T6).

### Hardware
A [VL53L0X breakout board](https://www.pololu.com/product/2490) can be purchased from Pololu's website.

Make the following connections between the STM32F103C8T6 and the VL53L0X board:

    STM32F103   VL53L0X board
    -------   -------------
        3V3 - VIN
        GND - GND
        PB6 - SCL
        PB7 - SDA
        PB5 - XSHUT (optional)
and don't forget the pullup resistors on SDA and SCL.

