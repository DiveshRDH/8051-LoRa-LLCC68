# LoRa Library for LLCC68/SX1261/1262

A lightweight library for interfacing the LLCC68 LoRa module with Nuvoton 8051-based microcontrollers, specifically the MS51 series. Supports LoRa communication at 915 MHz for transmitting and receiving packets.

## Features
- Initialize and configure LLCC68 module
- Send and receive LoRa packets (synchronous and asynchronous)
- SPI and UART support for communication and debugging
- Optimized for Nuvoton MS51 microcontrollers

## Requirements
- Nuvoton MS51 microcontroller
- LLCC68 LoRa module
- Compiler supporting Nuvoton 8051 (e.g., Keil uVision)
- Hardware connections:
  - NSS: P1.5 (SPI Slave Select)
  - DIO1: P1.1 (Interrupt)
  - RESET: P1.3 (Reset)
- Note: Adjustments may be required for other 8051-based MCUs.

## Installation
1. Clone or download this repository.
2. Include `LoRa.h` and `LoRa.c` in your project.
3. Ensure the MS51_16K library is included.
4. Configure hardware pins as defined in `LoRa.h`.

## Example Usage
```c
#include <stdio.h>
#include "LoRa.h"
#include "MS51_16K.h"

#define MODE_TX // Comment out for RX mode

void main(void) {
    char buffer[64];
    unsigned char len;
    unsigned int counter = 0;

    UART_Init();
    Spi_Init();
    begin();
    printf("LoRa Example Start\n");

#ifdef MODE_TX
    while (1) {
        sprintf(buffer, "%u", counter);
        printf("Sending: %s\n", buffer);
        LoRa_Send("Hello Bro", strlen("Hello Wrold"));
        printf("Sent!\n");
        counter++;
        Timer0_Delay(16000000, 1000, 5000); // 5s delay
    }
#else
    while (1) {
        printf("Listening...\n");
        len = LoRa_Receive(buffer, 64);
        buffer[len] = '\0';
        printf("Received: %s\n", buffer);
    }
#endif
}
```

## Key Functions
- `begin()`: Initializes the LLCC68 module and SPI.
- `LoRa_Send(msg, len)`: Sends a LoRa packet.
- `LoRa_Receive(buf, maxLen)`: Receives a LoRa packet (non-blocking).
- `lora_receive_blocking(buf, maxLen)`: Receives a LoRa packet (blocking).

## Notes
- Default frequency is 915 MHz; modify `freq_hz` in `LoRa.c` for other frequencies.
- Ensure proper SPI and GPIO setup for your hardware.
- Refer to the LLCC68 datasheet for command and register details.

### Built with ❤️ by **DiveshRDH**

Want more help? Reach out or fork and improve this repo!

