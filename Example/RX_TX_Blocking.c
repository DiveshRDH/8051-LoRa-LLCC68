#include <stdio.h>
#include <string.h>
#include "LoRa.h"
#include "MS51_16K.h"

    UART_Init();
    Spi_Init();
    begin();
// #define MODE_TX // Comment out for RX mode

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
        LoRa_Send(buffer, strlen(buffer));
        printf("Sent!\n");
        counter++;
        Timer0_Delay(24000000, 1000, 2000); // 2s delay
    }
#else
    while (1) {
        printf("Listening...\n");
        len = LoRa_Receive(buffer, 64);
        if (len > 0) { // Only print if valid packet received
            buffer[len] = '\0'; // Null-terminate the buffer
            printf("Received: %s\n", buffer);
            memset(buffer, 0, sizeof(buffer)); // Clear buffer
        }
        Timer0_Delay(24000000, 1000, 2000); // 2s delay
    }
#endif
}
