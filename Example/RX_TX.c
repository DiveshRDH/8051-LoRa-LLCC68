#include <stdio.h>
#include "LoRa.h"
#include "MS51_16K.h"

#define MODE_TX     // comment out for RX mode





UART_Init();
Spi_Init();
begin();

void main(void)
{
    char buffer[64];
    unsigned char len;
    unsigned int counter = 0;  // For TX mode

		UART_Init();
		Spi_Init();
		begin();
    printf("LoRa Example Start\n");


#ifdef MODE_TX
    while (1)
    {
        sprintf(buffer, "%u", counter);
        printf("Sending: %s\n", buffer);
        LoRa_Send("Hello Bro", strlen("Hello Wrold"));
        printf("Sent!\n");
        counter++;
        Timer0_Delay(16000000, 1000, 5000);  // 1s delay
    }
#else
    while (1)
    {
        printf("Listening...\n");
        len = LoRa_Receive(buffer, 64);
        buffer[len] = '\0';
        printf("Received: %s\n", buffer);
    }
#endif
}
