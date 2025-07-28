#ifndef LORA_H
#define LORA_H

#include "MS51_16K.h"
#include <stdio.h>

// Pin definitions
#define NSS     P15    // SPI Slave Select
#define DIO1    P11    // DIO1 for interrupts
#define RESET   P13    // Reset pin

// Function prototypes
void UART_Init(void);
void Spi_Init(void);
void Delay(unsigned char delay);
void Reset(void);
void SetStandby(void);
void SetModulationParams(void);
void SetPacketParams(void);
void WriteBuffer(void);
void SetTx(void);
void getStatus(void);
void SetRX(void);
void ClearIrqStatus(void);
void GetRxBufferStatus(void);
void handshake(void);
void SetDIO2AsRfSwitchCtrl(void);
unsigned long frequencyToPLL(unsigned long rfFreq);
void SetRfFrequency(void);
void SetPacketType(void);
void StopTimerOnPreamble(void);
void SetPaConfig(void);
void SetTxParams(void);
void SetLoRaSymbNumTimeout(void);
void SetDioIrqParams(void);
void GetIrqStatus(unsigned int *irqStatus);
void setModeReceive(void);
void lora_receive_async(void);
void Essensial(void);
void begin(void);
void LoRa_Send(char* msg, unsigned char len);
unsigned char LoRa_Receive(char* buf, unsigned char buffMaxLen);
int lora_receive_blocking(unsigned char *buf, int buffMaxLen);

#endif // LORA_H