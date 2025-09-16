#ifndef __LORA_H__
#define __LORA_H__

#include <stdio.h>
#include <string.h>
#include "MS51_16K.H"

// Pin mode definitions
#define PUSH_PULL_MODE    0
#define INPUT_MODE        1
#define QUASI_MODE        2
#define OPEN_DRAIN_MODE   3

#define SET_PIN_MODE(port, pin, mode) \
  if ((mode) == PUSH_PULL_MODE) { clr_P##port##M1_##pin; set_P##port##M2_##pin; } \
  else if ((mode) == INPUT_MODE) { set_P##port##M1_##pin; clr_P##port##M2_##pin; } \
  else if ((mode) == QUASI_MODE) { clr_P##port##M1_##pin; clr_P##port##M2_##pin; } \
  else if ((mode) == OPEN_DRAIN_MODE) { set_P##port##M1_##pin; set_P##port##M2_##pin; }

// Pin definitions
sbit NSS = P1^5;    // SPI Slave Select (Chip Select for LoRa)
sbit DIO1 = P1^1;   // DIO1 for interrupts
sbit RESET = P1^3;  // Reset pin

// Preset definitions
#define PRESET_DEFAULT   0
#define PRESET_LONGRANGE 1
#define PRESET_FAST      2
#define CUSTOM           3
#define CUSTOM2          4

// Function prototypes
void UART_Init(void);
void Spi_Init(void);
void Spi_Write_Byte(unsigned char dat);
unsigned char Spi_Read_Byte(unsigned char dummy);
void Delay_ms(unsigned int ms);
unsigned long frequencyToPLL(long rfFreq);
void Lora_reset(void);
unsigned char Lora_sanityCheck(void);
unsigned char Lora_waitForRadioCommandCompletion(unsigned long timeout_ms);
void Lora_configureRadioEssentials(void);
unsigned char Lora_begin(void);
void Lora_transmit(unsigned char *PKT, int dataLen);
void Lora_setModeReceive(void);
void Lora_setModeStandby(void);
int Lora_receive_async(unsigned char *buff, int buffMaxLen);
int Lora_receive_blocking(unsigned char *buff, int buffMaxLen, unsigned long timeout_ms);
void Lora_updateRadioFrequency(void);
unsigned char Lora_configSetFrequency(long frequencyInHz);
void Lora_updateModulationParameters(void);
unsigned char Lora_configSetPreset(int preset);
unsigned char Lora_configSetBandwidth(unsigned char bw);
unsigned char Lora_configSetCodingRate(unsigned char cr);
unsigned char Lora_configSetSpreadingFactor(unsigned char sf);
void delay(unsigned char timer);

// Global configuration variables
extern xdata unsigned char spiBuff[256];
extern unsigned long pllFrequency;
extern unsigned long freq_hz;
extern unsigned char spreadingFactor;
extern unsigned char bandwidth;
extern unsigned char codingRate;
extern unsigned char lowDataRateOptimize;
extern unsigned long transmitTimeout;
extern unsigned char inReceiveMode;
extern int rssi;
extern int snr;
extern int signalRssi;

#endif // __LORA_H__
