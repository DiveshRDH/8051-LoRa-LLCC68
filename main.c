#include "MS51_16K.h"
#include <stdio.h>

// Pin and constant definitions
#define NSS     P15
#define DIO0    P11
#define RESET   P13
#define BUSY    P12
#define SPI_BUFF_SIZE 64
#define PRESET_DEFAULT 0
#define PRESET_FAST 1
#define PRESET_LONGRANGE 2

// Global variables
unsigned char spiBuff[SPI_BUFF_SIZE];
unsigned long pllFrequency;
unsigned char chunk;
unsigned char buff_size = SPI_BUFF_SIZE;
unsigned int txTimeout = 7000;
unsigned char status, rssi_raw, snr_raw, signal_raw;
int rssi, snr, signalRssi;
unsigned char status1, payloadLen, startAddress;
unsigned char k;
unsigned long freq_hz = 915000000UL;
unsigned char spreadingFactor = 0x07, bandwidth = 0x05, codingRate = 0x01, lowDataRateOptimize = 0x00;
bit inReceiveMode = 0;
volatile unsigned long ms_counter = 0;

// Function prototypes
void Timer0_ISR(void);
unsigned long millis(void);
void UART_Init(void);
void Timer0_Init(void);
void Spi_Begin(void);
void SPI_Write(unsigned int address, unsigned char val);
unsigned int SPI_Read(unsigned int address);
void SPI_Write_Buffer(unsigned int address, unsigned char len);
void LoRa_Reset(void);
bit sanityCheck(void);
unsigned char LoRa_GetStatus(void);
void updateModulationParameters(void);
unsigned long frequencyToPLL(unsigned long rfFreq);
void configureRadioEssentials(void);
bit begin(void);
void setModeStandby(void);
void transmit(unsigned char* msg, unsigned char dataLen);
bit waitForRadioCommandCompletion(unsigned int timeout_ms);
void setModeReceive(void);
signed char lora_receive_async(unsigned char* buff, unsigned char buffMaxLen);
signed char lora_receive_blocking(unsigned char* buff, unsigned char buffMaxLen, unsigned long timeout);
void updateRadioFrequency(void);
bit configSetPreset(int preset);
bit configSetFrequency(unsigned long frequencyInHz);
bit configSetBandwidth(int bandwidth);
bit configSetCodingRate(int codingRate);
bit configSetSpreadingFactor(int spreadingFactor);
void main(void);

// Function definitions
void Timer0_ISR(void) interrupt 1 {
    ms_counter++;
    TH0 = (65536 - 24000) >> 8;
    TL0 = (65536 - 24000) & 0xFF;
}

unsigned long millis(void) {
    return ms_counter;
}

void UART_Init(void) {
    MODIFY_HIRC(HIRC_24);
    P06_QUASI_MODE;
    UART_Open(24000000, UART0_Timer3, 9600);
    ENABLE_UART0_PRINTF;
}

void Timer0_Init(void) {
    TMOD |= 0x01;
    TH0 = (65536 - 24000) >> 8;
    TL0 = (65536 - 24000) & 0xFF;
    set_TR0;
    set_ET0;
    set_EA;
}

void Spi_Begin(void) {
    P15_QUASI_MODE; P10_QUASI_MODE; P00_QUASI_MODE; P01_QUASI_MODE;
    P12_QUASI_MODE; P13_QUASI_MODE;
    set_SPSR_DISMODF;
    clr_SPCR_SSOE;
    clr_SPCR_LSBFE;
    clr_SPCR_CPOL;
    clr_SPCR_CPHA;
    set_SPCR_MSTR;
    SPICLK_FSYS_DIV16;
    set_SPCR_SPIEN;
    clr_SPSR_SPIF;
}

void SPI_Write(unsigned int address, unsigned char val) {
    while (BUSY == 1); // Wait for BUSY to be low (SX126x ready for SPI)
    NSS = 0;
    Spi_Write_Byte(0x0D);          // WriteRegister command
    Spi_Write_Byte((address >> 8) & 0xFF); // Address MSB
    Spi_Write_Byte(address & 0xFF);        // Address LSB
    Spi_Write_Byte(val);           // Write value
    NSS = 1;
}

unsigned int SPI_Read(unsigned int address) {
    unsigned char msb, lsb;
    while (BUSY == 1); // Wait for BUSY to be low (SX126x ready for SPI)
    
    NSS = 0;
    Spi_Write_Byte(0x1D);          // ReadRegister command
    Spi_Write_Byte((address >> 8) & 0xFF); // Address MSB
    Spi_Write_Byte(address & 0xFF);        // Address LSB
    Spi_Write_Byte(0x00);          // NOP byte
    msb = Spi_Read_Byte(0xFF);     // Read MSB
    lsb = Spi_Read_Byte(0xFF);     // Read LSB
    NSS = 1;
    
    return ((unsigned int)msb << 8) | lsb;
}

void SPI_Write_Buffer(unsigned int address, unsigned char len) {
    unsigned char i;
    while (BUSY == 1); // Wait for BUSY to be low (SX126x ready for SPI)
    NSS = 0;
    Spi_Write_Byte(0x0E);          // WriteBuffer command
    Spi_Write_Byte(address & 0xFF); // Buffer address
    for (i = 0; i < len; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
}

void LoRa_Reset(void) {
    RESET = 0; Timer0_Delay(24000000, 100, 1);
    RESET = 1; Timer0_Delay(24000000, 100, 1);
    RESET = 0; Timer0_Delay(24000000, 100, 1);
    RESET = 1; Timer0_Delay(24000000, 100, 1);
}

bit sanityCheck(void) {
    unsigned int regValue;
    Timer0_Delay(24000000, 10, 1);
    while (BUSY == 1);
    NSS = 0;
    Spi_Write_Byte(0x1D);          // ReadRegister command
    Spi_Write_Byte(0x07);          // Address MSB (0x0740)
    Spi_Write_Byte(0x40);          // Address LSB
    Spi_Write_Byte(0x00);          // NOP byte
    regValue = Spi_Read_Byte(0xFF); // Read value
    NSS = 1;
   printf("Register 0x0740:0x%02X)\n", regValue);
    return (regValue == 0x14);
}

unsigned char LoRa_GetStatus(void) {
    unsigned char resp;
    while (BUSY == 1);
    NSS = 0;
    Spi_Write_Byte(0xC0); // GetStatus command
    resp = Spi_Read_Byte(0x00);
    NSS = 1;
  //  printf("Status: 0x%02X\n", resp);
    return resp;
}

void updateModulationParameters(void) {
    unsigned char i;
    spiBuff[0] = 0x8B; spiBuff[1] = spreadingFactor; spiBuff[2] = bandwidth; spiBuff[3] = codingRate; spiBuff[4] = lowDataRateOptimize;
    while (BUSY == 1);
    NSS = 0;
    Spi_Write_Byte(0x0D); // WriteRegister command
    Spi_Write_Byte(0x08); // Address 0x08BB
    Spi_Write_Byte(0xBB);
    for (i = 1; i <= 4; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    Timer0_Delay(24000000, 100, 1);

    switch (spreadingFactor) {
        case 12: txTimeout = 252000; break;
        case 11: txTimeout = 160000; break;
        case 10: txTimeout = 60000; break;
        case 9:  txTimeout = 40000; break;
        case 8:  txTimeout = 20000; break;
        case 7:  txTimeout = 12000; break;
        case 6:  txTimeout = 7000; break;
        default: txTimeout = 5000; break;
    }
}

unsigned long frequencyToPLL(unsigned long rfFreq) {
    unsigned long q = rfFreq / 15625UL;
    unsigned long r = rfFreq % 15625UL;
    q *= 16384UL;
    r *= 16384UL;
    return q + (r / 15625UL);
}

void configureRadioEssentials(void) {
    unsigned char i;

    // Set DIO2 as RF switch
    while (BUSY == 1);
    NSS = 0;
    spiBuff[0] = 0x9D; spiBuff[1] = 0x01;
    for (i = 0; i < 2; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    Timer0_Delay(24000000, 100, 1);

    // Set LoRa mode
    while (BUSY == 1);
    NSS = 0;
    spiBuff[0] = 0x8A; spiBuff[1] = 0x01;
    for (i = 0; i < 2; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    Timer0_Delay(24000000, 100, 1);

    // Set frequency (915 MHz)
    pllFrequency = frequencyToPLL(freq_hz); // freq_hz = 915000000UL
    spiBuff[0] = 0x86;
    spiBuff[1] = (pllFrequency >> 24) & 0xFF;
    spiBuff[2] = (pllFrequency >> 16) & 0xFF;
    spiBuff[3] = (pllFrequency >> 8) & 0xFF;
    spiBuff[4] = pllFrequency & 0xFF;
    while (BUSY == 1);
    NSS = 0;
    for (i = 0; i < 5; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    Timer0_Delay(24000000, 100, 1);

    // Set modulation parameters (SF7, BW 250 kHz, CR_4_5)
    updateModulationParameters();

    // Set PA config
    spiBuff[0] = 0x95; spiBuff[1] = 0x04; spiBuff[2] = 0x07; spiBuff[3] = 0x00; spiBuff[4] = 0x01;
    while (BUSY == 1);
    NSS = 0;
    for (i = 0; i < 5; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    Timer0_Delay(24000000, 100, 1);

    // Set TX params
    spiBuff[0] = 0x8E; spiBuff[1] = 0x16; spiBuff[2] = 0x02;
    while (BUSY == 1);
    NSS = 0;
    for (i = 0; i < 3; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    Timer0_Delay(24000000, 100, 1);

    // Set symbol timeout
    spiBuff[0] = 0xA0; spiBuff[1] = 0x00;
    while (BUSY == 1);
    NSS = 0;
    for (i = 0; i < 2; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    Timer0_Delay(24000000, 100, 1);

    // Set RX timeout
    spiBuff[0] = 0x9F; spiBuff[1] = 0x00;
    while (BUSY == 1);
    NSS = 0;
    for (i = 0; i < 2; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    Timer0_Delay(24000000, 100, 1);

    // Set interrupts (using DIO0)
    spiBuff[0] = 0x08; spiBuff[1] = 0x00; spiBuff[2] = 0x02;
    spiBuff[3] = 0x00; spiBuff[4] = 0x00; // Disable DIO1
    spiBuff[5] = 0xFF; spiBuff[6] = 0xFF; // Enable DIO0 (mapped to DIO2)
    spiBuff[7] = 0x00; spiBuff[8] = 0x00; // Disable DIO3
    while (BUSY == 1);
    NSS = 0;
    for (i = 0; i < 9; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    Timer0_Delay(24000000, 100, 1);

    // Optional: TCXO configuration (if your module uses a TCXO)
    // spiBuff[0] = 0x97; spiBuff[1] = 0x05; spiBuff[2] = 0x00; spiBuff[3] = 0x00; spiBuff[4] = 0x01;
    // while (BUSY == 1);
    // NSS = 0;
    // for (i = 0; i < 5; i++) {
    //     Spi_Write_Byte(spiBuff[i]);
    // }
    // NSS = 1;
    // Timer0_Delay(24000000, 100, 1);
}

bit begin(void) {
    NSS = 1; P15_QUASI_MODE;
    RESET = 1; P13_QUASI_MODE;
    P11_INPUT_MODE;
    P12_INPUT_MODE;
    Spi_Begin();
    LoRa_Reset();
    if (!sanityCheck()) return 0;
    configureRadioEssentials();
    return 1;
}

void setModeStandby(void) {
    unsigned char i;
    spiBuff[0] = 0x80; spiBuff[1] = 0x01;
    while (BUSY == 1);
    NSS = 0;
    for (i = 0; i < 2; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    waitForRadioCommandCompletion(100);
    inReceiveMode = 0;
}

void transmit(unsigned char* msg, unsigned char dataLen) {
    unsigned char i = 0, j;
    bit success;
    if (inReceiveMode) setModeStandby();

    spiBuff[0] = 0x8C; spiBuff[1] = 0x00; spiBuff[2] = 0x0C; spiBuff[3] = 0x00;
    spiBuff[4] = dataLen; spiBuff[5] = 0x00; spiBuff[6] = 0x00;
    while (BUSY == 1);
    NSS = 0;
    for (i = 0; i < 7; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    Timer0_Delay(24000000, 100, 1);

    //SPI_Write_Buffer(0x00, dataLen);
    while (i < dataLen) {
        chunk = dataLen - i;
        if (chunk > buff_size) chunk = buff_size;
        for (j = 0; j < chunk; j++) {
            spiBuff[j] = msg[i + j];
        }
        SPI_Write_Buffer(0x00, chunk);
        i += chunk;
    }
    Timer0_Delay(24000000, 1000, 1);

    spiBuff[0] = 0x83; spiBuff[1] = 0xFF; spiBuff[2] = 0xFF; spiBuff[3] = 0xFF;
    while (BUSY == 1);
    NSS = 0;
    for (i = 0; i < 4; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    success = waitForRadioCommandCompletion(txTimeout);
    if (!success) {
        printf("TX failed: Timeout\n");
    } else {
        LoRa_GetStatus();
    }
    inReceiveMode = 0;
}

bit waitForRadioCommandCompletion(unsigned int timeout_ms) {
    unsigned long startTime = millis();
    unsigned char chipMode, commandStatus;

    while (1) {
        Timer0_Delay(24000000, 5, 1);
        while (BUSY == 1);
        NSS = 0;
        Spi_Write_Byte(0xC0); // GetStatus command
        chipMode = Spi_Read_Byte(0x00);
        NSS = 1;

        chipMode = (chipMode >> 4) & 0x07;
        commandStatus = (chipMode >> 1) & 0x07;

        if (commandStatus > 2 || chipMode == 0x02 || chipMode == 0x03) {
            return 1;
        }
        if (millis() - startTime >= timeout_ms) {
            return 0;
        }
    }
}

void setModeReceive(void) {
    unsigned char i;
    if (inReceiveMode) return;

    spiBuff[0] = 0x8C; spiBuff[1] = 0x00; spiBuff[2] = 0x0C; spiBuff[3] = 0x00;
    spiBuff[4] = 0xFF; spiBuff[5] = 0x00; spiBuff[6] = 0x00;
    while (BUSY == 1);
    NSS = 0;
    for (i = 0; i < 7; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    waitForRadioCommandCompletion(100);

    spiBuff[0] = 0x82; spiBuff[1] = 0xFF; spiBuff[2] = 0xFF; spiBuff[3] = 0xFF;
    while (BUSY == 1);
    NSS = 0;
    for (i = 0; i < 4; i++) {
        Spi_Write_Byte(spiBuff[i]);
    }
    NSS = 1;
    waitForRadioCommandCompletion(100);
    inReceiveMode = 1;
}

signed char lora_receive_async(unsigned char* buff, unsigned char buffMaxLen) {
    setModeReceive();
    if (DIO0 == 0) {
        printf("No packet: DIO0 low\n");
        return -1;
    }

    while (DIO0) {
    while (BUSY == 1);
NSS = 0;
Spi_Write_Byte(0x02); // ClearIrqStatus
Spi_Write_Byte(0xFF);
Spi_Write_Byte(0xFF);
NSS = 1;

        Timer0_Delay(24000000, 100, 1);
    }

    while (BUSY == 1);
    NSS = 0;
    spiBuff[0] = 0x14;
    SPI_Write_Buffer(0x00, 1); // GetRxBufferStatus command
    status = Spi_Read_Byte(0xFF);
    rssi_raw = Spi_Read_Byte(0xFF);
    snr_raw = Spi_Read_Byte(0xFF);
    signal_raw = Spi_Read_Byte(0xFF);
    NSS = 1;
    rssi = -((int)rssi_raw) / 2;
    snr = ((signed char)snr_raw) / 4;
    signalRssi = -((int)signal_raw) / 2;
    printf("RX Status: 0x%02X, RSSI: %d, SNR: %d\n", status, rssi, snr);

    while (BUSY == 1);
    NSS = 0;
    spiBuff[0] = 0x13;
    SPI_Write_Buffer(0x00, 1); // GetPacketStatus command
    status1 = Spi_Read_Byte(0xFF);
    payloadLen = Spi_Read_Byte(0xFF);
    startAddress = Spi_Read_Byte(0xFF);
    NSS = 1;
    printf("Payload Length: %d, Start Address: 0x%02X\n", payloadLen, startAddress);

    if (buffMaxLen < payloadLen) payloadLen = buffMaxLen;

    while (BUSY == 1);
    NSS = 0;
    spiBuff[0] = 0x1E; spiBuff[1] = startAddress; spiBuff[2] = 0x00;
    SPI_Write_Buffer(startAddress, 3); // ReadBuffer command
    for (k = 0; k < payloadLen; k++) {
        buff[k] = Spi_Read_Byte(0xFF);
    }
    NSS = 1;

    return payloadLen;
}

signed char lora_receive_blocking(unsigned char* buff, unsigned char buffMaxLen, unsigned long timeout) {
    unsigned long startTime = millis();
    unsigned long elapsed;

    setModeReceive();
    while (!DIO0) {
        if (timeout > 0) {
            elapsed = millis() - startTime;
            if (elapsed >= timeout) {
                printf("RX Timeout\n");
                return -1;
            }
        }
    }
    return lora_receive_async(buff, buffMaxLen);
}

void updateRadioFrequency(void) {
    pllFrequency = frequencyToPLL(freq_hz);
    while (BUSY == 1);
    NSS = 0;
    Spi_Write_Byte(0x0D); // WriteRegister command
    Spi_Write_Byte(0x08); // Address 0x0860
    Spi_Write_Byte(0x60);
    Spi_Write_Byte((pllFrequency >> 24) & 0xFF);
    Spi_Write_Byte((pllFrequency >> 16) & 0xFF);
    Spi_Write_Byte((pllFrequency >> 8) & 0xFF);
    Spi_Write_Byte(pllFrequency & 0xFF);
    NSS = 1;
    Timer0_Delay(24000000, 100, 1);
}

bit configSetPreset(int preset) {
    if (preset == PRESET_DEFAULT) {
        spreadingFactor = 0x07; bandwidth = 0x05; codingRate = 0x01; lowDataRateOptimize = 0x00;
        updateModulationParameters();
        return 1;
    }
    if (preset == PRESET_LONGRANGE) {
        spreadingFactor = 0x0C; bandwidth = 0x04; codingRate = 0x01; lowDataRateOptimize = 0x01;
        updateModulationParameters();
        return 1;
    }
    if (preset == PRESET_FAST) {
        spreadingFactor = 0x05; bandwidth = 0x06; codingRate = 0x01; lowDataRateOptimize = 0x00;
        updateModulationParameters();
        return 1;
    }
    return 0;
}

bit configSetFrequency(unsigned long frequencyInHz) {
    if (frequencyInHz < 150000000UL || frequencyInHz > 960000000UL) return 0;
    freq_hz = frequencyInHz;
    updateRadioFrequency();
    return 1;
}

bit configSetBandwidth(int bandwidth) {
    if (bandwidth < 0 || bandwidth > 0x0A || bandwidth == 7) return 0;
    spreadingFactor = spreadingFactor; bandwidth = bandwidth; codingRate = codingRate; lowDataRateOptimize = lowDataRateOptimize;
    updateModulationParameters();
    return 1;
}

bit configSetCodingRate(int codingRate) {
    if (codingRate < 1 || codingRate > 4) return 0;
    spreadingFactor = spreadingFactor; bandwidth = bandwidth; codingRate = codingRate; lowDataRateOptimize = lowDataRateOptimize;
    updateModulationParameters();
    return 1;
}

bit configSetSpreadingFactor(int spreadingFactor) {
    if (spreadingFactor < 5 || spreadingFactor > 12) return 0;
    lowDataRateOptimize = (spreadingFactor >= 11) ? 1 : 0;
    spreadingFactor = spreadingFactor; bandwidth = bandwidth; codingRate = codingRate; lowDataRateOptimize = lowDataRateOptimize;
    updateModulationParameters();
    return 1;
}

void main(void) {
    unsigned char rxBuff[255];
    signed char len;
    unsigned char i;
    UART_Init();
    Timer0_Init();
    if (!begin()) {
        printf("Failed to initialize radio\n");
        while (1);
    }
    printf("Receiver started\n");
    while (1) {
        len = lora_receive_blocking(rxBuff, 255, 1000);
        if (len >= 0) {
            printf("Received %d bytes: ", len);
            for (i = 0; i < len; i++) {
                printf("%c", rxBuff[i]);
            }
            printf("\n");
        }
    }
}