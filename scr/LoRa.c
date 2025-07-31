#include "LoRa.h"

// Constants
unsigned long pllFrequency;
unsigned long freq_hz = 915000000UL; // 915 MHz

// Initialize UART at 9600 baud
void UART_Init(void) {
    MODIFY_HIRC(HIRC_24);
    P06_QUASI_MODE;
    UART_Open(24000000, UART0_Timer3, 9600);
    ENABLE_UART0_PRINTF;
}

// Initialize SPI for LLCC68
void Spi_Init(void) {
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

// Delay in milliseconds
void Delay(unsigned char delay) {
    Timer0_Delay(24000000, 1000, delay);
}

// Reset LLCC68
void Reset(void) {
    RESET = 0;
    Delay(100);
    RESET = 1;
    Delay(100);
    RESET = 0;
    Delay(100);
    RESET = 1;
}

// Set standby mode
void SetStandby(void) {
    NSS = 0;
    Spi_Write_Byte(0x80);
    Spi_Write_Byte(0x01);
    NSS = 1;
}

// Set LoRa modulation parameters
void SetModulationParams(void) {
    NSS = 0;
    Spi_Write_Byte(0x8B);
    Spi_Write_Byte(0x07); // SF7
    Spi_Write_Byte(0x05); // BW 125 kHz
    Spi_Write_Byte(0x01); // CR 4/5
    Spi_Write_Byte(0x00);
    NSS = 1;
}

// Set packet parameters
void SetPacketParams(void) {
    NSS = 0;
    Spi_Write_Byte(0x8C);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x0C);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x01);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x00);
    NSS = 1;
    Delay(100);
}

// Write to buffer
void WriteBuffer(void) {
    NSS = 0;
    Spi_Write_Byte(0x0E);
    Spi_Write_Byte(0x00);
    NSS = 1;
    Delay(100);
    NSS = 0;
    Spi_Write_Byte(0x0E);
    Spi_Write_Byte(0x0A);
    NSS = 1;
    Delay(100);
}

// Set TX mode
void SetTx(void) {
    NSS = 0;
    Spi_Write_Byte(0x83);
    Spi_Write_Byte(0xFF);
    Spi_Write_Byte(0xFF);
    Spi_Write_Byte(0xFF);
    NSS = 1;
}

// Get status
void getStatus(void) {
    NSS = 0;
    Spi_Write_Byte(0xC0);
    Spi_Write_Byte(0xC0);
    NSS = 1;
}

// Set RX mode
void SetRX(void) {
    NSS = 0;
    Spi_Write_Byte(0x82);
    Spi_Write_Byte(0xFF);
    Spi_Write_Byte(0xFF);
    Spi_Write_Byte(0xFF);
    NSS = 1;
}

// Clear IRQ status
void ClearIrqStatus(void) {
    NSS = 0;
    Spi_Write_Byte(0x02);
    Spi_Write_Byte(0xFF);
    Spi_Write_Byte(0xFF);
    NSS = 1;
}

// Get RX buffer status
void GetRxBufferStatus(void) {
    unsigned char Status, PayloadLengthRx, RxStartBufferPointer;
    NSS = 0;
    Spi_Write_Byte(0x13);
    Status = Spi_Read_Byte(0xFF);
    PayloadLengthRx = Spi_Read_Byte(0xFF);
    RxStartBufferPointer = Spi_Read_Byte(0xFF);
    NSS = 1;
}

// Handshake with LLCC68
void handshake(void) {
    unsigned int regValue;
    Delay(100);
    NSS = 0;
    Spi_Write_Byte(0x1D);
    Spi_Write_Byte(0x07);
    Spi_Write_Byte(0x40);
    Spi_Write_Byte(0x00);
    regValue = Spi_Read_Byte(0xFF);
    NSS = 1;
    printf("Register 0x0740: 0x%02X\n", regValue);
}

// Set DIO2 as RF switch control
void SetDIO2AsRfSwitchCtrl(void) {
    NSS = 0;
    Spi_Write_Byte(0x9D);
    Spi_Write_Byte(0x01);
    NSS = 1;
}

// Convert RF frequency to PLL
unsigned long frequencyToPLL(unsigned long rfFreq) {
    unsigned long q = rfFreq / 15625UL;
    unsigned long r = rfFreq % 15625UL;
    q *= 16384UL;
    r *= 16384UL;
    return q + (r / 15625UL);
}

// Set RF frequency
void SetRfFrequency(void) {
    pllFrequency = frequencyToPLL(freq_hz);
    NSS = 0;
    Spi_Write_Byte(0x86);
    Spi_Write_Byte(0x08);
    Spi_Write_Byte(0x60);
    Spi_Write_Byte((pllFrequency >> 24) & 0xFF);
    Spi_Write_Byte((pllFrequency >> 16) & 0xFF);
    Spi_Write_Byte((pllFrequency >> 8) & 0xFF);
    Spi_Write_Byte(pllFrequency & 0xFF);
    NSS = 1;
}

// Set packet type to LoRa
void SetPacketType(void) {
    NSS = 0;
    Spi_Write_Byte(0x8A);
    Spi_Write_Byte(0x01);
    NSS = 1;
    Delay(100);
}

// Stop timer on preamble
void StopTimerOnPreamble(void) {
    NSS = 0;
    Spi_Write_Byte(0x9F);
    Spi_Write_Byte(0x00);
    NSS = 1;
    Delay(100);
}

// Set PA configuration
void SetPaConfig(void) {
    NSS = 0;
    Spi_Write_Byte(0x95);
    Spi_Write_Byte(0x04);
    Spi_Write_Byte(0x07);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x01);
    NSS = 1;
    Delay(100);
}

// Set TX parameters
void SetTxParams(void) {
    NSS = 0;
    Spi_Write_Byte(0x8E);
    Spi_Write_Byte(0x16);
    Spi_Write_Byte(0x02);
    NSS = 1;
    Delay(100);
}

// Set LoRa symbol timeout
void SetLoRaSymbNumTimeout(void) {
    NSS = 0;
    Spi_Write_Byte(0xA0);
    Spi_Write_Byte(0x00);
    NSS = 1;
    Delay(100);
}

// Set DIO IRQ parameters
void SetDioIrqParams(void) {
    NSS = 0;
    Spi_Write_Byte(0x08);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x02);
    Spi_Write_Byte(0xFF);
    Spi_Write_Byte(0xFF);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x00);
    NSS = 1;
    Delay(100);
}

// Get IRQ status
void GetIrqStatus(unsigned int *irqStatus) {
    unsigned char status, msb, lsb;
    NSS = 0;
    Spi_Write_Byte(0x12);
    status = Spi_Read_Byte(0x00);
    msb = Spi_Read_Byte(0x00);
    lsb = Spi_Read_Byte(0x00);
    *irqStatus = ((char)msb << 8) | lsb;
    NSS = 1;
    Delay(100);
}

// Set receive mode
void setModeReceive(void) {
    NSS = 0;
    Spi_Write_Byte(0x8C);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x0C);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0xFF);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x00);
    NSS = 1;
    Delay(100);
    SetRX();
}

// Async LoRa receive
void lora_receive_async(void) {
    unsigned regValue;
    setModeReceive();
    ClearIrqStatus();
    GetRxBufferStatus();
    NSS = 0;
    Spi_Write_Byte(0x1E);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x00);
    regValue = Spi_Read_Byte(0xFF);
    NSS = 1;
}

// Configure essential settings
void Essensial(void) {
    SetDIO2AsRfSwitchCtrl();
    SetRfFrequency();
    SetPacketType();
    StopTimerOnPreamble();
    SetModulationParams();
    SetPaConfig();
    SetTxParams();
    SetLoRaSymbNumTimeout();
    SetDioIrqParams();
}

// Initialize LLCC68
void begin(void) {
    Spi_Init();
    Reset();
    handshake();
    Essensial();
}

// Send LoRa packet
void LoRa_Send(char* msg, unsigned char len) {
    unsigned char i;
    if (len > 255) { len = 255; }
    SetStandby();
    NSS = 0;
    Spi_Write_Byte(0x8C);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x0C);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(len);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x00);
    NSS = 1;
    Delay(100);
    NSS = 0;
    Spi_Write_Byte(0x8F);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x80);
    NSS = 1;
    NSS = 0;
    Spi_Write_Byte(0x0E);
    Spi_Write_Byte(0x00);
    for (i = 0; i < len; i++) {
        Spi_Write_Byte(msg[i]);
    }
    NSS = 1;
    SetTx();
}

// Receive LoRa packet
unsigned char LoRa_Receive(char* buf, unsigned char buffMaxLen) {
    unsigned char Status, payloadLen, startAddress, i;
    setModeReceive();
    if (DIO1 == 0) { return -1; }
    ClearIrqStatus();
    NSS = 0;
    Spi_Write_Byte(0x13);
    Status = Spi_Read_Byte(0xFF);
    payloadLen = Spi_Read_Byte(0xFF);
    startAddress = Spi_Read_Byte(0xFF);
    NSS = 1;
    if (payloadLen > buffMaxLen) {
        payloadLen = buffMaxLen;
    }
    NSS = 0;
    Spi_Write_Byte(0x1E);
    Spi_Write_Byte(startAddress);
    Spi_Write_Byte(0x00);
    for (i = 0; i < payloadLen; i++) {
        buf[i] = Spi_Read_Byte(0x00);
    }
    NSS = 1;
    return payloadLen;
}

// Blocking LoRa receive
int lora_receive_blocking(unsigned char *buf, int buffMaxLen) {
    setModeReceive();
    while (DIO1 == 0);
    return LoRa_Receive(buf, buffMaxLen);
}
