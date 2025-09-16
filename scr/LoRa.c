#include "LoRa.h"

// Global configuration variables
xdata unsigned char spiBuff[256];
unsigned long pllFrequency;
unsigned long freq_hz = 852000000UL; // Default 915 MHz
unsigned char spreadingFactor = 0x07; // SF7 default
unsigned char bandwidth = 0x05;       // 250 kHz default
unsigned char codingRate = 0x01;      // CR_4_5 default
unsigned char lowDataRateOptimize = 0x00; // Off default
unsigned long transmitTimeout = 12000; // Default for SF7 (in ms)
unsigned char inReceiveMode = 0;
int rssi = 0;
int snr = 0;
int signalRssi = 0;

// Initialize UART at 9600 baud (for printf debugging)
void UART_Init(void) {
    MODIFY_HIRC(HIRC_24);
    SET_PIN_MODE(0,6,QUASI_MODE);
    UART_Open(24000000, UART0_Timer3, 9600);
    ENABLE_UART0_PRINTF;
}

// Initialize SPI Master for LLCC68
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

// SPI Write Byte
void Spi_Write_Byte(unsigned char u8SpiWB) {
    SPDR = u8SpiWB;
    while(!(SPSR&0x80));
    clr_SPSR_SPIF;
}

// SPI Read Byte
unsigned char Spi_Read_Byte(unsigned char u8SpiWB) {
    unsigned char u8SpiRB;
    SPDR = u8SpiWB;
    while(!(SPSR&0x80));
    u8SpiRB = SPDR;
    clr_SPSR_SPIF;
    return u8SpiRB;
}

// Delay in milliseconds
void Delay_ms(unsigned int ms) {
    Timer0_Delay(24000000, 1000, ms);
}

// Convert frequency to PLL
unsigned long frequencyToPLL(long rfFreq) {
    unsigned long q = rfFreq / 15625UL;
    unsigned long r = rfFreq % 15625UL;
    q *= 16384UL;
    r *= 16384UL;
    return q + (r / 15625UL);
}

// Hardware reset the radio
void Lora_reset(void) {
    RESET = 0; Delay_ms(100);
    RESET = 1; Delay_ms(100);
    RESET = 0; Delay_ms(100);
    RESET = 1; Delay_ms(100);
}

// Sanity check: Read register 0x0740, should be 0x14
unsigned char Lora_sanityCheck(void) {
    unsigned char regValue;
    NSS = 0;
    Spi_Write_Byte(0x1D); // Opcode: Read register
    Spi_Write_Byte(0x07); // Address MSB
    Spi_Write_Byte(0x40); // Address LSB
    Spi_Write_Byte(0x00); // Dummy
    regValue = Spi_Read_Byte(0xFF);
    NSS = 1;
    printf("Sanity check register 0x0740: 0x%02X\n", regValue);
    return (regValue == 0x14) ? 1 : 0;
}

// Wait for radio command completion
unsigned char Lora_waitForRadioCommandCompletion(unsigned long timeout_ms) {
    int loops;
    unsigned char done = 0;
    unsigned char status;
    unsigned char chipMode;
    unsigned char commandStatus;
    loops = (int)(timeout_ms / 5); // Poll every 5ms
    while (loops > 0 && !done) {
        Delay_ms(5);
        loops--;
        NSS = 0;
        Spi_Write_Byte(0xC0); // Opcode: Get status
        status = Spi_Read_Byte(0x00);
        NSS = 1;
        chipMode = (status >> 4) & 0x07;
        commandStatus = (status >> 1) & 0x07;
        if (commandStatus != 0 && commandStatus != 1 && commandStatus != 2) {
            done = 1;
        }
        if (chipMode == 0x02 || chipMode == 0x03) {
            done = 1;
        }
    }
    return done;
}

// Configure essential radio settings
void Lora_configureRadioEssentials(void) {
    // Enable DIO2 as RF switch control
    NSS = 0;
    Spi_Write_Byte(0x9D);
    Spi_Write_Byte(0x01);
    NSS = 1;
    Delay_ms(100);

    // Set frequency (default 915MHz)
    Lora_configSetFrequency(850000000);

    // Set packet type to LoRa
    NSS = 0;
    Spi_Write_Byte(0x8A);
    Spi_Write_Byte(0x01);
    NSS = 1;
    Delay_ms(100);

    // Stop timer on sync word/header
    NSS = 0;
    Spi_Write_Byte(0x9F);
    Spi_Write_Byte(0x00);
    NSS = 1;
    Delay_ms(100);

    // Set modulation parameters (default preset)
    Lora_configSetPreset(CUSTOM);

    // Set PA config
    NSS = 0;
    Spi_Write_Byte(0x95);
    Spi_Write_Byte(0x04); // paDutyCycle
    Spi_Write_Byte(0x07); // hpMax (max power)
    Spi_Write_Byte(0x00); // deviceSel (SX1262)
    Spi_Write_Byte(0x01); // paLut
    NSS = 1;
    Delay_ms(100);

    // Set TX params
    NSS = 0;
    Spi_Write_Byte(0x8E);
    Spi_Write_Byte(0x16); // Power +22 dBm
    Spi_Write_Byte(0x02); // Ramp time 40us
    NSS = 1;
    Delay_ms(100);

    // Set LoRa symbol timeout
    NSS = 0;
    Spi_Write_Byte(0xA0);
    Spi_Write_Byte(0x00); // 0 symbols
    NSS = 1;
    Delay_ms(100);

    // Enable interrupts
    NSS = 0;
    Spi_Write_Byte(0x08);
    Spi_Write_Byte(0x00); // IRQ mask MSB
    Spi_Write_Byte(0x02); // IRQ mask LSB
    Spi_Write_Byte(0xFF); // DIO1 mask MSB
    Spi_Write_Byte(0xFF); // DIO1 mask LSB
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x00);
    NSS = 1;
    Delay_ms(100);
}

// Begin LoRa setup
unsigned char Lora_begin(void) {
    unsigned char success;

    Spi_Init();

    // Set pin modes
    NSS = 1;
    RESET = 1;

    Lora_reset();

    success = Lora_sanityCheck();
    if (!success) {
        printf("Sanity check failed!\n");
        return 0;
    }

    Lora_configureRadioEssentials();
    printf("LoRa initialization successful.\n");
    return 1;
}

// Transmit data
void Lora_transmit(unsigned char *PKT, int dataLen) {
    int i;
    if (dataLen > 255) dataLen = 255;

    if (inReceiveMode) {
        Lora_setModeStandby();
    }

    // Set packet parameters
    NSS = 0;
    Spi_Write_Byte(0x8C);
    Spi_Write_Byte(0x00); // Preamble MSB
    Spi_Write_Byte(0x0C); // Preamble LSB
    Spi_Write_Byte(0x00); // Variable length
    Spi_Write_Byte(dataLen); // Payload length
    Spi_Write_Byte(0x00); // CRC off
    Spi_Write_Byte(0x00); // IQ standard
    NSS = 1;
    Lora_waitForRadioCommandCompletion(100);

    // Write buffer
    NSS = 0;
    Spi_Write_Byte(0x0E); // Opcode WriteBuffer
    Spi_Write_Byte(0x00); // Offset
    for (i = 0; i < dataLen; i++) {
        Spi_Write_Byte(PKT[i]);
    }
    NSS = 1;
    Lora_waitForRadioCommandCompletion(1000);

    // Set TX
    NSS = 0;
    Spi_Write_Byte(0x83);
    Spi_Write_Byte(0xFF); // Timeout (no timeout)
    Spi_Write_Byte(0xFF);
    Spi_Write_Byte(0xFF);
    NSS = 1;
    Lora_waitForRadioCommandCompletion(transmitTimeout);

    inReceiveMode = 0;
}

// Set mode to receive
void Lora_setModeReceive(void) {
    if (inReceiveMode) return;

    // Set packet parameters
    NSS = 0;
    Spi_Write_Byte(0x8C);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x0C);
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0xFF); // Max payload
    Spi_Write_Byte(0x00);
    Spi_Write_Byte(0x00);
    NSS = 1;
    Lora_waitForRadioCommandCompletion(100);

    // Set RX
    NSS = 0;
    Spi_Write_Byte(0x82);
    Spi_Write_Byte(0xFF); // Continuous RX
    Spi_Write_Byte(0xFF);
    Spi_Write_Byte(0xFF);
    NSS = 1;
    Lora_waitForRadioCommandCompletion(100);

    inReceiveMode = 1;
}

// Set mode to standby
void Lora_setModeStandby(void) {
    NSS = 0;
    Spi_Write_Byte(0x80);
    Spi_Write_Byte(0x01); // STDBY_XOSC
    NSS = 1;
    Lora_waitForRadioCommandCompletion(100);
    inReceiveMode = 0;
}

// Async receive (non-blocking)
int Lora_receive_async(unsigned char *buff, int buffMaxLen) {
    unsigned char rssiPkt;
    unsigned char snrPkt;
    unsigned char signalRssiPkt;
    unsigned char payloadLen;
    unsigned char startAddress;
    int i;

    Lora_setModeReceive();

    if (DIO1 == 0) return -1; // No packet

    // Clear IRQ
    while (DIO1 == 1) {
        NSS = 0;
        Spi_Write_Byte(0x02);
        Spi_Write_Byte(0xFF);
        Spi_Write_Byte(0xFF);
        NSS = 1;
    }

    // Get packet status
    NSS = 0;
    Spi_Write_Byte(0x14);
    Spi_Read_Byte(0x00); // Dummy status
    rssiPkt = Spi_Read_Byte(0x00);
    snrPkt = Spi_Read_Byte(0x00);
    signalRssiPkt = Spi_Read_Byte(0x00);
    NSS = 1;
    rssi = -((int)rssiPkt) / 2;
    snr = ((signed char)snrPkt) / 4;
    signalRssi = -((int)signalRssiPkt) / 2;

    // Get RX buffer status
    NSS = 0;
    Spi_Write_Byte(0x13);
    Spi_Read_Byte(0x00); // Dummy status
    payloadLen = Spi_Read_Byte(0x00);
    startAddress = Spi_Read_Byte(0x00);
    NSS = 1;

    if (payloadLen > buffMaxLen) payloadLen = buffMaxLen;

    // Read buffer
    NSS = 0;
    Spi_Write_Byte(0x1E);
    Spi_Write_Byte(startAddress);
    Spi_Write_Byte(0x00); // NOP
    for (i = 0; i < payloadLen; i++) {
        buff[i] = Spi_Read_Byte(0x00);
    }
    NSS = 1;

    return (int)payloadLen;
}

// Blocking receive
int Lora_receive_blocking(unsigned char *buff, int buffMaxLen, unsigned long timeout_ms) {
    Lora_setModeReceive();

    if (timeout_ms == 0) {
        while (DIO1 == 0); // Infinite wait
        return Lora_receive_async(buff, buffMaxLen);
    } else {
        int loops = (int)(timeout_ms / 10); // Poll every 10ms
        while (DIO1 == 0 && loops > 0) {
            Delay_ms(10);
            loops--;
        }
        if (DIO1 == 0) return -1;
        return Lora_receive_async(buff, buffMaxLen);
    }
}

// Update radio frequency
void Lora_updateRadioFrequency(void) {
    NSS = 0;
    Spi_Write_Byte(0x86);
    Spi_Write_Byte((pllFrequency >> 24) & 0xFF);
    Spi_Write_Byte((pllFrequency >> 16) & 0xFF);
    Spi_Write_Byte((pllFrequency >> 8) & 0xFF);
    Spi_Write_Byte(pllFrequency & 0xFF);
    NSS = 1;
    Delay_ms(100);
}

// Set frequency
unsigned char Lora_configSetFrequency(long frequencyInHz) {
    if (frequencyInHz < 150000000 || frequencyInHz > 850000000) return 0;
    pllFrequency = frequencyToPLL(frequencyInHz);
    Lora_updateRadioFrequency();
    return 1;
}

// Update modulation parameters
void Lora_updateModulationParameters(void) {
    NSS = 0;
    Spi_Write_Byte(0x8B);
    Spi_Write_Byte(spreadingFactor);
    Spi_Write_Byte(bandwidth);
    Spi_Write_Byte(codingRate);
    Spi_Write_Byte(lowDataRateOptimize);
    NSS = 1;
    Delay_ms(100);

    // Update transmit timeout based on SF
    switch (spreadingFactor) {
        case 0x0C: transmitTimeout = 252000; break;
        case 0x0B: transmitTimeout = 160000; break;
        case 0x0A: transmitTimeout = 60000; break;
        case 0x09: transmitTimeout = 40000; break;
        case 0x08: transmitTimeout = 20000; break;
        case 0x07: transmitTimeout = 12000; break;
        case 0x06: transmitTimeout = 7000; break;
        default: transmitTimeout = 5000; break; // SF5
    }
}

// Set preset
unsigned char Lora_configSetPreset(int preset) {
    switch (preset) {
        case PRESET_DEFAULT:
            bandwidth = 0x05; // 250 kHz
            codingRate = 0x01;
            spreadingFactor = 0x07;
            lowDataRateOptimize = 0x00;
            Lora_updateModulationParameters();
            return 1;
        case PRESET_LONGRANGE:
            bandwidth = 0x04; // 125 kHz
            codingRate = 0x01;
            spreadingFactor = 0x0C;
            lowDataRateOptimize = 0x01;
            Lora_updateModulationParameters();
            return 1;
        case PRESET_FAST:
            bandwidth = 0x06; // 500 kHz
            codingRate = 0x01;
            spreadingFactor = 0x05;
            lowDataRateOptimize = 0x00;
            Lora_updateModulationParameters();
            return 1;
        case CUSTOM:
            bandwidth = 0x04; // 125 kHz
            codingRate = 0x04;
            spreadingFactor = 0x0B; // SF11
            lowDataRateOptimize = 0x01;
            Lora_updateModulationParameters();
            return 1;
        default:
            return 0;
    }
}

// Set bandwidth
unsigned char Lora_configSetBandwidth(unsigned char bw) {
    if (bw > 0x0A || bw == 0x07) return 0; // Invalid
    bandwidth = bw;
    Lora_updateModulationParameters();
    return 1;
}

// Set coding rate
unsigned char Lora_configSetCodingRate(unsigned char cr) {
    if (cr < 0x01 || cr > 0x04) return 0;
    codingRate = cr;
    Lora_updateModulationParameters();
    return 1;
}

// Set spreading factor
unsigned char Lora_configSetSpreadingFactor(unsigned char sf) {
    if (sf < 0x05 || sf > 0x0C) return 0;
    lowDataRateOptimize = (sf >= 0x0B) ? 0x01 : 0x00;
    spreadingFactor = sf;
    Lora_updateModulationParameters();
    return 1;
}

// Delay function
void delay(unsigned char timer) {
    Timer0_Delay(16000000, 200, timer);
}
