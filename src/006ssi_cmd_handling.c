/*
 * 006ssi_tx_led_master.c
 *
 *  Created on: 04-Aug-2025
 *      Author: tabrez
 */

#include "tm4c123x.h"
#include <string.h>

#define BTN_PRESSED 0U
#define COMMAND_LED_CTRL 0x00
#define COMMAND_STATUS_READ 0x01  // Read status of specific pin

#define CMD_LED1 0x01  // Command to toggle LED 1
#define CMD_LED2 0x02  // Command to toggle LED 2
#define CMD_LED3 0x03  // Command to toggle LED 3
#define CMD_LED4 0x04  // Command to toggle LED 4

//STM32F407VGT6 LED PIN NUMBERS
#define LED3_PIN 13 //PD13
#define LED4_PIN 12  // PD12
#define LED5_PIN 14  // PD14
#define LED6_PIN 15  // PD15

#define LED_ON 1
#define LED_OFF 0
int count = 0;
uint8_t status;
uint8_t debug_rx[3];

void delay(void)
{
    uint32_t i = 200000; // Adjust for ~10-20 ms
    while (i)
    {
        i--;
    }
}

// at top, add small delay helper
static inline void short_delay(void) {
    volatile uint32_t d = 2000;
    while (d--) __asm__ volatile(" nop");
}

void SSI_GPIOInit(void)
{
    GPIO_Handle_t ssiPins;
    ssiPins.pGPIOx = GPIOB;

    // Ensure GPIOB clock enabled (do this explicitly)
    GPIO_PeriClockControl(GPIOB, ENABLE);

    // Common configuration for SSI2 pins
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 2;  // AF = 2 for SSI2
    ssiPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
    ssiPins.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_4MA;
    ssiPins.GPIO_PinConfig.GPIO_PinSlewRate = GPIO_SLEW_OFF;
    ssiPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;

    // Configure PB4 (SSI2CLK) - AF
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_OUT;  // Clock output
    GPIO_Init(&ssiPins);

    // Configure PB6 (SSI2RX) - AF
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_IN;   // RX input
    GPIO_Init(&ssiPins);

    // Configure PB7 (SSI2TX) - AF
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_OUT;  // TX output
    GPIO_Init(&ssiPins);

    // Configure PB5 as manual CS GPIO output (default HIGH)
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
    // Keep other fields (OPType/Drive/Slew/PuPd) as set earlier
    GPIO_Init(&ssiPins);
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, 1); // CS idle HIGH
}

void SSI_MasterInit(void)
{
    SSI_Handle_t ssi2Handle;
    ssi2Handle.pSSIx = SSI2;

    ssi2Handle.SSIConfig.SSI_DeviceMode = SSI_DEVICE_MODE_MASTER;
    ssi2Handle.SSIConfig.SSI_BusConfig = SSI_BUS_CONFIG_FD;   // Full duplex
    ssi2Handle.SSIConfig.SSI_SclkSpeed = SSI_SCLK_SPEED_DIV8; // SysClk/8 = 2MHz if SysClk=16MHz
    ssi2Handle.SSIConfig.SSI_DSS = SSI_DSS_8BIT;        // 8-bit frame
    ssi2Handle.SSIConfig.SSI_FRF = SSI_FRF_SSI_Freescale;
    ssi2Handle.SSIConfig.SSI_SPO = SSI_SPO_LOW;
    ssi2Handle.SSIConfig.SSI_SPH = SSI_SPH_1ST_EDGE;

    SSI_Init(&ssi2Handle);
}

// CS: Manual control functions (improved)
void CS_Select(void) {
    // Ensure peripheral not toggling CS in AF mode: PB5 is GPIO now.
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, 0); // Drive CS LOW
    short_delay(); // let slave detect NSS low
}

void CS_Deselect(void) {
//    // Wait for SPI transfer fully complete before releasing CS
//    while (SSI_GetFlagStatus(SSI2, SSI_FLAG_BSY)); // wait until not busy
//    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, 1); // Drive CS HIGH
//    short_delay();

    // Wait for TX FIFO empty then for SPI transfer fully complete before releasing CS
       while (!SSI_GetFlagStatus(SSI2, SSI_FLAG_TFE)); // wait until TX FIFO empty
       while (SSI_GetFlagStatus(SSI2, SSI_FLAG_BSY));  // now wait until not busy
       GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, 1);    // Drive CS HIGH
       short_delay();
}

uint8_t SSI_VerifyResponse(uint8_t ackbyte)
{
    if (ackbyte == 0xF5)
    {
        return 1; //ack
    }
    else
    {
        return 0; // nack
    }
}

int main()
{
    // GPIO configuration for button SW1
    GPIO_Handle_t GPIOBtn;
    GPIOBtn.pGPIOx = GPIOF;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_PeriClockControl(GPIOF, ENABLE);
    GPIO_Init(&GPIOBtn);

    // Initialize SSI2 pins and master
    SSI_GPIOInit();
    SSI_MasterInit();

    // Initially disable SSI2 clock to save power
    SSI_PeriClockControl(SSI2, DISABLE);

    uint8_t dummy_write = 0xFF; // Initialize with a value (0xFF is commonly used)
    uint8_t dummy_read; // Dummy read to clear RXNE
    static uint8_t led_toggle_state = 0; // Toggle state for LED
    while (1)
    {
        // Wait for button press (active low - returns 0 when pressed)
        while (GPIO_ReadFromInputPin(GPIOF, GPIO_PIN_4) != BTN_PRESSED);
        delay();  // Debounce

        // Enable SSI2 peripheral clock
        SSI_PeriClockControl(SSI2, ENABLE);

        //1. CMD_LED_CTRL <pin no> <value>
        uint8_t command_code = COMMAND_LED_CTRL;
        uint8_t ackbyte;
        uint8_t args[2];

        // CS: Manual control for LED command sequence
        CS_Select();
                // flush any stale RX data before starting
                while (SSI_GetFlagStatus(SSI2, SSI_FLAG_RNE)) {
                    SSI_ReceiveData(SSI2, &dummy_read, 1);
                }
        SSI_SendData(SSI2, &command_code, 1);
        SSI_SendData(SSI2, &dummy_write, 1);    // clock ACK
        SSI_ReceiveData(SSI2, &ackbyte, 1);
                // --- IMPORTANT: flush any lingering bytes (e.g. old ACK) before sending pin ---
                while (SSI_GetFlagStatus(SSI2, SSI_FLAG_RNE)) {
                    SSI_ReceiveData(SSI2, &dummy_read, 1);
                }
        if (SSI_VerifyResponse(ackbyte))
        {
            //send arguments
            args[0] = LED3_PIN; // Pin number for LED 3 (Orange)
            led_toggle_state = !led_toggle_state; // Toggle the state
            args[1] = led_toggle_state; // Send current toggle state

            SSI_SendData(SSI2, args, 2);
            count++;
            // Non-blocking conditional drain of RX FIFO
            uint8_t dummy_byte;
            while (SSI_GetFlagStatus(SSI2, SSI_FLAG_RNE)) {
                SSI_ReceiveData(SSI2, &dummy_byte, 1);
            }
        }
        CS_Deselect();

        //2. CMD_STATUS_READ<pin number>
        while (GPIO_ReadFromInputPin(GPIOF, GPIO_PIN_4) != BTN_PRESSED);
        delay();  // debounce
        command_code = COMMAND_STATUS_READ;
        uint8_t pin = LED3_PIN;
        uint8_t junk;

        CS_Select();
        // flush any stale RX data before starting
               while (SSI_GetFlagStatus(SSI2, SSI_FLAG_RNE)) {
                    SSI_ReceiveData(SSI2, &dummy_read, 1);
                }
        SSI_SendData(SSI2, &command_code, 1);
        SSI_SendData(SSI2, &dummy_write, 1);    // clock ACK
        SSI_ReceiveData(SSI2, &ackbyte, 1);
                while (SSI_GetFlagStatus(SSI2, SSI_FLAG_RNE)) {
                    SSI_ReceiveData(SSI2, &dummy_read, 1);
                }
        if (SSI_VerifyResponse(ackbyte)) {
            SSI_SendData(SSI2, &pin, 1);
            SSI_ReceiveData(SSI2, &junk, 1);    // slave received pin
            SSI_SendData(SSI2, &dummy_write, 1); // clock status out
            SSI_ReceiveData(SSI2, &status, 1);
        }
        debug_rx[0] = ackbyte;
        debug_rx[1] = junk;
        debug_rx[2] = status;
        CS_Deselect();

        SSI_PeriClockControl(SSI2, DISABLE);
        while (GPIO_ReadFromInputPin(GPIOF, GPIO_PIN_4) == BTN_PRESSED);
    }
    return 0;
}
