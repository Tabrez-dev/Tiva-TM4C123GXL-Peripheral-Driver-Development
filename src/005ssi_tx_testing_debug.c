/*
 * 005ssi_tx_testing_debug.c
 *
 *  Created on: 15-Jul-2025
 *      Author: tabrez
 *  
 *  Debug version with additional checks
 */

/*
 * PB4->SSI2CLK
 * PB5->SSI2FSS
 * PB6->SSI2RX
 * PB7->SSI2TX
 * AF=2
 * */

#include "tm4c123x.h"
#include <string.h>

void SSI_GPIOInit(void)
{
    GPIO_Handle_t ssiPins;

    //1. Set the GPIO port to PORTB
    ssiPins.pGPIOx = GPIOB;

    //2. Common configuration for all SSI2 pins
    ssiPins.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode  = 2;               // AF = 2 for SSI2
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_OUT; // TX
    ssiPins.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OPTYPE_PP;
    ssiPins.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_SPEED_MED;
    ssiPins.GPIO_PinConfig.GPIO_PinSlewRate    = GPIO_SLEW_OFF;
    ssiPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;

    //4. Configure PB4 (SSI2CLK)
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    GPIO_Init(&ssiPins);

    //5. Configure PB5 (SSI2FSS) - Even in SW mode, configure as GPIO for debugging
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    GPIO_Init(&ssiPins);
    
    // Set FSS high initially (inactive)
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

    //6. Configure PB7 (SSI2Tx)
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    GPIO_Init(&ssiPins);
}

void SSI_MasterInit(void)
{
    SSI_Handle_t ssi2Handle;

    ssi2Handle.pSSIx = SSI2;

    ssi2Handle.SSIConfig.SSI_DeviceMode   = SSI_DEVICE_MODE_MASTER;
    ssi2Handle.SSIConfig.SSI_BusConfig    = SSI_BUS_CONFIG_FD;   // full duplex
    ssi2Handle.SSIConfig.SSI_SclkSpeed    = SSI_SCLK_SPEED_DIV8; // SysClk/8
    ssi2Handle.SSIConfig.SSI_DSS          = SSI_DSS_8BIT;         // 8-bit frame
    ssi2Handle.SSIConfig.SSI_FRF          = SSI_FRF_SSI_FreeScale;
    ssi2Handle.SSIConfig.SSI_SPO          = SSI_SPO_LOW;
    ssi2Handle.SSIConfig.SSI_SPH          = SSI_SPH_1ST_EDGE;
    ssi2Handle.SSIConfig.SSI_FSSControl   = SSI_FSS_SW;           // software-controlled FSS

    //Call driver init
    SSI_Init(&ssi2Handle);
}

// Function to send a single byte with manual CS control
void SSI_SendByteWithCS(uint8_t data)
{
    // Pull CS low
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    
    // Small delay
    for(volatile uint32_t i = 0; i < 100; i++);
    
    // Send byte
    SSI_SendData(SSI2, &data, 1);
    
    // Small delay
    for(volatile uint32_t i = 0; i < 100; i++);
    
    // Pull CS high
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    
    // Delay between bytes
    for(volatile uint32_t i = 0; i < 1000; i++);
}

int main()
{
    //1. Init GPIO pins to behave as SSI2 pins
    SSI_GPIOInit();
    
    //2. SSI Init
    SSI_MasterInit();

    //3. Longer delay to ensure SSI is ready
    for(volatile uint32_t i = 0; i < 100000; i++);

    //4. Send test pattern first
    uint8_t test_pattern[] = {0xAA, 0x55, 0xFF, 0x00};
    for(int i = 0; i < 4; i++) {
        SSI_SendByteWithCS(test_pattern[i]);
    }
    
    //5. Delay before sending actual data
    for(volatile uint32_t i = 0; i < 100000; i++);
    
    //6. Send "Hello World" byte by byte with CS control
    char user_data[] = "Hello World";
    for(int i = 0; i < strlen(user_data); i++) {
        SSI_SendByteWithCS(user_data[i]);
    }
    
    //7. Keep the program running
    while(1);
    
    return 0;
}
