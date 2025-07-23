/*
 * 005ssi_tx_simple_test.c
 *
 *  Created on: 15-Jul-2025
 *      Author: tabrez
 *  
 *  Simple test - sends 'A' repeatedly
 */

/*
 * PB4->SSI2CLK
 * PB5->SSI2FSS
 * PB6->SSI2RX
 * PB7->SSI2TX
 * AF=2
 * */

#include "tm4c123x.h"

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

    //3. Configure PB4 (SSI2CLK)
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    GPIO_Init(&ssiPins);

    //4. Configure PB7 (SSI2Tx)
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    GPIO_Init(&ssiPins);
}

void SSI_MasterInit(void)
{
    SSI_Handle_t ssi2Handle;

    ssi2Handle.pSSIx = SSI2;

    ssi2Handle.SSIConfig.SSI_DeviceMode   = SSI_DEVICE_MODE_MASTER;
    ssi2Handle.SSIConfig.SSI_BusConfig    = SSI_BUS_CONFIG_FD;   // full duplex
    ssi2Handle.SSIConfig.SSI_SclkSpeed    = SSI_SCLK_SPEED_DIV16; // Slower speed for debugging
    ssi2Handle.SSIConfig.SSI_DSS          = SSI_DSS_8BIT;         // 8-bit frame
    ssi2Handle.SSIConfig.SSI_FRF          = SSI_FRF_SSI_FreeScale;
    ssi2Handle.SSIConfig.SSI_SPO          = SSI_SPO_LOW;
    ssi2Handle.SSIConfig.SSI_SPH          = SSI_SPH_1ST_EDGE;
    //Call driver init
    SSI_Init(&ssi2Handle);
}

int main()
{
    uint8_t test_char = 'A';  // 0x41 in hex
    
    //1. Init GPIO pins to behave as SSI2 pins
    SSI_GPIOInit();
    
    //2. SSI Init
    SSI_MasterInit();

    //3. Delay to ensure SSI is ready
    for(volatile uint32_t i = 0; i < 100000; i++);

    //4. Send 'A' repeatedly with delays
    while(1) {
        // Send single character
        SSI_SendData(SSI2, &test_char, 1);
        
        // Long delay between transmissions
        for(volatile uint32_t i = 0; i < 1000000; i++);
    }
    
    return 0;
}
