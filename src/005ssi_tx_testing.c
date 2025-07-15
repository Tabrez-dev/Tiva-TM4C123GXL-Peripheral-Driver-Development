/*
 * 005ssi_tx_testing.c
 *
 *  Created on: 14-Jul-2025
 *      Author: tabrez
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

     //5. Configure PB5 (SSI2FSS)
     //ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
     //GPIO_Init(&ssiPins);

     //6. Configure PB6 (SSI2Rx)
     //ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
     //GPIO_Init(&ssiPins);

     //7. Configure PB7 (SSI2Tx)
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

int main()
{
    char user_data[]="Hello World";
    //1. Init GPIO pins to behave as SSI2 pins
    SSI_GPIOInit();
    //2. SSI Init
    SSI_MasterInit();

    //3. Small delay to ensure SSI is ready
    for(volatile uint32_t i = 0; i < 10000; i++);

    //4. Send data byte by byte with small delays
    for(int i = 0; i < strlen(user_data); i++) {
        SSI_SendData(SSI2, (uint8_t *)&user_data[i], 1);
        // Small delay between bytes
        for(volatile uint32_t j = 0; j < 1000; j++);
    }
    
    //5. Keep the program running
    while(1);
    return 0;
}
