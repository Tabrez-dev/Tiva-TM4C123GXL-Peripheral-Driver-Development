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
 * 
 * SW1 Button -> PF4
 * */

#include "tm4c123x.h"
#include <string.h>

#define BTN_PRESSED 0U

void delay(void)
{
    uint32_t i = 50000;
    while(i){
        i--;
    };
}
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

     //5. Configure PB5 (SSI2FSS)- Hardware automatic CS control
     ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
     GPIO_Init(&ssiPins);

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

    /*for manual control of fss use this*/

    // Manual CS control during transmission
    /*GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
    SSI_SendData(SSI2, data, len);
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High*/

    // Call driver init
    SSI_Init(&ssi2Handle);
}

int main()
{
    char user_data[]="Hello World";    
    // GPIO configuration for button SW1
    GPIO_Handle_t GPIOBtn;
    GPIOBtn.pGPIOx = GPIOF;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    
    // Enable GPIO clock for button
    GPIO_PeriClockControl(GPIOF, ENABLE);
    GPIO_Init(&GPIOBtn);
    
    //1. Init GPIO pins to behave as SSI2 pins
    SSI_GPIOInit();
    //2. SSI Init
    SSI_MasterInit();
    
    // Initially disable SSI2 clock to save power
    SSI_PeriClockControl(SSI2, DISABLE);

    while(1)
    {
        if(GPIO_ReadFromInputPin(GPIOF, GPIO_PIN_4) == BTN_PRESSED)
        {
            delay();  // Debounce before reading again
            
            // Enable SSI2 peripheral clock
            SSI_PeriClockControl(SSI2, ENABLE);
            
            // Small delay for clock stabilization
            //for(volatile uint32_t i = 0; i < 1000; i++);
            
            // Better approach - with length header
            uint8_t msgLen = strlen(user_data);
            SSI_SendData(SSI2, &msgLen, 1);  // Send length first
            SSI_SendData(SSI2, user_data, msgLen);  // Send all data at once
            
            // Disable SSI2 peripheral clock to save power
            SSI_PeriClockControl(SSI2, DISABLE);
            
            // Wait until button is released (avoid repeated transmissions)
            while(GPIO_ReadFromInputPin(GPIOF, GPIO_PIN_4) == BTN_PRESSED);
            
            delay();  // Debounce after release
        }
    }
    
    return 0;
}
