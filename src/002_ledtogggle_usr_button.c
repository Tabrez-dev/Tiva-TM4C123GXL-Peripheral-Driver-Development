/*
 * 002_ledtogggle_usr_button.c
 *
 *  Created on: 28-Jun-2025
 *      Author: tabrez
 */

#include "tm4c123x.h"
#include "tm4c123x_gpio_driver.h"

#define BTN_PRESSED 0U

void delay(void)
{
    uint32_t i=50000;
    while(i){
      i--;
    };
}

int main(void)
{
    GPIO_Handle_t GpioLed, GPIOBtn;
    GpioLed.pGPIOx = GPIOF;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;


    GPIOBtn.pGPIOx = GPIOF;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_PeriClockControl(GPIOF, ENABLE);
    GPIO_Init(&GpioLed);
    GPIO_Init(&GPIOBtn);

    while(1)
    {
        if(GPIO_ReadFromInputPin(GPIOF, GPIO_PIN_4)== BTN_PRESSED)
        {
            delay();  // Debounce before reading again
            GPIO_ToggleOutputPin(GPIOF, GPIO_PIN_1);

            // Wait until button is released (avoid repeated toggles)
            while(GPIO_ReadFromInputPin(GPIOF, GPIO_PIN_4) == BTN_PRESSED);

            delay();  // Debounce after release
        }
    }
    return 0;
}
