/*
 *003_led_toggle_ext_btn.c
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
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GpioLed);


    GPIOBtn.pGPIOx = GPIOB;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_PeriClockControl(GPIOB, ENABLE);
    GPIO_Init(&GPIOBtn);

    while(1)
    {
        if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_5)== BTN_PRESSED)
        {
            delay();  // Debounce before reading again
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);

            // Wait until button is released (avoid repeated toggles)
            while(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_5) == BTN_PRESSED);

            delay();  // Debounce after release
        }
    }
    return 0;
}
