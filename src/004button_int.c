/*
 * 002_ledtogggle_usr_button.c
 *
 *  Created on: 28-Jun-2025
 *      Author: tabrez
 */

#include "tm4c123x.h"
#include "tm4c123x_gpio_driver.h"
#include <string.h>
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
    memset(&GpioLed, 0, sizeof(GpioLed));
    memset(&GPIOBtn, 0, sizeof(GPIOBtn));


    GpioLed.pGPIOx = GPIOF;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;

    GPIO_PeriClockControl(GPIOF, ENABLE);
    GPIO_Init(&GpioLed);

    GPIOBtn.pGPIOx = GPIOF;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    GPIOBtn.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    
    GPIO_Init(&GPIOBtn);
    //IRQ_Config for GPIO Port F (IRQ 30)
    // Ensure this line is executed in debug mode to see NVIC_ISER0 bit 30 set for enabling the interrupt
    GPIO_IRQPriorityConfig(30, 5);
    GPIO_IRQInterruptConfig(30, 1);
    while(1)
    {
//        if(GPIO_ReadFromInputPin(GPIOF, GPIO_PIN_0)== BTN_PRESSED)
//        {
//            delay();  // Debounce before reading again
//            GPIO_ToggleOutputPin(GPIOF, GPIO_PIN_1);
//
//            // Wait until button is released (avoid repeated toggles)
//            while(GPIO_ReadFromInputPin(GPIOF, GPIO_PIN_0) == BTN_PRESSED);
//
//            delay();  // Debounce after release
//        }
    }
    return 0;
}

void GPIOF_IRQHandler(void)
{
    delay();
    //advantage of EXTI is that u dont have to worry about which port to be sent to GPIO_IRQHandling function
    GPIO_IRQHandling(GPIOF, 0);
    GPIO_ToggleOutputPin(GPIOF, 1);

}
