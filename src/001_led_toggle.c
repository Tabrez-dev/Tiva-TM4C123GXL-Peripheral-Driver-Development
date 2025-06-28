

/**
 * main.c
 */

#include "tm4c123x.h"
#include "tm4c123x_gpio_driver.h"


void delay(void)
{
    uint32_t i=50000;
    while(i){
      i--;
    };
}

int main(void)
{
    GPIO_Handle_t GpioLed;
    GpioLed.pGPIOx = GPIOF;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;

    GPIO_PeriClockControl(GPIOF, ENABLE);
    GPIO_Init(&GpioLed);
    while(1)
    {
        GPIO_ToggleOutputPin(GPIOF, GPIO_PIN_1);
        delay();
    }
    return 0;
}
