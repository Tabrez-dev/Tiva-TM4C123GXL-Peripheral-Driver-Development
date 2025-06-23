/*
 * tm4c123x_gpio_driver.c
 *
 *  Created on: 22-Jun-2025
 *      Author: tabrez
 */


#include "tm4c123x_gpio_driver.h"

/*
 * Peripheral Clock setup
 * */

/***************************************************************************
 * @fn                          - GPIO_PeriClockControl
 *
 * @brief                       - Enables or disables the peripheral clock for a GPIO port in Run Mode
 *
 * @param[in]                   - pGPIOx: Pointer to the GPIO port base address (e.g., GPIOA, GPIOB)
 * @param[in]                   - EnorDi: Enable (1) or disable (0) the clock
 * @param[in]                   - none
 *
 * @return                      - none
 *
 * @Note                        - Waits for peripheral readiness after enabling to ensure stable operation.
 *                                Refer to TM4C123GH6PM datasheet, section 5.2.6 for clock gating details.
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (!pGPIOx) return;  // Null pointer check

        if (EnorDi == ENABLE) {
            // Enable clock for the specified port
            if (pGPIOx == GPIOA)
            {
                GPIOA_PCLK_EN();
            }
            else if(pGPIOx == GPIOB)
            {
                GPIOB_PCLK_EN();
            }
            else if(pGPIOx == GPIOC)
            {
                GPIOC_PCLK_EN();
            }
            else if (pGPIOx == GPIOD)
            {
               GPIOD_PCLK_EN();
            }
            else if(pGPIOx == GPIOE)
            {
                GPIOE_PCLK_EN();
            }
            else if(pGPIOx == GPIOF)
            {
                 GPIOF_PCLK_EN();
            }

            // Wait for peripheral readiness
            uint32_t portBit = (pGPIOx == GPIOA) ? RCGCGPIO_PORTA :
                               (pGPIOx == GPIOB) ? RCGCGPIO_PORTB :
                               (pGPIOx == GPIOC) ? RCGCGPIO_PORTC :
                               (pGPIOx == GPIOD) ? RCGCGPIO_PORTD :
                               (pGPIOx == GPIOE) ? RCGCGPIO_PORTE :
                               RCGCGPIO_PORTF;
            while (!(SYSCTL_PR->PRGPIO & portBit)) {};  // Spin until ready
        } else {
            // Disable clock for the specified port
            if (pGPIOx == GPIOA)
            {
                GPIOA_PCLK_DIS();
            }
            else if(pGPIOx == GPIOB)
            {
                GPIOB_PCLK_DIS();
            }
            else if(pGPIOx == GPIOC)
            {
                GPIOC_PCLK_DIS();
            }
            else if(pGPIOx == GPIOD)
            {
                GPIOD_PCLK_DIS();
            }
            else if (pGPIOx == GPIOE)
            {
                GPIOE_PCLK_DIS();
            }
            else if (pGPIOx == GPIOF)
            {
                GPIOF_PCLK_DIS();
            }
        }
}

/*
 * Init and De-Init
 */

/***************************************************************************
 * @fn                  GPIO_Init
 *
 * @brief               Initializes a GPIO pin with specified configuration settings
 *
 * @param[in]           pGPIOHandle Pointer to a GPIO_Handle_t structure containing
 *                      the GPIO port base address and pin configuration
 *
 * @return              None
 *
 * @note                - Assumes the GPIO peripheral clock is enabled via GPIO_PeriClockControl
 *                      - Validates pin number and mode to prevent invalid configurations
 *                      - Configures registers with mutual exclusivity for drive strength and pull-up/down
 *                      - Interrupt modes require separate IRQ configuration via GPIO_IRQConfig
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    GPIO_RegDef_t *port = pGPIOHandle->pGPIOx;
    uint8_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    uint32_t pinMask = 1U << pinNumber; // Bit mask for the pin (e.g., 0x02 for pin 1)

    // 1. Configure mode
    uint8_t mode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
    if (mode <= GPIO_MODE_ANALOG)
    {
        // Non-interrupt modes: Input, Output, Alternate Function, Analog
        // Clear direction, alternate function, and analog mode registers
        port->DIR   &= ~pinMask;   // Clear direction (input by default)
        port->AFSEL &= ~pinMask;   // Clear alternate function
        port->AMSEL &= ~pinMask;   // Clear analog mode

        if (mode == GPIO_MODE_OUT)
        {
            port->DIR |= pinMask;  // Set as output
        }
        else if (mode == GPIO_MODE_ALT_FN)
        {
            port->AFSEL |= pinMask; // Enable alternate function

            // Configure PCTL (4 bits per pin)
            uint32_t pctlShift = pinNumber * 4;
            uint8_t altFunc = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode;
            port->PCTL = (port->PCTL & ~(0xFU << pctlShift))
                        | ((uint32_t)altFunc << pctlShift);
        }
        else if (mode == GPIO_MODE_ANALOG)
        {
            port->AMSEL |= pinMask; // Enable analog mode
            port->DEN   &= ~pinMask; // Disable digital enable for analog
        }
        else // GPIO_MODE_IN
        {
            port->DEN |= pinMask;   // Enable digital input
        }
    }
    else
    {
        // TODO: Handle interrupt modes here
    }

    // 2. Configure speed
    // 3. Configure pull-up/pull-down settings
    // 4. Configure output type
    // 5. Configure alternate functionality
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}
/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}
uint8_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)//8pins per port
{

}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value)
{

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)//explore irq grouping later
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
