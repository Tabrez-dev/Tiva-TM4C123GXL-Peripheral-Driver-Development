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
            while (!(SYSCTL_PR->PRGPIO & portBit)) {}  // Spin until ready
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
 * @fn                          -
 *
 * @brief                       -
 *
 * @param[in]                   -
 * @param[in]                   -
 * @param[in]                   -
 *
 * @return                      -
 *
 * @Note                        -
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    //1. configure mode
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {

    }
    //2. configure speed
    //3. configure pupd settings
    //4. configure output type
    //5. configure alt functionality
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
