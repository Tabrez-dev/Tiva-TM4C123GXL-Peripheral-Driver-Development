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
            uint32_t portBit = (pGPIOx == GPIOA) ? (1U<<0) :
                               (pGPIOx == GPIOB) ? 1U<<1 :
                               (pGPIOx == GPIOC) ? 1U<<2 :
                               (pGPIOx == GPIOD) ? 1U<<3 :
                               (pGPIOx == GPIOE) ? 1U<<4 :
                               1U<<5;
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
 *                      - Automatically enables GPIO peripheral clock internally.
 *                      - You do NOT need to call GPIO_PeriClockControl() separately.
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    // --- Input Validation ---
    // Ensure the handle and GPIO port pointer are valid
    if (!pGPIOHandle || !pGPIOHandle->pGPIOx)
    {
        return; // Exit if handle or port is null
    }

    // Check if pin number is valid (TM4C123 ports have 8 pins, 0-7)
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber > GPIO_PIN_7)
    {
        return; // Exit if pin number exceeds 7
    }

    // Verify mode is within supported range
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode > GPIO_MODE_IT_RFT)
    {
        return; // Exit if mode is invalid
    }

    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    // --- Local Variables for Clarity ---
    GPIO_RegDef_t *port = pGPIOHandle->pGPIOx;                // GPIO port base address
    uint8_t pin         = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber; // Pin number (0-7)
    uint32_t mask       = (1U << pin);                        // Bit mask for the pin

    // --- Special Case: JTAG/SWD Pins ---
    // Prevent reconfiguration of pins reserved for JTAG/SWD or NMI
    if ((port == GPIOC && pin <= 3) || 
        (port == GPIOD && pin == 7 && (SYSCTL_CORE->DID0 & 0x00FF0000) <= 0x00010000)) 
    {
        return; // Exit if pin is JTAG/SWD (PC0-PC3) or NMI (PD7 on certain revisions)
    }

    /* Check if the pin requires unlocking based on the port */
    if ((port == GPIOF && (mask & GPIO_LOCKED_PIN_PF0)) ||
        (port == GPIOD && (mask & GPIO_LOCKED_PIN_PD7))) 
    {
        port->LOCK = GPIO_LOCK_KEY;  /* Unlock the port */
        port->CR |= mask;            /* Commit the pin configuration */
    }

    // --- 1. Clear Previous Configuration ---
    // Reset all relevant registers to a known state
    port->DEN   &= ~mask;  // Disable digital function
    port->AFSEL &= ~mask;  // Disable alternate function
    port->AMSEL &= ~mask;  // Disable analog function
    port->DIR   &= ~mask;  // Set pin as input (default)
    port->PCTL  &= ~(0xFU << (pin * 4)); // Clear alternate function selection
    port->DR2R  &= ~mask;  // Clear 2mA drive
    port->DR4R  &= ~mask;  // Clear 4mA drive
    port->DR8R  &= ~mask;  // Clear 8mA drive
    port->ODR   &= ~mask;  // Disable open-drain
    port->PUR   &= ~mask;  // Disable pull-up
    port->PDR   &= ~mask;  // Disable pull-down

    // --- 2. Configure Pin Mode ---
    uint8_t mode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
    if (mode <= GPIO_MODE_ANALOG) 
    {
        // Non-Interrupt Modes (Input, Output, Analog, Alternate Function)
        if (mode == GPIO_MODE_ANALOG) 
        {
            port->AMSEL |= mask;  // Enable analog mode (disables digital)
            // DEN remains 0
        } 
        else 
        {
            port->DEN |= mask;    // Enable digital mode
            if (mode == GPIO_MODE_OUT)
            {
                port->DIR |= mask; // Set pin as output
            }
            else if (mode == GPIO_MODE_ALT_FN)
            {
                port->AFSEL |= mask; // Enable alternate function
                // Set Alternate Function in PCTL
                uint32_t shift = pin * 4;
                uint32_t altfun = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode & 0xF;
                port->PCTL = (port->PCTL & ~(0xF << shift)) | (altfun << shift);
                if (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltDir == GPIO_DIR_OUT)
                {
                    port->DIR |= mask;
                }
                else
                {
                    port->DIR &= ~mask;
                }
            }
            // GPIO_MODE_IN: DIR remains 0 (input) by default
        }
    } 
    else
    {
        //1. Configure the GPIO pin as **input** mode  
        // Interrupt Modes (Rising Edge, Falling Edge, Both Edges)
        port->DEN  |= mask;   // Enable digital function
        port->DIR  &= ~mask;  // Ensure pin is input

        // 2. Configure interrupt trigger type
        if (mode == GPIO_MODE_IT_RT) 
        {
            port->IS  &= ~mask; // Edge-sensitive (not level)
            port->IBE &= ~mask; // Single edge (not both)
            port->IEV |= mask;  // Rising edge

        } 
        else if (mode == GPIO_MODE_IT_FT) 
        {
            port->IS  &= ~mask; // Edge-sensitive
            port->IBE &= ~mask; // Single edge
            port->IEV &= ~mask; // Falling edge
        } 
        else if (mode == GPIO_MODE_IT_RFT) 
        {
            port->IS  &= ~mask; // Edge-sensitive
            port->IBE |= mask;  // Both edges
        }
        //3. Enable the interrupt at the **GPIO peripheral level**  
        // Clear any pending interrupt and enable interrupt mask
        port->ICR |= mask; // Clear interrupt flag
        port->IM  |= mask; // Enable interrupt for this pin
        // Note: NVIC setup (IRQ number and enable) is handled separately
       
    }

    // --- 3. Set Drive Strength ---
    // Configure output current capability (2mA, 4mA, or 8mA)
    uint8_t speed = pGPIOHandle->GPIO_PinConfig.GPIO_PinDriveStrength;
    if (speed == GPIO_SPEED_LOW) 
    {
        port->DR2R |= mask; // 2mA drive
    } else if (speed == GPIO_SPEED_MED) 
    {
        port->DR4R |= mask; // 4mA drive
    } else 
    { // GPIO_SPEED_HIGH
        port->DR8R |= mask; // 8mA drive
    }

    // --- 3.5. Configure Slew Rate ---
    // Enable or disable slew rate control for smoother transitions
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinSlewRate == GPIO_SLEW_ON) 
    {
        port->SLR |= mask;  // Enable slew rate control
    } 
    else 
    {
        port->SLR &= ~mask; // Disable slew rate control
    }

    // --- 4. Set Output Type and Pull Resistors ---
    uint8_t otype = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType;
    uint8_t pupd  = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl;

    if (otype == GPIO_OPTYPE_OD) 
    {
        port->ODR |= mask; // Set pin as open-drain
    }
    else // GPIO_OPTYPE_PP
    {
        port->ODR &= ~mask; // Set pin as push-pull (default)
    }
    
    if (pupd == GPIO_PIN_PU) 
    {
        port->PUR |= mask; // Enable pull-up resistor
    } else if (pupd == GPIO_PIN_PD) 
    {
        port->PDR |= mask; // Enable pull-down resistor
    }
    // GPIO_NO_PUPD: No pull resistors (default)

    // --- 5. Re-lock Protected Pins ---
    // Restore lock state for protected pins
    if (mask & GPIO_PORT_LOCKED_PINS(port))
    {
        port->LOCK = 0; // Lock the pin again
    }
}

/***************************************************************************
 * @fn                  GPIO_DeInit
 *
 * @brief               De-initializes a GPIO port, resetting it to default state
 *
 * @param[in]           pGPIOx Pointer to the GPIO port base address (e.g., GPIOA, GPIOB)
 *
 * @return              None
 *
 * @note                Uses the Software Reset Control Register (SRCR2) to perform
 *                      a complete peripheral reset, which is more thorough than
 *                      just disabling the clock.
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (!pGPIOx) return;  // Null pointer check
    
    // Reset the appropriate GPIO port
    if (pGPIOx == GPIOA)
    {
        GPIOA_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_RESET();
    }
}
/***************************************************************************
 * @fn GPIO_ReadFromInputPin
 *
 * @brief Reads the state of a specific GPIO input pin (high or low)
 *
 * @param[in] pGPIOx: Pointer to the GPIO port base address (e.g., GPIOA, GPIOB)
 * @param[in] PinNumber: The pin number to read (0 to 7)
 *
 * @return uint8_t: State of the pin (1 for high, 0 for low)
 *
 * @note Returns 0 if the input parameters are invalid or if the pin is not configured as input.
 * Assumes the GPIO peripheral clock is enabled and the pin is configured as input.
 * The DATA register uses address masking [9:2] for pin-specific access.
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    if (!pGPIOx || PinNumber > GPIO_PIN_7)
    {
        return 0;
    }
    
    uint32_t mask = (1U << PinNumber);
    // Convert byte offset to word index: (mask << 2) >> 2 = mask
    return (pGPIOx->DATA[mask] & mask) ? 1 : 0;
}

/***************************************************************************
 * @fn GPIO_ReadFromInputPort
 *
 * @brief Reads the state of all 8 GPIO input pins of a port
 *
 * @param[in] pGPIOx: Pointer to the GPIO port base address (e.g., GPIOA, GPIOB)
 *
 * @return uint8_t: State of the port pins (each bit represents a pin, 1 for high, 0 for low)
 *
 * @note Returns 0 if the input parameter is invalid.
 * Assumes the GPIO peripheral clock is enabled and the pins are configured as input.
 * The DATA register uses address masking [9:2] for pin-specific access; a mask of 0xFF reads all 8 pins.
 */
uint8_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    if (!pGPIOx)
    {
        return 0;
    }
    
    // 0xFF means: address bits [9:2] = 11111111 → all 8 bits accessed
    // Convert byte offset to word index: (0xFF << 2) >> 2 = 0xFF
    return (uint8_t)(pGPIOx->DATA[0xFF]);
}

/***************************************************************************
 * @fn GPIO_WriteToOutputPin
 *
 * @brief Writes a value (high or low) to a specific GPIO output pin
 *
 * @param[in] pGPIOx: Pointer to the GPIO port base address (e.g., GPIOA, GPIOB)
 * @param[in] PinNumber: The pin number to write to (0 to 7)
 * @param[in] value: The value to write (GPIO_PIN_SET for high, GPIO_PIN_RESET for low)
 *
 * @return None
 *
 * @note Does nothing if the input parameters are invalid.
 * Assumes the GPIO peripheral clock is enabled and the pin is configured as output.
 * The DATA register uses address masking [9:2] for pin-specific access.
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
    if (!pGPIOx || PinNumber > GPIO_PIN_7)
    {
        return;
    }
    
    uint32_t mask = (1U << PinNumber);
    
    if (value == GPIO_PIN_SET)
    {
        // Convert byte offset to word index: (mask << 2) >> 2 = mask
        pGPIOx->DATA[mask] = mask; // Set bit
    }
    else
    {
        pGPIOx->DATA[mask] = 0; // Clear bit
    }
}

/***************************************************************************
 * @fn GPIO_WriteToOutputPort
 *
 * @brief Writes values to all 8 GPIO output pins of a port
 *
 * @param[in] pGPIOx: Pointer to the GPIO port base address (e.g., GPIOA, GPIOB)
 * @param[in] value: The 8-bit value to write to the port (each bit represents a pin)
 *
 * @return None
 *
 * @note Does nothing if the input parameter is invalid.
 * Assumes the GPIO peripheral clock is enabled and the pins are configured as output.
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value)
{
    if (!pGPIOx)
    {
        return;
    }
    
    // Write to all 8 pins: 0xFF
    pGPIOx->DATA[0xFF] = value;
}

/***************************************************************************
 * @fn GPIO_ToggleOutputPin
 *
 * @brief Toggles the state of a specific GPIO output pin using XOR operation
 *
 * @param[in] pGPIOx: Pointer to the GPIO port base address (e.g., GPIOA, GPIOB)
 * @param[in] PinNumber: The pin number to toggle (0 to 7)
 *
 * @return None
 *
 * @note Does nothing if the input parameters are invalid.
 * Assumes the GPIO peripheral clock is enabled and the pin is configured as output.
 * The DATA register uses address masking [9:2] for pin-specific access.
 * XOR with pin mask efficiently toggles: 0 XOR 1 = 1, 1 XOR 1 = 0
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    if (!pGPIOx || PinNumber > GPIO_PIN_7)
    {
        return;
    }
    
    uint32_t mask = (1U << PinNumber);
    
    // Toggle using XOR: current_state XOR mask = toggled_state
    pGPIOx->DATA[mask] = pGPIOx->DATA[mask] ^ mask;
}

/*
 * IRQ configuration and ISR handling
 */
/***************************************************************************
 * @fn                  GPIO_IRQInterruptConfig
 *
 * @brief               Configures the NVIC to enable or disable interrupts for a GPIO port
 *
 * @param[in]           IRQNumber: The IRQ number corresponding to the GPIO port
 * @param[in]           EnorDi: Enable (1) or Disable (0) the interrupt
 *
 * @return              None
 *
 * @note                Uses NVIC registers to enable/disable interrupts.
 *                      IRQ numbers for GPIO ports are:
 *                      - Port A: 0, Port B: 1, Port C: 2, Port D: 3, Port E: 4, Port F: 30
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        // Enable IRQ in NVIC using the NVIC_ENx registers
        if (IRQNumber < 32) {
            NVIC_EN0 |= (1 << IRQNumber);
        } else if (IRQNumber < 64) {
            NVIC_EN1 |= (1 << (IRQNumber - 32));
        } else if (IRQNumber < 96) {
            NVIC_EN2 |= (1 << (IRQNumber - 64));
        } else if (IRQNumber < 128) {
            NVIC_EN3 |= (1 << (IRQNumber - 96));
        }
    }
    else
    {
        // Disable IRQ in NVIC using the NVIC_DISx registers
        if (IRQNumber < 32) {
            NVIC_DIS0 |= (1 << IRQNumber);
        } else if (IRQNumber < 64) {
            NVIC_DIS1 |= (1 << (IRQNumber - 32));
        } else if (IRQNumber < 96) {
            NVIC_DIS2 |= (1 << (IRQNumber - 64));
        } else if (IRQNumber < 128) {
            NVIC_DIS3 |= (1 << (IRQNumber - 96));
        }
    }
}

/***************************************************************************
 * @fn                  GPIO_IRQPriorityConfig
 *
 * @brief               Sets the priority for a specific IRQ in the NVIC
 *
 * @param[in]           IRQNumber: The IRQ number corresponding to the GPIO port
 * @param[in]           IRQPriority: Priority level for the interrupt (0-7)
 *
 * @return              None
 *
 * @note                Sets the priority using NVIC IP registers.
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
    // Set priority
    uint8_t iprx = IRQNumber / 4;
    uint8_t section = IRQNumber % 4;
    // ARM Cortex-M4 usually supports 3 bits for priority (high 3 bits of 8)
    uint32_t shift = (8 * section) + (8 - __NVIC_PRIO_BITS); // __NVIC_PRIO_BITS = 3
    
    // Calculate the address of the appropriate NVIC_PRIx register
    volatile uint32_t* priReg = (volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + (iprx * 4));
    
    // Update the priority
    *priReg = (*priReg & ~(0xFF << shift)) | ((IRQPriority & 0x7) << shift);
}


/***************************************************************************
 * @fn                  GPIO_IRQHandling
 *
 * @brief               Handles GPIO interrupt for a specific pin on a given port
 *
 * @param[in]           pGPIOx: Pointer to the GPIO port base address (e.g., GPIOA, GPIOB)
 * @param[in]           PinNumber: The pin number (0-7) that triggered the interrupt
 *
 * @return              None
 *
 * @note                This function should be called from the specific GPIO port handler.
 *                      It checks both RIS (Raw Interrupt Status) and MIS (Masked Interrupt Status)
 *                      registers to confirm the interrupt source and clears the interrupt flag in ICR.
 *                      Custom handling logic can be added.
 */
void GPIO_IRQHandling(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    if (!pGPIOx || PinNumber > GPIO_PIN_7)
    {
        return; // Input validation
    }
    
    uint32_t mask = (1U << PinNumber);
    // Check Raw Interrupt Status to see if the pin meets interrupt conditions
    if (pGPIOx->RIS & mask)
    {
        // Check Masked Interrupt Status to confirm if it's sent to the interrupt controller
        if (pGPIOx->MIS & mask)
        {
            pGPIOx->ICR |= mask; // Clear interrupt
            // Add custom handling logic here, e.g., toggle a flag or call a callback
        }
    }
}
