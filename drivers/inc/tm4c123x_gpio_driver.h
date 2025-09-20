/*
 * tm4c123x_gpio_driver.h
 *
 *  Created on: 22-Jun-2025
 *      Author: tabrez
 */

#ifndef DRIVERS_INC_TM4C123X_GPIO_DRIVER_H_
#define DRIVERS_INC_TM4C123X_GPIO_DRIVER_H_


#include "tm4c123x.h"
/*
 * This is handle structure for GPIO pin
 */
typedef struct
{
    uint8_t GPIO_PinNumber;                     /*!<Possible values from @GPIO_PIN_NUMBERS*/
    uint8_t GPIO_PinMode;                       /*!<Possible values from @GPIO_PIN_MODES*/
    uint8_t GPIO_PinAltDir;                     /*!<GPIO_DIR_IN or GPIO_DIR_OUT for ALT_FN*/
    uint8_t GPIO_PinDriveStrength;              /*!<Possible values from @GPIO_PIN_DRIVE*/
    uint8_t GPIO_PinSlewRate;                   /*!<Possible values from @GPIO_SLEW_RATE*/
    uint8_t GPIO_PinPuPdControl;                /*!<Possible values from @GPIO_PULL_UP_DOWN*/
    uint8_t GPIO_PinOPType;                     /*!<Possible values from @GPIO_OUTPUT_TYPES*/
    uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is handle structure for GPIO pin
 */
typedef struct
{
    GPIO_RegDef_t *pGPIOx;                   /*!<This holds the base address of the GPIO port to which the pin belongs   >*/
    GPIO_PinConfig_t GPIO_PinConfig;          /*!< This holds GPIO pin configuration settings>*/
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO Pin Numbers
 */
#define GPIO_PIN_0    0U
#define GPIO_PIN_1    1U
#define GPIO_PIN_2    2U
#define GPIO_PIN_3    3U
#define GPIO_PIN_4    4U
#define GPIO_PIN_5    5U
#define GPIO_PIN_6    6U
#define GPIO_PIN_7    7U

/*
 * @GPIO_PIN_MODES
 * GPIO Pin Modes
 */
#define GPIO_MODE_IN          0U  /* Digital input */
#define GPIO_MODE_OUT         1U  /* Digital output */
#define GPIO_MODE_ALT_FN      2U  /* Alternate function (e.g., UART, PWM) */
#define GPIO_MODE_ANALOG      3U  /* Analog function (e.g., ADC) */
#define GPIO_MODE_IT_FT       4U  /* Input with falling edge trigger interrupt */
#define GPIO_MODE_IT_RT       5U  /* Input with rising edge trigger interrupt */
#define GPIO_MODE_IT_RFT      6U  /* Input with rising and falling edge trigger interrupt */

/*
 * @GPIO_ALT_DIR
 * Direction configuration when using alternate function mode
 */
#define GPIO_DIR_IN   0U   /*!< Alternate function pin configured as input */
#define GPIO_DIR_OUT  1U   /*!< Alternate function pin configured as output */

/*
 * @GPIO_PIN_DRIVE
 * GPIO Pin Drive (Drive Strength)
 */
#define GPIO_DRV_2MA        0U  /* 2mA drive strength */
#define GPIO_DRV_4MA        1U  /* 4mA drive strength */
#define GPIO_DRV_8MA        2U  /* 8mA drive strength */

/*
 * @GPIO_SLEW_RATE
 * GPIO Slew Rate Control
 */
#define GPIO_SLEW_OFF         0U  /* Slew rate control disabled */
#define GPIO_SLEW_ON          1U  /* Slew rate control enabled */

/*
 * @GPIO_PULL_UP_DOWN
 * GPIO Pull-Up/Pull-Down Settings
 */
#define GPIO_PIN_PUPD_NONE        0U  /* No pull-up or pull-down */
#define GPIO_PIN_PU               1U  /* Pull-up resistor */
#define GPIO_PIN_PD               2U  /* Pull-down resistor */

/*
 * @GPIO_OUTPUT_TYPES
 * GPIO Output Types
 */
#define GPIO_OPTYPE_PP  0U  /* Push-pull output */
#define GPIO_OPTYPE_OD  1U  /* Open-drain output */

/* GPIO Lock and Commit */
#define GPIO_LOCK_KEY                 0x4C4F434B  /* Unlock key for GPIOLOCK register */
#define GPIO_LOCKED_PIN_PF0           (1U << 0)  /* PF0 requires unlock */
#define GPIO_LOCKED_PIN_PD7           (1U << 7)  /* PD7 requires unlock */

// Define a port-specific macro for locked pins
#define GPIO_PORT_LOCKED_PINS(port)   ((port == GPIOF) ? GPIO_LOCKED_PIN_PF0 : \
                                      ((port == GPIOD) ? GPIO_LOCKED_PIN_PD7 : 0))

/* GPIO Speed/Drive Strength */
#define GPIO_SPEED_LOW                GPIO_DRV_2MA
#define GPIO_SPEED_MED                GPIO_DRV_4MA
#define GPIO_SPEED_HIGH               GPIO_DRV_8MA




/**************************************************************************
 *                                 APIs Supported by this driver
 *         For more information about the APIs check the function definitions
 *
 * **********************************************************************************/

/*
 * Peripheral Clock setup
 * */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint8_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);//8pins per port
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);





#endif /* DRIVERS_INC_TM4C123X_GPIO_DRIVER_H_ */
