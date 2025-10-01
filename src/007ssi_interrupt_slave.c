/*
 * 007ssi_interrupt_slave.c
 *
 *  Created on: 20-Sep-2025
 *      Author: tabrez
 *
 * DESCRIPTION:
 * TM4C123GXL (master) receives string messages from STM32F407VGTx (slave)
 * using interrupt-driven GPIO signaling and blocking SPI communication.
 * Displays received messages via UART1 output.
 *
 * OPERATION:
 * 1. Master waits for data available signal on PD6 (GPIO interrupt)
 * 2. When interrupt fires, master reads message character-by-character via SSI2
 * 3. Master sends dummy bytes to generate SPI clock, slave responds with message characters
 * 4. Message ends when slave sends null terminator '\0'
 * 5. Master displays complete message via UART1 and waits for next signal
 * 6. Cycle repeats for next message in sequence
 *
 * IMPLEMENTATION DETAILS:
 * - Uses blocking SPI calls (SSI_SendData/SSI_ReceiveData) rather than interrupts
 * - GPIO interrupt on PD6 triggers message read sequence
 * - Small delays between SPI transactions for slave processing time
 * - UART1 output for real-time message monitoring
 * - Handles slave state management through null terminator detection
 */

/*
 * HARDWARE CONNECTIONS:
 *
 * SPI Communication (SSI2 ↔ SPI2):
 * TM4C123GXL (Master)     STM32F407VGTx (Slave)
 * PB4 → SSI2CLK           PB13 → SPI2_SCLK
 * PB7 → SSI2TX (MOSI)     PB15 → SPI2_MOSI
 * PB6 → SSI2RX (MISO)     PB14 → SPI2_MISO
 * PB5 → SSI2FSS (CS)      PB12 → SPI2_NSS
 *
 * Data Available Signaling:
 * PD6 → GPIO Input        PC6 → Data Available Signal
 *
 * UART Debug Output:
 * PB0 → UART1_RX (unused)
 * PB1 → UART1_TX (connect to USB-TTL adapter)
 */

#include "tm4c123x_ssi_driver.h"
#include "tm4c123x_uart_driver.h"
#include <string.h>

/* Handle structures */
SSI_Handle_t SSI2handle;
UART_Handle_t UART1handle;

/* Communication buffers */
#define MAX_LEN 500
char RcvBuff[MAX_LEN];
volatile char ReadByte;

/* Control flags */
volatile uint8_t rcvStop = 0;
volatile uint8_t dataAvailable = 0;

/* Function prototypes */
void delay(void);
void SSI2_GPIOInits(void);
void SSI2_Inits(void);
void UART1_GPIOInits(void);
void UART1_Inits(void);
void Slave_GPIO_InterruptPinInit(void);

/***************************************************************************
 * @fn                          - delay
 *
 * @brief                       - Simple delay function
 */
void delay(void)
{
    for(uint32_t i = 0; i < 500000/2; i++);
}

/***************************************************************************
 * @fn                          - SSI2_GPIOInits
 *
 * @brief                       - Initializes GPIO pins for SSI2 operation
 */
void SSI2_GPIOInits(void)
{
    GPIO_Handle_t ssiPins;

    // Enable GPIOB clock first
    GPIO_PeriClockControl(GPIOB, ENABLE);

    //1. Set the GPIO port to PORTB
    ssiPins.pGPIOx = GPIOB;

    //2. Common configuration for all SSI2 pins
    ssiPins.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode  = 2;               // AF = 2 for SSI2
    ssiPins.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OPTYPE_PP;
    ssiPins.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_SPEED_MED;
    ssiPins.GPIO_PinConfig.GPIO_PinSlewRate    = GPIO_SLEW_OFF;
    ssiPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;

    //3. Configure PB4 (SSI2CLK) - OUTPUT
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_OUT;  // Master drives clock
    GPIO_Init(&ssiPins);

    //4. Configure PB5 (SSI2FSS) - Hardware CS control for interrupt mode
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_OUT;  // Master drives CS
    GPIO_Init(&ssiPins);

    //5. Configure PB6 (SSI2RX) - MISO
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_IN;   // Master receives data
    GPIO_Init(&ssiPins);

    //6. Configure PB7 (SSI2TX) - MOSI
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_OUT;  // Master sends data
    GPIO_Init(&ssiPins);
}

/***************************************************************************
 * @fn                          - SSI2_Inits
 *
 * @brief                       - Initializes SSI2 peripheral for master mode
 */
void SSI2_Inits(void)
{
    SSI2handle.pSSIx = SSI2;

    SSI2handle.SSIConfig.SSI_DeviceMode   = SSI_DEVICE_MODE_MASTER;
    SSI2handle.SSIConfig.SSI_BusConfig    = SSI_BUS_CONFIG_FD;   // full duplex
    SSI2handle.SSIConfig.SSI_SclkSpeed    = SSI_SCLK_SPEED_DIV32; // SysClk/32 for compatibility
    SSI2handle.SSIConfig.SSI_DSS          = SSI_DSS_8BIT;         // 8-bit frame
    SSI2handle.SSIConfig.SSI_FRF          = SSI_FRF_SSI_Freescale;
    SSI2handle.SSIConfig.SSI_SPO          = SSI_SPO_LOW;
    SSI2handle.SSIConfig.SSI_SPH          = SSI_SPH_1ST_EDGE;

    //Initialize SSI handle with default states
    SSI2handle.pTxBuffer = NULL;
    SSI2handle.pRxBuffer = NULL;
    SSI2handle.TxLen = 0;
    SSI2handle.RxLen = 0;
    SSI2handle.TxState = SSI_READY;
    SSI2handle.RxState = SSI_READY;

    //Call driver init
    SSI_Init(&SSI2handle);
}

/***************************************************************************
 * @fn                          - UART1_GPIOInits
 *
 * @brief                       - Initializes GPIO pins for UART1 operation
 */
void UART1_GPIOInits(void)
{
    GPIO_Handle_t uartPins;

    //1. Enable GPIOB clock and set the GPIO port to PORTB
    GPIO_PeriClockControl(GPIOB, ENABLE);
    uartPins.pGPIOx = GPIOB;

    //2. Common configuration for UART1 pins
    uartPins.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALT_FN;
    uartPins.GPIO_PinConfig.GPIO_PinAltFunMode  = 1;               // AF = 1 for UART1
    uartPins.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OPTYPE_PP;
    uartPins.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_SPEED_MED;
    uartPins.GPIO_PinConfig.GPIO_PinSlewRate    = GPIO_SLEW_OFF;
    uartPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;

    //3. Configure PB0 (UART1RX) - not used in this application
    uartPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
    GPIO_Init(&uartPins);

    //4. Configure PB1 (UART1TX)
    uartPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
    GPIO_Init(&uartPins);
}

/***************************************************************************
 * @fn                          - UART1_Inits
 *
 * @brief                       - Initializes UART1 peripheral
 */
void UART1_Inits(void)
{
    UART1handle.pUARTx = UART1;

    UART1handle.UARTConfig.UART_BaudRate = UART_BAUD_115200;
    UART1handle.UARTConfig.UART_WordLength = UART_WORDLEN_8BITS;
    UART1handle.UARTConfig.UART_StopBits = UART_STOPBITS_1;
    UART1handle.UARTConfig.UART_Parity = UART_PARITY_NONE;
    UART1handle.UARTConfig.UART_HWFlowControl = UART_HW_FLOW_CTRL_NONE;

    // Step 1: Test SYSCTL read access
    GPIO_WriteToOutputPin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
    volatile uint32_t test_read = SYSCTL_RUNCLK->RCGCUART;
    GPIO_WriteToOutputPin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);

    // Step 2: Test SYSCTL write access
    GPIO_WriteToOutputPin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
    SYSCTL_RUNCLK->RCGCUART |= (1U << 1);  // Enable UART1 clock
    GPIO_WriteToOutputPin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);

    // Step 3: Test peripheral ready wait
    GPIO_WriteToOutputPin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
    while(!(SYSCTL_PR->PRUART & (1U << 1)));
    GPIO_WriteToOutputPin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);

    // Step 4: Test UART register access
    GPIO_WriteToOutputPin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
    UART1->CTL = 0;  // Disable UART
    UART1->IBRD = 8;
    UART1->FBRD = 44;
    UART1->LCRH = (0x3 << 5) | (1 << 4);
    UART1->CTL = (1 << 0) | (1 << 8) | (1 << 9);
    GPIO_WriteToOutputPin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
}

/***************************************************************************
 * @fn                          - Slave_GPIO_InterruptPinInit
 *
 * @brief                       - Configures PD6 as interrupt input for data available signal
 */
void Slave_GPIO_InterruptPinInit(void)
{
    GPIO_Handle_t dataAvailPin;

    dataAvailPin.pGPIOx = GPIOD;
    dataAvailPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
    dataAvailPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT; // Rising edge trigger
    dataAvailPin.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_SPEED_LOW;
    dataAvailPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull-up

    GPIO_Init(&dataAvailPin);

    // Configure interrupt priority and enable
    GPIO_IRQPriorityConfig(IRQ_NO_GPIOD, 15);
    GPIO_IRQInterruptConfig(IRQ_NO_GPIOD, ENABLE);
}

/***************************************************************************
 * @fn                          - main
 *
 * @brief                       - Main application function
 */
int main(void)
{
    uint8_t dummy = 0xFF;

    // Initialize LED for basic debugging (PF1 - Red LED)
    GPIO_Handle_t led;
    led.pGPIOx = GPIOF;
    led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
    led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
    led.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_SPEED_MED;
    led.GPIO_PinConfig.GPIO_PinSlewRate = GPIO_SLEW_OFF;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;

    GPIO_PeriClockControl(GPIOF, ENABLE);
    GPIO_Init(&led);

    // Initialize UART for output
    UART1_GPIOInits();
    UART1_Inits();

    // Test UART immediately after init
    UART_SendString(&UART1handle, "UART Test\r\n");

    // Add delay to ensure transmission
    delay();
    delay();

    // Send startup message
    UART_SendString(&UART1handle, "\r\n=== TM4C123GXL SSI Interrupt Master ===\r\n");
    UART_SendString(&UART1handle, "Waiting for data from STM32F407VGTx slave...\r\n");

    // Initialize data available interrupt
    Slave_GPIO_InterruptPinInit();

    // Initialize SSI2 pins and peripheral
    SSI2_GPIOInits();
    SSI2_Inits();

    // Configure SSI2 interrupt
    SSI_IRQInterruptConfig(IRQ_NO_SSI2, ENABLE);

    // Ensure SSI peripheral is enabled by setting SSE bit in CR1
    SSI2->CR1 |= (1U << 1);  // Set SSE bit (bit 1) to enable SSI

    // Debug: Test UART output first
    UART_SendString(&UART1handle, "Master ready - waiting for slave data...\r\n");

    while(1)
    {
        // Wait for slave to signal data available
        while(!dataAvailable) {
            static uint32_t debug_counter = 0;
            debug_counter++;
            if (debug_counter >= 1000000) {
                UART_SendString(&UART1handle, "Waiting for slave...\r\n");
                debug_counter = 0;
            }
        }

        UART_SendString(&UART1handle, "Slave ready! Reading message...\r\n");

        // Shorter wait for fast slave response (no LED delays)
        for(volatile uint32_t d = 0; d < 50000; d++);

        // Clear buffer and prepare for clean message read
        memset(RcvBuff, 0, MAX_LEN);
        rcvStop = 0;

        // Read characters until null terminator with proper timing
        uint8_t msg_len = 0;
        for(uint8_t i = 0; i < 20; i++) {  // Reasonable limit for message length
            // Simple blocking SPI exchange
            SSI_SendData(SSI2, &dummy, 1);      // Send dummy byte to generate clock
            SSI_ReceiveData(SSI2, (uint8_t*)&ReadByte, 1);  // Read response from slave

            // Reduced delay for fast slave response (no LED delays in slave interrupt)
            for(volatile uint32_t d = 0; d < 10000; d++);

            // Only store valid characters (not null on first read)
            if(ReadByte != '\0' || i > 0) {
                RcvBuff[i] = ReadByte;
                if(ReadByte == '\0') {
                    msg_len = i;
                    break;
                }
            } else if(i == 0 && ReadByte == '\0') {
                // If first character is null, slave not ready yet - wait and try again
                for(volatile uint32_t d = 0; d < 50000; d++);
                i--;  // Retry this position
                continue;
            }
        }

        // Ensure null termination
        if(msg_len < MAX_LEN - 1) {
            RcvBuff[msg_len] = '\0';
        }

        // Print received message
        UART_SendString(&UART1handle, "Received: ");
        UART_SendString(&UART1handle, RcvBuff);
        UART_SendString(&UART1handle, "\r\n\r\n");

        // Reset for next message and wait longer
        dataAvailable = 0;

        // Shorter delay for faster system (no LED delays in slave)
        // Provides enough separation without excessive waiting
        for(int i = 0; i < 5; i++) {
            delay();
        }
    }

    return 0;
}

/***************************************************************************
 * @fn                          - SSI2_IRQHandler
 *
 * @brief                       - SSI2 interrupt service routine
 */
void SSI2_IRQHandler(void)
{
    SSI_IRQHandling(&SSI2handle);
}

/***************************************************************************
 * @fn                          - SSI_ApplicationEventCallback
 *
 * @brief                       - Application callback for SSI events
 *
 * @param[in]                   - pSSIHandle: Pointer to the SSI handle structure
 * @param[in]                   - AppEv: Event that occurred
 */
void SSI_ApplicationEventCallback(SSI_Handle_t *pSSIHandle, uint8_t AppEv)
{
    static uint32_t i = 0;

    if (AppEv == SSI_EVENT_RX_CMPLT)
    {
        // Debug: Show what byte we received
        if (i < 5) { // Only debug first few bytes
            UART_SendString(&UART1handle, "Rx byte ");
            // Simple decimal output
            char num[4];
            uint8_t byte_val = ReadByte;
            num[0] = '0' + (byte_val / 100);
            num[1] = '0' + ((byte_val / 10) % 10);
            num[2] = '0' + (byte_val % 10);
            num[3] = '\0';
            UART_SendString(&UART1handle, num);
            UART_SendString(&UART1handle, "\r\n");
        }

        // Store received byte in buffer
        RcvBuff[i++] = ReadByte;

        // Check for end of message ('\0' or buffer full)
        if (ReadByte == '\0' || i == MAX_LEN) {
            rcvStop = 1;
            RcvBuff[i-1] = '\0'; // Ensure null termination
            i = 0;
        }
    }
    else if (AppEv == SSI_EVENT_TX_CMPLT)
    {
        // TX complete - can be used for additional logic if needed
    }
    else if (AppEv == SSI_EVENT_OVR_ERR)
    {
        // Handle overrun error
        UART_SendString(&UART1handle, "SSI Overrun Error!\r\n");
        SSI_ClearOvrFlag(pSSIHandle->pSSIx);
    }
    else if (AppEv == SSI_EVENT_TIMEOUT)
    {
        // Handle timeout - could indicate end of packet
        UART_SendString(&UART1handle, "SSI Timeout Event\r\n");
    }
}

/***************************************************************************
 * @fn                          - GPIOD_IRQHandler
 *
 * @brief                       - GPIOD interrupt handler for data available signal
 */
void GPIOD_IRQHandler(void)
{
    GPIO_IRQHandling(GPIOD, GPIO_PIN_6);
    dataAvailable = 1;
}
