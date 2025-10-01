/*
 * tm4c123x_uart_driver.c
 *
 *  Created on: 20-Sep-2025
 *      Author: tabrez
 */

#include "tm4c123x_uart_driver.h"

/***************************************************************************
 * @fn                          - UART_PeriClockControl
 *
 * @brief                       - Enables or disables the peripheral clock for a UART port
 *
 * @param[in]                   - pUARTx: Pointer to the UART port base address (e.g., UART0, UART1)
 * @param[in]                   - EnorDi: Enable (1) or disable (0) the clock
 *
 * @return                      - none
 *
 * @Note                        - Enables UART peripheral clock in Run Mode Clock Gating Control
 */
void UART_PeriClockControl(UART_RegDef_t *pUARTx, uint8_t EnorDi)
{
    if (!pUARTx) return;  // Null pointer check

    if (EnorDi == ENABLE) {
        // Enable clock for the specified UART port
        if (pUARTx == UART0) {
            UART0_PCLK_EN();
        } else if (pUARTx == UART1) {
            UART1_PCLK_EN();
        } else if (pUARTx == UART2) {
            UART2_PCLK_EN();
        } else if (pUARTx == UART3) {
            UART3_PCLK_EN();
        } else if (pUARTx == UART4) {
            UART4_PCLK_EN();
        } else if (pUARTx == UART5) {
            UART5_PCLK_EN();
        } else if (pUARTx == UART6) {
            UART6_PCLK_EN();
        } else if (pUARTx == UART7) {
            UART7_PCLK_EN();
        }

        // Wait for peripheral readiness
        uint32_t portBit = (pUARTx == UART0) ? (1U << 0) :
                           (pUARTx == UART1) ? (1U << 1) :
                           (pUARTx == UART2) ? (1U << 2) :
                           (pUARTx == UART3) ? (1U << 3) :
                           (pUARTx == UART4) ? (1U << 4) :
                           (pUARTx == UART5) ? (1U << 5) :
                           (pUARTx == UART6) ? (1U << 6) :
                           (1U << 7);  // UART7

        uint32_t timeout = 10000;
        while (!(SYSCTL_PR->PRUART & portBit) && timeout > 0) {
            timeout--;
        }
    } else {
        // Disable clock for the specified UART port
        if (pUARTx == UART0) {
            UART0_PCLK_DIS();
        } else if (pUARTx == UART1) {
            UART1_PCLK_DIS();
        } else if (pUARTx == UART2) {
            UART2_PCLK_DIS();
        } else if (pUARTx == UART3) {
            UART3_PCLK_DIS();
        } else if (pUARTx == UART4) {
            UART4_PCLK_DIS();
        } else if (pUARTx == UART5) {
            UART5_PCLK_DIS();
        } else if (pUARTx == UART6) {
            UART6_PCLK_DIS();
        } else if (pUARTx == UART7) {
            UART7_PCLK_DIS();
        }
    }
}

/***************************************************************************
 * @fn                          - UART_Init
 *
 * @brief                       - Initializes the UART peripheral with the provided configuration
 *
 * @param[in]                   - pUARTHandle: Pointer to the UART handle structure containing configuration
 *
 * @return                      - none
 *
 * @Note                        - Configures baud rate, word length, parity, stop bits, and enables UART
 */
void UART_Init(UART_Handle_t *pUARTHandle)
{
    uint32_t tempLCRH = 0;

    //1. Enable UART Clock
    UART_PeriClockControl(pUARTHandle->pUARTx, ENABLE);

    //2. Disable UART before configuration
    pUARTHandle->pUARTx->CTL &= ~(1U << UART_UARTCTL_UARTEN);

    //3. Configure Baud Rate
    // BRD = BRDI + BRDF = UARTSysClk / (ClkDiv * Baud Rate)
    // For 16MHz system clock, ClkDiv = 16 (default)
    // BRDI = integer part, BRDF = fractional part (6 bits)
    uint32_t BRD = (16000000 * 64) / (16 * pUARTHandle->UARTConfig.UART_BaudRate);
    uint32_t BRDI = BRD / 64;
    uint32_t BRDF = BRD % 64;

    pUARTHandle->pUARTx->IBRD = BRDI;
    pUARTHandle->pUARTx->FBRD = BRDF;

    //4. Configure Word Length, Parity, Stop Bits, FIFO
    tempLCRH |= (pUARTHandle->UARTConfig.UART_WordLength & 0x3U) << UART_UARTLCRH_WLEN;

    // Configure Parity
    if (pUARTHandle->UARTConfig.UART_Parity != UART_PARITY_NONE) {
        tempLCRH |= (1U << UART_UARTLCRH_PEN);  // Enable parity
        if (pUARTHandle->UARTConfig.UART_Parity == UART_PARITY_EVEN) {
            tempLCRH |= (1U << UART_UARTLCRH_EPS);  // Even parity
        }
        // Odd parity: PEN=1, EPS=0 (default)
    }

    // Configure Stop Bits
    if (pUARTHandle->UARTConfig.UART_StopBits == UART_STOPBITS_2) {
        tempLCRH |= (1U << UART_UARTLCRH_STP2);
    }

    // Enable FIFO
    tempLCRH |= (1U << UART_UARTLCRH_FEN);

    //5. Write Line Control Register
    pUARTHandle->pUARTx->LCRH = tempLCRH;

    //6. Configure Control Register (Enable TX, RX)
    uint32_t tempCTL = 0;
    tempCTL |= (1U << UART_UARTCTL_TXE);  // Enable transmit
    tempCTL |= (1U << UART_UARTCTL_RXE);  // Enable receive
    tempCTL |= (1U << UART_UARTCTL_UARTEN);  // Enable UART

    pUARTHandle->pUARTx->CTL = tempCTL;
}

/***************************************************************************
 * @fn                          - UART_DeInit
 *
 * @brief                       - Resets the UART peripheral to its default state
 *
 * @param[in]                   - pUARTx: Pointer to the UART base address
 *
 * @return                      - none
 *
 * @Note                        - Disables the UART peripheral
 */
void UART_DeInit(UART_RegDef_t *pUARTx)
{
    // Disable UART
    pUARTx->CTL &= ~(1U << UART_UARTCTL_UARTEN);

    // Reset registers to default values
    pUARTx->CTL = 0x00000300;   // Default: TXE=1, RXE=1, UARTEN=0
    pUARTx->LCRH = 0x00000000;
    pUARTx->IBRD = 0x00000000;
    pUARTx->FBRD = 0x00000000;
}

/***************************************************************************
 * @fn                          - UART_GetFlagStatus
 *
 * @brief                       - Checks the status of a specific flag in the UART Flag Register
 *
 * @param[in]                   - pUARTx: Pointer to the UART peripheral base address
 * @param[in]                   - FlagName: Flag to check (UART_FLAG_TXFE, UART_FLAG_RXFE, etc.)
 *
 * @return                      - FLAG_SET or FLAG_RESET
 *
 * @Note                        - Used to check FIFO status, busy status, etc.
 */
uint8_t UART_GetFlagStatus(UART_RegDef_t *pUARTx, uint32_t FlagName)
{
    if (pUARTx->FR & FlagName) {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/***************************************************************************
 * @fn                          - UART_SendData
 *
 * @brief                       - Sends data over the UART peripheral
 *
 * @param[in]                   - pUARTHandle: Pointer to the UART handle structure
 * @param[in]                   - pTxBuffer: Pointer to the transmit data buffer
 * @param[in]                   - Len: Number of bytes to transmit
 *
 * @return                      - none
 *
 * @Note                        - Blocking transmission, waits until all data is sent
 */
void UART_SendData(UART_Handle_t *pUARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    for (uint32_t i = 0; i < Len; i++) {
        // Wait until transmit FIFO is not full
        while (UART_GetFlagStatus(pUARTHandle->pUARTx, UART_FLAG_TXFF));

        // Send data
        pUARTHandle->pUARTx->DR = pTxBuffer[i];
    }

    // Wait until transmission is complete (FIFO empty and not busy)
    while (!UART_GetFlagStatus(pUARTHandle->pUARTx, UART_FLAG_TXFE));
    while (UART_GetFlagStatus(pUARTHandle->pUARTx, UART_FLAG_BUSY));
}

/***************************************************************************
 * @fn                          - UART_ReceiveData
 *
 * @brief                       - Receives data from the UART peripheral
 *
 * @param[in]                   - pUARTHandle: Pointer to the UART handle structure
 * @param[in]                   - pRxBuffer: Pointer to the receive data buffer
 * @param[in]                   - Len: Number of bytes to receive
 *
 * @return                      - none
 *
 * @Note                        - Blocking reception, waits until all data is received
 */
void UART_ReceiveData(UART_Handle_t *pUARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    for (uint32_t i = 0; i < Len; i++) {
        // Wait until receive FIFO is not empty
        while (UART_GetFlagStatus(pUARTHandle->pUARTx, UART_FLAG_RXFE));

        // Read data (only lower 8 bits for data, upper bits contain error flags)
        pRxBuffer[i] = (uint8_t)(pUARTHandle->pUARTx->DR & 0xFF);
    }
}

/***************************************************************************
 * @fn                          - UART_SendString
 *
 * @brief                       - Sends a null-terminated string over UART
 *
 * @param[in]                   - pUARTHandle: Pointer to the UART handle structure
 * @param[in]                   - str: Pointer to the null-terminated string
 *
 * @return                      - none
 *
 * @Note                        - Utility function for easy string transmission
 */
void UART_SendString(UART_Handle_t *pUARTHandle, char *str)
{
    while (*str) {
        // Wait until transmit FIFO is not full
        while (UART_GetFlagStatus(pUARTHandle->pUARTx, UART_FLAG_TXFF));

        // Send character
        pUARTHandle->pUARTx->DR = *str++;
    }

    // Wait until transmission is complete
    while (!UART_GetFlagStatus(pUARTHandle->pUARTx, UART_FLAG_TXFE));
    while (UART_GetFlagStatus(pUARTHandle->pUARTx, UART_FLAG_BUSY));
}
