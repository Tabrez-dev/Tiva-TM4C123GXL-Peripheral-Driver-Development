/*
 * tm4c123x_uart_driver.h
 *
 *  Created on: 20-Sep-2025
 *      Author: tabrez
 */

#ifndef DRIVERS_INC_TM4C123X_UART_DRIVER_H_
#define DRIVERS_INC_TM4C123X_UART_DRIVER_H_

#include "tm4c123x.h"

/* UART Configuration Structure */
typedef struct {
    uint32_t UART_BaudRate;     /* Baud rate (9600, 115200, etc.) */
    uint8_t UART_WordLength;    /* Data bits: 5, 6, 7, 8 bits */
    uint8_t UART_StopBits;      /* Stop bits: 1 or 2 */
    uint8_t UART_Parity;        /* Parity: None, Even, Odd */
    uint8_t UART_HWFlowControl; /* Hardware flow control: Enable/Disable */
} UART_Config_t;

/**
 * @brief UART Handle Structure
 */
typedef struct {
    UART_RegDef_t* pUARTx;      /* Pointer to the UART register set (e.g., UART0) */
    UART_Config_t UARTConfig;   /* Configuration settings for the UART peripheral */
} UART_Handle_t;

/**
 * @UART_BaudRate
 * Standard baud rates for UART communication
 */
#define UART_BAUD_1200      1200
#define UART_BAUD_2400      2400
#define UART_BAUD_9600      9600
#define UART_BAUD_19200     19200
#define UART_BAUD_38400     38400
#define UART_BAUD_57600     57600
#define UART_BAUD_115200    115200

/**
 * @UART_WordLength
 */
#define UART_WORDLEN_5BITS  0x0
#define UART_WORDLEN_6BITS  0x1
#define UART_WORDLEN_7BITS  0x2
#define UART_WORDLEN_8BITS  0x3

/**
 * @UART_StopBits
 */
#define UART_STOPBITS_1     0x0
#define UART_STOPBITS_2     0x1

/**
 * @UART_Parity
 */
#define UART_PARITY_NONE    0x0
#define UART_PARITY_EVEN    0x1
#define UART_PARITY_ODD     0x2

/**
 * @UART_HWFlowControl
 */
#define UART_HW_FLOW_CTRL_NONE  0x0
#define UART_HW_FLOW_CTRL_RTS   0x1
#define UART_HW_FLOW_CTRL_CTS   0x2
#define UART_HW_FLOW_CTRL_RTS_CTS 0x3

/*
 * ----------------------------------------------------------------------------
 * Bit position definitions for UART Control Register (UARTCTL)
 * ----------------------------------------------------------------------------
 */
#define UART_UARTCTL_UARTEN     0   // UART Enable
#define UART_UARTCTL_SIREN      1   // SIR Enable (not used)
#define UART_UARTCTL_SIRLP      2   // SIR Low-Power (not used)
#define UART_UARTCTL_EOT        4   // End of Transmission
#define UART_UARTCTL_HSE        5   // High-Speed Enable
#define UART_UARTCTL_LBE        7   // Loopback Enable
#define UART_UARTCTL_TXE        8   // Transmit Enable
#define UART_UARTCTL_RXE        9   // Receive Enable
#define UART_UARTCTL_RTS        11  // Request to Send
#define UART_UARTCTL_RTSEN      14  // RTS Hardware Flow Control Enable
#define UART_UARTCTL_CTSEN      15  // CTS Hardware Flow Control Enable

/*
 * ----------------------------------------------------------------------------
 * Bit position definitions for UART Flag Register (UARTFR)
 * ----------------------------------------------------------------------------
 */
#define UART_UARTFR_CTS         0   // Clear to Send
#define UART_UARTFR_BUSY        3   // UART Busy
#define UART_UARTFR_RXFE        4   // Receive FIFO Empty
#define UART_UARTFR_TXFF        5   // Transmit FIFO Full
#define UART_UARTFR_RXFF        6   // Receive FIFO Full
#define UART_UARTFR_TXFE        7   // Transmit FIFO Empty

/*
 * ----------------------------------------------------------------------------
 * Bit position definitions for UART Line Control Register (UARTLCRH)
 * ----------------------------------------------------------------------------
 */
#define UART_UARTLCRH_BRK       0   // Send Break
#define UART_UARTLCRH_PEN       1   // Parity Enable
#define UART_UARTLCRH_EPS       2   // Even Parity Select
#define UART_UARTLCRH_STP2      3   // Two Stop Bits Select
#define UART_UARTLCRH_FEN       4   // FIFO Enable
#define UART_UARTLCRH_WLEN      5   // Word Length [6:5]
#define UART_UARTLCRH_SPS       7   // Stick Parity Select

/*
 * UART Flag definitions
 */
#define UART_FLAG_CTS   (1U << UART_UARTFR_CTS)
#define UART_FLAG_BUSY  (1U << UART_UARTFR_BUSY)
#define UART_FLAG_RXFE  (1U << UART_UARTFR_RXFE)
#define UART_FLAG_TXFF  (1U << UART_UARTFR_TXFF)
#define UART_FLAG_RXFF  (1U << UART_UARTFR_RXFF)
#define UART_FLAG_TXFE  (1U << UART_UARTFR_TXFE)

/***************************************************************
 *                    APIs supported by this driver            *
 *    For more information about the APIs check the function   *
 *                     definitions                             *
 ***************************************************************/

/*
 * Peripheral Clock setup
 */
void UART_PeriClockControl(UART_RegDef_t *pUARTx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void UART_Init(UART_Handle_t *pUARTHandle);
void UART_DeInit(UART_RegDef_t *pUARTx);

/*
 * Data Send and Receive
 */
void UART_SendData(UART_Handle_t *pUARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void UART_ReceiveData(UART_Handle_t *pUARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Other Peripheral Control APIs
 */
uint8_t UART_GetFlagStatus(UART_RegDef_t *pUARTx, uint32_t FlagName);

/*
 * Application utility
 */
void UART_SendString(UART_Handle_t *pUARTHandle, char *str);

#endif /* DRIVERS_INC_TM4C123X_UART_DRIVER_H_ */
