/*
 * tm4c123x_ssi_driver.h
 *
 *  Created on: 09-Jul-2025
 *      Author: tabrez
 */

#ifndef DRIVERS_INC_TM4C123X_SSI_DRIVER_H_
#define DRIVERS_INC_TM4C123X_SSI_DRIVER_H_

#include "tm4c123x.h"


/* SSI Configuration Structure */
typedef struct {
    uint8_t SSI_DeviceMode;   /* Master (1) or Slave (0) mode */
    uint8_t SSI_BusConfig;  /* FD/HD config handled in software; hardware supports only Full-Duplex */
    uint32_t SSI_SclkSpeed;   /* Encoded CPSDVSR and SCR */
    uint8_t SSI_DSS;          /* Data Frame Format (DSS values) */
    uint8_t SSI_FRF;          /* Frame Format (SPI, SSI, Microwire) */
    uint8_t SSI_SPO;         /* Clock Polarity: 0=low, 1=high */
    uint8_t SSI_SPH;         /* Clock Phase: 0=first edge, 1=second edge */
    /*
     * FSS (Frame Select/Chip Select) Pin Behavior:
     *
     * MASTER MODE:
     * - FSS pin as ALT_FN: Hardware automatically drives FSS during transmission
     * - FSS pin as GPIO/not configured: Manual CS control required via GPIO
     *
     * SLAVE MODE:
     * - FSS pin as ALT_FN: Hardware monitors FSS input, responds only when selected
     * - FSS pin as GPIO/not configured: Slave always enabled (no CS checking)
     *
     * Note: There are NO register bits to control this - it's determined by GPIO AFSEL
     */
} SSI_Config_t;

/**
 * @brief SPI Handle Structure
 */
typedef struct {
    SSI_RegDef_t* pSSIx;     /* Pointer to the SSI register set (e.g., SSI0) */
    SSI_Config_t SSIConfig;  /* Configuration settings for the SPI peripheral */
} SSI_Handle_t;

/**
 * @SSI_DeviceMode
 */

#define SSI_DEVICE_MODE_MASTER 1
#define SSI_DEVICE_MODE_SLAVE  0

/**
 * @SSI_BusConfig
 */
#define SSI_BUS_CONFIG_FD                   1
#define SSI_BUS_CONFIG_HD                   2
#define SSI_BUS_CONFIG_SIMPLEX_RXONLY       3

/**
 * @SSI_SclkSpeed : The SSI module’s maximum clock speed is typically 25 MHz, Total divisor = CPSDVSR × (1 + SCR)
 */
#define SSI_SCLK_SPEED_DIV2   ((2 << 8) | 0)   /* CPSDVSR=2, SCR=0 → SysClk / 2 */
#define SSI_SCLK_SPEED_DIV4   ((4 << 8) | 0)   /* CPSDVSR=4, SCR=0 → SysClk / 4 */
#define SSI_SCLK_SPEED_DIV8   ((8 << 8) | 0)   /* CPSDVSR=8, SCR=0 → SysClk / 8 */
#define SSI_SCLK_SPEED_DIV16  ((16 << 8) | 0)  /* CPSDVSR=16, SCR=0 → SysClk / 16 */
#define SSI_SCLK_SPEED_DIV32  ((32 << 8) | 0)  /* CPSDVSR=32, SCR=0 → SysClk / 32 */
#define SSI_SCLK_SPEED_DIV64  ((64 << 8) | 0)  /* CPSDVSR=64, SCR=0 → SysClk / 64 */
#define SSI_SCLK_SPEED_DIV128 ((128 << 8) | 0) /* CPSDVSR=128, SCR=0 → SysClk / 128 */

/**
 * @SSI_FRF
 */
#define SSI_FRF_SSI_FreeScale       0x0  /* Freescale SPI */
#define SSI_FRF_SSI_TI       0x1  /* Texas Instruments SSI */
#define SSI_FRF_MICROWIRE 0x2  /* Microwire */

/**
 * @SSI_DSS
 */
#define SSI_DSS_4BIT  0x3
#define SSI_DSS_5BIT  0x4
#define SSI_DSS_6BIT  0x5
#define SSI_DSS_7BIT  0x6
#define SSI_DSS_8BIT  0x7
#define SSI_DSS_9BIT  0x8
#define SSI_DSS_10BIT 0x9
#define SSI_DSS_11BIT 0xA
#define SSI_DSS_12BIT 0xB
#define SSI_DSS_13BIT 0xC
#define SSI_DSS_14BIT 0xD
#define SSI_DSS_15BIT 0xE
#define SSI_DSS_16BIT 0xF

/**
 * ----------------------------------------------------------------------------
 * These settings apply **only** when SSI_FRF = SSI_FRF_SSI_FreeScale (0x0)
 *
 * - SPO and SPH: Clock polarity and phase settings
 *   → These are used *only* in Freescale SPI mode. They are ignored in TI or Microwire modes.
 *
 * - FSSControl:
 *   → Controls whether FSS (Frame Select) is driven by software or hardware.
 *   → In Master mode with Freescale SPI, you may choose SW or HW control.
 *   → In Slave mode, FSS is always hardware-driven.
 *
 * Attempting to use SPO/SPH in TI or Microwire mode has no effect.
 * ----------------------------------------------------------------------------
 */

/**
 * @SSI_SPO
 * Clock Polarity for Freescale SPI (SSICR0.SPO)
 */
#define SSI_SPO_LOW  0x0  /* Clock idles low */
#define SSI_SPO_HIGH 0x1  /* Clock idles high */

/**
 * @SSI_SPH
 * Clock Phase for Freescale SPI (SSICR0.SPH)
 */
#define SSI_SPH_1ST_EDGE 0x0  /* Data sampled on first clock edge */
#define SSI_SPH_2ND_EDGE 0x1  /* Data sampled on second clock edge */

/*
 * ----------------------------------------------------------------------------
 * Bit position definitions for SSI Control Register 0 (SSICR0)
 * ----------------------------------------------------------------------------
 */
#define SSI_SSICR0_DSS       0   // Data Size Select [3:0]
#define SSI_SSICR0_FRF       4   // Frame Format Select [5:4]
#define SSI_SSICR0_SPO       6   // Serial Clock Polarity
#define SSI_SSICR0_SPH       7   // Serial Clock Phase
#define SSI_SSICR0_SCR       8   // Serial Clock Rate [15:8]

/*
 * ----------------------------------------------------------------------------
 * Bit position definitions for SSI Control Register 1 (SSICR1)
 * ----------------------------------------------------------------------------
 */
#define SSI_SSICR1_LBM       0   // Loopback Mode
#define SSI_SSICR1_SSE       1   // SSI Synchronous Serial Port Enable
#define SSI_SSICR1_MS        2   // Master/Slave Select
#define SSI_SSICR1_EOT       4   // End of Transmission

/*
 * ----------------------------------------------------------------------------
 * Bit position definitions for SSI Status Register (SSISR)
 * ----------------------------------------------------------------------------
 */
#define SSI_SSISR_TFE        0   // Transmit FIFO Empty
#define SSI_SSISR_TNF        1   // Transmit FIFO Not Full
#define SSI_SSISR_RNE        2   // Receive FIFO Not Empty
#define SSI_SSISR_RFF        3   // Receive FIFO Full
#define SSI_SSISR_BSY        4   // SSI Busy

/*
 * ----------------------------------------------------------------------------
 * Bit position definitions for SSI Interrupt Mask Register (SSIIM)
 * ----------------------------------------------------------------------------
 */
#define SSI_SSIIM_RORIM      0   // Receive Overrun Interrupt Mask
#define SSI_SSIIM_RTIM       1   // Receive Timeout Interrupt Mask
#define SSI_SSIIM_RXIM       2   // Receive FIFO Interrupt Mask
#define SSI_SSIIM_TXIM       3   // Transmit FIFO Interrupt Mask

/*
 * ----------------------------------------------------------------------------
 * Bit position definitions for SSI Raw Interrupt Status Register (SSIRIS)
 * ----------------------------------------------------------------------------
 */
#define SSI_SSIRIS_RORRIS    0   // Raw Overrun Interrupt Status
#define SSI_SSIRIS_RTRIS     1   // Raw Timeout Interrupt Status
#define SSI_SSIRIS_RXRIS     2   // Raw Receive FIFO Interrupt Status
#define SSI_SSIRIS_TXRIS     3   // Raw Transmit FIFO Interrupt Status

/*
 * ----------------------------------------------------------------------------
 * Bit position definitions for SSI Masked Interrupt Status Register (SSIMIS)
 * ----------------------------------------------------------------------------
 */
#define SSI_SSIMIS_RORMIS    0   // Masked Overrun Interrupt Status
#define SSI_SSIMIS_RTMIS     1   // Masked Timeout Interrupt Status
#define SSI_SSIMIS_RXMIS     2   // Masked Receive FIFO Interrupt Status
#define SSI_SSIMIS_TXMIS     3   // Masked Transmit FIFO Interrupt Status

/*
 * ----------------------------------------------------------------------------
 * Bit position definitions for SSI Interrupt Clear Register (SSIICR)
 * ----------------------------------------------------------------------------
 */
#define SSI_SSIICR_RORIC     0   // Clear Overrun Interrupt
#define SSI_SSIICR_RTIC      1   // Clear Timeout Interrupt

#define SSI_SSIDMACTL_RXDMAE   0  // Receive DMA Enable
#define SSI_SSIDMACTL_TXDMAE   1  // Transmit DMA Enable

#define SSI_SSICC_CS           0  // Clock Source select [2:0], usually only 0 or 5

/*
 * SSI (SPI) related status flag definitions
 */
#define SSI_FLAG_TFE   (1U << SSI_SSISR_TFE)
#define SSI_FLAG_TNF   (1U << SSI_SSISR_TNF)
#define SSI_FLAG_RNE   (1U << SSI_SSISR_RNE)
#define SSI_FLAG_RFF   (1U << SSI_SSISR_RFF)
#define SSI_FLAG_BSY   (1U << SSI_SSISR_BSY)

#define SSI_IRQ_RX_OVERRUN   (1U << SSI_SSIIM_RORIM)
#define SSI_IRQ_RX_TIMEOUT   (1U << SSI_SSIIM_RTIM)
#define SSI_IRQ_RX_FIFO      (1U << SSI_SSIIM_RXIM)
#define SSI_IRQ_TX_FIFO      (1U << SSI_SSIIM_TXIM)



/***************************************************************
 *                    APIs supported by this driver            *
 *    For more information about the APIs check the function   *
 *                     definitions                             *
 ***************************************************************/

/*
 * Peripheral Clock setup
 * */

void SSI_PeriClockControl(SSI_RegDef_t *pSSIx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void SSI_Init(SSI_Handle_t *pSSIHandle);
void SSI_DeInit(SSI_RegDef_t *pSSIx);

/*
 * Data Send and Receive
 */

void SSI_SendData(SSI_RegDef_t *pSSIx, void *pTxBuffer, uint32_t Len);
void SSI_ReceiveData(SSI_RegDef_t *pSSIx, void *pRxBuffer, uint32_t Len);


/*
 * IRQ Configuration and ISR handling
 */

void SSI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SSI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SSI_IRQHandling(SSI_RegDef_t *pHandle);



/*
 * Other peripheral control APIs
 */



#endif /* DRIVERS_INC_TM4C123X_SSI_DRIVER_H_ */
