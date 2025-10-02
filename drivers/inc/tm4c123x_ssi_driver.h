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
 * @brief Flash Operation Types for W25Q64FV
 */
typedef enum {
    FLASH_OP_NONE = 0,
    FLASH_OP_READ_ID,           /* Read JEDEC ID */
    FLASH_OP_READ_STATUS,       /* Read Status Register */
    FLASH_OP_WRITE_ENABLE,      /* Write Enable command */
    FLASH_OP_PAGE_PROGRAM,      /* Page Program operation */
    FLASH_OP_SECTOR_ERASE,      /* 4KB Sector Erase */
    FLASH_OP_READ_DATA          /* Read data from flash */
} FlashOperationType_t;

/**
 * @brief Flash Operation States for command sequences
 */
typedef enum {
    FLASH_STATE_IDLE = 0,
    FLASH_STATE_CMD_PHASE,      /* Sending command byte */
    FLASH_STATE_ADDR_PHASE,     /* Sending address bytes */
    FLASH_STATE_DATA_PHASE,     /* Sending/receiving data */
    FLASH_STATE_STATUS_POLL,    /* Polling status register */
    FLASH_STATE_COMPLETE,       /* Operation complete */
    FLASH_STATE_ERROR           /* Error occurred */
} FlashOperationState_t;

/**
 * @brief Flash Operation Request structure for client operations
 */
typedef struct {
    FlashOperationType_t opType;    /* Type of operation */
    uint32_t address;               /* Flash address (for read/write/erase) */
    uint8_t *pData;                 /* Data buffer pointer */
    uint32_t dataLen;               /* Data length */
    uint8_t clientId;               /* Client identifier (0-255) */
    uint8_t priority;               /* Operation priority (0=highest) */
    volatile uint8_t status;        /* Operation status flags */
    void (*callback)(uint8_t clientId, uint8_t status);  /* Completion callback */
} FlashRequest_t;

/**
 * @brief Multi-Client Flash Resource Manager
 */
#define MAX_FLASH_CLIENTS     8     /* Maximum simultaneous clients */
#define FLASH_QUEUE_SIZE      16    /* Maximum queued operations */

typedef struct {
    FlashRequest_t queue[FLASH_QUEUE_SIZE];  /* Operation queue */
    volatile uint8_t queueHead;              /* Queue head index */
    volatile uint8_t queueTail;              /* Queue tail index */
    volatile uint8_t queueCount;             /* Number of queued operations */

    volatile FlashRequest_t *pCurrentOp;     /* Currently executing operation */
    volatile FlashOperationState_t currentState;  /* Current state machine state */
    volatile uint8_t cmdBuffer[8];           /* Command/address buffer */
    volatile uint8_t cmdIndex;               /* Current command byte index */
    volatile uint8_t cmdLength;              /* Total command length */

    volatile uint32_t statusPollCount;       /* Status polling counter */
    volatile uint8_t lastStatus;             /* Last read status register value */

    volatile uint32_t eventFlags;            /* Event flags for interrupt communication */
    volatile uint8_t resourceLocked;         /* Resource lock flag */
} FlashResourceManager_t;

/**
 * @brief SSI Handle Structure for interrupt-based operations with flash support
 */
typedef struct {
    SSI_RegDef_t* pSSIx;     /* Pointer to the SSI register set (e.g., SSI0) */
    SSI_Config_t SSIConfig;  /* Configuration settings for the SSI peripheral */
    uint8_t *pTxBuffer;      /* To store the app. Tx buffer address */
    uint8_t *pRxBuffer;      /* To store the app. Rx buffer address */
    uint32_t TxLen;          /* To store Tx len */
    uint32_t RxLen;          /* To store Rx len */
    uint8_t TxState;         /* To store Tx state */
    uint8_t RxState;         /* To store Rx state */

    /* Flash-specific extensions for multi-client support */
    FlashResourceManager_t flashManager;  /* Flash resource manager */
    uint8_t isFlashInterface;             /* Flag: 1 if this SSI is for flash, 0 for generic */
} SSI_Handle_t;

/**
 * @SSI_State
 * SSI peripheral states for interrupt-based operations
 */
#define SSI_READY                0  /* SSI peripheral is ready for new operation */
#define SSI_BUSY_IN_RX          1  /* SSI peripheral is busy in reception */
#define SSI_BUSY_IN_TX          2  /* SSI peripheral is busy in transmission */

/**
 * Possible SSI application Events
 */
#define SSI_EVENT_TX_CMPLT      0   /* Data transmission complete */
#define SSI_EVENT_RX_CMPLT      1   /* Data reception complete */
#define SSI_EVENT_OVR_ERR       2   /* Overrun error occurred */
#define SSI_EVENT_TIMEOUT       3   /* RX timeout occurred */

/**
 * Flash Operation Status Flags
 */
#define FLASH_STATUS_PENDING    0x01    /* Operation queued, waiting */
#define FLASH_STATUS_ACTIVE     0x02    /* Operation in progress */
#define FLASH_STATUS_COMPLETE   0x04    /* Operation completed successfully */
#define FLASH_STATUS_ERROR      0x08    /* Operation failed */
#define FLASH_STATUS_TIMEOUT    0x10    /* Operation timed out */
#define FLASH_STATUS_CANCELLED  0x20    /* Operation was cancelled */

/**
 * Flash Event Flags for interrupt communication
 */
#define FLASH_EVENT_OP_COMPLETE     (1U << 0)   /* Current operation completed */
#define FLASH_EVENT_STATUS_READY    (1U << 1)   /* Status register read complete */
#define FLASH_EVENT_CMD_COMPLETE    (1U << 2)   /* Command phase complete */
#define FLASH_EVENT_ADDR_COMPLETE   (1U << 3)   /* Address phase complete */
#define FLASH_EVENT_DATA_COMPLETE   (1U << 4)   /* Data phase complete */
#define FLASH_EVENT_ERROR           (1U << 5)   /* Error occurred */
#define FLASH_EVENT_QUEUE_READY     (1U << 6)   /* Queue has space available */

/**
 * W25Q64FV Command Definitions
 */
#define W25QXX_CMD_READ_JEDEC_ID        0x9F    /* Read JEDEC ID */
#define W25QXX_CMD_READ_STATUS_REG1     0x05    /* Read Status Register-1 */
#define W25QXX_CMD_WRITE_ENABLE         0x06    /* Write Enable */
#define W25QXX_CMD_WRITE_DISABLE        0x04    /* Write Disable */
#define W25QXX_CMD_PAGE_PROGRAM         0x02    /* Page Program */
#define W25QXX_CMD_SECTOR_ERASE_4KB     0x20    /* Sector Erase (4KB) */
#define W25QXX_CMD_READ_DATA            0x03    /* Read Data */

/**
 * W25Q64FV Status Register Bits
 */
#define W25QXX_STATUS_BUSY              (1U << 0)   /* Erase/Write in Progress */
#define W25QXX_STATUS_WEL               (1U << 1)   /* Write Enable Latch */
#define W25QXX_STATUS_BP0               (1U << 2)   /* Block Protect bit 0 */
#define W25QXX_STATUS_BP1               (1U << 3)   /* Block Protect bit 1 */
#define W25QXX_STATUS_BP2               (1U << 4)   /* Block Protect bit 2 */
#define W25QXX_STATUS_TB                (1U << 5)   /* Top/Bottom Protect */
#define W25QXX_STATUS_SEC               (1U << 6)   /* Sector Protect */
#define W25QXX_STATUS_SRP0              (1U << 7)   /* Status Register Protect 0 */

/**
 * Flash Operation Timeouts (in interrupt ticks)
 */
#define FLASH_TIMEOUT_PAGE_PROGRAM      500     /* ~0.45ms * safety margin */
#define FLASH_TIMEOUT_SECTOR_ERASE      60000   /* ~60ms * safety margin */
#define FLASH_TIMEOUT_COMMAND           100     /* Command execution timeout */

/**
 * @SSI_DeviceMode
 */

#define SSI_DEVICE_MODE_MASTER 1
#define SSI_DEVICE_MODE_SLAVE  0

/**
 * @SSI_BusConfig
 * Note: TM4C123x SSI hardware only supports Full-Duplex mode.
 * Half-duplex and simplex modes would require software emulation.
 */
#define SSI_BUS_CONFIG_FD                   1   /* Full-Duplex (hardware native mode) */

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
#define SSI_FRF_SSI_Freescale       0x0  /* Freescale SPI */
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
 * These settings apply **only** when SSI_FRF = SSI_FRF_SSI_Freescale (0x0)
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

uint8_t SSI_SendDataIT(SSI_Handle_t *pSSIHandle, void *pTxBuffer, uint32_t Len);
uint8_t SSI_ReceiveDataIT(SSI_Handle_t *pSSIHandle, void *pRxBuffer, uint32_t Len);

uint8_t SSI_GetFlagStatus(SSI_RegDef_t *pSSIx, uint32_t Flag);


/*
 * IRQ Configuration and ISR handling
 */

void SSI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SSI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SSI_IRQHandling(SSI_Handle_t *pSSIHandle);


void SSI_ClearOvrFlag(SSI_RegDef_t *pSSIx);
void SSI_CloseTransmission(SSI_Handle_t *pSSIHandle);
void SSI_CloseReception(SSI_Handle_t *pSSIHandle);

/*
 * Application callback
 */

void SSI_ApplicationEventCallback(SSI_Handle_t *pSSIHandle, uint8_t AppEv);

/*
 * Flash Multi-Client Resource Manager APIs
 */

/* Flash resource manager initialization */
void SSI_FlashInit(SSI_Handle_t *pSSIHandle);
void SSI_FlashResourceManagerInit(SSI_Handle_t *pSSIHandle);

/* Client operation APIs */
uint8_t SSI_FlashQueueOperation(SSI_Handle_t *pSSIHandle, FlashRequest_t *pRequest);
uint8_t SSI_FlashCancelOperation(SSI_Handle_t *pSSIHandle, uint8_t clientId);
uint8_t SSI_FlashGetOperationStatus(SSI_Handle_t *pSSIHandle, uint8_t clientId);

/* Resource manager processing */
void SSI_FlashProcessQueue(SSI_Handle_t *pSSIHandle);
void SSI_FlashStateMachine(SSI_Handle_t *pSSIHandle);
void SSI_FlashInterruptHandler(SSI_Handle_t *pSSIHandle);

/* Utility functions */
uint8_t SSI_FlashIsResourceAvailable(SSI_Handle_t *pSSIHandle);
void SSI_FlashSetEventFlag(SSI_Handle_t *pSSIHandle, uint32_t flag);
uint32_t SSI_FlashGetEventFlags(SSI_Handle_t *pSSIHandle);
void SSI_FlashClearEventFlag(SSI_Handle_t *pSSIHandle, uint32_t flag);





#endif /* DRIVERS_INC_TM4C123X_SSI_DRIVER_H_ */
