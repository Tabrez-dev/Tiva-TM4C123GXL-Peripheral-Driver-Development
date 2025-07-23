/*
 * tm4c123x_ssi_driver.c
 *
 *  Created on: 09-Jul-2025
 *      Author: tabrez
 */

#include "tm4c123x_ssi_driver.h"

/***************************************************************************
 * @fn                          - SSI_PeriClockControl
 *
 * @brief                       - Enables or disables the peripheral clock for an SSI port in Run Mode
 *
 * @param[in]                   - pSSIx: Pointer to the SSI port base address (e.g., SSI0, SSI1)
 * @param[in]                   - EnorDi: Enable (1) or disable (0) the clock
 * @param[in]                   - none
 *
 * @return                      - none
 *
 * @Note                        - Waits for peripheral readiness after enabling to ensure stable operation.
 *                                Refer to TM4C123GH6PM datasheet, section 5.2.6 for clock gating details.
 */
void SSI_PeriClockControl(SSI_RegDef_t *pSSIx, uint8_t EnorDi)
{
    if (!pSSIx) return;  // Null pointer check

    if (EnorDi == ENABLE) {
        // Enable clock for the specified SSI port
        if (pSSIx == SSI0) {
            SSI0_PCLK_EN();
        } else if (pSSIx == SSI1) {
            SSI1_PCLK_EN();
        } else if (pSSIx == SSI2) {
            SSI2_PCLK_EN();
        } else if (pSSIx == SSI3) {
            SSI3_PCLK_EN();
        }

        // Wait for peripheral readiness
        uint32_t portBit = (pSSIx == SSI0) ? (1U << 0) :
                           (pSSIx == SSI1) ? (1U << 1) :
                           (pSSIx == SSI2) ? (1U << 2) :
                           (1U << 3);  // SSI3
        while (!(SYSCTL_PR->PRSSI  & portBit)) {};  // Spin until ready
    } else {
        // Disable clock for the specified SSI port
        if (pSSIx == SSI0) {
            SSI0_PCLK_DIS();
        } else if (pSSIx == SSI1) {
            SSI1_PCLK_DIS();
        } else if (pSSIx == SSI2) {
            SSI2_PCLK_DIS();
        } else if (pSSIx == SSI3) {
            SSI3_PCLK_DIS();
        }
    }
}

/***************************************************************************
 * @fn                          - SSI_Init
 *
 * @brief                       - Initializes the SSI peripheral with the provided configuration
 *
 * @param[in]                   - pSSIHandle: Pointer to the SSI handle structure containing configuration
 *
 * @return                      - none
 *
 * @Note                        - Automatically enables the peripheral clock for the given SSI instance.
 *                              - You do NOT need to call SSI_PeriClockControl() separately.
 *                              - Configures SSI mode, clock speed, and other parameters as per handle settings.

 */
void SSI_Init(SSI_Handle_t *pSSIHandle)
{
    uint32_t tempCR0 = 0;
    uint32_t tempCR1 = 0;
    uint32_t cpsdvsr = (pSSIHandle->SSIConfig.SSI_SclkSpeed >> 8) & 0xFF;
    uint32_t scr     = (pSSIHandle->SSIConfig.SSI_SclkSpeed & 0xFF);

    //0. Enable SSI Clock
    SSI_PeriClockControl(pSSIHandle->pSSIx, ENABLE);
    // 1. Disable SSI before configuration
    pSSIHandle->pSSIx->CR1 &= ~(1U << SSI_SSICR1_SSE);  // Clear SSE to disable SSI

    // 2. Set device mode (Master/Slave)
    if (pSSIHandle->SSIConfig.SSI_DeviceMode == SSI_DEVICE_MODE_MASTER) {
        tempCR1 &= ~(1U << SSI_SSICR1_MS);  // MS = 0 → Master
    } else {
        tempCR1 |=  (1U << SSI_SSICR1_MS);  // MS = 1 → Slave
    }

    // 3. Frame Format: Freescale SPI / TI / Microwire
    tempCR0 |= (pSSIHandle->SSIConfig.SSI_FRF & 0x3U) << SSI_SSICR0_FRF;

    // 4. Data Size (4–16 bits) – DSS field [3:0]
    tempCR0 |= (pSSIHandle->SSIConfig.SSI_DSS & 0xFU) << SSI_SSICR0_DSS;

    // 5. Clock Polarity and Phase
    tempCR0 |= (pSSIHandle->SSIConfig.SSI_SPO & 0x1U) << SSI_SSICR0_SPO;
    tempCR0 |= (pSSIHandle->SSIConfig.SSI_SPH & 0x1U) << SSI_SSICR0_SPH;

    // 6. Serial Clock Rate (SCR) field [15:8]
    tempCR0 |= (scr & 0xFFU) << SSI_SSICR0_SCR;

    // 7. Configure CR0 and CR1
    pSSIHandle->pSSIx->CR0 = tempCR0;
    pSSIHandle->pSSIx->CR1 = tempCR1;

    // 8. Configure Clock Prescale (CPSDVSR)
    if ((cpsdvsr < 2) || (cpsdvsr & 1)) {
        cpsdvsr = 2; // Must be even and ≥ 2
    }
    pSSIHandle->pSSIx->CPSR = cpsdvsr;

    // 9. BusConfig is handled in software: full-duplex by hardware default.
    // Higher-level APIs (e.g., transmit/receive) interpret SSI_BusConfig.

    // 10. Enable SSI
    pSSIHandle->pSSIx->CR1 |= (1U << SSI_SSICR1_SSE);
}



/***************************************************************************
 * @fn                          - SSI_DeInit
 *
 * @brief                       - Resets the SSI peripheral to its default state
 *
 * @param[in]                   - pSSIx: Pointer to the SSI base address (e.g., SSI0, SSI1)
 *
 * @return                      - none
 *
 * @Note                        - Disables the peripheral and resets all associated configuration.
 */
void SSI_DeInit(SSI_RegDef_t *pSSIx)
{
    // 1. Disable the SSI peripheral
    pSSIx->CR1 &= ~(1U << SSI_SSICR1_SSE);  // Clear SSE bit to disable SSI

    // 2. Clear control registers
    pSSIx->CR0 = 0x00000000;
    pSSIx->CR1 = 0x00000000;

    // 3. Clear prescaler
    pSSIx->CPSR = 0x00000000;

    // 4. Clear interrupt mask
    pSSIx->IM = 0x00000000;

    // 5. Clear interrupt status (write 1s to clear)
    pSSIx->ICR = (1U << SSI_SSIICR_RORIC) | (1U << SSI_SSIICR_RTIC);

    // 6. Optionally, clear DMA control
    pSSIx->DMACTL = 0x00000000;

}

/**
 * @fn      - SSI_GetFlagStatus
 * @brief   - Checks the status of a specific flag in the SSI Status Register
 *
 * @param[in]  pSSIx : Pointer to the SSI peripheral base address
 * @param[in]  Flag  : One of the SSI_FLAG_x macros (e.g., SSI_FLAG_TNF)
 *
 * @return     FLAG_SET or FLAG_RESET
 */
uint8_t SSI_GetFlagStatus(SSI_RegDef_t *pSSIx, uint32_t Flag)
{
    if (pSSIx->SR & Flag)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}


/***************************************************************************
 * @fn                          - SSI_SendData
 *
 * @brief                       - Sends data over the SSI (SPI) peripheral
 *
 * @param[in]                   - pSSIx: Pointer to the SSI base address (e.g., SSI0, SSI1)
 * @param[in]                   - pTxBuffer: Pointer to the transmit data buffer
 * @param[in]                   - Len: Number of data elements to transmit
 *
 * @return                      - none
 *
 * @Note                        - Blocks until all data is transmitted.
 *                                DSS config in CR0 determines whether each element is 8 or 16 bits.
 */
void SSI_SendData(SSI_RegDef_t *pSSIx, void *pTxBuffer, uint32_t Len)
{
    uint8_t *pData = (uint8_t *)pTxBuffer;  // Cast void* to uint8_t* for byte manipulation
    
    while (Len > 0)
    {
        // 1. Wait until Transmit FIFO is not full
        while (SSI_GetFlagStatus(pSSIx, SSI_FLAG_TNF) == FLAG_RESET);

        // 2. Get DSS (Data Size Select): bits [3:0] of CR0
        uint8_t dss = (pSSIx->CR0 & 0xF);  // DSS encoding from 0x3 to 0xF (4 to 16 bits)

        if (dss <= 0x7)  // DSS = 0x3 to 0x7 → 4 to 8 bits
        {
            pSSIx->DR = *pData;
            pData++;
            Len--;
        }
        else // DSS = 0x8 to 0xF → 9 to 16 bits
        {
            uint16_t *pData16 = (uint16_t *)pData;
            pSSIx->DR = *pData16;
            pData += 2;
            Len--;
        }
    }
    
    // 3. Wait for transmission to complete
    // Wait until transmit FIFO is empty
    while (SSI_GetFlagStatus(pSSIx, SSI_FLAG_TFE) == FLAG_RESET);
    
    // Wait until SSI is not busy
    while (SSI_GetFlagStatus(pSSIx, SSI_FLAG_BSY) == FLAG_SET);
}


/***************************************************************************
 * @fn                          - SSI_ReceiveData
 *
 * @brief                       - Receives data from the SSI peripheral
 *
 * @param[in]                   - pSSIx: Pointer to the SSI base address (e.g., SSI0, SSI1)
 * @param[in]                   - pRxBuffer: Pointer to the receive data buffer
 * @param[in]                   - Len: Number of data elements to receive
 *
 * @return                      - none
 *
 * @Note                        - Blocks until all data is received.
 *                                DSS config in CR0 determines whether each element is 8 or 16 bits.
 */
void SSI_ReceiveData(SSI_RegDef_t *pSSIx, void *pRxBuffer, uint32_t Len)
{
    uint8_t *pData = (uint8_t *)pRxBuffer;  // Cast void* to uint8_t* for byte manipulation
    
    while (Len > 0)
    {
        // 1. Wait until Receive FIFO is not empty (RNE flag)
        while (SSI_GetFlagStatus(pSSIx, SSI_FLAG_RNE) == FLAG_RESET);

        // 2. Get DSS (Data Size Select): bits [3:0] of CR0
        uint8_t dss = (pSSIx->CR0 & 0xF);  // DSS encoding from 0x3 to 0xF (4 to 16 bits)

        if (dss <= 0x7)  // DSS = 0x3 to 0x7 → 4 to 8 bits
        {
            // Read DR for 1 byte of data and increment the rx buffer address
            *pData = (uint8_t)(pSSIx->DR & 0xFF);  
            /* Masking with 0xFF ensures we only get the lower 8 bits.
             * The DR register is 16 bits wide, but when configured for 8-bit data,
             * only the lower 8 bits contain valid data. The upper 8 bits may contain
             * garbage or undefined values. Masking prevents these unwanted bits
             * from being stored in our receive buffer.
             */
            pData++;
            Len--;
        }
        else // DSS = 0x8 to 0xF → 9 to 16 bits
        {
            // Read DR for 2 bytes of data and increment the rx buffer address
            uint16_t *pData16 = (uint16_t *)pData;
            *pData16 = (uint16_t)(pSSIx->DR & 0xFFFF);  
            /* Masking with 0xFFFF ensures we only get the lower 16 bits.
             * Although DR is already 16 bits, this mask is a defensive programming
             * practice that ensures no sign extension or compiler-specific behavior
             * affects our data. It explicitly shows we want exactly 16 bits of data,
             * making the code more portable and clear in intent.
             */
            pData += 2;
            Len--;
        }
    }
}

/***************************************************************************
 * @fn                          - SPI_IRQInterruptConfig
 *
 * @brief                       - Configures the SPI interrupt in the NVIC
 *
 * @param[in]                   - IRQNumber: Interrupt number for the SPI peripheral
 * @param[in]                   - EnorDi: Enable (1) or disable (0) the interrupt
 * @param[in]                   - none
 *
 * @return                      - none
 *
 * @Note                        - Configures the NVIC for the specified interrupt number.
 */
void SSI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
}

/***************************************************************************
 * @fn                          - SPI_IRQPriorityConfig
 *
 * @brief                       - Configures the priority of the SPI interrupt
 *
 * @param[in]                   - IRQNumber: Interrupt number for the SPI peripheral
 * @param[in]                   - IRQPriority: Priority level for the interrupt
 * @param[in]                   - none
 *
 * @return                      - none
 *
 * @Note                        - Sets the priority in the NVIC for the specified interrupt.
 */
void SSI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
}

/***************************************************************************
 * @fn                          - SPI_IRQHandling
 *
 * @brief                       - Handles the SPI interrupt service routine
 *
 * @param[in]                   - pHandle: Pointer to the SPI handle structure
 * @param[in]                   - none
 * @param[in]                   - none
 *
 * @return                      - none
 *
 * @Note                        - Processes interrupt events and clears interrupt flags.
 */
void SSI_IRQHandling(SSI_RegDef_t *pHandle)
{
}
