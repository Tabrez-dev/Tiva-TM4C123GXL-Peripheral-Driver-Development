/*
 * tm4c123x_ssi_driver.c
 *
 *  Created on: 09-Jul-2025
 *      Author: tabrez
 */

#include "tm4c123x_ssi_driver.h"

static void SSI_TXE_InterruptHandle(SSI_Handle_t *pSSIHandle);
static void SSI_RXNE_InterruptHandle(SSI_Handle_t *pSSIHandle);
static void SSI_RT_InterruptHandle(SSI_Handle_t *pSSIHandle);
static void SSI_OVR_ErrInterruptHandle(SSI_Handle_t *pSSIHandle);

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

        // Wait for peripheral readiness with timeout
        uint32_t portBit = (pSSIx == SSI0) ? (1U << 0) :
                           (pSSIx == SSI1) ? (1U << 1) :
                           (pSSIx == SSI2) ? (1U << 2) :
                           (1U << 3);  // SSI3

        uint32_t timeout = 10000; // Maximum iterations to wait
        while (!(SYSCTL_PR->PRSSI & portBit) && timeout > 0) {
            timeout--;
        }
        // Note: Could add error handling here if timeout == 0
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
    // Validate DSS is in range 0x3 to 0xF (4 to 16 bits)
    uint8_t dss_value = pSSIHandle->SSIConfig.SSI_DSS & 0xFU;
    if (dss_value < 0x3) {
        dss_value = 0x3; // Default to minimum 4-bit data size
    }
    tempCR0 |= dss_value << SSI_SSICR0_DSS;

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
 * @fn      - SSI_GetStatus
 * @brief   - Checks the status of a specific  in the SSI Status Register
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
 * @param[in]                   - Len: Number of data frames to transmit (not bytes)
 *                                For DSS 4-8 bits: each frame = 1 byte
 *                                For DSS 9-16 bits: each frame = 2 bytes (little-endian)
 *
 * @return                      - none
 *
 * @Note                        - Blocks until all data is transmitted.
 *                                DSS config in CR0 determines frame size (4-16 bits).
 *                                For 16-bit transfers, data is read as little-endian.
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
            // Safe 16-bit data read avoiding alignment issues
            uint16_t data16 = pData[0] | (pData[1] << 8);
            pSSIx->DR = data16;
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
            // Read DR for 2 bytes of data - safe alignment method
            uint16_t data16 = (uint16_t)(pSSIx->DR & 0xFFFF);
            /* Masking with 0xFFFF ensures we only get the lower 16 bits.
             * Although DR is already 16 bits, this mask is a defensive programming
             * practice that ensures no sign extension or compiler-specific behavior
             * affects our data. It explicitly shows we want exactly 16 bits of data,
             * making the code more portable and clear in intent.
             */

            // Copy bytes individually to avoid alignment issues
            pData[0] = (uint8_t)(data16 & 0xFF);        // Low byte
            pData[1] = (uint8_t)((data16 >> 8) & 0xFF); // High byte
            pData += 2;
            Len--;
        }
    }
}

/***************************************************************************
 * @fn                          - SSI_SendDataIT
 *
 * @brief                       - Sends data over SSI using interrupt mode
 *
 * @param[in]                   - pSSIHandle: Pointer to the SSI handle structure
 * @param[in]                   - pTxBuffer: Pointer to the transmit data buffer
 * @param[in]                   - Len: Number of data frames to transmit
 *
 * @return                      - none
 *
 * @Note                        - Non-blocking transmission using FIFO interrupts
 */
uint8_t SSI_SendDataIT(SSI_Handle_t *pSSIHandle, void *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSSIHandle->TxState;
    if (state != SSI_BUSY_IN_TX)
    {
        // 1. Save the Tx buffer address and Len information in some global variables
        pSSIHandle->pTxBuffer = (uint8_t *)pTxBuffer;
        pSSIHandle->TxLen = Len;
        // 2. Mark the SSI state as busy in transmission so that
        //    no other code can take over same SSI peripheral until transmission is over
        pSSIHandle->TxState = SSI_BUSY_IN_TX;
        // 3. Enable the TXIM control bit to get interrupt whenever TXE flag is set in SR
        pSSIHandle->pSSIx->IM |= (1 << SSI_SSIIM_TXIM);
        // 4. Data Transmission will be handled by the ISR code (will implement later)
    }
    return state;
}
/***************************************************************************
 * @fn                          - SSI_ReceiveDataIT
 *
 * @brief                       - Receives data over SSI using interrupt mode
 *
 * @param[in]                   - pSSIHandle: Pointer to the SSI handle structure
 * @param[in]                   - pRxBuffer: Pointer to the receive data buffer
 * @param[in]                   - Len: Number of data frames to receive
 *
 * @return                      - none
 *
 * @Note                        - Non-blocking reception using FIFO interrupts
 */
uint8_t SSI_ReceiveDataIT(SSI_Handle_t *pSSIHandle, void *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSSIHandle->RxState;
    if (state != SSI_BUSY_IN_RX)
    {
        // 1. Save the Rx buffer address and Len information in some global variables
        pSSIHandle->pRxBuffer = (uint8_t *)pRxBuffer;
        pSSIHandle->RxLen = Len;
        // 2. Mark the SSI state as busy in reception so that
        //    no other code can take over same SSI peripheral until reception is over
        pSSIHandle->RxState = SSI_BUSY_IN_RX;
        // 3. Enable the RXIM control bit to get interrupt whenever RNE flag is set in SR
        pSSIHandle->pSSIx->IM |= (1 << SSI_SSIIM_RXIM);
    }
    return state;
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
    if (EnorDi == ENABLE)
    {
        // Enable IRQ in NVIC using the NVIC_ENx registers
        if (IRQNumber < 32)
        {
            NVIC_EN0 |= (1 << IRQNumber);
        }
        else if (IRQNumber < 64)
        {
            NVIC_EN1 |= (1 << (IRQNumber - 32));
        }
        else if (IRQNumber < 96)
        {
            NVIC_EN2 |= (1 << (IRQNumber - 64));
        }
        else if (IRQNumber < 128)
        {
            NVIC_EN3 |= (1 << (IRQNumber - 96));
        }
    }
    else
    {
        // Disable IRQ in NVIC using the NVIC_DISx registers
        if (IRQNumber < 32)
        {
            NVIC_DIS0 |= (1 << IRQNumber);
        }
        else if (IRQNumber < 64)
        {
            NVIC_DIS1 |= (1 << (IRQNumber - 32));
        }
        else if (IRQNumber < 96)
        {
            NVIC_DIS2 |= (1 << (IRQNumber - 64));
        }
        else if (IRQNumber < 128)
        {
            NVIC_DIS3 |= (1 << (IRQNumber - 96));
        }
    }
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
void SSI_IRQHandling(SSI_Handle_t *pSSIHandle)
{
    // Enter ISR

    // Understand which event caused interrupt to trigger (check SSIMIS)
    uint32_t temp1, temp2;

    // Check for TX FIFO interrupt (TXMIS)
    temp1 = pSSIHandle->pSSIx->MIS & (1 << SSI_SSIMIS_TXMIS);
    temp2 = pSSIHandle->pSSIx->IM & (1 << SSI_SSIIM_TXIM);

    if (temp1 && temp2)
    {
        // Interrupt is due to setting of TNF flag (TX FIFO not full)
        // Handle TNF event (TX transmission)
        SSI_TXE_InterruptHandle(pSSIHandle);
    }

    // Check for RX FIFO interrupt (RXMIS)
    temp1 = pSSIHandle->pSSIx->MIS & (1 << SSI_SSIMIS_RXMIS);
    temp2 = pSSIHandle->pSSIx->IM & (1 << SSI_SSIIM_RXIM);

    if (temp1 && temp2)
    {
        // Interrupt is due to setting of RNE flag (RX FIFO not empty)
        // Handle RNE event (RX reception)
        SSI_RXNE_InterruptHandle(pSSIHandle);
    }

    // Check for RX Timeout interrupt (RTMIS)
    temp1 = pSSIHandle->pSSIx->MIS & (1 << SSI_SSIMIS_RTMIS);
    temp2 = pSSIHandle->pSSIx->IM & (1 << SSI_SSIIM_RTIM);

    if (temp1 && temp2)
    {
        // Interrupt is due to setting of RX Timeout flag (≥32 bit-times)
        // Handle RX Timeout event
        SSI_RT_InterruptHandle(pSSIHandle);
    }

    // Check for RX Overrun Error (RORMIS)
    temp1 = pSSIHandle->pSSIx->MIS & (1 << SSI_SSIMIS_RORMIS);
    temp2 = pSSIHandle->pSSIx->IM & (1 << SSI_SSIIM_RORIM);

    if (temp1 && temp2)
    {
        // Interrupt is due to setting of RX Overrun ERROR flag
        // Handle Error (clear error and notify application)
        SSI_OVR_ErrInterruptHandle(pSSIHandle);
    }
}

static void SSI_TXE_InterruptHandle(SSI_Handle_t *pSSIHandle)
{
    //1. Get DSS (Data Size Select): bits [3:0] of CR0
    uint8_t dss = (pSSIHandle->pSSIx->CR0 & 0xF); // DSS encoding from 0x3 to 0xF (4 to 16 bits)

    if (dss <= 0x7) // DSS = 0x3 to 0x7 → 4 to 8 bits
    {
        //2. Write one byte to SSI Data register (DR)
        pSSIHandle->pSSIx->DR = *pSSIHandle->pTxBuffer;
        pSSIHandle->pTxBuffer++;
        pSSIHandle->TxLen--;
    }
    else // DSS = 0x8 to 0xF → 9 to 16 bits
    {
        //3. Write 2 bytes to SSI Data register (DR) - safe alignment method
        uint16_t data16 = pSSIHandle->pTxBuffer[0] | (pSSIHandle->pTxBuffer[1] << 8);
        pSSIHandle->pSSIx->DR = data16;
        pSSIHandle->pTxBuffer += 2;
        pSSIHandle->TxLen--;
    }

    if (!pSSIHandle->TxLen)
    {
        //4. TxLen is zero, so close the ssi transmission and inform the application that
        //   TX is over
        //5. This prevents interrupts from setting up of TXE flag
        SSI_CloseTransmission(pSSIHandle);
        SSI_ApplicationEventCallback(pSSIHandle, SSI_EVENT_TX_CMPLT);
    }
}

static void SSI_RXNE_InterruptHandle(SSI_Handle_t *pSSIHandle)
{
    //1. Do RX as long as RNE flag is set and we have data to receive
    while ((pSSIHandle->pSSIx->SR & SSI_FLAG_RNE) && (pSSIHandle->RxLen > 0))
    {
        //2. Get DSS (Data Size Select): bits [3:0] of CR0
        uint8_t dss = (pSSIHandle->pSSIx->CR0 & 0xF); // DSS encoding from 0x3 to 0xF (4 to 16 bits)

        if (dss <= 0x7) // DSS = 0x3 to 0x7 → 4 to 8 bits
        {
            //3. Read one byte from SSI Data register (DR) and mask to 8 bits
            *pSSIHandle->pRxBuffer = (uint8_t)(pSSIHandle->pSSIx->DR & 0xFF);
            pSSIHandle->pRxBuffer++;
            pSSIHandle->RxLen--;
        }
        else // DSS = 0x8 to 0xF → 9 to 16 bits
        {
            //4. Read 2 bytes from SSI Data register (DR) - safe alignment method
            uint16_t data16 = (uint16_t)(pSSIHandle->pSSIx->DR & 0xFFFF);
            // Copy bytes individually to avoid alignment issues
            pSSIHandle->pRxBuffer[0] = (uint8_t)(data16 & 0xFF);        // Low byte
            pSSIHandle->pRxBuffer[1] = (uint8_t)((data16 >> 8) & 0xFF); // High byte
            pSSIHandle->pRxBuffer += 2;
            pSSIHandle->RxLen--;
        }
    }

    if (!pSSIHandle->RxLen)
    {
        //5. RxLen is zero, so close the SSI reception and inform the application that
        //   RX is over

        //6. This prevents interrupts from setting up of RNE flag
        SSI_CloseReception(pSSIHandle);
        SSI_ApplicationEventCallback(pSSIHandle, SSI_EVENT_RX_CMPLT);
    }
}

static void SSI_RT_InterruptHandle(SSI_Handle_t *pSSIHandle)
{
    //1. Read any remaining data from RX FIFO before clearing timeout
    //   Datasheet: "RX timeout occurs when FIFO has data but no new data for ≥32 bit-times"
    //   This means FIFO may still contain valid data that needs to be read
    while ((pSSIHandle->pSSIx->SR & SSI_FLAG_RNE) && (pSSIHandle->RxLen > 0))
    {
        //2. Get DSS (Data Size Select): bits [3:0] of CR0
        uint8_t dss = (pSSIHandle->pSSIx->CR0 & 0xF); // DSS encoding from 0x3 to 0xF (4 to 16 bits)

        if (dss <= 0x7) // DSS = 0x3 to 0x7 → 4 to 8 bits
        {
            //3. Read one byte from SSI Data register (DR) and mask to 8 bits
            *pSSIHandle->pRxBuffer = (uint8_t)(pSSIHandle->pSSIx->DR & 0xFF);
            pSSIHandle->pRxBuffer++;
            pSSIHandle->RxLen--;
        }
        else // DSS = 0x8 to 0xF → 9 to 16 bits
        {
            //4. Read 2 bytes from SSI Data register (DR) - safe alignment method
            uint16_t data16 = (uint16_t)(pSSIHandle->pSSIx->DR & 0xFFFF);
            // Copy bytes individually to avoid alignment issues
            pSSIHandle->pRxBuffer[0] = (uint8_t)(data16 & 0xFF);        // Low byte
            pSSIHandle->pRxBuffer[1] = (uint8_t)((data16 >> 8) & 0xFF); // High byte
            pSSIHandle->pRxBuffer += 2;
            pSSIHandle->RxLen--;
        }
    }

    //5. Clear the timeout flag (must be done AFTER reading FIFO per datasheet)
    //   Datasheet: "ISR should clear timeout interrupt just after reading out the RX FIFO"
    pSSIHandle->pSSIx->ICR |= (1 << SSI_SSIICR_RTIC);

    //6. Inform the application about timeout event
    //   Application can decide if this indicates end of packet or communication error
    SSI_ApplicationEventCallback(pSSIHandle, SSI_EVENT_TIMEOUT);
}

static void SSI_OVR_ErrInterruptHandle(SSI_Handle_t *pSSIHandle)
{
    /* temp variable used to read DR/SR registers without storing the values
     * This prevents compiler warnings about unused return values from register reads
     * while still allowing the hardware side effects of reading these registers */
    uint8_t temp;

    /* Clear OVR flag - TM4C123GXL requires writing to ICR register */
    if (pSSIHandle->TxState != SSI_BUSY_IN_TX)
    {
        temp = pSSIHandle->pSSIx->DR;  // Read any available data
        temp = pSSIHandle->pSSIx->SR;  // Read status
    }
    /* TM4C123GXL specific: Write to ICR to clear overrun interrupt */
    pSSIHandle->pSSIx->ICR |= (1 << SSI_SSIICR_RORIC);
    (void)temp;  // Explicitly mark temp as unused to avoid compiler warnings

    /* Inform application */
    SSI_ApplicationEventCallback(pSSIHandle, SSI_EVENT_OVR_ERR);
}

/***************************************************************************
 * @fn                          - SSI_ClearOvrFlag
 *
 * @brief                       - Clears the SSI overrun error flag
 *
 * @param[in]                   - pSSIx: Pointer to the SSI base address (e.g., SSI0, SSI1)
 *
 * @return                      - none
 *
 * @Note                        - Used by application to manually clear overrun error after handling
 */
void SSI_ClearOvrFlag(SSI_RegDef_t *pSSIx)
{
    pSSIx->ICR |= (1 << SSI_SSIICR_RORIC);
}

/***************************************************************************
 * @fn                          - SSI_CloseTransmission
 *
 * @brief                       - Closes SSI transmission and resets TX state
 *
 * @param[in]                   - pSSIHandle: Pointer to the SSI handle structure
 *
 * @return                      - none
 *
 * @Note                        - Disables TX interrupt, clears buffers, resets state to READY
 */
void SSI_CloseTransmission(SSI_Handle_t *pSSIHandle)
{
    pSSIHandle->pSSIx->IM &= ~(1 << SSI_SSIIM_TXIM);
    pSSIHandle->pTxBuffer = NULL;
    pSSIHandle->TxLen = 0;
    pSSIHandle->TxState = SSI_READY;
}

/***************************************************************************
 * @fn                          - SSI_CloseReception
 *
 * @brief                       - Closes SSI reception and resets RX state
 *
 * @param[in]                   - pSSIHandle: Pointer to the SSI handle structure
 *
 * @return                      - none
 *
 * @Note                        - Disables RX interrupt, clears buffers, resets state to READY
 */
void SSI_CloseReception(SSI_Handle_t *pSSIHandle)
{
    pSSIHandle->pSSIx->IM &= ~(1 << SSI_SSIIM_RXIM);
    pSSIHandle->pRxBuffer = NULL;
    pSSIHandle->RxLen = 0;
    pSSIHandle->RxState = SSI_READY;
}

/***************************************************************************
 * @fn                          - SSI_ApplicationEventCallback
 *
 * @brief                       - Application callback function for SSI events
 *
 * @param[in]                   - pSSIHandle: Pointer to the SSI handle structure
 * @param[in]                   - AppEv: Event that triggered the callback (SSI_EVENT_TX_CMPLT, etc.)
 *
 * @return                      - none
 *
 * @Note                        - Weak implementation - application should override this function
 *                                to handle SSI events (TX complete, RX complete, errors, etc.)
 */
__weak void SSI_ApplicationEventCallback(SSI_Handle_t *pSSIHandle, uint8_t AppEv)
{
    // This is a weak implementation. The application may override this function.
}
