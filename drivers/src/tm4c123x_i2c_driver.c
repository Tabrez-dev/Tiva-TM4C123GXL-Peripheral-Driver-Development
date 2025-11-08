/*
 * tm4c123x_i2c_driver.c
 *
 *  Created on: Jan 2025
 *      Author: Tabrez
 */

#include "tm4c123x_i2c_driver.h"

/*******************************************************************************
 *                          PRIVATE HELPER FUNCTIONS
 ******************************************************************************/

/*
 * Crystal frequency lookup table
 * Maps XTAL field value to actual crystal frequency in Hz
 * Based on TM4C123GH6PM datasheet Table 5-5
 */
static const uint32_t XTAL_FREQ_TABLE[] = {
    [0x06] = 4000000UL,     /* 4 MHz */
    [0x07] = 4096000UL,     /* 4.096 MHz */
    [0x08] = 4915200UL,     /* 4.9152 MHz */
    [0x09] = 5000000UL,     /* 5 MHz */
    [0x0A] = 5120000UL,     /* 5.12 MHz */
    [0x0B] = 6000000UL,     /* 6 MHz */
    [0x0C] = 6144000UL,     /* 6.144 MHz */
    [0x0D] = 7372800UL,     /* 7.3728 MHz */
    [0x0E] = 8000000UL,     /* 8 MHz */
    [0x0F] = 8192000UL,     /* 8.192 MHz */
    [0x10] = 10000000UL,    /* 10 MHz */
    [0x11] = 12000000UL,    /* 12 MHz */
    [0x12] = 12288000UL,    /* 12.288 MHz */
    [0x13] = 13560000UL,    /* 13.56 MHz */
    [0x14] = 14318180UL,    /* 14.31818 MHz */
    [0x15] = 16000000UL,    /* 16 MHz (TM4C123GXL LaunchPad default) */
    [0x16] = 16384000UL,    /* 16.384 MHz */
    [0x17] = 18000000UL,    /* 18 MHz */
    [0x18] = 20000000UL,    /* 20 MHz */
    [0x19] = 24000000UL,    /* 24 MHz */
    [0x1A] = 25000000UL     /* 25 MHz */
};

/* Oscillator source values */
#define OSC_SOURCE_MOSC         0   /* Main oscillator */
#define OSC_SOURCE_PIOSC        1   /* Precision internal oscillator (16 MHz) */
#define OSC_SOURCE_PIOSC_DIV4   2   /* PIOSC/4 (4 MHz) */
#define OSC_SOURCE_LFIOSC       3   /* Low frequency internal oscillator (30 kHz) */

/* PLL output frequency (always 400 MHz when enabled) */
#define PLL_OUTPUT_FREQ         400000000UL

/* PIOSC frequency (precision internal oscillator) */
#define PIOSC_FREQ              16000000UL

/* I2C clock generation constants */
#define I2C_SCL_LP_NORMAL       6   /* Low period count (standard/fast/fast+) */
#define I2C_SCL_HP_NORMAL       4   /* High period count (standard/fast/fast+) */
#define I2C_CLOCK_FACTOR        (I2C_SCL_LP_NORMAL + I2C_SCL_HP_NORMAL)  /* = 10 */

/* TPR limits */
#define I2C_TPR_MIN             1
#define I2C_TPR_MAX             127

/***********************************************************************************
 * @fn              - RCC_GetSystemClockFreq
 *
 * @brief           - Reads RCC/RCC2 registers to determine actual system clock frequency
 *
 * @return          - System clock frequency in Hz
 *
 * @Note            - Analyzes clock source (MOSC/PIOSC/PLL) and dividers
 */
static uint32_t RCC_GetSystemClockFreq(void)
{
    uint32_t systemClock;
    uint32_t rcc = SYSCTL_CORE->RCC;
    uint32_t rcc2 = SYSCTL_CORE->RCC2;

    /* Step 1: Check if RCC2 is active (bit 31: USERCC2) */
    uint8_t useRCC2 = (rcc2 >> 31) & 0x1;

    /* Step 2: Determine oscillator source */
    uint32_t oscSource;
    if (useRCC2) {
        /* RCC2 OSCSRC2 field (bits 6:4) */
        oscSource = (rcc2 >> 4) & 0x7;
    } else {
        /* RCC OSCSRC field (bits 5:4) */
        oscSource = (rcc >> 4) & 0x3;
    }

    /* Step 3: Get base clock from selected source */
    switch (oscSource) {
        case OSC_SOURCE_MOSC:
        {
            /* Main oscillator - read crystal frequency from XTAL field */
            uint32_t xtal = (rcc >> 6) & 0x1F;  /* XTAL bits 10:6 */
            systemClock = XTAL_FREQ_TABLE[xtal];
            break;
        }

        case OSC_SOURCE_PIOSC:
            /* Precision internal oscillator - always 16 MHz */
            systemClock = PIOSC_FREQ;
            break;

        case OSC_SOURCE_PIOSC_DIV4:
            /* PIOSC divided by 4 */
            systemClock = PIOSC_FREQ / 4;
            break;

        case OSC_SOURCE_LFIOSC:
            /* Low frequency internal oscillator - 30 kHz */
            systemClock = 30000;
            break;

        default:
            /* Fallback to PIOSC */
            systemClock = PIOSC_FREQ;
            break;
    }

    /* Step 4: Check if PLL is being used */
    uint8_t pllBypassed;
    if (useRCC2) {
        /* RCC2 BYPASS2 bit (bit 11) */
        pllBypassed = (rcc2 >> 11) & 0x1;
    } else {
        /* RCC BYPASS bit (bit 11) */
        pllBypassed = (rcc >> 11) & 0x1;
    }

    if (!pllBypassed) {
        /* PLL is active - output is always 400 MHz */
        systemClock = PLL_OUTPUT_FREQ;
    }

    /* Step 5: Apply system clock divider */
    uint8_t useSysDiv;
    uint32_t divisor = 1;

    if (useRCC2) {
        /* Check DIV400 bit (bit 30) */
        uint8_t div400 = (rcc2 >> 30) & 0x1;

        if (div400) {
            /* Using enhanced divider for precise clock frequencies */
            /* SYSDIV2LSB (bit 22) + SYSDIV2 (bits 28:23) form 7-bit divisor */
            uint32_t sysdiv2lsb = (rcc2 >> 22) & 0x1;
            uint32_t sysdiv2 = (rcc2 >> 23) & 0x3F;
            divisor = (sysdiv2 << 1) | sysdiv2lsb;
            divisor += 1;  /* Actual divisor is register value + 1 */
        } else {
            /* Using standard divider */
            useSysDiv = (rcc2 >> 22) & 0x1;  /* USESYSDIV2 bit */
            if (useSysDiv) {
                divisor = ((rcc2 >> 23) & 0x3F) + 1;  /* SYSDIV2 field + 1 */
            }
        }
    } else {
        /* RCC: Check USESYSDIV bit (bit 22) */
        useSysDiv = (rcc >> 22) & 0x1;
        if (useSysDiv) {
            divisor = ((rcc >> 23) & 0xF) + 1;  /* SYSDIV field (4 bits) + 1 */
        }
    }

    /* Apply divisor to get final system clock */
    systemClock = systemClock / divisor;

    return systemClock;
}

/***********************************************************************************
 * @fn              - I2C_CalculateTPR
 *
 * @brief           - Calculates Timer Period Register value for desired I2C SCL speed
 *
 * @param[in]       - Desired I2C SCL frequency in Hz (100000, 400000, 1000000)
 *
 * @return          - TPR value (1-127)
 *
 * @Note            - Formula: TPR = (System_Clock / (2 × 10 × I2C_Speed)) - 1
 *                    Clamps result to valid range (1-127)
 */
static uint8_t I2C_CalculateTPR(uint32_t sclSpeed)
{
    /* Get current system clock frequency */
    uint32_t systemClock = RCC_GetSystemClockFreq();

    /* Calculate TPR using formula from datasheet
     * SCL_PERIOD = 2 × (1 + TPR) × (SCL_LP + SCL_HP) × CLK_PRD
     * Rearranged: TPR = (System_Clock / (2 × 10 × SCL_Frequency)) - 1
     */
    uint32_t tpr = (systemClock / (2 * I2C_CLOCK_FACTOR * sclSpeed)) - 1;

    /* Clamp to valid range */
    if (tpr < I2C_TPR_MIN) {
        tpr = I2C_TPR_MIN;
    }
    if (tpr > I2C_TPR_MAX) {
        tpr = I2C_TPR_MAX;
    }

    return (uint8_t)tpr;
}

/*******************************************************************************
 *                          PUBLIC API IMPLEMENTATIONS
 ******************************************************************************/

/***********************************************************************************
 * @fn              - I2C_PeriClockControl
 *
 * @brief           - Enables or disables peripheral clock for I2C module
 *
 * @param[in]       - pI2Cx: Base address of I2C peripheral (I2C0, I2C1, I2C2, I2C3)
 * @param[in]       - EnorDi: ENABLE or DISABLE
 *
 * @return          - none
 *
 * @Note            - Must be called before accessing I2C registers
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        /* Enable clock by setting appropriate bit in RCGCI2C register */
        if (pI2Cx == I2C0) {
            I2C0_PCLK_EN();
        } else if (pI2Cx == I2C1) {
            I2C1_PCLK_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_EN();
        }

        /* Wait for peripheral to be ready (check PRI2C register) */
        while (!(SYSCTL_PR->PRI2C & (1 << ((((uint32_t)pI2Cx - I2C_0_BASEADDR) / 0x1000)))));

    } else {
        /* Disable clock by clearing bit in RCGCI2C register */
        if (pI2Cx == I2C0) {
            SYSCTL_RUNCLK->RCGCI2C &= ~(1U << 0);
        } else if (pI2Cx == I2C1) {
            SYSCTL_RUNCLK->RCGCI2C &= ~(1U << 1);
        } else if (pI2Cx == I2C2) {
            SYSCTL_RUNCLK->RCGCI2C &= ~(1U << 2);
        } else if (pI2Cx == I2C3) {
            SYSCTL_RUNCLK->RCGCI2C &= ~(1U << 3);
        }
    }
}

/***********************************************************************************
 * @fn              - I2C_MasterInit
 *
 * @brief           - Initializes I2C peripheral in Master mode
 *
 * @param[in]       - pI2CHandle: Pointer to I2C handle structure
 * @param[in]       - I2C_SCLSpeed: Desired SCL clock speed in Hz
 *                    - I2C_SCL_SPEED_SM (100000 Hz - Standard Mode)
 *                    - I2C_SCL_SPEED_FM (400000 Hz - Fast Mode)
 *                    - I2C_SCL_SPEED_FMP (1000000 Hz - Fast Mode Plus)
 *
 * @return          - none
 *
 * @Note            - GPIO pins must be configured separately for I2C alternate function
 *                    Peripheral clock must be enabled before calling this function
 */
void I2C_MasterInit(I2C_Handle_t *pI2CHandle, uint32_t I2C_SCLSpeed)
{
    /* Note: Peripheral clock must be enabled and peripheral reset must be done
     * by application BEFORE calling this function.
     * For loopback mode: Reset should be done ONCE before both init functions.
     */

    /* Step 1: Clear only master-specific bits, preserve slave configuration
     * Don't use MCR = 0 as it would clear SFE if slave was initialized first
     * After reset, MCR is already 0, so we can skip this step
     */

    /* Step 2: Calculate and configure Timer Period for desired SCL speed
     * TPR determines the I2C clock frequency
     */
    uint8_t tpr = I2C_CalculateTPR(I2C_SCLSpeed);
    pI2CHandle->pI2Cx->MTPR = (tpr & 0x7F);  /* Only lower 7 bits, HS=0 */

    /* Step 3: Enable glitch filter (optional but recommended)
     * Filters out noise spikes on SCL/SDA lines
     * Use |= to preserve any existing slave configuration bits (SFE)
     */
    pI2CHandle->pI2Cx->MCR |= (1 << I2C_MCR_GFE);

    /* Step 4: Enable I2C Master Function
     * Sets MFE bit - peripheral now operates as I2C master
     * Use |= to allow dual master+slave operation (required for loopback mode)
     */
    pI2CHandle->pI2Cx->MCR |= (1 << I2C_MCR_MFE);
}

/***********************************************************************************
 * @fn              - I2C_SlaveInit
 *
 * @brief           - Initializes I2C peripheral in Slave mode
 *
 * @param[in]       - pI2CHandle: Pointer to I2C handle structure
 * @param[in]       - SlaveAddr: 7-bit slave address (0x00 to 0x7F)
 *
 * @return          - none
 *
 * @Note            - GPIO pins must be configured separately for I2C alternate function
 *                    Peripheral clock must be enabled before calling this function
 *                    Slave does not configure clock speed (uses master's clock)
 */
void I2C_SlaveInit(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr)
{
    /* Note: Peripheral clock must be enabled by application BEFORE calling this function */

    /* Step 1: Enable glitch filter FIRST (optional but recommended)
     * Filters out noise spikes on SCL/SDA lines
     * Note: Use OR to preserve any existing MCR bits (e.g., MFE for loopback mode)
     */
    pI2CHandle->pI2Cx->MCR |= (1 << I2C_MCR_GFE);

    /* Step 2: Enable I2C Slave Function BEFORE writing SOAR
     * CRITICAL: SOAR register is write-protected unless SFE is enabled first!
     * This is an undocumented hardware requirement discovered through testing
     */
    pI2CHandle->pI2Cx->MCR |= (1 << I2C_MCR_SFE);

    /* Step 3: Set slave own address (NOW that SFE is enabled)
     * This is the address the slave will respond to on the I2C bus
     * SOAR writes will be ignored if SFE=0!
     */
    pI2CHandle->pI2Cx->SOAR = (SlaveAddr & 0x7F);  /* 7-bit address only */

    /* Step 4: Activate slave device (CRITICAL for slave operation!)
     * Per datasheet Section 16.6 Register 14:
     * "Once this bit has been set, it should not be set again unless it has
     *  been cleared by writing a 0 or by a reset, otherwise transfer failures
     *  may occur."
     *
     * Solution: Always clear DA first, then set it to avoid transfer failures
     * from repeated initializations during debugging
     */
    pI2CHandle->pI2Cx->SCSR = 0;                 /* Clear DA bit first */
    pI2CHandle->pI2Cx->SCSR = (1 << I2C_SCSR_DA); /* Now activate slave */
}

/***********************************************************************************
 * @fn              - I2C_DeInit
 *
 * @brief           - De-initializes I2C peripheral (resets to default state)
 *
 * @param[in]       - pI2Cx: Base address of I2C peripheral
 *
 * @return          - none
 *
 * @Note            - Uses software reset via SRI2C register
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
    /* Reset peripheral using appropriate reset macro */
    if (pI2Cx == I2C0) {
        I2C0_RESET();
    } else if (pI2Cx == I2C1) {
        I2C1_RESET();
    } else if (pI2Cx == I2C2) {
        I2C2_RESET();
    } else if (pI2Cx == I2C3) {
        I2C3_RESET();
    }
}

/***********************************************************************************
 * @fn              - I2C_GetFlagStatus
 *
 * @brief           - Returns the status of specified I2C flag
 *
 * @param[in]       - pI2Cx: Base address of I2C peripheral
 * @param[in]       - FlagName: Flag to check (I2C_FLAG_BUSY, I2C_FLAG_ERROR, etc.)
 *
 * @return          - FLAG_SET or FLAG_RESET
 *
 * @Note            - Reads MCS register status bits
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
    if (pI2Cx->MCS & FlagName) {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/***********************************************************************************
 * @fn              - I2C_PeripheralControl
 *
 * @brief           - Enables or disables I2C peripheral
 *
 * @param[in]       - pI2Cx: Base address of I2C peripheral
 * @param[in]       - EnorDi: ENABLE or DISABLE
 *
 * @return          - none
 *
 * @Note            - Controls MFE/SFE bits in MCR register
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        /* Enable both master and slave functions if they were previously configured */
        pI2Cx->MCR |= (1 << I2C_MCR_MFE) | (1 << I2C_MCR_SFE);
    } else {
        /* Disable both master and slave functions */
        pI2Cx->MCR &= ~((1 << I2C_MCR_MFE) | (1 << I2C_MCR_SFE));
    }
}

/***********************************************************************************
 * @fn              - I2C_LoopbackControl
 *
 * @brief           - Enable/disable I2C internal loopback mode for testing
 *
 * @param[in]       - pI2Cx: Base address of I2C peripheral
 * @param[in]       - EnorDi: ENABLE or DISABLE
 *
 * @return          - none
 *
 * @Note            - Loopback mode (MCR.LPBK bit 0) routes master SDA/SCL signals
 *                    to the slave module internally for diagnostic testing
 *                    Datasheet Section 16.3.4: "The SDA and SCL signals from the
 *                    master are tied to the SDA and SCL signals of the slave module"
 */
void I2C_LoopbackControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        /* Enable loopback mode: Master → Slave internal routing */
        pI2Cx->MCR |= (1 << I2C_MCR_LPBK);
    } else {
        /* Disable loopback mode: Normal I2C operation */
        pI2Cx->MCR &= ~(1 << I2C_MCR_LPBK);
    }
}

/***********************************************************************************
 *                     I2C MASTER CONTROL SEQUENCE HELPERS
 ***********************************************************************************/

/***********************************************************************************
 * @fn              - I2C_MasterSendSingle
 *
 * @brief           - Generates single byte transmit sequence (START + RUN + STOP)
 *
 * @param[in]       - pI2Cx: Base address of I2C peripheral
 *
 * @return          - none
 *
 * @Note            - MCS = 0x07 (START + RUN + STOP)
 *                    Figure 16-8: Master Single TRANSMIT
 */
void I2C_MasterSendSingle(I2C_RegDef_t *pI2Cx)
{
    /* Single byte transmit: START + RUN + STOP (0x07) */
    pI2Cx->MCS = (1 << I2C_MCS_STOP) | (1 << I2C_MCS_START) | (1 << I2C_MCS_RUN);
}

/***********************************************************************************
 * @fn              - I2C_GenerateStartCondition
 *
 * @brief           - Generates START condition (initiates I2C transfer or repeated START)
 *
 * @param[in]       - pI2Cx: Base address of I2C peripheral
 *
 * @return          - none
 *
 * @Note            - MCS = 0x03 (START + RUN)
 *                    Used for: Burst first byte, repeated START
 *                    Figure 16-10: Master TRANSMIT of Multiple Data Bytes
 */
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    /* Generate START + RUN (0x03) */
    pI2Cx->MCS = (1 << I2C_MCS_START) | (1 << I2C_MCS_RUN);
}

/***********************************************************************************
 * @fn              - I2C_MasterBurstContinue
 *
 * @brief           - Continues burst transfer (middle bytes or hold bus for repeated START)
 *
 * @param[in]       - pI2Cx: Base address of I2C peripheral
 *
 * @return          - none
 *
 * @Note            - MCS = 0x01 (RUN only)
 *                    Used for: Burst middle bytes, holding bus without STOP
 *                    Figure 16-10: Master TRANSMIT of Multiple Data Bytes - Middle bytes
 */
void I2C_MasterBurstContinue(I2C_RegDef_t *pI2Cx)
{
    /* Burst continue: RUN only (0x01) */
    pI2Cx->MCS = (1 << I2C_MCS_RUN);
}

/***********************************************************************************
 * @fn              - I2C_GenerateStopCondition
 *
 * @brief           - Generates STOP condition (terminates I2C transfer)
 *
 * @param[in]       - pI2Cx: Base address of I2C peripheral
 *
 * @return          - none
 *
 * @Note            - MCS = 0x05 (STOP + RUN)
 *                    Used for: Burst last byte, terminating transaction
 *                    Figure 16-10: Master TRANSMIT of Multiple Data Bytes - Last byte
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    /* Generate STOP + RUN (0x05) */
    pI2Cx->MCS = (1 << I2C_MCS_STOP) | (1 << I2C_MCS_RUN);
}

/*******************************************************************************
 *                  STUB IMPLEMENTATIONS (TO BE COMPLETED)
 ******************************************************************************/

/***********************************************************************************
 * @fn              - I2C_MasterSendData
 *
 * @brief           - Transmits data to I2C slave device (blocking mode)
 *
 * @param[in]       - pI2CHandle: Pointer to I2C handle structure
 * @param[in]       - pTxbuffer: Pointer to data buffer to transmit
 * @param[in]       - Len: Number of bytes to send
 * @param[in]       - SlaveAddr: 7-bit I2C slave address
 * @param[in]       - Sr: Repeated start control (I2C_ENABLE_SR or I2C_DISABLE_SR)
 *
 * @return          - none
 *
 * @Note            - This is a blocking call - function waits until transmission completes
 *                    Based on TM4C datasheet Figures 16-8 (single) and 16-10 (burst)
 *                    MCS control sequences:
 *                      Single byte: 0x07 (START + RUN + STOP)
 *                      Burst first: 0x03 (START + RUN)
 *                      Burst middle: 0x01 (RUN only)
 *                      Burst last: 0x05 (STOP + RUN) or 0x01 (RUN if repeated start)
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    /* Step 1: Configure slave address in MSA register
     * Bits 7:1 = slave address
     * Bit 0 (R/S) = 0 for transmit operation
     */
    pI2CHandle->pI2Cx->MSA = (SlaveAddr << 1);  /* Shift addr left, R/S bit = 0 */

    /* Step 2: Check if single byte or multi-byte transmission */
    if (Len == 1) {
        /*******************************************************************
         * SINGLE BYTE TRANSMISSION (Figure 16-8)
         * Uses simplified single-byte control sequence
         *******************************************************************/

        /* Write data byte to Master Data Register */
        pI2CHandle->pI2Cx->MDR = *pTxbuffer;

        /* Generate single byte transmit sequence: START + RUN + STOP
         * This completes entire transaction in hardware automatically
         */
        I2C_MasterSendSingle(pI2CHandle->pI2Cx);

        /* Wait for operation to complete by polling BUSY bit
         * BUSY bit (bit 0) in MCS clears when operation finishes
         * Add timeout to prevent infinite hang
         */
        volatile uint32_t timeout = 100000;
        while (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BUSY) == FLAG_SET && timeout--);

        /* If timeout, return immediately - check MCS for error flags */
        if (timeout == 0) {
            return;  /* Timeout - transaction didn't complete */
        }

        /* Check for address acknowledgment error (slave didn't respond) */
        if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADRACK) == FLAG_SET) {
            return;  /* Slave address not acknowledged - slave not present or wrong address */
        }

        /* Check for other transmission errors
         * ERROR bit indicates general error condition
         */
        if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ERROR) == FLAG_SET) {
            return;  /* General error occurred */
        }

    } else {
        /*******************************************************************
         * MULTI-BYTE BURST TRANSMISSION (Figure 16-10)
         * Uses three-phase burst control sequence
         *******************************************************************/

        volatile uint32_t timeout;  /* Timeout for BUSY waits */

        /* PHASE 1: First byte - START condition */
        pI2CHandle->pI2Cx->MDR = pTxbuffer[0];  /* Load first data byte */

        /* Generate START condition to initiate burst transfer
         * This initiates burst transfer sequence
         */
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        /* Wait for first byte transmission to complete */
        timeout = 100000;
        while (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BUSY) == FLAG_SET && timeout--);
        if (timeout == 0) return;

        /* Check for address acknowledgment error (slave didn't respond) */
        if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADRACK) == FLAG_SET) {
            return;  /* Slave address not acknowledged - slave not present or wrong address */
        }

        /* Check for other errors (arbitration lost, etc.) */
        if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ERROR) == FLAG_SET) {
            return;  /* General error occurred */
        }

        /* PHASE 2: Middle bytes (if any) - Continue transfer */
        for (uint32_t i = 1; i < (Len - 1); i++) {
            pI2CHandle->pI2Cx->MDR = pTxbuffer[i];  /* Load next data byte */

            /* Generate burst continue sequence: RUN only (no START, no STOP)
             * This continues the burst transfer
             */
            I2C_MasterBurstContinue(pI2CHandle->pI2Cx);

            /* Wait for byte transmission to complete */
            timeout = 100000;
            while (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BUSY) == FLAG_SET && timeout--);
            if (timeout == 0) return;

            /* Check for errors (data NACK, arbitration lost) */
            if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ERROR) == FLAG_SET) {
                return;
            }
        }

        /* PHASE 3: Last byte - Terminate transfer */
        pI2CHandle->pI2Cx->MDR = pTxbuffer[Len - 1];  /* Load last data byte */

        if (Sr == I2C_DISABLE_SR) {
            /* Normal termination - generate STOP condition
             * This completes the burst transfer and releases the bus
             */
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        } else {
            /* Repeated START mode - hold bus without STOP
             * Bus remains held for next transaction (useful for read-after-write)
             */
            I2C_MasterBurstContinue(pI2CHandle->pI2Cx);
        }

        /* Wait for last byte transmission to complete */
        timeout = 100000;
        while (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BUSY) == FLAG_SET && timeout--);
        if (timeout == 0) return;

        /* Final error check */
        if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ERROR) == FLAG_SET) {
            return;
        }
    }
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    /* TODO: Implement master receive data (blocking mode) */
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    /* TODO: Implement interrupt-based master send */
    return I2C_READY;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    /* TODO: Implement interrupt-based master receive */
    return I2C_READY;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    /* TODO: Implement close receive logic */
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    /* TODO: Implement close send logic */
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
    /* TODO: Implement slave send */
    pI2Cx->SDR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
    volatile uint32_t timeout = 100000;
    uint8_t data;

    /* Wait for receive request - first RREQ might be for address byte */
    while (!(pI2Cx->SCSR & (1 << I2C_SCSR_RREQ)) && timeout--);

    /* Check if this is the first byte (data) or just address match
     * FBR bit is set when we have actual data, not just address
     */
    if (pI2Cx->SCSR & (1 << I2C_SCSR_FBR)) {
        /* FBR is set - we have data, read it directly */
        data = (uint8_t)(pI2Cx->SDR & 0xFF);
    } else {
        /* FBR not set - might be address byte still in SDR
         * Do a dummy read to clear it, then wait for actual data
         */
        (void)(pI2Cx->SDR);  /* Dummy read to clear address byte */

        /* Wait for next RREQ - this should be the actual data */
        timeout = 100000;
        while (!(pI2Cx->SCSR & (1 << I2C_SCSR_RREQ)) && timeout--);

        /* Now read the actual data */
        data = (uint8_t)(pI2Cx->SDR & 0xFF);
    }

    return data;
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    /* TODO: Implement IRQ configuration */
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    /* TODO: Implement IRQ priority configuration */
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    /* TODO: Implement event IRQ handling */
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    /* TODO: Implement error IRQ handling */
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    /* Note: ACK is controlled per-transaction in MCS register for TM4C123x
     * This is a placeholder for API compatibility
     */
}

__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
    /* Weak implementation - application should override this function */
}
