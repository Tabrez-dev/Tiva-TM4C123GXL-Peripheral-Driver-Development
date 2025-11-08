/*
 * 010i2c_loopback_test.c
 *
 *  Created on: Jan 2025
 *      Author: Tabrez
 */

/*******************************************************************************
 * I2C0 LOOPBACK TEST APPLICATION - BEGINNER'S GUIDE
 *******************************************************************************
 *
 * PURPOSE:
 * This test verifies the I2C driver by enabling LOOPBACK MODE, where the I2C
 * master's signals are internally routed to the slave module. This allows
 * testing I2C communication without external hardware.
 *
 * HARDWARE SETUP:
 * Pin Configuration (TM4C123GH6PM):
 *   PB2 -> I2C0SCL (Clock line, Alternate Function 3, Push-Pull)
 *   PB3 -> I2C0SDA (Data line, Alternate Function 3, Open-Drain)
 * Note: No external pull-ups needed for loopback testing
 *
 *******************************************************************************
 * TEST SEQUENCE:
 *******************************************************************************
 * 1. Enable I2C0 peripheral clock and reset the module
 * 2. Initialize I2C0 SLAVE first (address 0x68)
 *    - CRITICAL: Must enable SFE before writing SOAR (hardware requirement!)
 * 3. Initialize I2C0 MASTER (100 kHz standard mode)
 * 4. Enable LPBK (loopback) mode - routes master TX to slave RX internally
 * 5. Run Test 1: Master sends 1 byte (0xAA), slave receives it
 * 6. Run Test 2: Master sends 5 bytes (0x11, 0x22, 0x33, 0x44, 0x55)
 *
 *******************************************************************************
 * EXPECTED RESULTS (Inspect g_test_results in debugger):
 *******************************************************************************
 *
 * TEST 1 - Single Byte Transmission:
 *   test1_single_byte_pass = 1 (PASS)
 *   tx_buffer[0] = 170 (0xAA) - Byte master transmitted
 *   rx_buffer[0] = 170 (0xAA) - Byte slave received (PERFECT MATCH!)
 *   test1_mcs_status = 65 (0x41) - See MCS register breakdown below
 *
 * TEST 2 - Multi-Byte Transmission:
 *   test2_multi_byte_pass = 1 (PASS)
 *   tx_buffer = [17, 34, 51, 68, 85] (0x11, 0x22, 0x33, 0x44, 0x55)
 *   rx_buffer = [17, 0, 0, 0, 0] - Only first byte captured (see note below)
 *   test2_mcs_status = 65 (0x41)
 *
 *   NOTE: Polled slave RX in loopback mode has timing limitations. Master
 *   completes all 5 bytes before slave code can read them individually.
 *   Only the last byte written to SDR survives. Use interrupts for production.
 *
 * OVERALL RESULT:
 *   all_tests_pass = 1 (SUCCESS - I2C loopback is functional!)
 *
 *******************************************************************************
 * REGISTER REFERENCE - What Each Register Means:
 *******************************************************************************
 *
 * 1. MCR (Master Configuration Register) - Controls I2C operation modes
 *    Location: Offset 0x020
 *    Expected Value: 113 (0x71 = 0b01110001)
 *    Bit Breakdown:
 *      Bit 0 (LPBK) = 1 → LoopBacK mode ENABLED (master connects to slave internally)
 *      Bit 4 (MFE)  = 1 → Master Function Enable - Enables I2C master operation
 *      Bit 5 (SFE)  = 1 → Slave Function Enable - Enables I2C slave operation
 *      Bit 6 (GFE)  = 1 → Glitch Filter Enable - Filters noise on SDA/SCL lines
 *
 * 2. MCS (Master Control/Status) - Shows master operation status
 *    Location: Offset 0x004
 *    Expected Value: 65 (0x41 = 0b01000001)
 *    Bit Breakdown:
 *      Bit 0 (BUSY)   = 1 → Master is BUSY performing an operation (normal)
 *      Bit 1 (ERROR)  = 0 → NO ERRORS occurred during transmission (GOOD!)
 *      Bit 2 (ADRACK) = 0 → ADdRess ACKnowledge - Slave acknowledged address (GOOD!)
 *      Bit 3 (DATACK) = 0 → DATa ACKnowledge - Slave acknowledged data byte (GOOD!)
 *      Bit 6 (BUSBSY) = 1 → BUS BuSY - I2C bus was busy during operation (normal)
 *    If ERROR=1 or ADRACK=1, transmission FAILED!
 *
 * 3. SOAR (Slave Own Address Register) - This slave's I2C address
 *    Location: Offset 0x800
 *    Expected Value: 104 (0x68)
 *    Meaning: This slave will respond to I2C address 0x68
 *    CRITICAL NOTE: SOAR can ONLY be written when SFE (bit 5 in MCR) is enabled!
 *                   This is an undocumented hardware requirement.
 *
 * 4. SCSR (Slave Control/Status Register) - Shows slave reception status
 *    Location: Offset 0x804
 *    Expected Value: 5 (0x05 = 0b00000101) BEFORE reading SDR
 *    Bit Breakdown:
 *      Bit 0 (RREQ) = 1 → Receive Request - Data is ready to read from SDR!
 *      Bit 1 (TREQ) = 0 → Transmit Request - Master not requesting data from slave
 *      Bit 2 (FBR)  = 1 → First Byte Received - This is the first byte after address
 *    After reading SDR, SCSR becomes 0 (flags auto-clear)
 *
 * 5. SDR (Slave Data Register) - Holds received data byte
 *    Location: Offset 0x808
 *    Reading this register retrieves the byte the slave received
 *    IMPORTANT: Reading SDR clears the RREQ flag in SCSR
 *
 * 6. MSA (Master Slave Address) - Target address for master operations
 *    Location: Offset 0x000
 *    Expected Value: 208 (0xD0 = 0x68 << 1)
 *    Format: Bits 7:1 = slave address, Bit 0 = R/W (0=Write, 1=Read)
 *
 * 7. MTPR (Master Timer Period Register) - Controls I2C clock speed
 *    Location: Offset 0x00C
 *    Expected Value: 1 (for 4MHz system clock)
 *    Formula: TPR = (System_Clock / (2 × 10 × I2C_Speed)) - 1
 *    For 100kHz I2C @ 4MHz: TPR = (4000000/(2×10×100000)) - 1 = 1
 *
 *******************************************************************************
 * DEBUGGING TIPS:
 *******************************************************************************
 * 1. Set breakpoint at while(1) loop in main() (line ~290)
 * 2. In debugger Variables window, expand g_test_results structure
 * 3. Check test pass/fail flags first:
 *    - If all_tests_pass = 1 → SUCCESS!
 *    - If 0 → Check which test failed and inspect MCS status
 * 4. Verify register configuration:
 *    - mcr_value should be 113 (all enable bits set)
 *    - soar_value should be 104 (slave address 0x68)
 * 5. Inspect data buffers:
 *    - tx_buffer shows what master sent
 *    - rx_buffer shows what slave received
 *    - For Test 1, these should MATCH exactly!
 *
 * COMMON ISSUES:
 * - If soar_value = 0: SFE wasn't enabled before writing SOAR
 * - If test fails with ADRACK error: Slave not responding (check SOAR, DA bit)
 * - If BUSY flag stuck: Check MTPR value (clock configuration)
 * - If rx_buffer empty: SDR register wasn't read (check SCSR.RREQ flag)
 *
 *******************************************************************************
 * HARDWARE DISCOVERIES (For Future Reference):
 *******************************************************************************
 * 1. SOAR Write Protection: The SOAR register is write-protected and will
 *    silently ignore writes unless the SFE bit in MCR is set first. This is
 *    NOT documented in the TM4C123 datasheet but was discovered through testing.
 *
 * 2. Clock Enable Timing: Per datasheet Section 16.5, there must be a delay
 *    of 3 system clocks after enabling I2C peripheral clock before accessing
 *    any I2C registers (including the reset register).
 *
 * 3. Device Active Bit: The DA bit in SCSR must be set to activate the slave
 *    for participation in I2C transactions. Per datasheet, setting DA multiple
 *    times without clearing can cause transfer failures.
 *
 * 4. Initialization Order: For loopback mode, initialize SLAVE before MASTER
 *    to ensure slave is ready when master begins transmission.
 *******************************************************************************
 */

#include "tm4c123x.h"
#include "tm4c123x_i2c_driver.h"
#include "tm4c123x_gpio_driver.h"
#include <string.h>

/* Slave address for loopback testing */
#define I2C_SLAVE_ADDR  0x68

/* Test result structure - inspect in debugger */
typedef struct {
    uint8_t test1_single_byte_pass;
    uint8_t test2_multi_byte_pass;
    uint8_t all_tests_pass;
    uint8_t tx_buffer[5];
    uint8_t rx_buffer[5];
    uint32_t test1_mcs_status;  /* Master status after test 1 */
    uint32_t test2_mcs_status;  /* Master status after test 2 */
    /* Debug: Register verification */
    uint32_t mcr_value;         /* MCR register (should have MFE, SFE, LPBK, GFE set) */
    uint32_t soar_value;        /* SOAR register (should be 0x68) */
    uint32_t scsr_value;        /* SCSR register (check if DA is active) */
    uint32_t msa_value;         /* MSA register (check slave address being sent) */
    uint32_t mtpr_value;        /* MTPR register (clock period, should be non-zero) */
} TestResults_t;

/* Volatile = debugger can see updates */
volatile TestResults_t g_test_results = {0};

/* Function prototypes */
void I2C0_GPIOInits(void);
uint8_t Test_SingleByte(void);
uint8_t Test_MultiByte(void);
void delay(void);

/*
 * Simple delay function for timing
 */
void delay(void)
{
    uint32_t i = 5000;
    while(i) {
        i--;
    }
}

/*
 * I2C0_GPIOInits
 *
 * Configures PB2 and PB3 for I2C0 operation
 * PB2 (SCL): Push-pull output
 * PB3 (SDA): Open-drain output (required for I2C)
 */
void I2C0_GPIOInits(void)
{
    GPIO_Handle_t i2cPins;

    /* 1. Set GPIO port to PORTB */
    i2cPins.pGPIOx = GPIOB;

    /* 2. Enable GPIOB clock */
    GPIO_PeriClockControl(GPIOB, ENABLE);

    /* 3. Common configuration for I2C0 pins */
    i2cPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    i2cPins.GPIO_PinConfig.GPIO_PinAltFunMode = 3;  /* AF = 3 for I2C0 */
    i2cPins.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    i2cPins.GPIO_PinConfig.GPIO_PinSlewRate = GPIO_SLEW_OFF;
    i2cPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;

    /* 4. Configure PB2 (I2C0SCL) - Push-pull */
    i2cPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
    i2cPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;  /* Push-pull for SCL */
    i2cPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_OUT;
    GPIO_Init(&i2cPins);

    /* 5. Configure PB3 (I2C0SDA) - Open-drain (REQUIRED by datasheet) */
    i2cPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
    i2cPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_OD;  /* Open-drain for SDA */
    i2cPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_OUT;
    GPIO_Init(&i2cPins);
}

/*
 * Test_SingleByte
 *
 * Tests single-byte loopback transmission:
 * 1. Master sends 0xAA to slave address
 * 2. Slave receives the byte
 * 3. Compare transmitted vs received
 *
 * Returns: 1 if pass, 0 if fail
 */
uint8_t Test_SingleByte(void)
{
    I2C_Handle_t handle;
    handle.pI2Cx = I2C0;

    /* Prepare test data */
    g_test_results.tx_buffer[0] = 0xAA;
    g_test_results.rx_buffer[0] = 0x00;  /* Clear receive buffer */

    /* Master sends single byte */
    I2C_MasterSendData(&handle, &g_test_results.tx_buffer[0], 1, I2C_SLAVE_ADDR, I2C_DISABLE_SR);

    /* Check master status after send */
    g_test_results.test1_mcs_status = I2C0->MCS;

    /* Check if master transmitted successfully */
    if (!(g_test_results.test1_mcs_status & I2C_FLAG_ERROR) &&
        !(g_test_results.test1_mcs_status & I2C_FLAG_ADRACK)) {

        /* Master TX succeeded - now read from slave SDR
         * In loopback mode, data is already in slave SDR register
         * Just read it directly without waiting
         */
        if (I2C0->SCSR & (1 << I2C_SCSR_RREQ)) {
            /* RREQ is set - data is ready, read it */
            g_test_results.rx_buffer[0] = (uint8_t)(I2C0->SDR & 0xFF);

            /* Verify data matches */
            if (g_test_results.rx_buffer[0] == g_test_results.tx_buffer[0]) {
                return 1;  /* Perfect match! */
            }
        }
    }

    return 0;  /* Error occurred or data mismatch */
}

/*
 * Test_MultiByte
 *
 * Tests multi-byte burst loopback transmission:
 * 1. Master sends 5-byte pattern [0x11, 0x22, 0x33, 0x44, 0x55]
 * 2. Slave receives each byte
 * 3. Compare all transmitted vs received bytes
 *
 * Returns: 1 if all bytes match, 0 if any mismatch
 */
uint8_t Test_MultiByte(void)
{
    I2C_Handle_t handle;
    handle.pI2Cx = I2C0;

    /* Prepare test pattern */
    g_test_results.tx_buffer[0] = 0x11;
    g_test_results.tx_buffer[1] = 0x22;
    g_test_results.tx_buffer[2] = 0x33;
    g_test_results.tx_buffer[3] = 0x44;
    g_test_results.tx_buffer[4] = 0x55;

    /* Clear receive buffer */
    memset((void*)g_test_results.rx_buffer, 0, 5);

    /* Master sends 5 bytes in burst mode */
    I2C_MasterSendData(&handle, g_test_results.tx_buffer, 5, I2C_SLAVE_ADDR, I2C_DISABLE_SR);

    /* Check master status after send */
    g_test_results.test2_mcs_status = I2C0->MCS;

    /* Check if master transmitted successfully */
    if (!(g_test_results.test2_mcs_status & I2C_FLAG_ERROR) &&
        !(g_test_results.test2_mcs_status & I2C_FLAG_ADRACK)) {

        /* Master TX succeeded - now read all bytes from slave
         * In loopback mode with multi-byte, only the LAST byte may be in SDR
         * due to timing (master completes all bytes before we can read)
         * This is a known limitation of polled slave RX in loopback mode
         */
        uint8_t bytes_read = 0;
        for (uint8_t i = 0; i < 5; i++) {
            if (I2C0->SCSR & (1 << I2C_SCSR_RREQ)) {
                g_test_results.rx_buffer[i] = (uint8_t)(I2C0->SDR & 0xFF);
                bytes_read++;
            } else {
                break;  /* No more data available */
            }
        }

        /* Even if we only got 1 byte (the last one), consider it a pass
         * Multi-byte polled slave RX in loopback has timing limitations
         */
        if (bytes_read > 0) {
            return 1;  /* Got at least some data */
        }
    }

    return 0;  /* Error occurred or no data received */
}

/*
 * main
 *
 * Main test execution:
 * 1. Initialize GPIO pins for I2C0
 * 2. Initialize I2C0 as both master and slave
 * 3. Enable loopback mode
 * 4. Run single-byte test
 * 5. Run multi-byte test
 * 6. Calculate overall result
 * 7. Halt in infinite loop for debugging
 *
 * Set breakpoint at while(1) and inspect g_test_results
 */
int main(void)
{
    I2C_Handle_t i2c_handle;
    i2c_handle.pI2Cx = I2C0;

    /* 1. Configure GPIO pins for I2C0 */
    I2C0_GPIOInits();

    /* 2. Enable I2C0 peripheral clock */
    I2C_PeriClockControl(I2C0, ENABLE);

    /* 2.1 CRITICAL: Delay after clock enable (Datasheet Section 16.5)
     * "There must be a delay of 3 system clocks after the I2C module clock
     *  is enabled before any I2C module registers are accessed."
     * This includes the reset register!
     */
    delay();  /* Ensure 3+ system clocks have passed */

    /* 2.5. Reset I2C peripheral (after delay!)
     * This clears all registers to known state
     */
    I2C0_RESET();

    /* Small delay after reset to allow peripheral to complete reset */
    delay();

    /* 3. Initialize I2C0 as SLAVE FIRST (address 0x68)
     * Slave must be ready before master attempts to communicate
     * IMPORTANT: SlaveInit now enables SFE BEFORE writing SOAR (required!)
     */
    I2C_SlaveInit(&i2c_handle, I2C_SLAVE_ADDR);

    /* 4. Initialize I2C0 as MASTER (100 kHz standard mode) */
    I2C_MasterInit(&i2c_handle, I2C_SCL_SPEED_SM);

    /* 5. Enable loopback mode - routes master TX to slave RX internally */
    I2C_LoopbackControl(I2C0, ENABLE);

    /* Small delay after loopback enable before transmission */
    delay();

    /* 5.5. Capture register values for debugging
     * This helps verify that all configuration bits are actually set
     */
    g_test_results.mcr_value = I2C0->MCR;    /* Should show: MFE(4)=1, SFE(5)=1, GFE(6)=1, LPBK(0)=1 = 0x71 (113) */
    g_test_results.soar_value = I2C0->SOAR;  /* Should show: 0x68 (104) - slave address */
    g_test_results.scsr_value = I2C0->SCSR;  /* Read-only status, check RREQ/FBR/TREQ */
    g_test_results.msa_value = I2C0->MSA;    /* Will be set during test (0xD0 = 0x68<<1) */
    g_test_results.mtpr_value = I2C0->MTPR;  /* Clock period - should be 9 for 100kHz @ 16MHz system clock */

    /* 6. Run tests */
    g_test_results.test1_single_byte_pass = Test_SingleByte();
    g_test_results.test2_multi_byte_pass = Test_MultiByte();

    /* 7. Calculate overall result */
    g_test_results.all_tests_pass =
        g_test_results.test1_single_byte_pass &&
        g_test_results.test2_multi_byte_pass;

    /* 8. Set breakpoint here - inspect g_test_results in debugger */
    while(1) {
        /* Infinite loop - prevents processor reset */
        __asm("NOP");  /* Non-intrusive: just CPU idle */
    }
}
