/*
 * 011i2c_master_rx_test.c
 *
 *  Created on: Jan 2025
 *      Author: Tabrez
 */

/*******************************************************************************
 * I2C MASTER RECEIVE TEST - TM4C reads from external I2C device
 *******************************************************************************
 *
 * ⚠️ CRITICAL: TM4C I2C GPIO CONFIGURATION REQUIREMENTS ⚠️
 * ========================================================
 * TM4C has ASYMMETRIC GPIO requirements (different from typical I2C):
 *
 *   ✅ SCL (Clock): PUSH-PULL output (NOT open-drain!)
 *      - Reason: TM4C has active internal pull-up on SCL
 *      - Datasheet line 58487-58488: "should NOT be configured as open drain"
 *
 *   ✅ SDA (Data): OPEN-DRAIN output (standard I2C)
 *      - Reason: Allows multi-master and clock stretching
 *      - Datasheet line 44060: "should be set to open drain"
 *
 * ⚠️ IMPORTANT: Configuring SCL as open-drain will cause I2C to fail!
 *    - Loopback test may work (bypasses GPIO)
 *    - External device communication will fail (reads return 0x00)
 *
 * See: /findings/TM4C_I2C_GPIO_Configuration_Issue.md for full details
 *
 *******************************************************************************
 * PURPOSE:
 * This test verifies I2C Master Receive functionality by reading data from
 * an external I2C device (AT24C32 EEPROM or DS1307 RTC on HW-111 module).
 *
 * HARDWARE SETUP - HW-111 Module:
 * ================================
 *
 * HW-111 Module Description:
 * --------------------------
 * The HW-111 is a DS1307-based real-time clock module with I2C interface.
 * It contains two I2C devices on the same bus:
 *   1. DS1307 RTC chip     - I2C address 0x68 (Real-time clock with 56 bytes SRAM)
 *   2. AT24C32 EEPROM chip - I2C address 0x50 (4KB non-volatile memory)
 *
 * HW-111 Module Pinout (P1 and P2 headers):
 * ------------------------------------------
 * ⚠️ IMPORTANT: Both P1 and P2 have the SAME I2C signals - use either one!
 *
 *    P1 (Left Header)           P2 (Right Header)
 *    ================           =================
 *    Pin 1: SCL  ←──────────────→  SCL   (I2C Clock)
 *    Pin 2: SDA  ←──────────────→  SDA   (I2C Data)
 *    Pin 3: VCC  ←──────────────→  VCC   (Power 3.3V-5V)
 *    Pin 4: GND  ←──────────────→  GND   (Ground)
 *    Pin 5: DS   ←──────────────→  DS    (DS18B20 temp sensor - optional)
 *
 *    Additional pins (not duplicated):
 *    - SQW: Square wave output from DS1307 (1Hz, 4kHz, 8kHz, or 32kHz)
 *    - BAT: Battery backup connection for CR2032 coin cell
 *
 * Pull-up Resistors:
 * ------------------
 * HW-111 module has built-in 4.7kΩ pull-up resistors on SCL and SDA.
 * No external pull-ups needed!
 *
 * Connection Diagram:
 * -------------------
 * TM4C123 LaunchPad          HW-111 Module (P1 or P2)
 * =================          =======================
 *   PA6 (J1-23) ───────────→ SCL (Pin 1) [with onboard 4.7kΩ pull-up]
 *   PA7 (J2-24) ───────────→ SDA (Pin 2) [with onboard 4.7kΩ pull-up]
 *   3.3V ──────────────────→ VCC (Pin 3)
 *   GND ───────────────────→ GND (Pin 4)
 *
 * ⚠️ Use 3.3V from TM4C, NOT 5V! (DS1307 works with 3.3V-5V, but TM4C is 3.3V)
 *
 * I2C Bus Devices:
 * ----------------
 *   Device         | I2C Address | Memory Size | Purpose
 *   ---------------|-------------|-------------|---------------------------
 *   AT24C32 EEPROM | 0x50-0x57   | 4KB         | Non-volatile data storage
 *   DS1307 RTC     | 0x68        | 56 bytes    | Real-time clock + SRAM
 *
 * This test primarily uses the AT24C32 EEPROM for verification.
 *
 * I2C Configuration:
 *   - Module: I2C1 (PA6=SCL, PA7=SDA)
 *   - Speed: 100 kHz (Standard Mode)
 *   - Addressing: 7-bit
 *   - Pull-ups: Onboard 4.7kΩ (no external resistors needed)
 *
 *******************************************************************************
 * TEST SEQUENCE:
 *******************************************************************************
 * 1. Initialize I2C1 GPIO pins with CORRECT configuration:
 *    - PA6 (SCL): PUSH-PULL with pull-up
 *    - PA7 (SDA): OPEN-DRAIN with pull-up
 * 2. Initialize I2C1 as Master (100 kHz)
 * 3. Scan I2C bus for devices (0x50-0x57 for EEPROM, 0x68 for RTC)
 * 4. Write test pattern to EEPROM
 * 5. Read back and verify data matches
 * 6. Store results in structure for debugging
 *
 *******************************************************************************
 * EXPECTED RESULTS (Inspect g_test in debugger):
 *******************************************************************************
 * ✅ g_test.device_found = 1         (EEPROM or RTC detected)
 * ✅ g_test.device_addr = 0x50-0x57 or 0x68
 * ✅ g_test.write_success = 1        (Write succeeded)
 * ✅ g_test.read_success = 1         (Read succeeded)
 * ✅ g_test.data_matches = 1         (Written data == Read data)
 * ✅ g_test.i2c_verified = 1         (Overall success)
 *
 * If test fails:
 * - Check GPIO configuration (SCL = push-pull, SDA = open-drain)
 * - Verify external pull-ups (4.7kΩ on SCL/SDA to 3.3V)
 * - Check connections (PA6→SCL, PA7→SDA, GND→GND, 3.3V→VCC)
 * - Verify I2C device is powered and functional
 *******************************************************************************
 */

#include "tm4c123x.h"
#include "tm4c123x_i2c_driver.h"
#include "tm4c123x_gpio_driver.h"

/* Test result structure */
typedef struct {
    /* Device scan */
    uint8_t device_found;
    uint8_t device_addr;      /* 0x50-0x57 (EEPROM) or 0x68 (RTC) */

    /* Test data */
    uint8_t write_pattern[8];
    uint8_t read_back[8];

    /* Results */
    uint8_t write_success;
    uint8_t read_success;
    uint8_t data_matches;

    /* Status registers */
    uint32_t mcs_write;
    uint32_t mcs_read;

    /* Overall verification flag */
    uint8_t i2c_verified;     /* 1 = I2C driver works correctly! */
} I2C_Test_t;

volatile I2C_Test_t g_test = {0};

/*******************************************************************************
 * delay - Simple delay function
 ******************************************************************************/
void delay(void) {
    for (volatile uint32_t i = 0; i < 100000; i++);
}

/*******************************************************************************
 * eeprom_write_delay - Wait for EEPROM write cycle (10ms minimum)
 ******************************************************************************/
void eeprom_write_delay(void) {
    for (int i = 0; i < 20; i++) {
        delay();
    }
}

/*******************************************************************************
 * I2C1_GPIOInits - Configure PA6 and PA7 for I2C1 with CORRECT configuration
 *
 * ⚠️ CRITICAL CONFIGURATION:
 * =========================
 * PA6 (I2C1SCL): PUSH-PULL output (NOT open-drain!)
 * PA7 (I2C1SDA): OPEN-DRAIN output
 *
 * This is DIFFERENT from typical I2C implementations!
 * See findings/TM4C_I2C_GPIO_Configuration_Issue.md for details.
 ******************************************************************************/
void I2C1_GPIOInits(void)
{
    GPIO_Handle_t i2c_scl, i2c_sda;

    /* Enable GPIOA clock */
    GPIO_PeriClockControl(GPIOA, ENABLE);

    /**************************************************************************
     * PA6 (I2C1SCL) - PUSH-PULL Configuration
     **************************************************************************
     * Datasheet (line 58487-58488):
     * "I²C module 1 clock has active pull-up. The corresponding port pin
     *  should NOT be configured as open drain."
     *
     * ⚠️ Configuring as open-drain will cause external I2C communication to fail!
     *************************************************************************/
    i2c_scl.pGPIOx = GPIOA;
    i2c_scl.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
    i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = 3;                    /* AF3 = I2C1 */
    i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;           /* ✅ PUSH-PULL (NOT OD!) */
    i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;         /* Internal pull-up */
    i2c_scl.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    i2c_scl.GPIO_PinConfig.GPIO_PinSlewRate = GPIO_SLEW_OFF;
    i2c_scl.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_IN;
    GPIO_Init(&i2c_scl);

    /**************************************************************************
     * PA7 (I2C1SDA) - OPEN-DRAIN Configuration
     **************************************************************************
     * Datasheet (line 44060):
     * "I2CSDA pin should be set to open drain"
     *
     * This is standard I2C configuration for data line.
     *************************************************************************/
    i2c_sda.pGPIOx = GPIOA;
    i2c_sda.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = 3;                    /* AF3 = I2C1 */
    i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_OD;           /* ✅ OPEN-DRAIN */
    i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;         /* Internal pull-up */
    i2c_sda.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    i2c_sda.GPIO_PinConfig.GPIO_PinSlewRate = GPIO_SLEW_OFF;
    i2c_sda.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_IN;
    GPIO_Init(&i2c_sda);
}

/*******************************************************************************
 * Scan_I2C_Bus - Scan for I2C devices (EEPROM at 0x50-0x57, RTC at 0x68)
 *
 * Returns: Device address if found, 0 if not found
 ******************************************************************************/
uint8_t Scan_I2C_Bus(void)
{
    /* First scan for EEPROM (0x50-0x57) */
    for (uint8_t addr = 0x50; addr <= 0x57; addr++) {
        I2C1->MSA = (addr << 1) | 0;  /* Write mode */
        I2C1->MCS = 0x07;             /* START + RUN + STOP */

        volatile uint32_t timeout = 100000;
        while ((I2C1->MCS & 0x01) && timeout--);

        /* Check if device acknowledged */
        if (!(I2C1->MCS & (1 << 1)) && !(I2C1->MCS & (1 << 4))) {
            return addr;  /* EEPROM found! */
        }
        delay();
    }

    /* If no EEPROM, try RTC at 0x68 */
    I2C1->MSA = (0x68 << 1) | 0;
    I2C1->MCS = 0x07;

    volatile uint32_t timeout = 100000;
    while ((I2C1->MCS & 0x01) && timeout--);

    if (!(I2C1->MCS & (1 << 1)) && !(I2C1->MCS & (1 << 4))) {
        return 0x68;  /* RTC found! */
    }

    return 0;  /* No device found */
}

/*******************************************************************************
 * main - I2C Master Receive Test
 *
 * Tests I2C communication by:
 * 1. Scanning for external I2C device
 * 2. Writing test pattern to device
 * 3. Reading back data
 * 4. Verifying data matches
 *
 * Set breakpoint at while(1) and inspect g_test in debugger
 ******************************************************************************/
int main(void)
{
    I2C_Handle_t i2c;
    i2c.pI2Cx = I2C1;

    /**************************************************************************
     * STEP 1: Initialize GPIO with CORRECT configuration
     *************************************************************************/
    I2C1_GPIOInits();
    delay();

    /**************************************************************************
     * STEP 2: Initialize I2C1 peripheral
     *************************************************************************/
    I2C_PeriClockControl(I2C1, ENABLE);
    delay();

    I2C1_RESET();
    delay();

    I2C_MasterInit(&i2c, I2C_SCL_SPEED_SM);
    delay();

    /* Clear any stuck BUSY state */
    I2C1->MCS = 0x04;
    delay();

    /**************************************************************************
     * STEP 3: Scan for I2C device
     *************************************************************************/
    g_test.device_addr = Scan_I2C_Bus();
    if (!g_test.device_addr) {
        /* No device found - halt here */
        while(1) { __asm(" NOP"); }
    }
    g_test.device_found = 1;

    /**************************************************************************
     * STEP 4: Prepare test pattern
     *************************************************************************/
    g_test.write_pattern[0] = 0xAA;
    g_test.write_pattern[1] = 0x55;
    g_test.write_pattern[2] = 0x11;
    g_test.write_pattern[3] = 0x22;
    g_test.write_pattern[4] = 0x33;
    g_test.write_pattern[5] = 0x44;
    g_test.write_pattern[6] = 0x55;
    g_test.write_pattern[7] = 0x66;

    /**************************************************************************
     * STEP 5: Write test pattern to device
     *
     * For EEPROM (0x50-0x57): Write to address 0x0020
     * For RTC (0x68): Write to registers 0x08-0x0F (SRAM)
     *************************************************************************/
    uint8_t write_buffer[10];
    write_buffer[0] = 0x00;  /* Address high byte */
    write_buffer[1] = (g_test.device_addr >= 0x50 && g_test.device_addr <= 0x57) ? 0x20 : 0x08;
    write_buffer[2] = g_test.write_pattern[0];
    write_buffer[3] = g_test.write_pattern[1];
    write_buffer[4] = g_test.write_pattern[2];
    write_buffer[5] = g_test.write_pattern[3];
    write_buffer[6] = g_test.write_pattern[4];
    write_buffer[7] = g_test.write_pattern[5];
    write_buffer[8] = g_test.write_pattern[6];
    write_buffer[9] = g_test.write_pattern[7];

    I2C_MasterSendData(&i2c, write_buffer, 10, g_test.device_addr, I2C_DISABLE_SR);
    delay();

    g_test.mcs_write = I2C1->MCS;

    /* Check write success */
    if (!(g_test.mcs_write & (1 << 4)) && !(g_test.mcs_write & (1 << 1))) {
        g_test.write_success = 1;
    }

    /* Wait for write cycle to complete */
    eeprom_write_delay();

    /**************************************************************************
     * STEP 6: Read back data from device
     *************************************************************************/
    /* Set memory address pointer */
    uint8_t addr_bytes[2];
    addr_bytes[0] = 0x00;
    addr_bytes[1] = (g_test.device_addr >= 0x50 && g_test.device_addr <= 0x57) ? 0x20 : 0x08;

    I2C_MasterSendData(&i2c, addr_bytes, 2, g_test.device_addr, I2C_ENABLE_SR);
    delay();

    /* Read 8 bytes */
    I2C_MasterReceiveData(&i2c, (uint8_t*)g_test.read_back, 8, g_test.device_addr, I2C_DISABLE_SR);
    delay();

    g_test.mcs_read = I2C1->MCS;

    /* Check read success */
    if (!(g_test.mcs_read & (1 << 4)) && !(g_test.mcs_read & (1 << 1))) {
        g_test.read_success = 1;
    }

    /**************************************************************************
     * STEP 7: Verify data matches
     *************************************************************************/
    g_test.data_matches = 1;

    for (uint8_t i = 0; i < 8; i++) {
        if (g_test.read_back[i] != g_test.write_pattern[i]) {
            g_test.data_matches = 0;
            break;
        }
    }

    /**************************************************************************
     * STEP 8: Calculate overall result
     *************************************************************************/
    if (g_test.write_success && g_test.read_success && g_test.data_matches) {
        g_test.i2c_verified = 1;  /* ✅ I2C DRIVER VERIFIED! */
    }

    /**************************************************************************
     * Set breakpoint here and inspect g_test
     *
     * Expected results:
     * ✅ g_test.i2c_verified = 1
     * ✅ g_test.data_matches = 1
     * ✅ g_test.read_back == g_test.write_pattern
     *
     * If i2c_verified = 0, check:
     * - GPIO configuration (SCL = push-pull, SDA = open-drain)
     * - External pull-ups (4.7kΩ to 3.3V)
     * - Hardware connections
     * - Device power
     *************************************************************************/
    while(1) {
        __asm(" NOP");
    }
}

/*******************************************************************************
 * TROUBLESHOOTING:
 *******************************************************************************
 *
 * Problem: g_test.device_found = 0
 * Solution:
 *   - Check connections (PA6→SCL, PA7→SDA)
 *   - Verify device is powered (3.3V to VCC, GND to GND)
 *   - Check external pull-ups (4.7kΩ on SCL/SDA to 3.3V)
 *
 * Problem: g_test.write_success = 0 or g_test.read_success = 0
 * Solution:
 *   - Check mcs_write and mcs_read for error flags
 *   - Bit 4 = arbitration lost
 *   - Bit 1 = address not acknowledged
 *
 * Problem: g_test.data_matches = 0 (read_back != write_pattern)
 * Solution:
 *   - Check if SCL is configured as PUSH-PULL (not open-drain!)
 *   - This is the most common error - see findings document
 *   - Verify external pull-ups are present
 *
 * Problem: g_test.read_back = [0x00, 0x00, ...]
 * Solution:
 *   - SCL is likely configured as open-drain (WRONG!)
 *   - Change PA6 GPIO_PinOPType to GPIO_OPTYPE_PP
 *   - See: findings/TM4C_I2C_GPIO_Configuration_Issue.md
 *
 *******************************************************************************
 * REFERENCE:
 * For complete explanation of this GPIO configuration requirement, see:
 * /findings/TM4C_I2C_GPIO_Configuration_Issue.md
 *******************************************************************************
 */
