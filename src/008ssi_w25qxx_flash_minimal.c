/*
 * 008ssi_w25qxx_flash_minimal.c - MINIMAL VERSION TO DEBUG FAULTISR
 *
 * This is a stripped-down version to isolate the FaultISR issue.
 * Once this works, we'll add back the full Resource Manager.
 */

#include "tm4c123x_ssi_driver.h"
#include "tm4c123x_gpio_driver.h"
#include <stdint.h>
#include <stdbool.h>

/* W25QXX Commands */
#define W25QXX_CMD_READ_JEDEC_ID     0x9F

/* Minimal global variables */
static SSI_Handle_t SSI2handle;

/* Function prototypes */
static void W25QXX_SSI_Init(void);
static void W25QXX_GPIO_Init(void);
static void W25QXX_LED_Init(void);
static void W25QXX_DelayMs(uint32_t ms);
static uint8_t W25QXX_SendReceiveByte(uint8_t data);
static void W25QXX_CS_Enable(void);
static void W25QXX_CS_Disable(void);

/***************************************************************************
 * @fn                          - W25QXX_DelayMs
 *
 * @brief                       - Simple millisecond delay
 */
static void W25QXX_DelayMs(uint32_t ms)
{
    volatile uint32_t count = ms * 4000;  // Approximate for 16MHz
    while(count--);
}

/***************************************************************************
 * @fn                          - W25QXX_LED_Init
 *
 * @brief                       - Initialize onboard LEDs for test visualization
 */
static void W25QXX_LED_Init(void)
{
    GPIO_Handle_t ledPins;

    // Enable GPIOF clock for onboard LEDs
    GPIO_PeriClockControl(GPIOF, ENABLE);

    ledPins.pGPIOx = GPIOF;
    ledPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    ledPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
    ledPins.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_SPEED_LOW;
    ledPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;

    // Configure PF1 (RED LED)
    ledPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
    GPIO_Init(&ledPins);

    // Turn on RED LED to show we reached this point
    GPIO_WriteToOutputPin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
}

/***************************************************************************
 * @fn                          - W25QXX_GPIO_Init
 *
 * @brief                       - Initialize GPIO pins for SSI2
 */
static void W25QXX_GPIO_Init(void)
{
    GPIO_Handle_t ssiPins;

    // Enable GPIOB clock
    GPIO_PeriClockControl(GPIOB, ENABLE);

    ssiPins.pGPIOx = GPIOB;
    ssiPins.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode  = 2;               // AF = 2 for SSI2
    ssiPins.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OPTYPE_PP;
    ssiPins.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_SPEED_MED;
    ssiPins.GPIO_PinConfig.GPIO_PinSlewRate    = GPIO_SLEW_OFF;
    ssiPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;

    // Configure PB4 (SSI2CLK)
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    GPIO_Init(&ssiPins);

    // Configure PB6 (SSI2RX) - MISO
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
    GPIO_Init(&ssiPins);

    // Configure PB7 (SSI2TX) - MOSI
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    GPIO_Init(&ssiPins);

    // Configure PB5 (CS) - Manual control as GPIO output
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    GPIO_Init(&ssiPins);

    // Set CS high (inactive) initially
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
}

/***************************************************************************
 * @fn                          - W25QXX_SSI_Init
 *
 * @brief                       - Initialize SSI2 peripheral with slower speed
 */
static void W25QXX_SSI_Init(void)
{
    // Enable SSI2 peripheral clock
    SSI_PeriClockControl(SSI2, ENABLE);

    // Configure SSI2 with SLOWER speed for debugging
    SSI2handle.pSSIx = SSI2;
    SSI2handle.SSIConfig.SSI_DeviceMode   = SSI_DEVICE_MODE_MASTER;
    SSI2handle.SSIConfig.SSI_BusConfig    = SSI_BUS_CONFIG_FD;
    SSI2handle.SSIConfig.SSI_SclkSpeed    = SSI_SCLK_SPEED_DIV32;  // Much slower for debugging
    SSI2handle.SSIConfig.SSI_DSS          = SSI_DSS_8BIT;
    SSI2handle.SSIConfig.SSI_FRF          = SSI_FRF_SSI_Freescale;
    SSI2handle.SSIConfig.SSI_SPO          = SSI_SPO_LOW;           // Mode 0
    SSI2handle.SSIConfig.SSI_SPH          = SSI_SPH_1ST_EDGE;      // Mode 0

    // Initialize SSI2
    SSI_Init(&SSI2handle);
}

/***************************************************************************
 * @fn                          - W25QXX_CS_Enable/Disable
 *
 * @brief                       - Control chip select signal
 */
static void W25QXX_CS_Enable(void)
{
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  // Active low
}

static void W25QXX_CS_Disable(void)
{
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);    // Inactive high
}

/***************************************************************************
 * @fn                          - W25QXX_SendReceiveByte
 *
 * @brief                       - Send and receive a single byte via SPI
 */
static uint8_t W25QXX_SendReceiveByte(uint8_t data)
{
    uint8_t received_data;

    SSI_SendData(SSI2, &data, 1);
    SSI_ReceiveData(SSI2, &received_data, 1);

    return received_data;
}

/***************************************************************************
 * @fn                          - W25QXX_TestMISO
 *
 * @brief                       - Test MISO line functionality
 */
void W25QXX_TestMISO(void)
{
    // Test 1: Send dummy bytes and check for any response
    W25QXX_CS_Enable();
    W25QXX_DelayMs(1);  // Give flash time to respond

    uint8_t response1 = W25QXX_SendReceiveByte(0xFF);  // Dummy byte
    uint8_t response2 = W25QXX_SendReceiveByte(0xFF);  // Dummy byte
    uint8_t response3 = W25QXX_SendReceiveByte(0xFF);  // Dummy byte

    W25QXX_CS_Disable();
    W25QXX_DelayMs(10);

    // Test 2: Try JEDEC ID with longer CS setup time
    W25QXX_CS_Enable();
    W25QXX_DelayMs(1);  // Longer setup time

    uint8_t cmd_response = W25QXX_SendReceiveByte(W25QXX_CMD_READ_JEDEC_ID);
    W25QXX_DelayMs(1);  // Wait between command and data

    uint8_t id1 = W25QXX_SendReceiveByte(0xFF);
    uint8_t id2 = W25QXX_SendReceiveByte(0xFF);
    uint8_t id3 = W25QXX_SendReceiveByte(0xFF);

    W25QXX_CS_Disable();
    W25QXX_DelayMs(10);

    // Test 3: Try different command (Read Status Register)
    W25QXX_CS_Enable();
    W25QXX_DelayMs(1);

    uint8_t status_cmd = W25QXX_SendReceiveByte(0x05);  // Read Status Register
    uint8_t status = W25QXX_SendReceiveByte(0xFF);

    W25QXX_CS_Disable();

    // Suppress unused variable warnings
    (void)response1; (void)response2; (void)response3;
    (void)cmd_response; (void)id1; (void)id2; (void)id3;
    (void)status_cmd; (void)status;
}

/***************************************************************************
 * @fn                          - W25QXX_MinimalInit
 *
 * @brief                       - Minimal initialization to test basic functionality
 */
bool W25QXX_MinimalInit(void)
{
    // Step 1: LED init (should work)
    W25QXX_LED_Init();
    W25QXX_DelayMs(500);  // RED LED should be on for 500ms

    // Step 2: GPIO init
    W25QXX_GPIO_Init();
    W25QXX_DelayMs(100);

    // Step 3: SSI init
    W25QXX_SSI_Init();
    W25QXX_DelayMs(100);

    // Step 4: Flash might need power-up time
    W25QXX_DelayMs(100);  // W25Q64FV power-up time

    // Step 5: Test MISO line extensively
    W25QXX_TestMISO();

    // Turn off RED LED to indicate success
    GPIO_WriteToOutputPin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);

    return true;
}

/***************************************************************************
 * @fn                          - main
 *
 * @brief                       - Minimal test to isolate FaultISR issue
 */
int main(void)
{
    // This should work without any faults
    if (!W25QXX_MinimalInit()) {
        // Failed - RED LED stays on
        while(1);
    }

    // Success - RED LED turns off, system continues
    while(1) {
        // Success loop
        W25QXX_DelayMs(1000);
    }

    return 0;
}