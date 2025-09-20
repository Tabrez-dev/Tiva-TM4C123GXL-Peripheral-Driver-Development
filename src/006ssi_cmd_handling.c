/*
 * 006ssi_tx_led_master.c
 *
 *  Created on: 04-Aug-2025
 *      Author: tabrez
 */

#include "tm4c123x.h"
#include <string.h>

#define BTN_PRESSED 0U
#define COMMAND_LED_CTRL 0x00
#define COMMAND_STATUS_READ 0x01 // Read status of specific pin

#define CMD_LED1 0x01 // Command to toggle LED 1
#define CMD_LED2 0x02 // Command to toggle LED 2
#define CMD_LED3 0x03 // Command to toggle LED 3
#define CMD_LED4 0x04 // Command to toggle LED 4

// STM32F407VGT6 LED PIN NUMBERS
#define LED3_PIN 13 // PD13
#define LED4_PIN 12 // PD12
#define LED5_PIN 14 // PD14
#define LED6_PIN 15 // PD15

#define LED_ON 1
#define LED_OFF 0
int count = 0;
uint8_t status;
uint8_t debug_rx[3];

void delay(void)
{
    uint32_t i = 200000; // Adjust for ~10-20 ms
    while (i)
    {
        i--;
    }
}

// at top, add small delay helper
static inline void short_delay(void)
{
    volatile uint32_t d = 2000;
    while (d--)
        __asm__ volatile(" nop");
}

void SSI_GPIOInit(void)
{
    GPIO_Handle_t ssiPins;
    ssiPins.pGPIOx = GPIOB;

    // Ensure GPIOB clock enabled (do this explicitly)
    GPIO_PeriClockControl(GPIOB, ENABLE);

    // Common configuration for SSI2 pins
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 2; // AF = 2 for SSI2
    ssiPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
    ssiPins.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_4MA;
    ssiPins.GPIO_PinConfig.GPIO_PinSlewRate = GPIO_SLEW_OFF;
    ssiPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;

    // Configure PB4 (SSI2CLK) - AF
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_OUT; // Clock output
    GPIO_Init(&ssiPins);

    // Configure PB6 (SSI2RX) - AF
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_IN; // RX input
    GPIO_Init(&ssiPins);

    // Configure PB7 (SSI2TX) - AF
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_OUT; // TX output
    GPIO_Init(&ssiPins);

    // Configure PB5 as manual CS GPIO output (default HIGH)
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
    // Keep other fields (OPType/Drive/Slew/PuPd) as set earlier
    GPIO_Init(&ssiPins);
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, 1); // CS idle HIGH
}

void SSI_MasterInit(void)
{
    SSI_Handle_t ssi2Handle;
    ssi2Handle.pSSIx = SSI2;

    ssi2Handle.SSIConfig.SSI_DeviceMode = SSI_DEVICE_MODE_MASTER;
    ssi2Handle.SSIConfig.SSI_BusConfig = SSI_BUS_CONFIG_FD;   // Full duplex
    ssi2Handle.SSIConfig.SSI_SclkSpeed = SSI_SCLK_SPEED_DIV8; // SysClk/8 = 2MHz if SysClk=16MHz
    ssi2Handle.SSIConfig.SSI_DSS = SSI_DSS_8BIT;              // 8-bit frame
    ssi2Handle.SSIConfig.SSI_FRF = SSI_FRF_SSI_Freescale;
    ssi2Handle.SSIConfig.SSI_SPO = SSI_SPO_LOW;
    ssi2Handle.SSIConfig.SSI_SPH = SSI_SPH_1ST_EDGE;

    SSI_Init(&ssi2Handle);
}

// CS: Manual control functions (improved)
void CS_Select(void)
{
    // Ensure peripheral not toggling CS in AF mode: PB5 is GPIO now.
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, 0); // Drive CS LOW
    short_delay();                               // let slave detect NSS low
}

void CS_Deselect(void)
{
    //    // Wait for SPI transfer fully complete before releasing CS
    //    while (SSI_GetFlagStatus(SSI2, SSI_FLAG_BSY)); // wait until not busy
    //    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, 1); // Drive CS HIGH
    //    short_delay();

    // Wait for TX FIFO empty then for SPI transfer fully complete before releasing CS
    while (!SSI_GetFlagStatus(SSI2, SSI_FLAG_TFE))
        ; // wait until TX FIFO empty
    while (SSI_GetFlagStatus(SSI2, SSI_FLAG_BSY))
        ;                                        // now wait until not busy
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_5, 1); // Drive CS HIGH
    short_delay();
}

uint8_t SSI_VerifyResponse(uint8_t ackbyte)
{
    if (ackbyte == 0xF5)
    {
        return 1; // ack
    }
    else
    {
        return 0; // nack
    }
}

/*
 * MASTER-SLAVE SSI COMMUNICATION PROTOCOL DEMONSTRATION
 *
 * This program demonstrates robust embedded communication between two microcontrollers:
 * - Master: TM4C123GXL (this device) running at 80MHz
 * - Slave: STM32F407VGTx running at 168MHz
 *
 * COMMUNICATION PROTOCOL:
 * The master sends structured commands to control and monitor the slave's LED.
 * Each command follows a strict handshaking protocol with acknowledgments.
 *
 * COMMAND SEQUENCE:
 * 1. Master sends command code (0x00 for LED control, 0x01 for status read)
 * 2. Slave responds with ACK (0xF5) to confirm command received
 * 3. Master sends additional data (pin number, LED state, etc.)
 * 4. Slave processes command and sends response data
 *
 * WHY THIS PROGRAMMING APPROACH?
 *
 * PROBLEM: Two microcontrollers need to communicate reliably
 * - Different manufacturers (TI vs STMicroelectronics)
 * - Different speeds (80MHz vs 168MHz)
 * - Need guaranteed data delivery with error detection
 *
 * SOLUTION: Structured command-response protocol
 * Like a formal conversation where each party confirms they understood:
 * Person A: "Turn on the light" → Person B: "OK, I got it" → Person A: "Which light?" → Person B: "Got it, done"
 *
 * STEP-BY-STEP PROGRAM EXECUTION:
 *
 * SETUP PHASE - Prepare hardware for communication:
 * 1. Configure button SW1 as input (user will press to trigger commands)
 * 2. Setup SSI pins: PB4=clock, PB5=chip-select, PB6=receive, PB7=transmit
 * 3. Turn off SSI clock to save battery power (sleep mode)
 * 4. Initialize variables: dummy_write=0xFF (used to get responses), led_toggle_state=0 (LED starts OFF)
 *
 * MAIN PROGRAM LOOP - Wait for user and execute commands:
 *
 * WAITING PHASE:
 * - Sit in infinite loop waiting for user interaction
 * - When SW1 pressed (button reads 0), continue to command phase
 * - Add 200ms delay to prevent button "bounce" (electrical noise from mechanical contact)
 * - Wake up SSI hardware (turn on clock) to start communication
 *
 * COMMAND 1 - CONTROL LED (Tell slave to turn LED on/off):
 * WHY: We want to control a device on another microcontroller
 * - Set command_code = 0x00 (this means "LED control" to the slave)
 * - Pull CS pin low (tells slave "pay attention, message starting")
 * - Clear any old data from receive buffer (start with clean slate)
 * - Send 0x00 to slave (our command: "I want to control an LED")
 * - Send 0xFF dummy byte (this forces slave to send back its response)
 * - Read slave's acknowledgment (should be 0xF5 meaning "I understand")
 * - Clear buffer again (prepare for next data exchange)
 * - If slave acknowledged: send pin number (13) and desired state (ON/OFF)
 * - Clear any response data from this command
 * - Pull CS pin high (tells slave "message complete")
 *
 * COMMAND 2 - READ STATUS (Ask slave what state the LED is in):
 * WHY: We want to verify our command worked and get feedback
 * - Wait for user to press button again
 * - Set command_code = 0x01 (this means "read status" to the slave)
 * - Pull CS low (start new message)
 * - Clear old data from receive buffer
 * - Send 0x01 to slave (our command: "tell me LED status")
 * - Send 0xFF dummy to get slave's acknowledgment
 * - Read ACK response (should be 0xF5)
 * - Send pin number 13 (which pin status do you want?)
 * - Read slave's acknowledgment into "junk" variable (slave confirms it got pin number)
 * - Send 0xFF dummy to request the actual status
 * - Read status: 0=LED is OFF, 1=LED is ON
 * - Store all responses for debugging
 * - Pull CS high (end message)
 * - Turn off SSI clock (save power until next button press)
 * - Wait for button release before starting over
 *
 * UNDERSTANDING DUMMY/JUNK VARIABLES IN FULL-DUPLEX SPI:
 *
 * WHY DO WE NEED DUMMY BYTES AND JUNK VARIABLES?
 * SPI is "full-duplex" meaning data flows BOTH directions simultaneously.
 * Think of it like two people talking on walkie-talkies at the same time.
 *
 * THE CHALLENGE:
 * When master sends data, slave MUST send something back (even if meaningless).
 * When master wants slave's data, master MUST send something (even if meaningless).
 *
 * REAL EXAMPLE FROM OUR CODE:
 * Master action: "Send pin number 13 to slave"
 * - Master MOSI: sends 0x0D (meaningful data: pin 13)
 * - Slave MISO: sends ??? (random data, slave wasn't ready with useful response)
 * - We read this random data into "junk" variable and ignore it
 *
 * Master action: "Get LED status from slave"
 * - Master MOSI: sends 0xFF (meaningless dummy, just to clock slave's response)
 * - Slave MISO: sends 0x01 (meaningful data: LED is ON)
 * - We read this important data into "status" variable
 *
 * WHY NOT JUST IGNORE THE UNWANTED DATA?
 * - SSI hardware requires you to read EVERY byte that comes back
 * - If you don't read it, the receive buffer fills up and blocks future communication
 * - "dummy_read" and "junk" variables are "data trash cans" for unwanted bytes
 *
 * NAMING CONVENTION:
 * - "dummy_write" = meaningless data we send (usually 0xFF)
 * - "dummy_read" = meaningless data we receive and throw away
 * - "junk" = semi-meaningful data we receive but don't need (like acknowledgments)
 * - "status/ackbyte" = important data we actually want to use
 *
 * WHY THIS IS BETTER THAN SIMPLE APPROACHES:
 * - Error detection: If ACK ≠ 0xF5, we know communication failed
 * - Reliability: Each step is confirmed before proceeding
 * - Debugging: We can see exactly what each device sent/received
 * - Scalability: Easy to add more commands (0x02, 0x03, etc.)
 * - Professional: This is how real industrial systems communicate
 *
 * REAL-WORLD APPLICATIONS:
 * - Industrial sensor networks with master-slave topology
 * - Automotive ECU communication between control modules
 * - IoT device coordination where one controller manages multiple peripherals
 * - Multi-board systems requiring reliable inter-processor communication
 *
 * This demonstrates professional embedded systems engineering practices:
 * error handling, protocol validation, power management, and hardware abstraction.
 */
int main()
{
    // Setup button SW1 for user input
    GPIO_Handle_t GPIOBtn;
    GPIOBtn.pGPIOx = GPIOF;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_DRV_2MA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_PeriClockControl(GPIOF, ENABLE);
    GPIO_Init(&GPIOBtn);

    // Setup SSI hardware for master communication
    SSI_GPIOInit();
    SSI_MasterInit();

    // Save power when not communicating
    SSI_PeriClockControl(SSI2, DISABLE);

    uint8_t dummy_write = 0xFF;          // Used to get slave responses
    uint8_t dummy_read;                  // Discard unwanted RX data
    static uint8_t led_toggle_state = 0; // Track LED on/off state
    while (1)
    {
        // Wait for user to press button
        while (GPIO_ReadFromInputPin(GPIOF, GPIO_PIN_4) != BTN_PRESSED);
        delay(); // Prevent button bounce

        // Power up SSI for communication
        SSI_PeriClockControl(SSI2, ENABLE);

        // === COMMAND 1: Control LED (turn on/off) ===
        uint8_t command_code = COMMAND_LED_CTRL;
        uint8_t ackbyte;
        uint8_t args[2];

        CS_Select(); // Start SSI transaction
        // Clear any old data from RX buffer
        while (SSI_GetFlagStatus(SSI2, SSI_FLAG_RNE))
        {
            SSI_ReceiveData(SSI2, &dummy_read, 1);
        }
        SSI_SendData(SSI2, &command_code, 1);    // Send command: "LED control"
        SSI_SendData(SSI2, &dummy_write, 1);     // Trigger slave ACK response
        SSI_ReceiveData(SSI2, &ackbyte, 1);      // Read ACK (should be 0xF5)
        // Clear any leftover data before sending arguments
        while (SSI_GetFlagStatus(SSI2, SSI_FLAG_RNE))
        {
            SSI_ReceiveData(SSI2, &dummy_read, 1);
        }
        if (SSI_VerifyResponse(ackbyte))         // If slave acknowledged
        {
            args[0] = LED3_PIN;                  // Which LED (pin 13)
            led_toggle_state = !led_toggle_state; // Flip LED state
            args[1] = led_toggle_state;          // New LED state (on/off)

            SSI_SendData(SSI2, args, 2);         // Send pin + state
            count++;
            // Clear any response data from LED command
            uint8_t dummy_byte;
            while (SSI_GetFlagStatus(SSI2, SSI_FLAG_RNE))
            {
                SSI_ReceiveData(SSI2, &dummy_byte, 1);
            }
        }
        CS_Deselect(); // End LED control transaction

        // === COMMAND 2: Read LED status ===
        while (GPIO_ReadFromInputPin(GPIOF, GPIO_PIN_4) != BTN_PRESSED); // Wait for second button press
        delay(); // Prevent button bounce
        command_code = COMMAND_STATUS_READ;
        uint8_t pin = LED3_PIN;
        uint8_t junk;

        CS_Select(); // Start new SSI transaction
        // Clear any old data from RX buffer
        while (SSI_GetFlagStatus(SSI2, SSI_FLAG_RNE))
        {
            SSI_ReceiveData(SSI2, &dummy_read, 1);
        }
        SSI_SendData(SSI2, &command_code, 1);    // Send command: "read status"
        SSI_SendData(SSI2, &dummy_write, 1);     // Trigger slave ACK response
        SSI_ReceiveData(SSI2, &ackbyte, 1);      // Read ACK (should be 0xF5)
        while (SSI_GetFlagStatus(SSI2, SSI_FLAG_RNE))
        {
            SSI_ReceiveData(SSI2, &dummy_read, 1);
        }
        if (SSI_VerifyResponse(ackbyte))         // If slave acknowledged
        {
            SSI_SendData(SSI2, &pin, 1);         // Send which pin to read
            SSI_ReceiveData(SSI2, &junk, 1);     // Slave's "got it" response
            SSI_SendData(SSI2, &dummy_write, 1); // Request actual status
            SSI_ReceiveData(SSI2, &status, 1);   // Read LED state (0 or 1)
        }
        // Save communication results for debugging
        debug_rx[0] = ackbyte;
        debug_rx[1] = junk;
        debug_rx[2] = status;
        CS_Deselect(); // End status read transaction

        // Power down SSI to save energy
        SSI_PeriClockControl(SSI2, DISABLE);
        while (GPIO_ReadFromInputPin(GPIOF, GPIO_PIN_4) == BTN_PRESSED); // Wait for button release
    }
    return 0;
}
