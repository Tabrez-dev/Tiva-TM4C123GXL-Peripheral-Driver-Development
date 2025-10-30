/*
 * 009multi_client_flash_demo.c
 *
 * Multi-Client W25Q64FV Flash Memory Driver Demonstration
 *
 * This demonstration shows two independent clients (Temperature Sensor Logger
 * and Pressure Sensor Logger) simultaneously writing to different regions of
 * the same W25Q64FV flash memory through a transparent multi-client resource
 * manager. Both clients are unaware of each other's existence.
 *
 * Hardware Setup:
 * - SSI2: PB4(CLK), PB6(MISO), PB7(MOSI), PA3(Manual CS/FSS)
 *         PB5(SSI2FSS hardware pin - configured as input with pull-up, unused)
 * - UART1: PB1(TX) @ 115200 baud (TX-only for telemetry)
 * - LEDs: PF1(RED-Client1), PF2(BLUE-Client2), PF3(GREEN-Status)
 * - SysTick: 1ms tick for timestamps
 *
 * W25Q64FV Flash Connections:
 * - W25Q64FV VCC  → 3.3V
 * - W25Q64FV GND  → GND
 * - W25Q64FV CLK  → PB4 (SSI2CLK)
 * - W25Q64FV MISO → PB6 (SSI2RX)
 * - W25Q64FV MOSI → PB7 (SSI2TX)
 * - W25Q64FV CS   → PA3 (Manual GPIO control)
 *
 * FTDI USB-to-TTL Connection (Telemetry Output):
 * - FTDI GND → TM4C123 GND
 * - FTDI RXD → TM4C123 PB1 (UART1_TX)
 * - Set FTDI to 3.3V mode!
 *
 *  Created on: 02-Oct-2025
 *      Author: tabrez
 */

#include "tm4c123x.h"
#include "tm4c123x_gpio_driver.h"
#include "tm4c123x_ssi_driver.h"
#include "w25qxx_flash_client.h"

/***************************************************************************
 * Global Variables
 ***************************************************************************/

// Client handles (each thinks they own the flash)
FlashClient_t tempSensor;      // Client 1: Temperature Logger
FlashClient_t pressureSensor;  // Client 2: Pressure Logger

// Shared SSI flash interface
SSI_Handle_t ssi2Flash;

// Statistics tracking
typedef struct {
    uint32_t writeCount;
    uint32_t successCount;
    uint32_t errorCount;
    uint32_t totalLatencyMs;
    uint32_t lastOpStartTime;
} ClientStats_t;

ClientStats_t client1Stats = {0};
ClientStats_t client2Stats = {0};

// Queue statistics (exported for resource manager to update)
uint32_t queueMaxDepth = 0;
uint32_t totalOperations = 0;
uint32_t flashBusyTimeMs = 0;

// SysTick counter for timestamps
volatile uint32_t sysTick_ms = 0;

/***************************************************************************
 * Function Prototypes
 ***************************************************************************/

// System initialization
void SysTick_Init(void);
void UART1_Init(void);
void LED_Init(void);
void SSI2_Flash_HW_Init(void);

// Client tasks
void TempLogger_Task(void);
void PressureLogger_Task(void);

// Client callbacks
void TempLogger_Callback(uint8_t clientId, uint8_t status);
void PressureLogger_Callback(uint8_t clientId, uint8_t status);

// Utilities
void PrintStats(void);
uint32_t GetTick(void);
void delay_ms(uint32_t ms);

/***************************************************************************
 * SysTick Timer Functions
 ***************************************************************************/

/*
 * Function: SysTick_Init
 * Purpose: Configure 1ms system tick for timestamps
 */
void SysTick_Init(void) {
    // Configure SysTick for 1ms @ 16MHz system clock
    SysTick->LOAD = 16000 - 1;  // 16MHz / 1000Hz = 16000
    SysTick->VAL = 0;
    SysTick->CTRL = 0x07;  // Enable, interrupt, use system clock
}

/*
 * Function: SysTick_Handler
 * Purpose: Increment millisecond counter (ISR)
 */
void SysTick_Handler(void) {
    sysTick_ms++;
}

/*
 * Function: GetTick
 * Purpose: Return current timestamp in milliseconds
 */
uint32_t GetTick(void) {
    return sysTick_ms;
}

/*
 * Function: delay_ms
 * Purpose: Simple millisecond delay
 */
void delay_ms(uint32_t ms) {
    volatile uint32_t start = GetTick();
    while ((GetTick() - start) < ms) {
        __asm(" NOP");  // Prevent optimization
    }
}

/***************************************************************************
 * UART1 Initialization (TX-only for telemetry)
 ***************************************************************************/

/*
 * Function: UART1_Init
 * Purpose: Initialize UART1 for telemetry output (115200 baud, 8N1)
 */
void UART1_Init(void) {
    // Enable UART1 and GPIOB clocks
    SYSCTL_RUNCLK->RCGCUART |= (1 << 1);   // UART1
    SYSCTL_RUNCLK->RCGCGPIO |= (1 << 1);   // GPIOB

    // Wait for peripherals to be ready
    while (!(SYSCTL_PR->PRUART & (1 << 1)));
    while (!(SYSCTL_PR->PRGPIO & (1 << 1)));

    // Add small delay for clock stabilization
    for(volatile int i = 0; i < 1000; i++);

    // Configure PB1 as UART1_TX
    GPIOB->AFSEL |= (1 << 1);       // PB1 alternate function
    GPIOB->PCTL &= ~0x000000F0;     // Clear PB1
    GPIOB->PCTL |= 0x00000010;      // PB1 = U1TX (function 1)
    GPIOB->DEN |= (1 << 1);         // Digital enable PB1
    GPIOB->DIR |= (1 << 1);         // PB1 output (TX)

    // Configure UART1: 115200 baud, 8N1
    UART1->CTL = 0;                 // Disable UART during setup

    // Baud rate: 16MHz / (16 * 115200) = 8.680555
    // IBRD = 8, FBRD = int(0.680555 * 64 + 0.5) = 44
    UART1->IBRD = 8;
    UART1->FBRD = 44;

    // 8-bit, no parity, 1 stop bit, FIFO enabled
    UART1->LCRH = 0x60;             // 8-bit word length

    // Enable TX only (bit 8), enable UART (bit 0)
    UART1->CTL = (1 << 8) | (1 << 0);  // TXE=1, UARTEN=1
}

/*
 * Function: UART1_SendChar
 * Purpose: Send single character via UART1
 */
void UART1_SendChar(char c) {
    while (UART1->FR & (1 << 5));  // Wait until TX FIFO not full
    UART1->DR = c;
}

/*
 * Function: UART1_SendString
 * Purpose: Send string via UART1
 */
void UART1_SendString(const char *str) {
    while (*str) {
        if (*str == '\n') {
            UART1_SendChar('\r');
        }
        UART1_SendChar(*str++);
    }
}

/*
 * Function: UART1_SendNumber
 * Purpose: Send unsigned number via UART1
 */
void UART1_SendNumber(uint32_t num) {
    char buf[12];
    int i = 0;
    if (num == 0) {
        UART1_SendChar('0');
        return;
    }
    while (num > 0) {
        buf[i++] = '0' + (num % 10);
        num /= 10;
    }
    while (i > 0) {
        UART1_SendChar(buf[--i]);
    }
}

/*
 * Function: UART1_SendHex
 * Purpose: Send hex byte (2 digits)
 */
void UART1_SendHex(uint8_t byte) {
    const char hex[] = "0123456789ABCDEF";
    UART1_SendChar(hex[(byte >> 4) & 0xF]);
    UART1_SendChar(hex[byte & 0xF]);
}

/*
 * Helper macros for common log patterns
 */
#define LOG(source, func, msg) \
    do { \
        UART1_SendString("[" source ":" func "]["); \
        UART1_SendNumber(GetTick()); \
        UART1_SendString("ms] " msg "\n"); \
    } while(0)

#define LOG_NUM(source, func, msg, num) \
    do { \
        UART1_SendString("[" source ":" func "]["); \
        UART1_SendNumber(GetTick()); \
        UART1_SendString("ms] " msg); \
        UART1_SendNumber(num); \
        UART1_SendString("\n"); \
    } while(0)

/***************************************************************************
 * LED Initialization
 ***************************************************************************/

/*
 * Function: LED_Init
 * Purpose: Initialize PF1, PF2, PF3 as LED outputs
 */
void LED_Init(void) {
    // Enable GPIOF clock
    SYSCTL_RUNCLK->RCGCGPIO |= (1 << 5);  // GPIOF
    while (!(SYSCTL_PR->PRGPIO & (1 << 5)));

    // Configure PF1, PF2, PF3 as outputs
    GPIOF->DIR |= (1 << 1) | (1 << 2) | (1 << 3);
    GPIOF->DEN |= (1 << 1) | (1 << 2) | (1 << 3);

    // Turn off all LEDs initially (using bit-masked addressing)
    GPIOF->DATA[(1 << 1) | (1 << 2) | (1 << 3)] = 0;
}

/*
 * Function: LED_Toggle
 * Purpose: Toggle specified LED
 */
void LED_Toggle(uint8_t pin) {
    GPIOF->DATA[1 << pin] ^= (1 << pin);
}

/***************************************************************************
 * SSI2 Flash Hardware Initialization
 ***************************************************************************/

/*
 * Function: SSI2_Flash_HW_Init
 * Purpose: Initialize SSI2 hardware and flash interface
 */
void SSI2_Flash_HW_Init(void) {
    // Configure SSI2 pins
    GPIO_Handle_t ssiPins;

    // Enable clocks
    GPIO_PeriClockControl(GPIOB, ENABLE);
    GPIO_PeriClockControl(GPIOA, ENABLE);

    // Configure PB4 (SSI2CLK) - Output
    ssiPins.pGPIOx = GPIOB;
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    ssiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    ssiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_OUT;
    ssiPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
    ssiPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;
    ssiPins.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_SPEED_HIGH;
    GPIO_Init(&ssiPins);

    // Configure PB6 (SSI2RX/MISO) - Input
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_IN;
    GPIO_Init(&ssiPins);

    // Configure PB7 (SSI2TX/MOSI) - Output
    ssiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    ssiPins.GPIO_PinConfig.GPIO_PinAltDir = GPIO_DIR_OUT;
    GPIO_Init(&ssiPins);

    // Configure PA3 as manual CS (active LOW)
    GPIO_Handle_t csPin;
    csPin.pGPIOx = GPIOA;
    csPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
    csPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    csPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
    csPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PUPD_NONE;
    csPin.GPIO_PinConfig.GPIO_PinDriveStrength = GPIO_SPEED_HIGH;
    GPIO_Init(&csPin);

    // Set CS HIGH (inactive) initially
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

    // Configure SSI2 peripheral
    ssi2Flash.pSSIx = SSI2;
    ssi2Flash.SSIConfig.SSI_DeviceMode = SSI_DEVICE_MODE_MASTER;
    ssi2Flash.SSIConfig.SSI_BusConfig = SSI_BUS_CONFIG_FD;
    ssi2Flash.SSIConfig.SSI_SclkSpeed = SSI_SCLK_SPEED_DIV8;  // 2MHz
    ssi2Flash.SSIConfig.SSI_DSS = SSI_DSS_8BIT;
    ssi2Flash.SSIConfig.SSI_FRF = SSI_FRF_SSI_Freescale;
    ssi2Flash.SSIConfig.SSI_SPO = SSI_SPO_LOW;
    ssi2Flash.SSIConfig.SSI_SPH = SSI_SPH_1ST_EDGE;

    // Initialize flash interface
    SSI_FlashInit(&ssi2Flash);

    // Enable SSI2 interrupt
    SSI_IRQInterruptConfig(IRQ_NO_SSI2, ENABLE);
    SSI_IRQPriorityConfig(IRQ_NO_SSI2, 2);

    // Debug: Verify NVIC enable
    UART1_SendString("[SSI2_Flash_HW_Init] NVIC_EN1=0x");
    UART1_SendNumber(NVIC_EN1);
    UART1_SendString(", SSI2_IM=0x");
    UART1_SendNumber(SSI2->IM);
    UART1_SendString("\n");
}

/***************************************************************************
 * Client 1: Temperature Sensor Logger
 ***************************************************************************/

/*
 * Function: TempLogger_Task
 * Purpose: Client 1 main task - writes temperature data every 300ms
 * Flash Region: Pages 0-99
 */
void TempLogger_Task(void) {
    static uint32_t lastWriteTime = 0;
    static uint8_t currentPage = 0;
    uint8_t tempData[256];

    // Check if it's time to write (300ms interval)
    if ((GetTick() - lastWriteTime) < 300) {
        return;
    }

    // Prepare simulated temperature data
    for (int i = 0; i < 256; i++) {
        tempData[i] = 0xAA;  // Simulated sensor reading
    }
    tempData[0] = currentPage;  // Page marker
    tempData[1] = 0x01;         // Client ID marker
    tempData[2] = (uint8_t)(GetTick() & 0xFF);  // Timestamp LSB

    // Print from Client1
    UART1_SendString("[Client1:TempLogger_Task][");
    UART1_SendNumber(GetTick());
    UART1_SendString("ms] Queueing write to page ");
    UART1_SendNumber(currentPage);
    UART1_SendString("\n");

    // Record start time for latency measurement
    client1Stats.lastOpStartTime = GetTick();

    // Queue the write operation
    FlashResult_t result = FlashClient_WritePage(&tempSensor, currentPage, tempData);

    if (result != FLASH_RESULT_SUCCESS) {
        UART1_SendString("[Client1:TempLogger_Task][");
        UART1_SendNumber(GetTick());
        UART1_SendString("ms] Queue failed! result=");
        UART1_SendNumber(result);
        UART1_SendString("\n");
        client1Stats.errorCount++;
    } else {
        client1Stats.writeCount++;
    }

    lastWriteTime = GetTick();
    currentPage = (currentPage + 1) % 100;  // Wrap at page 100
}

/*
 * Function: TempLogger_Callback
 * Purpose: Called when Client 1 operation completes
 */
void TempLogger_Callback(uint8_t clientId, uint8_t status) {
    uint32_t latency = GetTick() - client1Stats.lastOpStartTime;

    if (status & FLASH_STATUS_COMPLETE) {
        UART1_SendString("[Client1:TempLogger_Callback][");
        UART1_SendNumber(GetTick());
        UART1_SendString("ms] Write complete (");
        UART1_SendNumber(latency);
        UART1_SendString("ms elapsed)\n");
        client1Stats.successCount++;
        client1Stats.totalLatencyMs += latency;

        // Toggle RED LED
        LED_Toggle(1);
    } else {
        UART1_SendString("[Client1:TempLogger_Callback][");
        UART1_SendNumber(GetTick());
        UART1_SendString("ms] Write FAILED! status=0x");
        UART1_SendHex(status);
        UART1_SendString("\n");
        client1Stats.errorCount++;

        // Blink GREEN LED for error
        LED_Toggle(3);
    }
}

/***************************************************************************
 * Client 2: Pressure Sensor Logger
 ***************************************************************************/

/*
 * Function: PressureLogger_Task
 * Purpose: Client 2 main task - writes pressure data every 500ms
 * Flash Region: Pages 100-199
 */
void PressureLogger_Task(void) {
    static uint32_t lastWriteTime = 0;
    static uint8_t currentPage = 100;
    uint8_t pressureData[256];

    // Check if it's time to write (500ms interval)
    if ((GetTick() - lastWriteTime) < 500) {
        return;
    }

    // Prepare simulated pressure data
    for (int i = 0; i < 256; i++) {
        pressureData[i] = 0x55;  // Simulated sensor reading
    }
    pressureData[0] = currentPage;  // Page marker
    pressureData[1] = 0x02;         // Client ID marker
    pressureData[2] = (uint8_t)(GetTick() & 0xFF);  // Timestamp LSB

    // Print from Client2
    UART1_SendString("[Client2:PressureLogger_Task][");
    UART1_SendNumber(GetTick());
    UART1_SendString("ms] Queueing write to page ");
    UART1_SendNumber(currentPage);
    UART1_SendString("\n");

    // Record start time for latency measurement
    client2Stats.lastOpStartTime = GetTick();

    // Queue the write operation
    FlashResult_t result = FlashClient_WritePage(&pressureSensor, currentPage, pressureData);

    if (result != FLASH_RESULT_SUCCESS) {
        UART1_SendString("[Client2:PressureLogger_Task][");
        UART1_SendNumber(GetTick());
        UART1_SendString("ms] Queue failed! result=");
        UART1_SendNumber(result);
        UART1_SendString("\n");
        client2Stats.errorCount++;
    } else {
        client2Stats.writeCount++;
    }

    lastWriteTime = GetTick();
    currentPage = 100 + ((currentPage - 100 + 1) % 100);  // Wrap 100-199
}

/*
 * Function: PressureLogger_Callback
 * Purpose: Called when Client 2 operation completes
 */
void PressureLogger_Callback(uint8_t clientId, uint8_t status) {
    uint32_t latency = GetTick() - client2Stats.lastOpStartTime;

    if (status & FLASH_STATUS_COMPLETE) {
        UART1_SendString("[Client2:PressureLogger_Callback][");
        UART1_SendNumber(GetTick());
        UART1_SendString("ms] Write complete (");
        UART1_SendNumber(latency);
        UART1_SendString("ms elapsed)\n");
        client2Stats.successCount++;
        client2Stats.totalLatencyMs += latency;

        // Toggle BLUE LED
        LED_Toggle(2);
    } else {
        UART1_SendString("[Client2:PressureLogger_Callback][");
        UART1_SendNumber(GetTick());
        UART1_SendString("ms] Write FAILED! status=0x");
        UART1_SendHex(status);
        UART1_SendString("\n");
        client2Stats.errorCount++;

        // Blink GREEN LED for error
        LED_Toggle(3);
    }
}

/***************************************************************************
 * Statistics & Monitoring
 ***************************************************************************/

/*
 * Function: PrintStats
 * Purpose: Print comprehensive statistics every 1 second
 */
void PrintStats(void) {
    static uint32_t lastPrintTime = 0;

    if ((GetTick() - lastPrintTime) < 1000) {
        return;
    }

    UART1_SendString("[Main:PrintStats][");
    UART1_SendNumber(GetTick());
    UART1_SendString("ms] === STATISTICS ===\n");

    // Client 1 stats
    uint32_t avgLatency1 = client1Stats.successCount > 0 ?
                          (client1Stats.totalLatencyMs / client1Stats.successCount) : 0;
    UART1_SendString("[Main:PrintStats][");
    UART1_SendNumber(GetTick());
    UART1_SendString("ms] Client1: ");
    UART1_SendNumber(client1Stats.writeCount);
    UART1_SendString(" writes (");
    UART1_SendNumber(client1Stats.successCount);
    UART1_SendString(" OK, ");
    UART1_SendNumber(client1Stats.errorCount);
    UART1_SendString(" ERR, avg=");
    UART1_SendNumber(avgLatency1);
    UART1_SendString("ms)\n");

    // Client 2 stats
    uint32_t avgLatency2 = client2Stats.successCount > 0 ?
                          (client2Stats.totalLatencyMs / client2Stats.successCount) : 0;
    UART1_SendString("[Main:PrintStats][");
    UART1_SendNumber(GetTick());
    UART1_SendString("ms] Client2: ");
    UART1_SendNumber(client2Stats.writeCount);
    UART1_SendString(" writes (");
    UART1_SendNumber(client2Stats.successCount);
    UART1_SendString(" OK, ");
    UART1_SendNumber(client2Stats.errorCount);
    UART1_SendString(" ERR, avg=");
    UART1_SendNumber(avgLatency2);
    UART1_SendString("ms)\n");

    // Queue stats
    UART1_SendString("[Main:PrintStats][");
    UART1_SendNumber(GetTick());
    UART1_SendString("ms] Queue: max_depth=");
    UART1_SendNumber(queueMaxDepth);
    UART1_SendString(", total_ops=");
    UART1_SendNumber(totalOperations);
    UART1_SendString("\n");

    // Flash utilization
    uint32_t uptime = GetTick();
    uint32_t utilization = uptime > 0 ? (flashBusyTimeMs * 1000) / uptime : 0;
    UART1_SendString("[Main:PrintStats][");
    UART1_SendNumber(GetTick());
    UART1_SendString("ms] Flash: busy=");
    UART1_SendNumber(flashBusyTimeMs);
    UART1_SendString("ms (");
    UART1_SendNumber(utilization / 10);
    UART1_SendChar('.');
    UART1_SendNumber(utilization % 10);
    UART1_SendString("%)\n");

    UART1_SendString("[Main:PrintStats][");
    UART1_SendNumber(GetTick());
    UART1_SendString("ms] ==================\n\n");

    lastPrintTime = GetTick();
}

/***************************************************************************
 * Main Function
 ***************************************************************************/

int main(void) {
    FlashDeviceInfo_t flashInfo;
    FlashResult_t result;

    // 1. Initialize SysTick for timestamps
    SysTick_Init();

    // 2. Initialize UART1 for telemetry
    UART1_Init();
    delay_ms(100);  // Let UART stabilize

    // Printf should now work via write() override
    LOG("Main", "main", "\n=== Multi-Client Flash Demo ===");
    LOG("Main", "main", "SysTick initialized (1ms tick)");
    LOG("Main", "main", "UART1 initialized (115200 baud, TX-only)");

    // 3. Initialize GPIO for LEDs
    LED_Init();
    LOG("Main", "main", "LEDs initialized (PF1=RED, PF2=BLUE, PF3=GREEN)");

    // Blink green to show system alive
    for (int i = 0; i < 3; i++) {
        LED_Toggle(3);
        delay_ms(200);
        LED_Toggle(3);
        delay_ms(200);
    }

    // 4. Initialize SSI2 flash interface
    SSI2_Flash_HW_Init();
    LOG("Main", "main", "SSI2 flash interface initialized");

    // 5. Initialize Client 1 (Temperature Sensor)
    result = FlashClient_Init(&tempSensor, &ssi2Flash, 1000);
    if (result != FLASH_RESULT_SUCCESS) {
        LOG("Main", "main", "ERROR: Client1 init failed!");
        while(1) {
            LED_Toggle(3);
            delay_ms(100);
        }
    }
    LOG_NUM("Main", "main", "Client1 initialized (ID=", tempSensor.clientId);

    // 6. Initialize Client 2 (Pressure Sensor)
    result = FlashClient_Init(&pressureSensor, &ssi2Flash, 1000);
    if (result != FLASH_RESULT_SUCCESS) {
        LOG("Main", "main", "ERROR: Client2 init failed!");
        while(1) {
            LED_Toggle(3);
            delay_ms(100);
        }
    }
    LOG_NUM("Main", "main", "Client2 initialized (ID=", pressureSensor.clientId);

    // 7. Enable global interrupts (MUST be before flash operations!)
    __enable_irq();
    LOG("Main", "main", "Interrupts enabled");

    // 8. Verify flash device
    result = FlashClient_ReadDeviceInfo(&tempSensor, &flashInfo);
    UART1_SendString("[Main:main][");
    UART1_SendNumber(GetTick());
    UART1_SendString("ms] ReadDeviceInfo returned: ");
    UART1_SendNumber(result);
    UART1_SendChar('\n');
    if (result == FLASH_RESULT_SUCCESS) {
        UART1_SendString("[Main:main][");
        UART1_SendNumber(GetTick());
        UART1_SendString("ms] Flash ID: 0x");
        UART1_SendHex(flashInfo.manufacturerId);
        UART1_SendHex(flashInfo.deviceType);
        UART1_SendHex(flashInfo.deviceId);
        UART1_SendChar(' ');

        if (flashInfo.manufacturerId == W25Q64FV_JEDEC_MANUFACTURER &&
            flashInfo.deviceId == W25Q64FV_JEDEC_DEVICE_ID) {
            UART1_SendString("(W25Q64FV - OK)\n");
        } else {
            UART1_SendString("(UNKNOWN - Check connections!)\n");
        }
    } else {
        LOG("Main", "main", "ERROR: Flash ID read failed!");
        while(1) {
            LED_Toggle(3);
            delay_ms(100);
        }
    }

    // 9. Erase test sectors (sector 0 and 1 for pages 0-199)
    LOG("Main", "main", "Erasing sectors 0-1...");

    result = FlashClient_EraseSector(&tempSensor, 0);
    if (result == FLASH_RESULT_SUCCESS) {
        FlashClient_WaitUntilReady(&tempSensor, 5000);
        LOG("Main", "main", "Sector 0 erased");
    }

    result = FlashClient_EraseSector(&tempSensor, 1);
    if (result == FLASH_RESULT_SUCCESS) {
        FlashClient_WaitUntilReady(&tempSensor, 5000);
        LOG("Main", "main", "Sector 1 erased");
    }

    LOG("Main", "main", "Erase complete");

    // 10. Start operation
    LOG("Main", "main", "Starting dual-writer demo\n");

    // Main loop
    while (1) {
        TempLogger_Task();       // Client 1 task
        PressureLogger_Task();   // Client 2 task
        PrintStats();            // Statistics output
    }
}
