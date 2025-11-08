# TM4C123GH6PM I2C Driver Implementation Plan

## Overview
This document outlines the complete implementation plan for I2C (Inter-Integrated Circuit) drivers for the TM4C123GH6PM microcontroller, following the existing driver architecture pattern used in GPIO and SSI drivers.

---

## Phase 1: Hardware Abstraction Layer (tm4c123x.h)

### 1.1 Items Already Implemented ✓
The following I2C-related definitions already exist in `tm4c123x.h`:

- **Base Addresses** (lines 264-267):
  - `I2C_0_BASEADDR` through `I2C_3_BASEADDR`

- **IRQ Numbers** (line 84):
  - `IRQ_NO_I2C0` (need to add I2C1, I2C2, I2C3)

- **Clock Control Macros** (lines 586-590):
  - `I2C0_PCLK_EN()` through `I2C3_PCLK_EN()`

- **Reset Macros** (lines 684-706):
  - `I2C0_RESET()` through `I2C3_RESET()`

- **System Control Registers** (lines 364, 388, 412, 436, 460, 484):
  - `PPI2C`, `SRI2C`, `RCGCI2C`, `SCGCI2C`, `DCGCI2C`, `PRI2C` in SYSCTL structure

### 1.2 Items to Add

#### 1.2.1 Missing IRQ Numbers
```c
/* I2C Interrupt Numbers - Add after line 84 */
#define IRQ_NO_I2C1             37      /* I2C1 Master and Slave Interrupt */
#define IRQ_NO_I2C2             68      /* I2C2 Master and Slave Interrupt */
#define IRQ_NO_I2C3             69      /* I2C3 Master and Slave Interrupt */
```

#### 1.2.2 I2C Register Structure Definition
Add near line 234 (after existing peripheral structures):

```c
/*
 * I2C Peripheral Register Structure
 */
typedef struct {
    /* I2C Master Registers */
    __vo uint32_t MSA;          /* 0x000: Master Slave Address */
    __vo uint32_t MCS;          /* 0x004: Master Control/Status */
    __vo uint32_t MDR;          /* 0x008: Master Data */
    __vo uint32_t MTPR;         /* 0x00C: Master Timer Period */
    __vo uint32_t MIMR;         /* 0x010: Master Interrupt Mask */
    __vo uint32_t MRIS;         /* 0x014: Master Raw Interrupt Status */
    __vo uint32_t MMIS;         /* 0x018: Master Masked Interrupt Status */
    __vo uint32_t MICR;         /* 0x01C: Master Interrupt Clear */
    __vo uint32_t MCR;          /* 0x020: Master Configuration */
    __vo uint32_t MCLKOCNT;     /* 0x024: Master Clock Low Timeout Count */
    uint32_t RESERVED0;         /* 0x028: Reserved */
    __vo uint32_t MBMON;        /* 0x02C: Master Bus Monitor */
    uint32_t RESERVED1[2];      /* 0x030-0x034: Reserved */
    __vo uint32_t MCR2;         /* 0x038: Master Configuration 2 */
    uint32_t RESERVED2[497];    /* 0x03C-0x7FC: Reserved */

    /* I2C Slave Registers */
    __vo uint32_t SOAR;         /* 0x800: Slave Own Address */
    __vo uint32_t SCSR;         /* 0x804: Slave Control/Status */
    __vo uint32_t SDR;          /* 0x808: Slave Data */
    __vo uint32_t SIMR;         /* 0x80C: Slave Interrupt Mask */
    __vo uint32_t SRIS;         /* 0x810: Slave Raw Interrupt Status */
    __vo uint32_t SMIS;         /* 0x814: Slave Masked Interrupt Status */
    __vo uint32_t SICR;         /* 0x818: Slave Interrupt Clear */
    __vo uint32_t SOAR2;        /* 0x81C: Slave Own Address 2 */
    __vo uint32_t SACKCTL;      /* 0x820: Slave ACK Control */
    uint32_t RESERVED3[487];    /* 0x824-0xFBC: Reserved */

    /* I2C Peripheral Properties */
    __vo uint32_t PP;           /* 0xFC0: Peripheral Properties */
    __vo uint32_t PC;           /* 0xFC4: Peripheral Configuration */
} I2C_RegDef_t;
```

#### 1.2.3 I2C Peripheral Definitions
Add near line 271 (after existing peripheral definitions):

```c
/* I2C Peripheral Definitions */
#define I2C0                ((I2C_RegDef_t*)I2C_0_BASEADDR)
#define I2C1                ((I2C_RegDef_t*)I2C_1_BASEADDR)
#define I2C2                ((I2C_RegDef_t*)I2C_2_BASEADDR)
#define I2C3                ((I2C_RegDef_t*)I2C_3_BASEADDR)
```

#### 1.2.4 I2C Register Bit Position Definitions
Add new section after line 456 (in the bit position definitions section):

```c
/***********************************************************************************
 * Bit position definitions of I2C peripheral
 ***********************************************************************************/

/* I2CMSA Register Bits */
#define I2C_MSA_SA              1       /* Slave Address bit position */
#define I2C_MSA_RS              0       /* Receive/Send bit */

/* I2CMCS Register Bits - Read (Status) */
#define I2C_MCS_CLKTO           7       /* Clock Timeout Error */
#define I2C_MCS_BUSBSY          6       /* Bus Busy */
#define I2C_MCS_IDLE            5       /* I2C Idle */
#define I2C_MCS_ARBLST          4       /* Arbitration Lost */
#define I2C_MCS_DATACK          3       /* Acknowledge Data */
#define I2C_MCS_ADRACK          2       /* Acknowledge Address */
#define I2C_MCS_ERROR           1       /* Error */
#define I2C_MCS_BUSY            0       /* I2C Busy */

/* I2CMCS Register Bits - Write (Control) */
#define I2C_MCS_HS              4       /* High-Speed Enable */
#define I2C_MCS_ACK             3       /* Data Acknowledge Enable */
#define I2C_MCS_STOP            2       /* Generate STOP */
#define I2C_MCS_START           1       /* Generate START */
#define I2C_MCS_RUN             0       /* I2C Master Enable */

/* I2CMTPR Register Bits */
#define I2C_MTPR_HS             7       /* High-Speed Enable */
#define I2C_MTPR_TPR            0       /* Timer Period (bits 6:0) */

/* I2CMIMR Register Bits */
#define I2C_MIMR_CLKIM          1       /* Clock Timeout Interrupt Mask */
#define I2C_MIMR_IM             0       /* Master Interrupt Mask */

/* I2CMRIS Register Bits */
#define I2C_MRIS_CLKRIS         1       /* Clock Timeout Raw Interrupt Status */
#define I2C_MRIS_RIS            0       /* Master Raw Interrupt Status */

/* I2CMMIS Register Bits */
#define I2C_MMIS_CLKMIS         1       /* Clock Timeout Masked Interrupt Status */
#define I2C_MMIS_MIS            0       /* Masked Interrupt Status */

/* I2CMICR Register Bits */
#define I2C_MICR_CLKIC          1       /* Clock Timeout Interrupt Clear */
#define I2C_MICR_IC             0       /* Master Interrupt Clear */

/* I2CMCR Register Bits */
#define I2C_MCR_GFE             6       /* I2C Glitch Filter Enable */
#define I2C_MCR_SFE             5       /* I2C Slave Function Enable */
#define I2C_MCR_MFE             4       /* I2C Master Function Enable */
#define I2C_MCR_LPBK            0       /* I2C Loopback */

/* I2CMCR2 Register Bits */
#define I2C_MCR2_GFPW           4       /* Glitch Filter Pulse Width (bits 6:4) */

/* I2CSCSR Register Bits - Read (Status) */
#define I2C_SCSR_OAR2SEL        3       /* OAR2 Address Matched */
#define I2C_SCSR_FBR            2       /* First Byte Received */
#define I2C_SCSR_TREQ           1       /* Transmit Request */
#define I2C_SCSR_RREQ           0       /* Receive Request */

/* I2CSCSR Register Bits - Write (Control) */
#define I2C_SCSR_DA             0       /* Device Active */

/* I2CSIMR Register Bits */
#define I2C_SIMR_STOPIM         2       /* Stop Condition Interrupt Mask */
#define I2C_SIMR_STARTIM        1       /* Start Condition Interrupt Mask */
#define I2C_SIMR_DATAIM         0       /* Data Interrupt Mask */

/* I2CSRIS Register Bits */
#define I2C_SRIS_STOPRIS        2       /* Stop Condition Raw Interrupt Status */
#define I2C_SRIS_STARTRIS       1       /* Start Condition Raw Interrupt Status */
#define I2C_SRIS_DATARIS        0       /* Data Raw Interrupt Status */

/* I2CSMIS Register Bits */
#define I2C_SMIS_STOPMIS        2       /* Stop Condition Masked Interrupt Status */
#define I2C_SMIS_STARTMIS       1       /* Start Condition Masked Interrupt Status */
#define I2C_SMIS_DATAMIS        0       /* Data Masked Interrupt Status */

/* I2CSICR Register Bits */
#define I2C_SICR_STOPIC         2       /* Stop Condition Interrupt Clear */
#define I2C_SICR_STARTIC        1       /* Start Condition Interrupt Clear */
#define I2C_SICR_DATAIC         0       /* Data Interrupt Clear */

/* I2CSOAR2 Register Bits */
#define I2C_SOAR2_OAR2EN        7       /* I2C Slave Own Address 2 Enable */

/* I2CSACKCTL Register Bits */
#define I2C_SACKCTL_ACKOVAL     1       /* I2C Slave ACK Override Value */
#define I2C_SACKCTL_ACKOEN      0       /* I2C Slave ACK Override Enable */

/* I2CPP Register Bits */
#define I2C_PP_HS               0       /* High-Speed Capable */

/* I2CPC Register Bits */
#define I2C_PC_HS               0       /* High-Speed Capable */
```

---

## Phase 2: Driver API Layer (tm4c123x_i2c_driver.h)

### 2.1 Configuration and Handle Structures

Following the SSI driver pattern:

```c
/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct {
    uint32_t I2C_SCLSpeed;          /* SCL clock speed: 100kHz, 400kHz, 1MHz, 3.33MHz */
    uint8_t I2C_DeviceAddress;      /* Slave device address (7-bit) */
    uint8_t I2C_ACKControl;         /* ACK control: Enable/Disable */
} I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct {
    I2C_RegDef_t *pI2Cx;            /* Peripheral base address pointer */
    I2C_Config_t I2C_Config;        /* Configuration settings */
    uint8_t *pTxBuffer;             /* Tx buffer address */
    uint8_t *pRxBuffer;             /* Rx buffer address */
    uint32_t TxLen;                 /* Tx length */
    uint32_t RxLen;                 /* Rx length */
    uint8_t TxRxState;              /* Communication state */
    uint8_t DevAddr;                /* Slave/device address */
    uint32_t RxSize;                /* Rx size */
    uint8_t Sr;                     /* Repeated start value */
} I2C_Handle_t;
```

### 2.2 I2C Macros and Definitions

```c
/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM        100000      /* Standard mode: 100 kHz */
#define I2C_SCL_SPEED_FM        400000      /* Fast mode: 400 kHz */
#define I2C_SCL_SPEED_FMP       1000000     /* Fast mode plus: 1 MHz */

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE          1
#define I2C_ACK_DISABLE         0

/*
 * I2C application states
 */
#define I2C_READY               0
#define I2C_BUSY_IN_RX          1
#define I2C_BUSY_IN_TX          2

/*
 * I2C application events
 */
#define I2C_EV_TX_CMPLT         0
#define I2C_EV_RX_CMPLT         1
#define I2C_EV_STOP             2
#define I2C_ERROR_BERR          3   /* Bus error */
#define I2C_ERROR_ARLO          4   /* Arbitration lost */
#define I2C_ERROR_AF            5   /* ACK failure */
#define I2C_ERROR_OVR           6   /* Overrun */
#define I2C_ERROR_TIMEOUT       7   /* Timeout */
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9

/*
 * I2C repeated start macros
 */
#define I2C_DISABLE_SR          0
#define I2C_ENABLE_SR           1

/*
 * I2C status flags (MCS register)
 */
#define I2C_FLAG_CLKTO          (1 << I2C_MCS_CLKTO)
#define I2C_FLAG_BUSBSY         (1 << I2C_MCS_BUSBSY)
#define I2C_FLAG_IDLE           (1 << I2C_MCS_IDLE)
#define I2C_FLAG_ARBLST         (1 << I2C_MCS_ARBLST)
#define I2C_FLAG_DATACK         (1 << I2C_MCS_DATACK)
#define I2C_FLAG_ADRACK         (1 << I2C_MCS_ADRACK)
#define I2C_FLAG_ERROR          (1 << I2C_MCS_ERROR)
#define I2C_FLAG_BUSY           (1 << I2C_MCS_BUSY)
```

### 2.3 API Function Prototypes

Following SSI driver naming convention:

```c
/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive (Blocking mode)
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * Data Send and Receive (Interrupt mode)
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/*
 * Slave mode data transfer
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
```

---

## Phase 3: Driver Implementation (tm4c123x_i2c_driver.c)

### 3.1 I2C Initialization Procedure (Generic - TM4C123 Specific)

The I2C initialization on TM4C123GH6PM follows a specific sequence that differs from STM32:

#### **Step 1: Configure the Mode (Standard or Fast)**
- Determine if using Standard mode (100 kHz) or Fast mode (400 kHz)
- This affects the TPR calculation

#### **Step 2: Configure the Speed of the Serial Clock (SCL)**
- Calculate TPR value based on system clock and desired I2C speed
- Program I2CMTPR register
- Formula: `TPR = (SysClock / (2 × 10 × I2C_Speed)) - 1`
  - Note: `10 = SCL_LP + SCL_HP` where SCL_LP=6, SCL_HP=4 (hardware fixed)

#### **Step 3: Configure Device Address (Applicable when device is slave)**
- Program I2CSOAR register with 7-bit slave address
- Optionally program I2CSOAR2 for dual-address support

#### **Step 4: Enable the Acking**
- For master: ACK is controlled via MCS register (ACK bit) during operation
- For slave: Automatic ACK generation (can override via I2CSACKCTL)

#### **Step 5: Configure Rise Time for I2C Pins**
- **TM4C123 does NOT have a dedicated TRISE register**
- Rise time is configured through GPIO settings:
  - **Drive Strength**: Set via GPIODR2R, GPIODR4R, or GPIODR8R
  - **Slew Rate Control**: Enable via GPIOSLR (for 8-mA drive only)
  - **Open-Drain**: Configure SDA (and optionally SCL) via GPIOODR
- See GPIO timing configuration section for details

**Important Note:** All above configuration must be done when the peripheral is **disabled** (MFE/SFE bits cleared in I2CMCR register).

---

### 3.2 Key Implementation Functions

#### 3.2.1 Clock Control
```c
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (pI2Cx == I2C0) I2C0_PCLK_EN();
        else if (pI2Cx == I2C1) I2C1_PCLK_EN();
        else if (pI2Cx == I2C2) I2C2_PCLK_EN();
        else if (pI2Cx == I2C3) I2C3_PCLK_EN();
    }
    // Clock disable logic similar
}
```

#### 3.2.2 I2C Initialization
Key steps in I2C_Init():

1. **Disable I2C peripheral** (clear MFE/SFE in MCR)
2. **Calculate and set TPR** value for SCL speed
3. **Configure glitch filter** (optional, via MCR and MCR2)
4. **Set slave own address** (if in slave mode, via SOAR)
5. **Enable master/slave function** (set MFE/SFE in MCR)

```c
void I2C_Init(I2C_Handle_t *pI2CHandle) {
    // 1. Disable I2C master function
    pI2CHandle->pI2Cx->MCR &= ~(1 << I2C_MCR_MFE);

    // 2. Calculate and set TPR
    uint8_t tpr = I2C_CalculateTPR(pI2CHandle->I2C_Config.I2C_SCLSpeed);
    pI2CHandle->pI2Cx->MTPR = (tpr & 0x7F);

    // 3. Configure glitch filter (optional)
    // pI2CHandle->pI2Cx->MCR |= (1 << I2C_MCR_GFE);
    // pI2CHandle->pI2Cx->MCR2 = (0x3 << I2C_MCR2_GFPW);

    // 4. Set slave address if configured
    if (pI2CHandle->I2C_Config.I2C_DeviceAddress != 0) {
        pI2CHandle->pI2Cx->SOAR = (pI2CHandle->I2C_Config.I2C_DeviceAddress & 0x7F);
    }

    // 5. Enable I2C master function
    pI2CHandle->pI2Cx->MCR |= (1 << I2C_MCR_MFE);
}
```

#### 3.2.3 TPR Calculation
```c
/* TPR = (System Clock / (2 × (SCL_LP + SCL_HP) × SCL_Speed)) - 1 */
/* For standard/fast mode: SCL_LP=6, SCL_HP=4 (total = 10) */
static uint8_t I2C_CalculateTPR(uint32_t sclSpeed) {
    uint32_t systemClock = 16000000; // 16 MHz default (should be configurable)
    uint32_t tpr = (systemClock / (2 * 10 * sclSpeed)) - 1;

    // Ensure TPR is within valid range (1-127)
    if (tpr < 1) tpr = 1;
    if (tpr > 127) tpr = 127;

    return (uint8_t)tpr;
}
```

**Example TPR Values:**
- 100 kHz @ 16 MHz: TPR = (16000000 / (2×10×100000)) - 1 = 7
- 400 kHz @ 16 MHz: TPR = (16000000 / (2×10×400000)) - 1 = 1
- 100 kHz @ 80 MHz: TPR = (80000000 / (2×10×100000)) - 1 = 39

#### 3.2.4 GPIO Configuration for I2C Timing

Since TM4C123 lacks TRISE register, GPIO must be properly configured:

```c
/* Example: Configure I2C0 GPIO pins (PB2=SCL, PB3=SDA) */
void I2C0_GPIO_Init(void) {
    // 1. Enable GPIO Port B clock
    SYSCTL->RCGCGPIO |= (1 << 1);
    while(!(SYSCTL->PRGPIO & (1 << 1)));

    // 2. Enable alternate function
    GPIOB->AFSEL |= (1 << 2) | (1 << 3);  // PB2, PB3

    // 3. Configure both SCL and SDA as open-drain (REQUIRED for I2C)
    GPIOB->ODR |= (1 << 2) | (1 << 3);  // PB2 (SCL), PB3 (SDA) open-drain

    // 4. Configure pin control for I2C (AF=3)
    GPIOB->PCTL &= ~0xFF00;
    GPIOB->PCTL |= 0x3300;  // I2C function on PB2, PB3

    // 5. Set drive strength (affects rise/fall time)
    GPIOB->DR8R |= (1 << 2) | (1 << 3);  // 8-mA drive

    // 6. Enable slew rate control (reduces overshoot/EMI)
    GPIOB->SLR |= (1 << 2) | (1 << 3);   // Slew rate control

    // 7. Enable digital function
    GPIOB->DEN |= (1 << 2) | (1 << 3);
}
```

#### 3.2.5 Master Send/Receive Implementation

**Master Control Sequences (MCS register):**

```c
/* Single byte send/receive */
#define I2C_MCS_SINGLE      ((1<<I2C_MCS_STOP) | (1<<I2C_MCS_START) | (1<<I2C_MCS_RUN))  // 0x07

/* Burst send */
#define I2C_MCS_BURST_SEND_START    ((1<<I2C_MCS_START) | (1<<I2C_MCS_RUN))          // 0x03
#define I2C_MCS_BURST_SEND_CONT     (1<<I2C_MCS_RUN)                                 // 0x01
#define I2C_MCS_BURST_SEND_FINISH   ((1<<I2C_MCS_STOP) | (1<<I2C_MCS_RUN))           // 0x05

/* Burst receive */
#define I2C_MCS_BURST_RECV_START    ((1<<I2C_MCS_ACK) | (1<<I2C_MCS_START) | (1<<I2C_MCS_RUN))  // 0x0B
#define I2C_MCS_BURST_RECV_CONT     ((1<<I2C_MCS_ACK) | (1<<I2C_MCS_RUN))                       // 0x09
#define I2C_MCS_BURST_RECV_FINISH   ((1<<I2C_MCS_STOP) | (1<<I2C_MCS_RUN))                      // 0x05
```

**Master Send Data (Blocking):**
- Set slave address in MSA register (RS=0 for transmit)
- For each byte:
  - Write data to MDR
  - Write appropriate control sequence to MCS
  - Wait for BUSY flag to clear
  - Check for errors (ARBLST, ADRACK, DATACK)

---

## Phase 4: Demo Applications

### 4.1 Example Applications to Create

1. **010i2c_master_tx_testing.c**
   - I2C master sending data to Arduino slave
   - Uses I2C0: PB2 (SCL), PB3 (SDA)

2. **011i2c_master_rx_testing.c**
   - I2C master receiving data from slave device

3. **012i2c_slave_tx_string.c**
   - I2C slave mode transmitting data

4. **013i2c_slave_rx_string.c**
   - I2C slave mode receiving data

---

## Hardware Specifications Summary

### I2C Module Details
- **4 I2C modules**: I2C0, I2C1, I2C2, I2C3
- **Base addresses**: 0x40020000 - 0x40023000
- **Interrupt numbers**: 8, 37, 68, 69

### GPIO Pin Configuration
| Module | SCL Pin | SDA Pin | Alt Function |
|--------|---------|---------|--------------|
| I2C0   | PB2     | PB3     | 3            |
| I2C1   | PA6     | PA7     | 3            |
| I2C2   | PE4     | PE5     | 3            |
| I2C3   | PD0     | PD1     | 3            |

**Important**:
- **Both SCL and SDA** pins must be configured as **open-drain** (GPIOODR register)
- Datasheet marks both signals as "I/O OD" (Open-Drain)
- SCL has internal pull-up assistance, but still requires open-drain configuration
- **External pull-up resistors required**:
  - Standard mode (100 kHz): 4.7-10 kΩ typical
  - Fast mode (400 kHz): 2.2-4.7 kΩ typical
  - Size based on bus speed and capacitance
- Both pins require **AFSEL** and **PCTL** configuration for I2C alternate function

### Supported Speed Modes
- **Standard Mode**: 100 kHz (TPR=7 @ 16MHz)
- **Fast Mode**: 400 kHz (TPR=1 @ 16MHz)
- **Fast Mode Plus**: 1 MHz
- **High-Speed Mode**: 3.33 MHz (requires HS bit set)

---

## Implementation Checklist

### Phase 1: tm4c123x.h
- [ ] Add missing I2C IRQ numbers (I2C1, I2C2, I2C3)
- [ ] Add I2C_RegDef_t structure
- [ ] Add I2C peripheral definitions (I2C0-I2C3)
- [ ] Add I2C register bit position definitions

### Phase 2: tm4c123x_i2c_driver.h
- [ ] Create I2C_Config_t structure
- [ ] Create I2C_Handle_t structure
- [ ] Define I2C macros (speed, ACK control, states, events, flags)
- [ ] Declare all API function prototypes

### Phase 3: tm4c123x_i2c_driver.c
- [ ] Implement I2C_PeriClockControl()
- [ ] Implement I2C_Init() and I2C_DeInit()
- [ ] Implement I2C_MasterSendData() (blocking)
- [ ] Implement I2C_MasterReceiveData() (blocking)
- [ ] Implement I2C_MasterSendDataIT() (interrupt)
- [ ] Implement I2C_MasterReceiveDataIT() (interrupt)
- [ ] Implement I2C_SlaveSendData()
- [ ] Implement I2C_SlaveReceiveData()
- [ ] Implement I2C_IRQInterruptConfig()
- [ ] Implement I2C_IRQPriorityConfig()
- [ ] Implement I2C_EV_IRQHandling()
- [ ] Implement I2C_ER_IRQHandling()
- [ ] Implement helper functions (GetFlagStatus, ManageAcking, etc.)

### Phase 4: Demo Applications
- [ ] Create 010i2c_master_tx_testing.c
- [ ] Create 011i2c_master_rx_testing.c
- [ ] Create 012i2c_slave_tx_string.c
- [ ] Create 013i2c_slave_rx_string.c

---

## TM4C123 vs STM32 I2C: Key Architectural Differences

### Timing Configuration Differences

| Feature | TM4C123GH6PM | STM32F4xx |
|---------|--------------|-----------|
| **Clock Control** | I2CMTPR register (TPR field) | I2C_CCR register |
| **Rise Time Config** | ❌ NO dedicated register | ✅ I2C_TRISE register |
| **Duty Cycle** | Fixed (6:4 ratio) | Programmable via CCR[14] |
| **SCL Low/High Period** | Hardware fixed (LP=6, HP=4) | Calculated from CCR value |
| **Timing Formula** | `TPR = (SysClock/(2×10×Speed)) - 1` | Complex CCR + TRISE calc |
| **GPIO Timing Control** | ✅ Drive strength + slew rate | Limited |
| **Glitch Filter** | ✅ Programmable (MCR, MCR2) | Basic noise filter |

### Control Register Differences

| Aspect | TM4C123GH6PM | STM32F4xx |
|--------|--------------|-----------|
| **Master Control** | I2CMCS (combined control/status) | I2C_CR1, I2C_CR2 separate |
| **Operation Control** | Sequence-based (0x03, 0x01, 0x05) | Bit-based (START, STOP bits) |
| **Interrupts** | Single shared IRQ per module | Separate event/error IRQs |
| **ACK Control** | Controlled per-transaction in MCS | Global control in CR1 |

### Key TM4C123 Specific Notes

1. **No TRISE Register**:
   - Rise time controlled via GPIO configuration only
   - Use GPIODR2R/DR4R/DR8R for drive strength
   - Use GPIOSLR for slew rate control on 8-mA drive

2. **GPIO Configuration Requirements**:
   - Enable GPIO clock before I2C clock
   - Configure alternate function (AFSEL, PCTL)
   - **Both SCL and SDA MUST be open-drain** (GPIOODR register)
   - Configure external pull-up resistors (4.7-10 kΩ for 100 kHz, 2.2-4.7 kΩ for 400 kHz)
   - Set appropriate drive strength (affects timing)
   - Enable digital function (DEN)

3. **Master Operation Sequences** (MCS register values):
   - Single byte TX/RX: `0x07` (START + RUN + STOP)
   - Burst start: `0x03` (START + RUN)
   - Burst continue: `0x01` (RUN only)
   - Burst finish: `0x05` (STOP + RUN)
   - Burst RX with ACK: `0x0B`, `0x09` (includes ACK bit)

4. **Error Handling** (MCS status bits):
   - CLKTO (bit 7): Clock timeout error
   - BUSBSY (bit 6): Bus busy
   - ARBLST (bit 4): Arbitration lost
   - DATACK (bit 3): Data not acknowledged
   - ADRACK (bit 2): Address not acknowledged
   - ERROR (bit 1): General error

5. **Interrupt Architecture**:
   - Single IRQ per I2C module (master + slave combined)
   - No separate event vs error interrupts
   - IRQ numbers: 8, 37, 68, 69 for I2C0-3
   - Must check status registers to determine interrupt source

---

## End of Implementation Plan
