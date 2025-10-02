/*
 * tm4c123x.h
 *
 *  Created on: 02-Jun-2025
 *      Author: tabrez
 */
/*
 * The TM4C123GH6PM microcontroller comes with 32 KB of bit-banded SRAM, internal ROM, 256 KB of Flash memory, and 2KB of EEPROM.
 * */


#ifndef DRIVERS_INC_TM4C123X_H_
#define DRIVERS_INC_TM4C123X_H_

#include<stdint.h>
#include<stddef.h>

#define __weak   __attribute__((weak))

/********************************** Interrupt Control Macros **********************************/
/* TI ARM Compiler interrupt control - map CMSIS names to TI intrinsics */
/* TI uses single underscore (_disable_IRQ) while CMSIS uses double (__disable_irq) */

/* TI ARM Compiler provides these intrinsics (see SPNU151 section 6.8.1) */
extern unsigned _disable_IRQ(void);    /* Disable IRQ, returns previous PRIMASK */
extern unsigned _enable_IRQ(void);     /* Enable IRQ, returns previous PRIMASK */
extern void _restore_interrupts(unsigned);  /* Restore interrupts to previous state */

/* Map CMSIS-style names to TI intrinsics for code portability */
#define __get_PRIMASK()         _disable_IRQ()    /* Returns PRIMASK and disables */
#define __set_PRIMASK(x)        _restore_interrupts(x)  /* Restore PRIMASK state */
#define __disable_irq()         (void)_disable_IRQ()    /* Just disable, ignore return */
#define __enable_irq()          (void)_enable_IRQ()     /* Just enable, ignore return */

/********************************** Memory Regions **********************************/
#define FLASH_BASEADDR                          0x00000000U             /* Starting address of the 256 KB Flash memory, used for program code and constant data */
#define SRAM_BASEADDR                           0x20000000U             /* Starting address of the 32 KB SRAM, used for runtime data storage */
#define EEPROM_BASEADDR                         0x400AF000U             /* Starting address of the 2 KB EEPROM, used for non-volatile data storage */

#define EXT_RAM                                 0x60000000U             /* Starting address for external RAM, if connected (not internal to TM4C123GH6PM) */

#define PERIPH_BASEADDR                         0x40000000U
// SRAM Bit-Band Region: A bit-band region maps each word in a bit-band alias region to a single bit in the bit-band region.
#define SRAM_BITBAND_REGION_START               SRAM_BASEADDR           /* Start of the SRAM region (1 MB) that supports bit-banding */
#define SRAM_BITBAND_REGION_END                 0x200FFFFFU             /* End of the SRAM bit-band region */
#define SRAM_BITBAND_ALIAS_START                0x22000000U             /* Start of the SRAM bit-band alias region, where each bit maps to a 32-bit word */
#define SRAM_BITBAND_ALIAS_END                  0x23FFFFFFU             /* End of the SRAM bit-band alias region */

// Peripheral Bit-Band Region
#define PERIPH_BITBAND_REGION_START             PERIPH_BASEADDR             /* Start of the peripheral region (1 MB) that supports bit-banding */
#define PERIPH_BITBAND_REGION_END               0x400FFFFFU             /* End of the peripheral bit-band region */
#define PERIPH_BITBAND_ALIAS_START              0x42000000U             /* Start of the peripheral bit-band alias region, for bit-level access to peripheral registers */
#define PERIPH_BITBAND_ALIAS_END                0x43FFFFFFU             /* End of the peripheral bit-band alias region */

/* Calculates the alias address for a specific bit in SRAM for atomic bit manipulation */
#define BITBAND_SRAM(address, bit)      ((SRAM_BITBAND_ALIAS_START + (((uint32_t)(address) - SRAM_BITBAND_REGION_START) * 32U) + ((bit) * 4U)))

/* Calculates the alias address for a specific bit in peripheral registers for atomic bit manipulation */
#define BITBAND_PERIPH(address, bit)    ((PERIPH_BITBAND_ALIAS_START + (((uint32_t)(address) - PERIPH_BITBAND_REGION_START) * 32U) + ((bit) * 4U)))

/********************************** IRQ Numbers **********************************/
/*
 * TM4C123GH6PM Interrupt Request (IRQ) Numbers
 * From TM4C123GH6PM Datasheet Table 2-9: Exception Types
 */

/* SSI Interrupt Numbers */
#define IRQ_NO_SSI0             7       /* SSI0 Interrupt */
#define IRQ_NO_SSI1             34      /* SSI1 Interrupt */
#define IRQ_NO_SSI2             57      /* SSI2 Interrupt */
#define IRQ_NO_SSI3             58      /* SSI3 Interrupt */

/* GPIO Interrupt Numbers */
#define IRQ_NO_GPIOA            0       /* GPIO Port A Interrupt */
#define IRQ_NO_GPIOB            1       /* GPIO Port B Interrupt */
#define IRQ_NO_GPIOC            2       /* GPIO Port C Interrupt */
#define IRQ_NO_GPIOD            3       /* GPIO Port D Interrupt */
#define IRQ_NO_GPIOE            4       /* GPIO Port E Interrupt */
#define IRQ_NO_GPIOF            30      /* GPIO Port F Interrupt */

/* Other Common Peripherals */
#define IRQ_NO_UART0            5       /* UART0 Interrupt */
#define IRQ_NO_UART1            6       /* UART1 Interrupt */
#define IRQ_NO_I2C0             8       /* I2C0 Master and Slave Interrupt */
#define IRQ_NO_PWM0_FAULT       9       /* PWM0 Fault Interrupt */
#define IRQ_NO_PWM0_GEN0        10      /* PWM0 Generator 0 Interrupt */
#define IRQ_NO_TIMER0A          19      /* 16/32-Bit Timer 0A Interrupt */
#define IRQ_NO_TIMER0B          20      /* 16/32-Bit Timer 0B Interrupt */
#define IRQ_NO_TIMER1A          21      /* 16/32-Bit Timer 1A Interrupt */
#define IRQ_NO_TIMER1B          22      /* 16/32-Bit Timer 1B Interrupt */

/********************************** SYSCTL Register **********************************/
#define SYSCTL_BASEADDR                         0x400FE000U
#define SYSCTL_GPIOHBCTL_R                      (*((volatile uint32_t *)(SYSCTL_BASEADDR + 0x06C)))

/********************************** GPIO Base Addresses **********************************/

/* GPIO Base Addresses (APB) */
#define GPIOA_APB_BASEADDR      (PERIPH_BASEADDR + 0x4000)
#define GPIOB_APB_BASEADDR      (PERIPH_BASEADDR + 0x5000)
#define GPIOC_APB_BASEADDR      (PERIPH_BASEADDR + 0x6000)
#define GPIOD_APB_BASEADDR      (PERIPH_BASEADDR + 0x7000)
#define GPIOE_APB_BASEADDR      (PERIPH_BASEADDR + 0x24000)
#define GPIOF_APB_BASEADDR      (PERIPH_BASEADDR + 0x25000)

/* GPIO Base Addresses (AHB) */
#define GPIOA_AHB_BASEADDR      (PERIPH_BASEADDR + 0x58000)
#define GPIOB_AHB_BASEADDR      (PERIPH_BASEADDR + 0x59000)
#define GPIOC_AHB_BASEADDR      (PERIPH_BASEADDR + 0x5A000)
#define GPIOD_AHB_BASEADDR      (PERIPH_BASEADDR + 0x5B000)
#define GPIOE_AHB_BASEADDR      (PERIPH_BASEADDR + 0x5C000)
#define GPIOF_AHB_BASEADDR      (PERIPH_BASEADDR + 0x5D000)

/* SYSCTL_GPIOHBCTL_R Bit Positions */
#define SYSCTL_GPIOHBCTL_PORTA  (1U << 0)
#define SYSCTL_GPIOHBCTL_PORTB  (1U << 1)
#define SYSCTL_GPIOHBCTL_PORTC  (1U << 2)
#define SYSCTL_GPIOHBCTL_PORTD  (1U << 3)
#define SYSCTL_GPIOHBCTL_PORTE  (1U << 4)
#define SYSCTL_GPIOHBCTL_PORTF  (1U << 5)

/* Select default GPIOx_BASEADDR depending on GPIOHBCTL enable status, Note: GPIOX_BASEADDR is not a true constant */
//#define GPIOA_BASEADDR  (((SYSCTL_GPIOHBCTL_R & SYSCTL_GPIOHBCTL_PORTA) != 0) ? GPIOA_AHB_BASEADDR : GPIOA_APB_BASEADDR)
//#define GPIOB_BASEADDR  (((SYSCTL_GPIOHBCTL_R & SYSCTL_GPIOHBCTL_PORTB) != 0) ? GPIOB_AHB_BASEADDR : GPIOB_APB_BASEADDR)
//#define GPIOC_BASEADDR  (((SYSCTL_GPIOHBCTL_R & SYSCTL_GPIOHBCTL_PORTC) != 0) ? GPIOC_AHB_BASEADDR : GPIOC_APB_BASEADDR)
//#define GPIOD_BASEADDR  (((SYSCTL_GPIOHBCTL_R & SYSCTL_GPIOHBCTL_PORTD) != 0) ? GPIOD_AHB_BASEADDR : GPIOD_APB_BASEADDR)
//#define GPIOE_BASEADDR  (((SYSCTL_GPIOHBCTL_R & SYSCTL_GPIOHBCTL_PORTE) != 0) ? GPIOE_AHB_BASEADDR : GPIOE_APB_BASEADDR)
//#define GPIOF_BASEADDR  (((SYSCTL_GPIOHBCTL_R & SYSCTL_GPIOHBCTL_PORTF) != 0) ? GPIOF_AHB_BASEADDR : GPIOF_APB_BASEADDR)

/* Inline functions to select GPIO base addresses based on GPIOHBCTL */
static inline uint32_t GPIOA_BASEADDR(void) {
    return (SYSCTL_GPIOHBCTL_R & SYSCTL_GPIOHBCTL_PORTA) ? GPIOA_AHB_BASEADDR : GPIOA_APB_BASEADDR;
}

static inline uint32_t GPIOB_BASEADDR(void) {
    return (SYSCTL_GPIOHBCTL_R & SYSCTL_GPIOHBCTL_PORTB) ? GPIOB_AHB_BASEADDR : GPIOB_APB_BASEADDR;
}

static inline uint32_t GPIOC_BASEADDR(void) {
    return (SYSCTL_GPIOHBCTL_R & SYSCTL_GPIOHBCTL_PORTC) ? GPIOC_AHB_BASEADDR : GPIOC_APB_BASEADDR;
}

static inline uint32_t GPIOD_BASEADDR(void) {
    return (SYSCTL_GPIOHBCTL_R & SYSCTL_GPIOHBCTL_PORTD) ? GPIOD_AHB_BASEADDR : GPIOD_APB_BASEADDR;
}

static inline uint32_t GPIOE_BASEADDR(void) {
    return (SYSCTL_GPIOHBCTL_R & SYSCTL_GPIOHBCTL_PORTE) ? GPIOE_AHB_BASEADDR : GPIOE_APB_BASEADDR;
}

static inline uint32_t GPIOF_BASEADDR(void) {
    return (SYSCTL_GPIOHBCTL_R & SYSCTL_GPIOHBCTL_PORTF) ? GPIOF_AHB_BASEADDR : GPIOF_APB_BASEADDR;
}


/************************NVIC*************************************************************** */
// NVIC (Nested Vectored Interrupt Controller) Registers - Base address + offset approach
#define NVIC_BASEADDR                          0xE000E000U

// NVIC Register Offsets
#define NVIC_EN_OFFSET                         0x100U  // Interrupt Set-Enable Registers offset
#define NVIC_DIS_OFFSET                        0x180U  // Interrupt Clear-Enable Registers offset
#define NVIC_PEND_OFFSET                       0x200U  // Interrupt Set-Pending Registers offset
#define NVIC_UNPEND_OFFSET                     0x280U  // Interrupt Clear-Pending Registers offset
#define NVIC_ACTIVE_OFFSET                     0x300U  // Interrupt Active Bit Registers offset
#define NVIC_PRI_OFFSET                        0x400U  // Interrupt Priority Registers offset

// NVIC Enable Registers (EN0-EN4)
#define NVIC_EN0                               (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_EN_OFFSET + 0x00U)))
#define NVIC_EN1                               (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_EN_OFFSET + 0x04U)))
#define NVIC_EN2                               (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_EN_OFFSET + 0x08U)))
#define NVIC_EN3                               (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_EN_OFFSET + 0x0CU)))
#define NVIC_EN4                               (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_EN_OFFSET + 0x10U)))

// NVIC Disable Registers (DIS0-DIS4)
#define NVIC_DIS0                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_DIS_OFFSET + 0x00U)))
#define NVIC_DIS1                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_DIS_OFFSET + 0x04U)))
#define NVIC_DIS2                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_DIS_OFFSET + 0x08U)))
#define NVIC_DIS3                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_DIS_OFFSET + 0x0CU)))
#define NVIC_DIS4                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_DIS_OFFSET + 0x10U)))

// NVIC Priority Registers (PRI0-PRI34)
#define NVIC_PRI0                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x00U)))
#define NVIC_PRI1                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x04U)))
#define NVIC_PRI2                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x08U)))
#define NVIC_PRI3                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x0CU)))
#define NVIC_PRI4                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x10U)))
#define NVIC_PRI5                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x14U)))
#define NVIC_PRI6                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x18U)))
#define NVIC_PRI7                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x1CU)))
#define NVIC_PRI8                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x20U)))
#define NVIC_PRI9                              (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x24U)))
#define NVIC_PRI10                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x28U)))
#define NVIC_PRI11                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x2CU)))
#define NVIC_PRI12                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x30U)))
#define NVIC_PRI13                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x34U)))
#define NVIC_PRI14                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x38U)))
#define NVIC_PRI15                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x3CU)))
#define NVIC_PRI16                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x40U)))
#define NVIC_PRI17                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x44U)))
#define NVIC_PRI18                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x48U)))
#define NVIC_PRI19                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x4CU)))
#define NVIC_PRI20                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x50U)))
#define NVIC_PRI21                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x54U)))
#define NVIC_PRI22                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x58U)))
#define NVIC_PRI23                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x5CU)))
#define NVIC_PRI24                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x60U)))
#define NVIC_PRI25                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x64U)))
#define NVIC_PRI26                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x68U)))
#define NVIC_PRI27                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x6CU)))
#define NVIC_PRI28                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x70U)))
#define NVIC_PRI29                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x74U)))
#define NVIC_PRI30                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x78U)))
#define NVIC_PRI31                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x7CU)))
#define NVIC_PRI32                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x80U)))
#define NVIC_PRI33                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x84U)))
#define NVIC_PRI34                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PRI_OFFSET + 0x88U)))

// NVIC Pending Registers (PEND0-PEND4)
#define NVIC_PEND0                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PEND_OFFSET + 0x00U)))
#define NVIC_PEND1                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PEND_OFFSET + 0x04U)))
#define NVIC_PEND2                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PEND_OFFSET + 0x08U)))
#define NVIC_PEND3                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PEND_OFFSET + 0x0CU)))
#define NVIC_PEND4                             (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_PEND_OFFSET + 0x10U)))

// NVIC Unpend Registers (UNPEND0-UNPEND4)
#define NVIC_UNPEND0                           (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_UNPEND_OFFSET + 0x00U)))
#define NVIC_UNPEND1                           (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_UNPEND_OFFSET + 0x04U)))
#define NVIC_UNPEND2                           (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_UNPEND_OFFSET + 0x08U)))
#define NVIC_UNPEND3                           (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_UNPEND_OFFSET + 0x0CU)))
#define NVIC_UNPEND4                           (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_UNPEND_OFFSET + 0x10U)))

// NVIC Active Bit Registers (ACTIVE0-ACTIVE4)
#define NVIC_ACTIVE0                           (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_ACTIVE_OFFSET + 0x00U)))
#define NVIC_ACTIVE1                           (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_ACTIVE_OFFSET + 0x04U)))
#define NVIC_ACTIVE2                           (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_ACTIVE_OFFSET + 0x08U)))
#define NVIC_ACTIVE3                           (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_ACTIVE_OFFSET + 0x0CU)))
#define NVIC_ACTIVE4                           (*((volatile uint32_t*)(NVIC_BASEADDR + NVIC_ACTIVE_OFFSET + 0x10U)))

#define __NVIC_PRIO_BITS 3  // TM4C123x implements 3 bits for priority levels (8 levels, 0-7)


/*APB Peripherals*/

/* SSI Base Addresses */
#define SSI0_BASEADDR                (PERIPH_BASEADDR + 0x08000)
#define SSI1_BASEADDR                (PERIPH_BASEADDR + 0x09000)
#define SSI2_BASEADDR                (PERIPH_BASEADDR + 0x0A000)
#define SSI3_BASEADDR                (PERIPH_BASEADDR + 0x0B000)

#define I2C_0_BASEADDR               (PERIPH_BASEADDR + 0x20000)
#define I2C_1_BASEADDR               (PERIPH_BASEADDR + 0x21000)
#define I2C_2_BASEADDR               (PERIPH_BASEADDR + 0x22000)
#define I2C_3_BASEADDR               (PERIPH_BASEADDR + 0x23000)

#define UART_0_BASEADDR              (PERIPH_BASEADDR + 0xC000)
#define UART_1_BASEADDR              (PERIPH_BASEADDR + 0xD000)
#define UART_2_BASEADDR              (PERIPH_BASEADDR + 0xE000)
#define UART_3_BASEADDR              (PERIPH_BASEADDR + 0xF000)
#define UART_4_BASEADDR              (PERIPH_BASEADDR + 0x10000)
#define UART_5_BASEADDR              (PERIPH_BASEADDR + 0x11000)
#define UART_6_BASEADDR              (PERIPH_BASEADDR + 0x12000)
#define UART_7_BASEADDR              (PERIPH_BASEADDR + 0x13000)

/*************************************peripheral structures register definition******************************************/

typedef struct {
    volatile uint32_t DATA[256];   /* Port data register (virtual array). Write/read using address mask (offsets 0x000–0x3FC) */
    volatile uint32_t DIR;         /* Data direction register. 1=Output, 0=Input (offset 0x400) */
    volatile uint32_t IS;          /* Interrupt sense: 0=edge, 1=level (offset 0x404) */
    volatile uint32_t IBE;         /* Interrupt both edges: 1=both edges trigger (offset 0x408) */
    volatile uint32_t IEV;         /* Interrupt event: 0=falling/low, 1=rising/high (offset 0x40C) */
    volatile uint32_t IM;          /* Interrupt mask: 1=unmask interrupt (offset 0x410) */
    volatile uint32_t RIS;         /* Raw interrupt status: shows interrupt before masking (offset 0x414) */
    volatile uint32_t MIS;         /* Masked interrupt status: shows interrupt after masking (offset 0x418) */
    volatile uint32_t ICR;         /* Interrupt clear: write 1 to clear interrupt (offset 0x41C) */
    volatile uint32_t AFSEL;       /* Alternate function select: 1=use peripheral instead of GPIO (offset 0x420) */
    volatile uint32_t RESERVED[55];/* Reserved space to align to 0x500 */
    volatile uint32_t DR2R;        /* 2-mA drive select for each pin (offset 0x500) */
    volatile uint32_t DR4R;        /* 4-mA drive select (offset 0x504) */
    volatile uint32_t DR8R;        /* 8-mA drive select (offset 0x508) */
    volatile uint32_t ODR;         /* Open drain enable: 1=OD mode (offset 0x50C) */
    volatile uint32_t PUR;         /* Pull-up resistor enable (offset 0x510) */
    volatile uint32_t PDR;         /* Pull-down resistor enable (offset 0x514) */
    volatile uint32_t SLR;         /* Slew rate control enable (offset 0x518) */
    volatile uint32_t DEN;         /* Digital enable: 1=digital function enabled (offset 0x51C) */
    volatile uint32_t LOCK;        /* GPIO lock register (offset 0x520) */
    volatile uint32_t CR;          /* Commit register for protected pins (offset 0x524) */
    volatile uint32_t AMSEL;       /* Analog mode select: 1=analog function (offset 0x528) */
    volatile uint32_t PCTL;        /* Port control for configuring alternate functions (offset 0x52C) */
    volatile uint32_t ADCCTL;      /* ADC control for each pin (offset 0x530) */
    volatile uint32_t DMACTL;      /* DMA control for each pin (offset 0x534) */
} GPIO_RegDef_t;

// Core System Control Registers
typedef struct {
    volatile uint32_t DID0;        // 0x000: Device Identification 0
    volatile uint32_t DID1;        // 0x004: Device Identification 1
    uint32_t RESERVED0[10];        // 0x008-0x02C: Reserved
    volatile uint32_t PBORCTL;     // 0x030: Brown-Out Reset Control
    uint32_t RESERVED1[3];         // 0x034-0x03C: Reserved
    volatile uint32_t SRCR0;       // 0x040: Software Reset Control 0
    volatile uint32_t SRCR1;       // 0x044: Software Reset Control 1
    volatile uint32_t SRCR2;       // 0x048: Software Reset Control 2
    uint32_t RESERVED2;            // 0x04C: Reserved
    volatile uint32_t RIS;         // 0x050: Raw Interrupt Status
    volatile uint32_t IMC;         // 0x054: Interrupt Mask Control
    volatile uint32_t MISC;        // 0x058: Masked Interrupt Status and Clear
    volatile uint32_t RESC;        // 0x05C: Reset Cause
    volatile uint32_t RCC;         // 0x060: Run Mode Clock Configuration
    uint32_t RESERVED3;            // 0x064: Reserved
    volatile uint32_t GPIOHBCTL;   // 0x068: GPIO High-Performance Bus Control
    uint32_t RESERVED4;            // 0x06C: Reserved
    volatile uint32_t RCC2;        // 0x070: RCC2 (extended Clock Configuration)
    uint32_t RESERVED5[2];         // 0x074-0x078: Reserved
    volatile uint32_t MOSCCTL;     // 0x07C: Main Oscillator Control
    uint32_t RESERVED6[49];        // 0x080-0x140: Reserved
    volatile uint32_t DSLPCLKCFG;  // 0x144: Deep Sleep Clock Configuration
    uint32_t RESERVED7;            // 0x148: Reserved
    volatile uint32_t SYSPROP;     // 0x14C: System Properties
    volatile uint32_t PIOSCCAL;    // 0x150: Precision Internal Oscillator Calibration
    volatile uint32_t PIOSCSTAT;   // 0x154: Precision Internal Oscillator Statistics
    uint32_t RESERVED8[2];         // 0x158-0x15C: Reserved
    volatile uint32_t PLLFREQ0;    // 0x160: PLL Frequency 0
    volatile uint32_t PLLFREQ1;    // 0x164: PLL Frequency 1
    volatile uint32_t PLLSTAT;     // 0x168: PLL Status
    uint32_t RESERVED9[7];         // 0x16C-0x184: Reserved
    volatile uint32_t SLPPWRCFG;   // 0x188: Sleep Power Configuration
    volatile uint32_t DSLPPWRCFG;  // 0x18C: Deep-Sleep Power Configuration
    uint32_t RESERVED10[4];        // 0x190-0x19C: Reserved
    volatile uint32_t NVMSTAT;     // 0x1A0: Non-Volatile Memory Information
    uint32_t RESERVED11[3];        // 0x1A4-0x1AC: Reserved
    volatile uint32_t LDOSPCTL;    // 0x1B4: LDO Sleep Power Control
    volatile uint32_t LDOSPCAL;    // 0x1B8: LDO Sleep Power Calibration
    volatile uint32_t LDODPCTL;    // 0x1BC: LDO Deep-Sleep Power Control
    volatile uint32_t LDODPCAL;    // 0x1C0: LDO Deep-Sleep Power Calibration
    uint32_t RESERVED12[2];        // 0x1C4-0x1C8: Reserved
    volatile uint32_t SDPMST;      // 0x1CC: Sleep/Deep-Sleep Power Mode Status
} SYSCTL_CoreRegDef_t;

// Peripheral Present Registers (offset 0x300-0x35C)
typedef struct {
    volatile uint32_t PPWD;           // 0x300: Watchdog Timer Present
    volatile uint32_t PPTIMER;        // 0x304: 16/32-Bit Timer Present
    volatile uint32_t PPGPIO;         // 0x308: GPIO Present
    volatile uint32_t PPDMA;          // 0x30C: uDMA Present
    uint32_t RESERVED0;               // 0x310: Reserved
    volatile uint32_t PPHIB;          // 0x314: Hibernation Module Present
    volatile uint32_t PPUART;         // 0x318: UART Present
    volatile uint32_t PPSSI;          // 0x31C: SSI Present
    volatile uint32_t PPI2C;          // 0x320: I2C Present
    uint32_t RESERVED1;               // 0x324: Reserved
    volatile uint32_t PPUSB;          // 0x328: USB Present
    uint32_t RESERVED2[2];            // 0x32C-0x330: Reserved
    volatile uint32_t PPCAN;          // 0x334: CAN Present
    volatile uint32_t PPADC;          // 0x338: ADC Present
    volatile uint32_t PPACMP;         // 0x33C: Analog Comparator Present
    volatile uint32_t PPPWM;          // 0x340: PWM Present
    volatile uint32_t PPQEI;          // 0x344: QEI Present
    uint32_t RESERVED3[4];            // 0x348-0x354: Reserved
    volatile uint32_t PPEEPROM;       // 0x358: EEPROM Present
    volatile uint32_t PPWTIMER;       // 0x35C: Wide Timer Present
} SYSCTL_PresRegDef_t;

// Software Reset Registers (offset 0x500-0x55C)
typedef struct {
    volatile uint32_t SRWD;           // 0x500: Watchdog Timer Reset
    volatile uint32_t SRTIMER;        // 0x504: Timer Reset
    volatile uint32_t SRGPIO;         // 0x508: GPIO Reset
    volatile uint32_t SRDMA;          // 0x50C: uDMA Reset
    uint32_t RESERVED0;               // 0x510: Reserved
    volatile uint32_t SRHIB;          // 0x514: Hibernation Reset
    volatile uint32_t SRUART;         // 0x518: UART Reset
    volatile uint32_t SRSSI;          // 0x51C: SSI Reset
    volatile uint32_t SRI2C;          // 0x520: I2C Reset
    uint32_t RESERVED1;               // 0x524: Reserved
    volatile uint32_t SRUSB;          // 0x528: USB Reset
    uint32_t RESERVED2[2];            // 0x52C-0x530: Reserved
    volatile uint32_t SRCAN;          // 0x534: CAN Reset
    volatile uint32_t SRADC;          // 0x538: ADC Reset
    volatile uint32_t SRACMP;         // 0x53C: Analog Comparator Reset
    volatile uint32_t SRPWM;          // 0x540: PWM Reset
    volatile uint32_t SRQEI;          // 0x544: QEI Reset
    uint32_t RESERVED3[4];            // 0x548-0x554: Reserved
    volatile uint32_t SREEPROM;       // 0x558: EEPROM Reset
    volatile uint32_t SRWTIMER;       // 0x55C: Wide Timer Reset
} SYSCTL_SwResetRegDef_t;

// Run-Mode Clock Gating Control Registers (offset 0x600-0x65C)
typedef struct {
    volatile uint32_t RCGCWD;         // 0x600: WDT Clock Gating
    volatile uint32_t RCGCTIMER;      // 0x604: Timer Clock Gating
    volatile uint32_t RCGCGPIO;       // 0x608: GPIO Clock Gating
    volatile uint32_t RCGCDMA;        // 0x60C: uDMA Clock Gating
    uint32_t RESERVED0;               // 0x610: Reserved
    volatile uint32_t RCGCHIB;        // 0x614: HIB Clock Gating
    volatile uint32_t RCGCUART;       // 0x618: UART Clock Gating
    volatile uint32_t RCGCSSI;        // 0x61C: SSI Clock Gating
    volatile uint32_t RCGCI2C;        // 0x620: I2C Clock Gating
    uint32_t RESERVED1;               // 0x624: Reserved
    volatile uint32_t RCGCUSB;        // 0x628: USB Clock Gating
    uint32_t RESERVED2[2];            // 0x62C-0x630: Reserved
    volatile uint32_t RCGCCAN;        // 0x634: CAN Clock Gating
    volatile uint32_t RCGCADC;        // 0x638: ADC Clock Gating
    volatile uint32_t RCGCACMP;       // 0x63C: ACMP Clock Gating
    volatile uint32_t RCGCPWM;        // 0x640: PWM Clock Gating
    volatile uint32_t RCGCQEI;        // 0x644: QEI Clock Gating
    uint32_t RESERVED3[4];            // 0x648-0x654: Reserved
    volatile uint32_t RCGCEEPROM;     // 0x658: EEPROM Clock Gating
    volatile uint32_t RCGCWTIMER;     // 0x65C: Wide Timer Clock Gating
} SYSCTL_RunModeClkGatingRegDef_t;

// Sleep-Mode Clock Gating Control Registers (offset 0x700-0x75C)
typedef struct {
    volatile uint32_t SCGCWD;         // 0x700: WDT Sleep Clock Gating
    volatile uint32_t SCGCTIMER;      // 0x704: Timer Sleep Clock Gating
    volatile uint32_t SCGCGPIO;       // 0x708: GPIO Sleep Clock Gating
    volatile uint32_t SCGCDMA;        // 0x70C: uDMA Sleep Clock Gating
    uint32_t RESERVED0;               // 0x710: Reserved
    volatile uint32_t SCGCHIB;        // 0x714: HIB Sleep Clock Gating
    volatile uint32_t SCGCUART;       // 0x718: UART Sleep Clock Gating
    volatile uint32_t SCGCSSI;        // 0x71C: SSI Sleep Clock Gating
    volatile uint32_t SCGCI2C;        // 0x720: I2C Sleep Clock Gating
    uint32_t RESERVED1;               // 0x724: Reserved
    volatile uint32_t SCGCUSB;        // 0x728: USB Sleep Clock Gating
    uint32_t RESERVED2[2];            // 0x72C-0x730: Reserved
    volatile uint32_t SCGCCAN;        // 0x734: CAN Sleep Clock Gating
    volatile uint32_t SCGCADC;        // 0x738: ADC Sleep Clock Gating
    volatile uint32_t SCGCACMP;       // 0x73C: ACMP Sleep Clock Gating
    volatile uint32_t SCGCPWM;        // 0x740: PWM Sleep Clock Gating
    volatile uint32_t SCGCQEI;        // 0x744: QEI Sleep Clock Gating
    uint32_t RESERVED3[4];            // 0x748-0x754: Reserved
    volatile uint32_t SCGCEEPROM;     // 0x758: EEPROM Sleep Clock Gating
    volatile uint32_t SCGCWTIMER;     // 0x75C: Wide Timer Sleep Clock Gating
} SYSCTL_SleepModeClkGatingRegDef_t;

// Deep-Sleep-Mode Clock Gating Control Registers (offset 0x800-0x85C)
typedef struct {
    volatile uint32_t DCGCWD;         // 0x800: WDT Deep-Sleep Clock Gating
    volatile uint32_t DCGCTIMER;      // 0x804: Timer Deep-Sleep Clock Gating
    volatile uint32_t DCGCGPIO;       // 0x808: GPIO Deep-Sleep Clock Gating
    volatile uint32_t DCGCDMA;        // 0x80C: uDMA Deep-Sleep Clock Gating
    uint32_t RESERVED0;               // 0x810: Reserved
    volatile uint32_t DCGCHIB;        // 0x814: HIB Deep-Sleep Clock Gating
    volatile uint32_t DCGCUART;       // 0x818: UART Deep-Sleep Clock Gating
    volatile uint32_t DCGCSSI;        // 0x81C: SSI Deep-Sleep Clock Gating
    volatile uint32_t DCGCI2C;        // 0x820: I2C Deep-Sleep Clock Gating
    uint32_t RESERVED1;               // 0x824: Reserved
    volatile uint32_t DCGCUSB;        // 0x828: USB Deep-Sleep Clock Gating
    uint32_t RESERVED2[2];            // 0x82C-0x830: Reserved
    volatile uint32_t DCGCCAN;        // 0x834: CAN Deep-Sleep Clock Gating
    volatile uint32_t DCGCADC;        // 0x838: ADC Deep-Sleep Clock Gating
    volatile uint32_t DCGCACMP;       // 0x83C: ACMP Deep-Sleep Clock Gating
    volatile uint32_t DCGCPWM;        // 0x840: PWM Deep-Sleep Clock Gating
    volatile uint32_t DCGCQEI;        // 0x844: QEI Deep-Sleep Clock Gating
    uint32_t RESERVED3[4];            // 0x848-0x854: Reserved
    volatile uint32_t DCGCEEPROM;     // 0x858: EEPROM Deep-Sleep Clock Gating
    volatile uint32_t DCGCWTIMER;     // 0x85C: Wide Timer Deep-Sleep Clock Gating
} SYSCTL_DeepSleepModeClkGatingRegDef_t;

// Peripheral Ready Registers (offset 0xA00-0xA5C)
typedef struct {
    volatile uint32_t PRWD;           // 0xA00: WDT Peripheral Ready
    volatile uint32_t PRTIMER;        // 0xA04: Timer Peripheral Ready
    volatile uint32_t PRGPIO;         // 0xA08: GPIO Peripheral Ready
    volatile uint32_t PRDMA;          // 0xA0C: uDMA Peripheral Ready
    uint32_t RESERVED0;               // 0xA10: Reserved
    volatile uint32_t PRHIB;          // 0xA14: HIB Peripheral Ready
    volatile uint32_t PRUART;         // 0xA18: UART Peripheral Ready
    volatile uint32_t PRSSI;          // 0xA1C: SSI Peripheral Ready
    volatile uint32_t PRI2C;          // 0xA20: I2C Peripheral Ready
    uint32_t RESERVED1;               // 0xA24: Reserved
    volatile uint32_t PRUSB;          // 0xA28: USB Peripheral Ready
    uint32_t RESERVED2[2];            // 0xA2C-0xA30: Reserved
    volatile uint32_t PRCAN;          // 0xA34: CAN Peripheral Ready
    volatile uint32_t PRADC;          // 0xA38: ADC Peripheral Ready
    volatile uint32_t PRACMP;         // 0xA3C: ACMP Peripheral Ready
    volatile uint32_t PRPWM;          // 0xA40: PWM Peripheral Ready
    volatile uint32_t PRQEI;          // 0xA44: QEI Peripheral Ready
    uint32_t RESERVED3[4];            // 0xA48-0xA54: Reserved
    volatile uint32_t PREEPROM;       // 0xA58: EEPROM Peripheral Ready
    volatile uint32_t PRWTIMER;       // 0xA5C: Wide Timer Peripheral Ready
} SYSCTL_PeriphReadyRegDef_t;

/* SSI Register Structure */
typedef struct {
    volatile uint32_t CR0;     /* Control 0: Data size, clock settings (offset 0x000) */
    volatile uint32_t CR1;     /* Control 1: Enable, master/slave mode (offset 0x004) */
    volatile uint32_t DR;      /* Data: Transmit/Receive data (offset 0x008) */
    volatile uint32_t SR;      /* Status: Busy, FIFO status (offset 0x00C) */
    volatile uint32_t CPSR;    /* Clock Prescale: Clock divider (offset 0x010) */
    volatile uint32_t IM;      /* Interrupt Mask (offset 0x014) */
    volatile uint32_t RIS;     /* Raw Interrupt Status (offset 0x018) */
    volatile uint32_t MIS;     /* Masked Interrupt Status (offset 0x01C) */
    volatile uint32_t ICR;     /* Interrupt Clear (offset 0x020) */
    volatile uint32_t DMACTL;  /* DMA Control (offset 0x024) */
} SSI_RegDef_t;

/* UART Register Structure */
typedef struct {
    volatile uint32_t DR;          /* Data Register (offset 0x000) */
    union {
        volatile uint32_t RSR;     /* Receive Status Register (offset 0x004) */
        volatile uint32_t ECR;     /* Error Clear Register (offset 0x004) */
    };
    uint32_t RESERVED0[4];         /* 0x008-0x014: Reserved */
    volatile uint32_t FR;          /* Flag Register (offset 0x018) */
    uint32_t RESERVED1;            /* 0x01C: Reserved */
    volatile uint32_t ILPR;        /* IrDA Low-Power Register (offset 0x020) */
    volatile uint32_t IBRD;        /* Integer Baud-Rate Divisor (offset 0x024) */
    volatile uint32_t FBRD;        /* Fractional Baud-Rate Divisor (offset 0x028) */
    volatile uint32_t LCRH;        /* Line Control (offset 0x02C) */
    volatile uint32_t CTL;         /* Control (offset 0x030) */
    volatile uint32_t IFLS;        /* Interrupt FIFO Level Select (offset 0x034) */
    volatile uint32_t IM;          /* Interrupt Mask (offset 0x038) */
    volatile uint32_t RIS;         /* Raw Interrupt Status (offset 0x03C) */
    volatile uint32_t MIS;         /* Masked Interrupt Status (offset 0x040) */
    volatile uint32_t ICR;         /* Interrupt Clear (offset 0x044) */
    volatile uint32_t DMACTL;      /* DMA Control (offset 0x048) */
} UART_RegDef_t;



/****************************************************************Peripheral definitions typecasted**************************************************************************/
/* Peripheral definitions typecasted */
#define GPIOA   ((GPIO_RegDef_t*)GPIOA_BASEADDR())
#define GPIOB   ((GPIO_RegDef_t*)GPIOB_BASEADDR())
#define GPIOC   ((GPIO_RegDef_t*)GPIOC_BASEADDR())
#define GPIOD   ((GPIO_RegDef_t*)GPIOD_BASEADDR())
#define GPIOE   ((GPIO_RegDef_t*)GPIOE_BASEADDR())
#define GPIOF   ((GPIO_RegDef_t*)GPIOF_BASEADDR())

#define SYSCTL_CORE      ((SYSCTL_CoreRegDef_t*)SYSCTL_BASEADDR)
#define SYSCTL_PRES      ((SYSCTL_PresRegDef_t*)(SYSCTL_BASEADDR + 0x300))
#define SYSCTL_SWRESET   ((SYSCTL_SwResetRegDef_t*)(SYSCTL_BASEADDR + 0x500))
#define SYSCTL_RUNCLK    ((SYSCTL_RunModeClkGatingRegDef_t*)(SYSCTL_BASEADDR + 0x600))
#define SYSCTL_SLEEPCLK  ((SYSCTL_SleepModeClkGatingRegDef_t*)(SYSCTL_BASEADDR + 0x700))
#define SYSCTL_DSLPCLK   ((SYSCTL_DeepSleepModeClkGatingRegDef_t*)(SYSCTL_BASEADDR + 0x800))
#define SYSCTL_PR        ((SYSCTL_PeriphReadyRegDef_t*)(SYSCTL_BASEADDR + 0xA00))

/* SSI Peripheral Instances */
#define SSI0             ((SSI_RegDef_t*)SSI0_BASEADDR)
#define SSI1             ((SSI_RegDef_t*)SSI1_BASEADDR)
#define SSI2             ((SSI_RegDef_t*)SSI2_BASEADDR)
#define SSI3             ((SSI_RegDef_t*)SSI3_BASEADDR)

/* UART Peripheral Instances */
#define UART0            ((UART_RegDef_t*)UART_0_BASEADDR)
#define UART1            ((UART_RegDef_t*)UART_1_BASEADDR)
#define UART2            ((UART_RegDef_t*)UART_2_BASEADDR)
#define UART3            ((UART_RegDef_t*)UART_3_BASEADDR)
#define UART4            ((UART_RegDef_t*)UART_4_BASEADDR)
#define UART5            ((UART_RegDef_t*)UART_5_BASEADDR)
#define UART6            ((UART_RegDef_t*)UART_6_BASEADDR)
#define UART7            ((UART_RegDef_t*)UART_7_BASEADDR)

/* Clock enable macros for GPIOx peripherals */
#define GPIOA_PCLK_EN() (SYSCTL_RUNCLK->RCGCGPIO |= 1U<<0)
#define GPIOB_PCLK_EN() (SYSCTL_RUNCLK->RCGCGPIO |= 1U<<1)
#define GPIOC_PCLK_EN() (SYSCTL_RUNCLK->RCGCGPIO |= 1U<<2)
#define GPIOD_PCLK_EN() (SYSCTL_RUNCLK->RCGCGPIO |= 1U<<3)
#define GPIOE_PCLK_EN() (SYSCTL_RUNCLK->RCGCGPIO |= 1U<<4)
#define GPIOF_PCLK_EN() (SYSCTL_RUNCLK->RCGCGPIO |= 1U<<5)

/* Clock disable macros for GPIOx peripherals */
#define GPIOA_PCLK_DIS()  (SYSCTL_RUNCLK->RCGCGPIO &= ~(1U << 0))
#define GPIOB_PCLK_DIS()  (SYSCTL_RUNCLK->RCGCGPIO &= ~(1U << 1))
#define GPIOC_PCLK_DIS()  (SYSCTL_RUNCLK->RCGCGPIO &= ~(1U << 2))
#define GPIOD_PCLK_DIS()  (SYSCTL_RUNCLK->RCGCGPIO &= ~(1U << 3))
#define GPIOE_PCLK_DIS()  (SYSCTL_RUNCLK->RCGCGPIO &= ~(1U << 4))
#define GPIOF_PCLK_DIS()  (SYSCTL_RUNCLK->RCGCGPIO &= ~(1U << 5))

/* Clock enable macros for I2Cx peripherals */
#define I2C0_PCLK_EN() (SYSCTL_RUNCLK->RCGCI2C |= 1U<<0)
#define I2C1_PCLK_EN() (SYSCTL_RUNCLK->RCGCI2C |= 1U<<1)
#define I2C2_PCLK_EN() (SYSCTL_RUNCLK->RCGCI2C |= 1U<<2)
#define I2C3_PCLK_EN() (SYSCTL_RUNCLK->RCGCI2C |= 1U<<3)


/*
 * Clock Enable macros for UARTx peripherals
 */

/* Clock enable macros for UARTx peripherals */
#define UART0_PCLK_EN() (SYSCTL_RUNCLK->RCGCUART |= 1U << 0)
#define UART1_PCLK_EN() (SYSCTL_RUNCLK->RCGCUART |= 1U << 1)
#define UART2_PCLK_EN() (SYSCTL_RUNCLK->RCGCUART |= 1U << 2)
#define UART3_PCLK_EN() (SYSCTL_RUNCLK->RCGCUART |= 1U << 3)
#define UART4_PCLK_EN() (SYSCTL_RUNCLK->RCGCUART |= 1U << 4)
#define UART5_PCLK_EN() (SYSCTL_RUNCLK->RCGCUART |= 1U << 5)
#define UART6_PCLK_EN() (SYSCTL_RUNCLK->RCGCUART |= 1U << 6)
#define UART7_PCLK_EN() (SYSCTL_RUNCLK->RCGCUART |= 1U << 7)

/* Clock disable macros for UARTx peripherals */
#define UART0_PCLK_DIS() (SYSCTL_RUNCLK->RCGCUART &= ~(1U << 0))
#define UART1_PCLK_DIS() (SYSCTL_RUNCLK->RCGCUART &= ~(1U << 1))
#define UART2_PCLK_DIS() (SYSCTL_RUNCLK->RCGCUART &= ~(1U << 2))
#define UART3_PCLK_DIS() (SYSCTL_RUNCLK->RCGCUART &= ~(1U << 3))
#define UART4_PCLK_DIS() (SYSCTL_RUNCLK->RCGCUART &= ~(1U << 4))
#define UART5_PCLK_DIS() (SYSCTL_RUNCLK->RCGCUART &= ~(1U << 5))
#define UART6_PCLK_DIS() (SYSCTL_RUNCLK->RCGCUART &= ~(1U << 6))
#define UART7_PCLK_DIS() (SYSCTL_RUNCLK->RCGCUART &= ~(1U << 7))

/*
 * Clock Enable macros for SSIx peripherals, implements 3 variants of SPI
 */

/* Clock enable macros for SSIx (SPIx) peripherals */
#define SSI0_PCLK_EN() (SYSCTL_RUNCLK->RCGCSSI |= 1U << 0)
#define SSI1_PCLK_EN() (SYSCTL_RUNCLK->RCGCSSI |= 1U << 1)
#define SSI2_PCLK_EN() (SYSCTL_RUNCLK->RCGCSSI |= 1U << 2)
#define SSI3_PCLK_EN() (SYSCTL_RUNCLK->RCGCSSI |= 1U << 3)


/* Clock disable macros for SSIx (SPIx) peripherals */
#define SSI0_PCLK_DIS() (SYSCTL_RUNCLK->RCGCSSI &= ~(1U << 0))
#define SSI1_PCLK_DIS() (SYSCTL_RUNCLK->RCGCSSI &= ~(1U << 1))
#define SSI2_PCLK_DIS() (SYSCTL_RUNCLK->RCGCSSI &= ~(1U << 2))
#define SSI3_PCLK_DIS() (SYSCTL_RUNCLK->RCGCSSI &= ~(1U << 3))

/*
 * GPIO Reset macros using SRCR2
 * These macros perform a complete reset of the GPIO peripheral:
 * 1. Trigger reset by setting the bit in SRCR2
 * 2. Release from reset by clearing the bit
 * 3. Wait until the peripheral is ready again (PRGPIO register)
 */
#define GPIOA_RESET() do { \
    SYSCTL_CORE->SRCR2 |= (1U << 0); \
    SYSCTL_CORE->SRCR2 &= ~(1U << 0); \
    while(!(SYSCTL_PR->PRGPIO & (1U << 0))) {}; \
} while(0)

#define GPIOB_RESET() do { \
    SYSCTL_CORE->SRCR2 |= (1U << 1); \
    SYSCTL_CORE->SRCR2 &= ~(1U << 1); \
    while(!(SYSCTL_PR->PRGPIO & (1U << 1))) {}; \
} while(0)

#define GPIOC_RESET() do { \
    SYSCTL_CORE->SRCR2 |= (1U << 2); \
    SYSCTL_CORE->SRCR2 &= ~(1U << 2); \
    while(!(SYSCTL_PR->PRGPIO & (1U << 2))) {}; \
} while(0)

#define GPIOD_RESET() do { \
    SYSCTL_CORE->SRCR2 |= (1U << 3); \
    SYSCTL_CORE->SRCR2 &= ~(1U << 3); \
    while(!(SYSCTL_PR->PRGPIO & (1U << 3))) {}; \
} while(0)

#define GPIOE_RESET() do { \
    SYSCTL_CORE->SRCR2 |= (1U << 4); \
    SYSCTL_CORE->SRCR2 &= ~(1U << 4); \
    while(!(SYSCTL_PR->PRGPIO & (1U << 4))) {}; \
} while(0)

#define GPIOF_RESET() do { \
    SYSCTL_CORE->SRCR2 |= (1U << 5); \
    SYSCTL_CORE->SRCR2 &= ~(1U << 5); \
    while(!(SYSCTL_PR->PRGPIO & (1U << 5))) {}; \
} while(0)

/*
 * I2C Reset macros using SRCR1
 * These macros perform a complete reset of the I2C peripheral:
 * 1. Trigger reset by setting the bit in SRCR1
 * 2. Release from reset by clearing the bit
 * 3. Wait until the peripheral is ready again (PRI2C register)
 */
#define I2C0_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 0); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 0); \
    while(!(SYSCTL_PR->PRI2C & (1U << 0))) {}; \
} while(0)

#define I2C1_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 1); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 1); \
    while(!(SYSCTL_PR->PRI2C & (1U << 1))) {}; \
} while(0)

#define I2C2_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 2); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 2); \
    while(!(SYSCTL_PR->PRI2C & (1U << 2))) {}; \
} while(0)

#define I2C3_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 3); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 3); \
    while(!(SYSCTL_PR->PRI2C & (1U << 3))) {}; \
} while(0)

/*
 * UART Reset macros using SRCR1
 * These macros perform a complete reset of the UART peripheral:
 * 1. Trigger reset by setting the bit in SRCR1
 * 2. Release from reset by clearing the bit
 * 3. Wait until the peripheral is ready again (PRUART register)
 */
#define UART0_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 0); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 0); \
    while(!(SYSCTL_PR->PRUART & (1U << 0))) {}; \
} while(0)

#define UART1_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 1); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 1); \
    while(!(SYSCTL_PR->PRUART & (1U << 1))) {}; \
} while(0)

#define UART2_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 2); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 2); \
    while(!(SYSCTL_PR->PRUART & (1U << 2))) {}; \
} while(0)

#define UART3_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 3); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 3); \
    while(!(SYSCTL_PR->PRUART & (1U << 3))) {}; \
} while(0)

#define UART4_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 4); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 4); \
    while(!(SYSCTL_PR->PRUART & (1U << 4))) {}; \
} while(0)

#define UART5_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 5); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 5); \
    while(!(SYSCTL_PR->PRUART & (1U << 5))) {}; \
} while(0)

#define UART6_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 6); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 6); \
    while(!(SYSCTL_PR->PRUART & (1U << 6))) {}; \
} while(0)

#define UART7_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 7); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 7); \
    while(!(SYSCTL_PR->PRUART & (1U << 7))) {}; \
} while(0)

/*
 * SSI Reset macros using SRCR1
 * These macros perform a complete reset of the SSI peripheral:
 * 1. Trigger reset by setting the bit in SRCR1
 * 2. Release from reset by clearing the bit
 * 3. Wait until the peripheral is ready again (PRSSI register)
 */
#define SSI0_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 0); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 0); \
    while(!(SYSCTL_PR->PRSSI & (1U << 0))) {}; \
} while(0)

#define SSI1_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 1); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 1); \
    while(!(SYSCTL_PR->PRSSI & (1U << 1))) {}; \
} while(0)

#define SSI2_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 2); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 2); \
    while(!(SYSCTL_PR->PRSSI & (1U << 2))) {}; \
} while(0)

#define SSI3_RESET() do { \
    SYSCTL_CORE->SRCR1 |= (1U << 3); \
    SYSCTL_CORE->SRCR1 &= ~(1U << 3); \
    while(!(SYSCTL_PR->PRSSI & (1U << 3))) {}; \
} while(0)


/*
 * Some generic macors
 */
#define ENABLE                          1
#define DISABLE                         0
#define SET                             ENABLE
#define RESET                           DISABLE
#define GPIO_PIN_SET                    SET
#define GPIO_PIN_RESET                  RESET

#define FLAG_SET   SET
#define FLAG_RESET RESET
#include "tm4c123x_ssi_driver.h"
#include "tm4c123x_gpio_driver.h"

#endif /* DRIVERS_INC_TM4C123X_H_ */
