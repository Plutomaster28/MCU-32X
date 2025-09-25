/*
 * MCU-32X Bootstrap Hardware Definitions
 * Hardware register definitions and memory map for bootstrap loader
 * This file provides constants and macros for MCU-32X hardware access
 */

#ifndef MCU32X_HARDWARE_H
#define MCU32X_HARDWARE_H

#include <stdint.h>

/* =============================================================================
 * MEMORY MAP DEFINITIONS
 * =============================================================================
 */

/* Flash Memory Layout */
#define FLASH_BASE        0x00000000U      /* Flash memory base address */
#define FLASH_SIZE        0x00080000U      /* 512KB total flash */
#define BOOTLOADER_BASE   0x00000000U      /* Bootloader area (64KB) */
#define BOOTLOADER_SIZE   0x00010000U      /* 64KB for bootloader */
#define APPLICATION_BASE  0x00010000U      /* Application area start */
#define APPLICATION_SIZE  0x00070000U      /* 448KB for application */

/* SRAM Memory Layout */
#define SRAM_BASE         0x20000000U      /* Internal SRAM base */
#define SRAM_SIZE         0x00010000U      /* 64KB internal SRAM */
#define STACK_TOP         0x20010000U      /* Top of SRAM (stack grows down) */
#define HEAP_START        0x20008000U      /* Heap starts at 32KB mark */

/* External DRAM Layout */
#define DRAM_BASE         0x40000000U      /* External DRAM base */
#define DRAM_SIZE         0x01000000U      /* 16MB external DRAM */

/* Cache Memory Layout */
#define ICACHE_BASE       0x10000000U      /* Instruction cache control */
#define DCACHE_BASE       0x10008000U      /* Data cache control */
#define CACHE_SIZE        0x00008000U      /* 32KB per cache */

/* =============================================================================
 * PERIPHERAL BASE ADDRESSES
 * =============================================================================
 */

/* UART Controllers */
#define UART0_BASE        0x40000000U      /* UART0 base address */
#define UART1_BASE        0x40000080U      /* UART1 base address */

/* GPIO Controllers */
#define GPIOA_BASE        0x40000100U      /* GPIO Port A base */
#define GPIOB_BASE        0x40000200U      /* GPIO Port B base */
#define GPIOC_BASE        0x40000300U      /* GPIO Port C base */
#define GPIOD_BASE        0x40000400U      /* GPIO Port D base */

/* SPI Controllers */
#define SPI0_BASE         0x40000500U      /* SPI0 base address */
#define SPI1_BASE         0x40000580U      /* SPI1 base address */

/* I2C Controllers */
#define I2C0_BASE         0x40000600U      /* I2C0 base address */
#define I2C1_BASE         0x40000680U      /* I2C1 base address */

/* System Controllers */
#define SYSCON_BASE       0x40001000U      /* System control base */
#define CLOCK_BASE        0x40001000U      /* Clock control (within SYSCON) */
#define RESET_BASE        0x40001040U      /* Reset control (within SYSCON) */
#define POWER_BASE        0x40001080U      /* Power management (within SYSCON) */

/* Memory Controllers */
#define MEMCON_BASE       0x40002000U      /* Memory controller base */
#define FLASH_CON_BASE    0x40002100U      /* Flash controller base */

/* Timer Controllers */
#define TIMER0_BASE       0x40003000U      /* Timer0 base address */
#define TIMER1_BASE       0x40003040U      /* Timer1 base address */
#define TIMER2_BASE       0x40003080U      /* Timer2 base address */
#define SYSTICK_BASE      0x400030C0U      /* System tick timer base */

/* DMA Controller */
#define DMA_BASE          0x40004000U      /* DMA controller base */

/* ADC Controller */
#define ADC_BASE          0x40005000U      /* ADC controller base */

/* RTC Controller */
#define RTC_BASE          0x40006000U      /* RTC controller base */

/* Watchdog Timer */
#define WDT_BASE          0x40007000U      /* Watchdog timer base */

/* Interrupt Controller */
#define INTC_BASE         0x80000000U      /* Interrupt controller base */

/* =============================================================================
 * UART REGISTER OFFSETS
 * =============================================================================
 */

#define UART_LCR          0x00U            /* Line control register */
#define UART_CR           0x04U            /* Control register */
#define UART_BRR          0x08U            /* Baud rate register */
#define UART_FCR          0x0CU            /* FIFO control register */
#define UART_SR           0x10U            /* Status register */
#define UART_TDR          0x14U            /* Transmit data register */
#define UART_RDR          0x18U            /* Receive data register */
#define UART_IER          0x1CU            /* Interrupt enable register */

/* UART Status Register Bits */
#define UART_SR_TXE       0x01U            /* Transmit buffer empty */
#define UART_SR_RXNE      0x02U            /* Receive buffer not empty */
#define UART_SR_TC        0x04U            /* Transmission complete */
#define UART_SR_IDLE      0x08U            /* IDLE line detected */
#define UART_SR_ORE       0x10U            /* Overrun error */
#define UART_SR_NF        0x20U            /* Noise flag */
#define UART_SR_FE        0x40U            /* Framing error */
#define UART_SR_PE        0x80U            /* Parity error */

/* =============================================================================
 * GPIO REGISTER OFFSETS
 * =============================================================================
 */

#define GPIO_DATA         0x00U            /* Data register */
#define GPIO_DIR          0x04U            /* Direction register */
#define GPIO_PUR          0x08U            /* Pull-up register */
#define GPIO_PDR          0x0CU            /* Pull-down register */
#define GPIO_ODR          0x10U            /* Open-drain register */
#define GPIO_IER          0x14U            /* Interrupt enable register */
#define GPIO_ISR          0x18U            /* Interrupt status register */
#define GPIO_ICR          0x1CU            /* Interrupt clear register */

/* =============================================================================
 * CLOCK CONTROL REGISTER OFFSETS
 * =============================================================================
 */

#define CLK_PLLCR         0x00U            /* PLL control register */
#define CLK_PLLSR         0x04U            /* PLL status register */
#define CLK_CLKSEL        0x08U            /* Clock select register */
#define CLK_CLKEN         0x0CU            /* Clock enable register */
#define CLK_CLKDIV        0x10U            /* Clock divider register */

/* PLL Control Register Bits */
#define CLK_PLL_EN        0x00000001U      /* PLL enable */
#define CLK_PLL_BYPASS    0x00000002U      /* PLL bypass */
#define CLK_PLL_MUL_MASK  0x000000FCU      /* PLL multiplier mask */
#define CLK_PLL_DIV_MASK  0x00000F00U      /* PLL divider mask */

/* PLL Status Register Bits */
#define CLK_PLL_LOCK      0x00000001U      /* PLL locked */

/* Clock Select Register Values */
#define CLK_SEL_OSC       0x00000000U      /* Select oscillator */
#define CLK_SEL_PLL       0x00000001U      /* Select PLL */
#define CLK_SEL_RC        0x00000002U      /* Select RC oscillator */

/* =============================================================================
 * MEMORY CONTROLLER REGISTER OFFSETS
 * =============================================================================
 */

#define MEM_DRAMCR        0x00U            /* DRAM control register */
#define MEM_DRAMCFG       0x04U            /* DRAM configuration register */
#define MEM_DRAMCTL       0x08U            /* DRAM control register */
#define MEM_DRAMSR        0x0CU            /* DRAM status register */
#define MEM_DRAMTMG       0x10U            /* DRAM timing register */

/* DRAM Control Register Bits */
#define MEM_DRAM_EN       0x00000001U      /* DRAM enable */
#define MEM_DRAM_INIT     0x00000002U      /* DRAM initialization */
#define MEM_DRAM_REF_EN   0x00000004U      /* Auto refresh enable */
#define MEM_DRAM_SELF_REF 0x00000008U      /* Self refresh mode */

/* =============================================================================
 * CACHE CONTROLLER REGISTER OFFSETS
 * =============================================================================
 */

#define CACHE_CR          0x00U            /* Cache control register */
#define CACHE_SR          0x04U            /* Cache status register */
#define CACHE_INV         0x08U            /* Cache invalidate register */
#define CACHE_CLEAN       0x0CU            /* Cache clean register */

/* Cache Control Register Bits */
#define CACHE_EN          0x00000001U      /* Cache enable */
#define CACHE_INV_ALL     0x00000002U      /* Invalidate all */
#define CACHE_CLEAN_ALL   0x00000004U      /* Clean all */
#define CACHE_WT          0x00000008U      /* Write-through mode */
#define CACHE_WB          0x00000000U      /* Write-back mode (default) */

/* =============================================================================
 * INTERRUPT CONTROLLER REGISTER OFFSETS
 * =============================================================================
 */

#define INTC_IER          0x00U            /* Interrupt enable register */
#define INTC_IPR          0x04U            /* Interrupt pending register */
#define INTC_ICR          0x08U            /* Interrupt clear register */
#define INTC_IPRIOR0      0x10U            /* Interrupt priority register 0 */
#define INTC_IPRIOR1      0x14U            /* Interrupt priority register 1 */
#define INTC_IPRIOR2      0x18U            /* Interrupt priority register 2 */
#define INTC_IPRIOR3      0x1CU            /* Interrupt priority register 3 */

/* =============================================================================
 * TIMER REGISTER OFFSETS
 * =============================================================================
 */

#define TMR_CR            0x00U            /* Timer control register */
#define TMR_CNT           0x04U            /* Timer counter register */
#define TMR_PRD           0x08U            /* Timer period register */
#define TMR_CMP           0x0CU            /* Timer compare register */
#define TMR_SR            0x10U            /* Timer status register */
#define TMR_IER           0x14U            /* Timer interrupt enable register */

/* Timer Control Register Bits */
#define TMR_EN            0x00000001U      /* Timer enable */
#define TMR_IE            0x00000002U      /* Interrupt enable */
#define TMR_AR            0x00000004U      /* Auto-reload enable */
#define TMR_URS           0x00000008U      /* Update request source */
#define TMR_OPM           0x00000010U      /* One-pulse mode */
#define TMR_DIR           0x00000020U      /* Direction (0=up, 1=down) */

/* =============================================================================
 * POWER MANAGEMENT REGISTER OFFSETS
 * =============================================================================
 */

#define PWR_CR            0x00U            /* Power control register */
#define PWR_SR            0x04U            /* Power status register */
#define PWR_WUCR          0x08U            /* Wake-up control register */
#define PWR_WUSR          0x0CU            /* Wake-up status register */

/* Power Control Register Bits */
#define PWR_LPDS          0x00000001U      /* Low-power deep sleep */
#define PWR_PDDS          0x00000002U      /* Power down deep sleep */
#define PWR_CWUF          0x00000004U      /* Clear wake-up flag */
#define PWR_CSBF          0x00000008U      /* Clear standby flag */
#define PWR_PVDE          0x00000010U      /* Power voltage detector enable */
#define PWR_PLS_MASK      0x000000E0U      /* PVD level selection mask */

/* =============================================================================
 * SYSTEM CONTROL REGISTER DEFINITIONS
 * =============================================================================
 */

/* System Control Register */
#define SYSCON_CTRL       0x00U            /* System control register */
#define SYSCON_STAT       0x04U            /* System status register */
#define SYSCON_ID         0x08U            /* System ID register */
#define SYSCON_REV        0x0CU            /* System revision register */

/* System Control Bits */
#define SYSCON_RST        0x00000001U      /* System reset */
#define SYSCON_CLK_EN     0x00000002U      /* System clock enable */
#define SYSCON_DBG_EN     0x00000004U      /* Debug enable */
#define SYSCON_JTAG_EN    0x00000008U      /* JTAG enable */

/* =============================================================================
 * RISC-V CSR DEFINITIONS
 * =============================================================================
 */

/* Machine-mode CSRs */
#define CSR_MSTATUS       0x300U           /* Machine status register */
#define CSR_MISA          0x301U           /* Machine ISA register */
#define CSR_MIE           0x304U           /* Machine interrupt enable */
#define CSR_MTVEC         0x305U           /* Machine trap vector base */
#define CSR_MSCRATCH      0x340U           /* Machine scratch register */
#define CSR_MEPC          0x341U           /* Machine exception program counter */
#define CSR_MCAUSE        0x342U           /* Machine trap cause */
#define CSR_MTVAL         0x343U           /* Machine trap value */
#define CSR_MIP           0x344U           /* Machine interrupt pending */

/* Machine Status Register Bits */
#define MSTATUS_MIE       0x00000008U      /* Machine interrupt enable */
#define MSTATUS_MPIE      0x00000080U      /* Previous interrupt enable */
#define MSTATUS_MPP_MASK  0x00001800U      /* Previous privilege mode */

/* Machine Interrupt Enable Bits */
#define MIE_MSIE          0x00000008U      /* Machine software interrupt enable */
#define MIE_MTIE          0x00000080U      /* Machine timer interrupt enable */
#define MIE_MEIE          0x00000800U      /* Machine external interrupt enable */

/* =============================================================================
 * INTERRUPT VECTOR DEFINITIONS
 * =============================================================================
 */

#define IRQ_UART0         0U               /* UART0 interrupt */
#define IRQ_UART1         1U               /* UART1 interrupt */
#define IRQ_SPI0          2U               /* SPI0 interrupt */
#define IRQ_SPI1          3U               /* SPI1 interrupt */
#define IRQ_I2C0          4U               /* I2C0 interrupt */
#define IRQ_I2C1          5U               /* I2C1 interrupt */
#define IRQ_GPIO          6U               /* GPIO interrupt */
#define IRQ_TIMER0        7U               /* Timer0 interrupt */
#define IRQ_TIMER1        8U               /* Timer1 interrupt */
#define IRQ_TIMER2        9U               /* Timer2 interrupt */
#define IRQ_DMA           10U              /* DMA interrupt */
#define IRQ_ADC           11U              /* ADC interrupt */
#define IRQ_RTC           12U              /* RTC interrupt */
#define IRQ_WDT           13U              /* Watchdog timer interrupt */
#define IRQ_FLASH         14U              /* Flash controller interrupt */
#define IRQ_ECC           15U              /* ECC error interrupt */

/* =============================================================================
 * UTILITY MACROS FOR C CODE
 * =============================================================================
 */

/* Bit manipulation macros */
#define BIT(n)            (1U << (n))
#define SET_BIT(reg, bit) ((reg) |= BIT(bit))
#define CLR_BIT(reg, bit) ((reg) &= ~BIT(bit))
#define CHK_BIT(reg, bit) ((reg) & BIT(bit))
#define TOG_BIT(reg, bit) ((reg) ^= BIT(bit))

/* Memory access macros */
#define REG32(addr)       (*(volatile uint32_t *)(addr))
#define REG16(addr)       (*(volatile uint16_t *)(addr))
#define REG8(addr)        (*(volatile uint8_t *)(addr))

/* Register field manipulation */
#define FIELD_PREP(mask, val) (((val) << __builtin_ctz(mask)) & (mask))
#define FIELD_GET(mask, reg)  (((reg) & (mask)) >> __builtin_ctz(mask))

/* Wait for bit condition macros (polling) */
#define WAIT_BIT_SET(addr, bit)   do { while (!(REG32(addr) & BIT(bit))); } while(0)
#define WAIT_BIT_CLR(addr, bit)   do { while (REG32(addr) & BIT(bit)); } while(0)

/* Memory barriers (compiler barriers for single-core) */
#define MEMORY_BARRIER()    __asm__ volatile ("" ::: "memory")
#define INST_BARRIER()      __asm__ volatile ("fence.i" ::: "memory")

/* =============================================================================
 * BOOTLOADER CONFIGURATION
 * =============================================================================
 */

#define BOOT_SIGNATURE    0x32584D43U      /* "MCU2" signature */
#define BOOT_VERSION      0x00010000U      /* Version 1.0.0.0 */
#define BOOT_MAGIC        0x12345678U      /* Boot magic number */

/* Application header structure offsets */
#define APP_SIGNATURE     0x00U            /* Application signature */
#define APP_ENTRY_POINT   0x04U            /* Application entry point */
#define APP_SIZE          0x08U            /* Application size */
#define APP_CHECKSUM      0x0CU            /* Application checksum */
#define APP_VERSION       0x10U            /* Application version */

/* Boot modes */
#define BOOT_MODE_NORMAL  0x00U            /* Normal boot mode */
#define BOOT_MODE_FORCE   0x01U            /* Force bootloader mode */
#define BOOT_MODE_SAFE    0x02U            /* Safe mode boot */
#define BOOT_MODE_UPDATE  0x03U            /* Update mode boot */

/* =============================================================================
 * REGISTER ACCESS HELPER MACROS
 * =============================================================================
 */

/* UART register access */
#define UART_REG(base, reg) REG32((base) + (reg))

/* GPIO register access */
#define GPIO_REG(base, reg) REG32((base) + (reg))

/* Timer register access */
#define TIMER_REG(base, reg) REG32((base) + (reg))

/* Clock register access */
#define CLK_REG(base, reg) REG32((base) + (reg))

/* Memory controller register access */
#define MEM_REG(base, reg) REG32((base) + (reg))

/* Cache controller register access */
#define CACHE_REG(base, reg) REG32((base) + (reg))

/* Interrupt controller register access */
#define INTC_REG(base, reg) REG32((base) + (reg))

/* Power management register access */
#define PWR_REG(base, reg) REG32((base) + (reg))

/* System control register access */
#define SYSCON_REG(base, reg) REG32((base) + (reg))

/* =============================================================================
 * APPLICATION HEADER STRUCTURE
 * =============================================================================
 */

typedef struct {
    uint32_t signature;     /* Application signature */
    uint32_t entry_point;   /* Application entry point */
    uint32_t size;          /* Application size in bytes */
    uint32_t checksum;      /* Application checksum */
    uint32_t version;       /* Application version */
    uint32_t reserved[11];  /* Reserved for future use */
} app_header_t;

/* Compile-time checks */
_Static_assert(sizeof(app_header_t) == 64, "Application header must be 64 bytes");

#endif /* MCU32X_HARDWARE_H */