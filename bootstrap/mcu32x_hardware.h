# MCU-32X Bootstrap Hardware Definitions
# Hardware register definitions and memory map for bootstrap loader
# This file provides constants and macros for MCU-32X hardware access

.ifndef MCU32X_HARDWARE_H
.set MCU32X_HARDWARE_H, 1

# =============================================================================
# MEMORY MAP DEFINITIONS
# =============================================================================

# Flash Memory Layout
.set FLASH_BASE,        0x00000000      # Flash memory base address
.set FLASH_SIZE,        0x00080000      # 512KB total flash
.set BOOTLOADER_BASE,   0x00000000      # Bootloader area (64KB)
.set BOOTLOADER_SIZE,   0x00010000      # 64KB for bootloader
.set APPLICATION_BASE,  0x00010000      # Application area start
.set APPLICATION_SIZE,  0x00070000      # 448KB for application

# SRAM Memory Layout  
.set SRAM_BASE,         0x20000000      # Internal SRAM base
.set SRAM_SIZE,         0x00010000      # 64KB internal SRAM
.set STACK_TOP,         0x20010000      # Top of SRAM (stack grows down)
.set HEAP_START,        0x20008000      # Heap starts at 32KB mark

# External DRAM Layout
.set DRAM_BASE,         0x40000000      # External DRAM base
.set DRAM_SIZE,         0x01000000      # 16MB external DRAM

# Cache Memory Layout
.set ICACHE_BASE,       0x10000000      # Instruction cache control
.set DCACHE_BASE,       0x10008000      # Data cache control
.set CACHE_SIZE,        0x00008000      # 32KB per cache

# =============================================================================
# PERIPHERAL BASE ADDRESSES
# =============================================================================

# UART Controllers
.set UART0_BASE,        0x40000000      # UART0 base address
.set UART1_BASE,        0x40000080      # UART1 base address

# GPIO Controllers
.set GPIOA_BASE,        0x40000100      # GPIO Port A base
.set GPIOB_BASE,        0x40000200      # GPIO Port B base
.set GPIOC_BASE,        0x40000300      # GPIO Port C base
.set GPIOD_BASE,        0x40000400      # GPIO Port D base

# SPI Controllers
.set SPI0_BASE,         0x40000500      # SPI0 base address
.set SPI1_BASE,         0x40000580      # SPI1 base address

# I2C Controllers
.set I2C0_BASE,         0x40000600      # I2C0 base address
.set I2C1_BASE,         0x40000680      # I2C1 base address

# System Controllers
.set SYSCON_BASE,       0x40001000      # System control base
.set CLOCK_BASE,        0x40001000      # Clock control (within SYSCON)
.set RESET_BASE,        0x40001040      # Reset control (within SYSCON)
.set POWER_BASE,        0x40001080      # Power management (within SYSCON)

# Memory Controllers
.set MEMCON_BASE,       0x40002000      # Memory controller base
.set FLASH_CON_BASE,    0x40002100      # Flash controller base

# Timer Controllers
.set TIMER0_BASE,       0x40003000      # Timer0 base address
.set TIMER1_BASE,       0x40003040      # Timer1 base address
.set TIMER2_BASE,       0x40003080      # Timer2 base address
.set SYSTICK_BASE,      0x400030C0      # System tick timer base

# DMA Controller
.set DMA_BASE,          0x40004000      # DMA controller base

# ADC Controller
.set ADC_BASE,          0x40005000      # ADC controller base

# RTC Controller
.set RTC_BASE,          0x40006000      # RTC controller base

# Watchdog Timer
.set WDT_BASE,          0x40007000      # Watchdog timer base

# Interrupt Controller
.set INTC_BASE,         0x80000000      # Interrupt controller base

# =============================================================================
# UART REGISTER OFFSETS
# =============================================================================

.set UART_LCR,          0x00            # Line control register
.set UART_CR,           0x04            # Control register
.set UART_BRR,          0x08            # Baud rate register
.set UART_FCR,          0x0C            # FIFO control register
.set UART_SR,           0x10            # Status register
.set UART_TDR,          0x14            # Transmit data register
.set UART_RDR,          0x18            # Receive data register
.set UART_IER,          0x1C            # Interrupt enable register

# UART Status Register Bits
.set UART_SR_TXE,       0x01            # Transmit buffer empty
.set UART_SR_RXNE,      0x02            # Receive buffer not empty
.set UART_SR_TC,        0x04            # Transmission complete
.set UART_SR_IDLE,      0x08            # IDLE line detected
.set UART_SR_ORE,       0x10            # Overrun error
.set UART_SR_NF,        0x20            # Noise flag
.set UART_SR_FE,        0x40            # Framing error
.set UART_SR_PE,        0x80            # Parity error

# =============================================================================
# GPIO REGISTER OFFSETS
# =============================================================================

.set GPIO_DATA,         0x00            # Data register
.set GPIO_DIR,          0x04            # Direction register
.set GPIO_PUR,          0x08            # Pull-up register
.set GPIO_PDR,          0x0C            # Pull-down register
.set GPIO_ODR,          0x10            # Open-drain register
.set GPIO_IER,          0x14            # Interrupt enable register
.set GPIO_ISR,          0x18            # Interrupt status register
.set GPIO_ICR,          0x1C            # Interrupt clear register

# =============================================================================
# CLOCK CONTROL REGISTER OFFSETS
# =============================================================================

.set CLK_PLLCR,         0x00            # PLL control register
.set CLK_PLLSR,         0x04            # PLL status register
.set CLK_CLKSEL,        0x08            # Clock select register
.set CLK_CLKEN,         0x0C            # Clock enable register
.set CLK_CLKDIV,        0x10            # Clock divider register

# PLL Control Register Bits
.set CLK_PLL_EN,        0x00000001      # PLL enable
.set CLK_PLL_BYPASS,    0x00000002      # PLL bypass
.set CLK_PLL_MUL_MASK,  0x000000FC      # PLL multiplier mask
.set CLK_PLL_DIV_MASK,  0x00000F00      # PLL divider mask

# PLL Status Register Bits
.set CLK_PLL_LOCK,      0x00000001      # PLL locked

# Clock Select Register Values
.set CLK_SEL_OSC,       0x00000000      # Select oscillator
.set CLK_SEL_PLL,       0x00000001      # Select PLL
.set CLK_SEL_RC,        0x00000002      # Select RC oscillator

# =============================================================================
# MEMORY CONTROLLER REGISTER OFFSETS
# =============================================================================

.set MEM_DRAMCR,        0x00            # DRAM control register
.set MEM_DRAMCFG,       0x04            # DRAM configuration register
.set MEM_DRAMCTL,       0x08            # DRAM control register
.set MEM_DRAMSR,        0x0C            # DRAM status register
.set MEM_DRAMTMG,       0x10            # DRAM timing register

# DRAM Control Register Bits
.set MEM_DRAM_EN,       0x00000001      # DRAM enable
.set MEM_DRAM_INIT,     0x00000002      # DRAM initialization
.set MEM_DRAM_REF_EN,   0x00000004      # Auto refresh enable
.set MEM_DRAM_SELF_REF, 0x00000008      # Self refresh mode

# =============================================================================
# CACHE CONTROLLER REGISTER OFFSETS
# =============================================================================

.set CACHE_CR,          0x00            # Cache control register
.set CACHE_SR,          0x04            # Cache status register
.set CACHE_INV,         0x08            # Cache invalidate register
.set CACHE_CLEAN,       0x0C            # Cache clean register

# Cache Control Register Bits
.set CACHE_EN,          0x00000001      # Cache enable
.set CACHE_INV_ALL,     0x00000002      # Invalidate all
.set CACHE_CLEAN_ALL,   0x00000004      # Clean all
.set CACHE_WT,          0x00000008      # Write-through mode
.set CACHE_WB,          0x00000000      # Write-back mode (default)

# =============================================================================
# INTERRUPT CONTROLLER REGISTER OFFSETS
# =============================================================================

.set INTC_IER,          0x00            # Interrupt enable register
.set INTC_IPR,          0x04            # Interrupt pending register
.set INTC_ICR,          0x08            # Interrupt clear register
.set INTC_IPRIOR0,      0x10            # Interrupt priority register 0
.set INTC_IPRIOR1,      0x14            # Interrupt priority register 1
.set INTC_IPRIOR2,      0x18            # Interrupt priority register 2
.set INTC_IPRIOR3,      0x1C            # Interrupt priority register 3

# =============================================================================
# TIMER REGISTER OFFSETS
# =============================================================================

.set TMR_CR,            0x00            # Timer control register
.set TMR_CNT,           0x04            # Timer counter register
.set TMR_PRD,           0x08            # Timer period register
.set TMR_CMP,           0x0C            # Timer compare register
.set TMR_SR,            0x10            # Timer status register
.set TMR_IER,           0x14            # Timer interrupt enable register

# Timer Control Register Bits
.set TMR_EN,            0x00000001      # Timer enable
.set TMR_IE,            0x00000002      # Interrupt enable
.set TMR_AR,            0x00000004      # Auto-reload enable
.set TMR_URS,           0x00000008      # Update request source
.set TMR_OPM,           0x00000010      # One-pulse mode
.set TMR_DIR,           0x00000020      # Direction (0=up, 1=down)

# =============================================================================
# POWER MANAGEMENT REGISTER OFFSETS
# =============================================================================

.set PWR_CR,            0x00            # Power control register
.set PWR_SR,            0x04            # Power status register
.set PWR_WUCR,          0x08            # Wake-up control register
.set PWR_WUSR,          0x0C            # Wake-up status register

# Power Control Register Bits
.set PWR_LPDS,          0x00000001      # Low-power deep sleep
.set PWR_PDDS,          0x00000002      # Power down deep sleep
.set PWR_CWUF,          0x00000004      # Clear wake-up flag
.set PWR_CSBF,          0x00000008      # Clear standby flag
.set PWR_PVDE,          0x00000010      # Power voltage detector enable
.set PWR_PLS_MASK,      0x000000E0      # PVD level selection mask

# =============================================================================
# SYSTEM CONTROL REGISTER DEFINITIONS
# =============================================================================

# System Control Register
.set SYSCON_CTRL,       0x00            # System control register
.set SYSCON_STAT,       0x04            # System status register
.set SYSCON_ID,         0x08            # System ID register
.set SYSCON_REV,        0x0C            # System revision register

# System Control Bits
.set SYSCON_RST,        0x00000001      # System reset
.set SYSCON_CLK_EN,     0x00000002      # System clock enable
.set SYSCON_DBG_EN,     0x00000004      # Debug enable
.set SYSCON_JTAG_EN,    0x00000008      # JTAG enable

# =============================================================================
# RISC-V CSR DEFINITIONS
# =============================================================================

# Machine-mode CSRs
.set CSR_MSTATUS,       0x300           # Machine status register
.set CSR_MISA,          0x301           # Machine ISA register
.set CSR_MIE,           0x304           # Machine interrupt enable
.set CSR_MTVEC,         0x305           # Machine trap vector base
.set CSR_MSCRATCH,      0x340           # Machine scratch register
.set CSR_MEPC,          0x341           # Machine exception program counter
.set CSR_MCAUSE,        0x342           # Machine trap cause
.set CSR_MTVAL,         0x343           # Machine trap value
.set CSR_MIP,           0x344           # Machine interrupt pending

# Machine Status Register Bits
.set MSTATUS_MIE,       0x00000008      # Machine interrupt enable
.set MSTATUS_MPIE,      0x00000080      # Previous interrupt enable
.set MSTATUS_MPP_MASK,  0x00001800      # Previous privilege mode

# Machine Interrupt Enable Bits
.set MIE_MSIE,          0x00000008      # Machine software interrupt enable
.set MIE_MTIE,          0x00000080      # Machine timer interrupt enable
.set MIE_MEIE,          0x00000800      # Machine external interrupt enable

# =============================================================================
# INTERRUPT VECTOR DEFINITIONS
# =============================================================================

.set IRQ_UART0,         0               # UART0 interrupt
.set IRQ_UART1,         1               # UART1 interrupt
.set IRQ_SPI0,          2               # SPI0 interrupt
.set IRQ_SPI1,          3               # SPI1 interrupt
.set IRQ_I2C0,          4               # I2C0 interrupt
.set IRQ_I2C1,          5               # I2C1 interrupt
.set IRQ_GPIO,          6               # GPIO interrupt
.set IRQ_TIMER0,        7               # Timer0 interrupt
.set IRQ_TIMER1,        8               # Timer1 interrupt
.set IRQ_TIMER2,        9               # Timer2 interrupt
.set IRQ_DMA,           10              # DMA interrupt
.set IRQ_ADC,           11              # ADC interrupt
.set IRQ_RTC,           12              # RTC interrupt
.set IRQ_WDT,           13              # Watchdog timer interrupt
.set IRQ_FLASH,         14              # Flash controller interrupt
.set IRQ_ECC,           15              # ECC error interrupt

# =============================================================================
# UTILITY MACROS
# =============================================================================

# Load 32-bit immediate value (RISC-V doesn't have direct 32-bit immediate)
.macro load_imm reg, value
    lui  \reg, %hi(\value)
    addi \reg, \reg, %lo(\value)
.endm

# Load address macro
.macro load_addr reg, label
    lui  \reg, %hi(\label)
    addi \reg, \reg, %lo(\label)
.endm

# Set bit macro
.macro set_bit reg, bit
    ori  \reg, \reg, (1 << \bit)
.endm

# Clear bit macro
.macro clear_bit reg, bit
    andi \reg, \reg, ~(1 << \bit)
.endm

# Wait for bit set macro
.macro wait_bit_set addr, bit, temp_reg
1:  lw   \temp_reg, (\addr)
    andi \temp_reg, \temp_reg, (1 << \bit)
    beqz \temp_reg, 1b
.endm

# Wait for bit clear macro
.macro wait_bit_clear addr, bit, temp_reg
1:  lw   \temp_reg, (\addr)
    andi \temp_reg, \temp_reg, (1 << \bit)
    bnez \temp_reg, 1b
.endm

# Memory barrier macro (for ordering memory operations)
.macro memory_barrier
    fence
.endm

# Instruction barrier macro
.macro instruction_barrier
    fence.i
.endm

# =============================================================================
# BOOTLOADER CONFIGURATION
# =============================================================================

.set BOOT_SIGNATURE,    0x32584D43      # "MCU2" signature
.set BOOT_VERSION,      0x00010000      # Version 1.0.0.0
.set BOOT_MAGIC,        0x12345678      # Boot magic number

# Application header structure offsets
.set APP_SIGNATURE,     0x00            # Application signature
.set APP_ENTRY_POINT,   0x04            # Application entry point
.set APP_SIZE,          0x08            # Application size
.set APP_CHECKSUM,      0x0C            # Application checksum
.set APP_VERSION,       0x10            # Application version

# Boot modes
.set BOOT_MODE_NORMAL,  0x00            # Normal boot mode
.set BOOT_MODE_FORCE,   0x01            # Force bootloader mode
.set BOOT_MODE_SAFE,    0x02            # Safe mode boot
.set BOOT_MODE_UPDATE,  0x03            # Update mode boot

.endif # MCU32X_HARDWARE_H