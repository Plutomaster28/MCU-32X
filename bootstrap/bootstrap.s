# MCU-32X Bootstrap Loader
# Complete system initialization and bootloader
# Compatible with MCU-32X RISC-V RV32I architecture

.section .vectors, "ax"
.global _vectors
.global _start

# =============================================================================
# INTERRUPT VECTOR TABLE
# =============================================================================
# MCU-32X requires 256-byte aligned vector table for fast dispatch

.align 8
_vectors:
    j    _start                     # Reset vector (0x00)
    j    nmi_handler               # NMI handler (0x04)
    j    hardfault_handler         # Hard fault (0x08)
    j    memfault_handler          # Memory fault (0x0C)
    j    busfault_handler          # Bus fault (0x10)
    j    usagefault_handler        # Usage fault (0x14)
    j    reserved_handler          # Reserved (0x18)
    j    reserved_handler          # Reserved (0x1C)
    j    reserved_handler          # Reserved (0x20)
    j    reserved_handler          # Reserved (0x24)
    j    svcall_handler            # SVCall (0x28)
    j    reserved_handler          # Reserved (0x2C)
    j    reserved_handler          # Reserved (0x30)
    j    pendsv_handler            # PendSV (0x34)
    j    systick_handler           # SysTick (0x38)
    
    # External interrupts (0x3C - 0xFC)
    j    uart0_handler             # UART0 interrupt (0x3C)
    j    uart1_handler             # UART1 interrupt (0x40)
    j    spi0_handler              # SPI0 interrupt (0x44)
    j    spi1_handler              # SPI1 interrupt (0x48)
    j    i2c0_handler              # I2C0 interrupt (0x4C)
    j    i2c1_handler              # I2C1 interrupt (0x50)
    j    gpio_handler              # GPIO interrupt (0x54)
    j    timer0_handler            # Timer0 interrupt (0x58)
    j    timer1_handler            # Timer1 interrupt (0x5C)
    j    timer2_handler            # Timer2 interrupt (0x60)
    j    dma_handler               # DMA interrupt (0x64)
    j    adc_handler               # ADC interrupt (0x68)
    j    rtc_handler               # RTC interrupt (0x6C)
    j    wdt_handler               # Watchdog timer (0x70)
    j    flash_handler             # Flash controller (0x74)
    j    ecc_handler               # ECC error (0x78)
    
    # Fill remaining vectors with default handler
    .rept 34
    j    default_handler
    .endr

# =============================================================================
# BOOTSTRAP ENTRY POINT
# =============================================================================
.section .startup, "ax"
.global _start
.type _start, @function

_start:
    # Disable interrupts during initialization
    csrci mstatus, 0x8             # Clear MIE bit in mstatus
    
    # Initialize stack pointer to end of SRAM
    li   sp, 0x20010000            # 64KB SRAM: 0x20000000 + 0x10000
    
    # Initialize global pointer for efficient data access
    .option push
    .option norelax
    la   gp, __global_pointer$     # Set global pointer
    .option pop
    
    # Clear BSS section
    la   t0, _bss_start            # Load BSS start address
    la   t1, _bss_end              # Load BSS end address
    j    2f                        # Jump to BSS clear check
    
1:  sw   zero, 0(t0)              # Clear word
    addi t0, t0, 4                 # Advance pointer
2:  bltu t0, t1, 1b               # Continue if not done
    
    # Copy initialized data from ROM to RAM
    la   t0, _data_load_start      # Source address (in ROM)
    la   t1, _data_start           # Destination address (in RAM)
    la   t2, _data_end             # End of data section
    j    4f                        # Jump to data copy check
    
3:  lw   t3, 0(t0)                # Load word from ROM
    sw   t3, 0(t1)                # Store word to RAM
    addi t0, t0, 4                 # Advance source pointer
    addi t1, t1, 4                 # Advance destination pointer
4:  bltu t1, t2, 3b               # Continue if not done
    
    # Initialize system hardware
    call system_init
    
    # Jump to main application or bootloader menu
    call bootloader_main
    
    # Should never reach here, but halt if we do
halt_loop:
    wfi                            # Wait for interrupt
    j    halt_loop

# =============================================================================
# SYSTEM INITIALIZATION
# =============================================================================
.section .text
.global system_init
.type system_init, @function

system_init:
    # Save return address
    addi sp, sp, -16
    sw   ra, 12(sp)
    sw   s0, 8(sp)
    
    # Initialize clock system
    call clock_init
    
    # Initialize memory subsystem
    call memory_init
    
    # Initialize cache subsystem
    call cache_init
    
    # Initialize interrupt controller
    call interrupt_init
    
    # Initialize UART for debug output
    call uart_init
    
    # Initialize GPIO
    call gpio_init
    
    # Initialize timers
    call timer_init
    
    # Print boot banner
    call print_boot_banner
    
    # Restore registers and return
    lw   ra, 12(sp)
    lw   s0, 8(sp)
    addi sp, sp, 16
    ret

# =============================================================================
# CLOCK INITIALIZATION
# =============================================================================
.global clock_init
.type clock_init, @function

clock_init:
    # MCU-32X clock initialization
    # Configure PLL for 100MHz operation from 25MHz crystal
    
    # PLL control register base
    li   t0, 0x40001000            # Clock control base
    
    # Configure PLL multiplier: 25MHz * 4 = 100MHz
    li   t1, 0x00000403            # PLL enable, multiply by 4, divide by 1
    sw   t1, 0x00(t0)              # Write to PLL control register
    
    # Wait for PLL to lock
1:  lw   t1, 0x04(t0)             # Read PLL status
    andi t1, t1, 0x01              # Check lock bit
    beqz t1, 1b                    # Wait for lock
    
    # Switch system clock to PLL
    li   t1, 0x00000001            # Select PLL as system clock
    sw   t1, 0x08(t0)              # Write to clock select register
    
    # Configure peripheral clocks
    li   t1, 0x0000FFFF            # Enable all peripheral clocks
    sw   t1, 0x0C(t0)              # Write to peripheral clock enable
    
    ret

# =============================================================================
# MEMORY SUBSYSTEM INITIALIZATION
# =============================================================================
.global memory_init
.type memory_init, @function

memory_init:
    # Initialize external DRAM controller
    li   t0, 0x40002000            # Memory controller base
    
    # Configure DRAM timing for 100MHz operation
    # tRAS=45ns, tRP=15ns, tRCD=15ns, CL=3
    li   t1, 0x12345678            # DRAM timing configuration
    sw   t1, 0x00(t0)              # DRAM timing register
    
    # Configure DRAM geometry (16MB, 32-bit width)
    li   t1, 0x00001000            # 16MB DRAM configuration
    sw   t1, 0x04(t0)              # DRAM config register
    
    # Enable DRAM controller
    li   t1, 0x00000001            # Enable DRAM
    sw   t1, 0x08(t0)              # DRAM control register
    
    # Wait for DRAM initialization complete
1:  lw   t1, 0x0C(t0)             # Read DRAM status
    andi t1, t1, 0x01              # Check ready bit
    beqz t1, 1b                    # Wait for ready
    
    # Perform DRAM test
    call dram_test
    
    ret

# =============================================================================
# CACHE INITIALIZATION
# =============================================================================
.global cache_init
.type cache_init, @function

cache_init:
    # Initialize L1 caches (32KB instruction + 32KB data)
    
    # Instruction cache initialization
    li   t0, 0x10000000            # I-cache control base
    
    # Invalidate entire instruction cache
    li   t1, 0x00000002            # Invalidate all
    sw   t1, 0x00(t0)              # Write to I-cache control
    
    # Wait for invalidation complete
1:  lw   t1, 0x04(t0)             # Read I-cache status
    andi t1, t1, 0x02              # Check invalidation done
    bnez t1, 1b                    # Wait while invalidating
    
    # Enable instruction cache
    li   t1, 0x00000001            # Enable I-cache
    sw   t1, 0x00(t0)              # Write to I-cache control
    
    # Data cache initialization
    li   t0, 0x10008000            # D-cache control base
    
    # Invalidate entire data cache
    li   t1, 0x00000002            # Invalidate all
    sw   t1, 0x00(t0)              # Write to D-cache control
    
    # Wait for invalidation complete
1:  lw   t1, 0x04(t0)             # Read D-cache status
    andi t1, t1, 0x02              # Check invalidation done
    bnez t1, 1b                    # Wait while invalidating
    
    # Enable data cache
    li   t1, 0x00000001            # Enable D-cache
    sw   t1, 0x00(t0)              # Write to D-cache control
    
    ret

# =============================================================================
# INTERRUPT CONTROLLER INITIALIZATION
# =============================================================================
.global interrupt_init
.type interrupt_init, @function

interrupt_init:
    # Initialize MCU-32X interrupt controller
    li   t0, 0x80000000            # Interrupt controller base
    
    # Disable all interrupts initially
    sw   zero, 0x00(t0)            # Interrupt enable register
    
    # Clear all pending interrupts
    li   t1, 0xFFFFFFFF
    sw   t1, 0x04(t0)              # Interrupt clear register
    
    # Set default interrupt priorities
    li   t1, 0x76543210            # Priority configuration
    sw   t1, 0x08(t0)              # Priority register 0
    sw   t1, 0x0C(t0)              # Priority register 1
    
    # Configure vector table base address
    la   t1, _vectors              # Load vector table address
    csrw mtvec, t1                 # Set machine trap vector
    
    # Enable machine-mode interrupts
    li   t1, 0x00000800            # Enable machine external interrupts
    csrw mie, t1                   # Set machine interrupt enable
    
    ret

# =============================================================================
# UART INITIALIZATION FOR DEBUG OUTPUT
# =============================================================================
.global uart_init
.type uart_init, @function

uart_init:
    # Initialize UART0 for 115200 baud at 100MHz system clock
    li   t0, 0x40000000            # UART0 base address
    
    # Calculate baud rate divisor: 100MHz / (16 * 115200) = 54.25 ≈ 54
    li   t1, 54                    # Baud rate divisor
    sw   t1, 0x08(t0)              # Baud rate register
    
    # Configure UART: 8N1, no flow control
    li   t1, 0x00000003            # 8 data bits, 1 stop bit, no parity
    sw   t1, 0x00(t0)              # Line control register
    
    # Enable UART transmitter and receiver
    li   t1, 0x00000003            # Enable TX and RX
    sw   t1, 0x04(t0)              # Control register
    
    # Clear UART FIFOs
    li   t1, 0x00000006            # Clear TX and RX FIFOs
    sw   t1, 0x0C(t0)              # FIFO control register
    
    ret

# =============================================================================
# GPIO INITIALIZATION
# =============================================================================
.global gpio_init
.type gpio_init, @function

gpio_init:
    # Initialize GPIO ports for basic I/O
    
    # Configure Port A (PA0-PA7)
    li   t0, 0x40000100            # GPIO Port A base
    
    # Set PA0 as output (LED), others as inputs
    li   t1, 0x00000001            # PA0 output, PA1-PA7 inputs
    sw   t1, 0x04(t0)              # Direction register
    
    # Enable pull-ups on input pins
    li   t1, 0x000000FE            # Pull-ups on PA1-PA7
    sw   t1, 0x08(t0)              # Pull-up enable register
    
    # Configure Port B (PB0-PB7) - all inputs with pull-ups
    li   t0, 0x40000200            # GPIO Port B base
    sw   zero, 0x04(t0)            # All inputs
    li   t1, 0x000000FF            # Pull-ups on all pins
    sw   t1, 0x08(t0)              # Pull-up enable register
    
    ret

# =============================================================================
# TIMER INITIALIZATION
# =============================================================================
.global timer_init
.type timer_init, @function

timer_init:
    # Initialize system timer for 1ms tick
    li   t0, 0x40003000            # Timer0 base address
    
    # Configure timer for 1ms period (100MHz / 1000 = 100000 cycles)
    li   t1, 100000                # Timer period
    sw   t1, 0x04(t0)              # Period register
    
    # Enable timer interrupt and auto-reload
    li   t1, 0x00000003            # Enable timer and interrupt
    sw   t1, 0x00(t0)              # Control register
    
    ret

# =============================================================================
# DRAM MEMORY TEST
# =============================================================================
.global dram_test
.type dram_test, @function

dram_test:
    # Simple DRAM test - write/read patterns
    addi sp, sp, -16
    sw   ra, 12(sp)
    
    # Test start address (external DRAM)
    li   t0, 0x40000000            # DRAM base address
    li   t1, 0x40100000            # Test 1MB
    
    # Write test pattern
    li   t2, 0x5A5A5A5A            # Test pattern
1:  sw   t2, 0(t0)                # Write pattern
    addi t0, t0, 4                 # Next address
    bltu t0, t1, 1b               # Continue until end
    
    # Read and verify test pattern
    li   t0, 0x40000000            # Reset to start
2:  lw   t3, 0(t0)                # Read data
    bne  t2, t3, dram_error       # Check for mismatch
    addi t0, t0, 4                 # Next address
    bltu t0, t1, 2b               # Continue until end
    
    # DRAM test passed
    la   a0, dram_ok_msg
    call uart_print_string
    
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

dram_error:
    # DRAM test failed
    la   a0, dram_error_msg
    call uart_print_string
    
    # Halt system on DRAM failure
    j    halt_loop

# =============================================================================
# BOOTLOADER MAIN FUNCTION
# =============================================================================
.global bootloader_main
.type bootloader_main, @function

bootloader_main:
    addi sp, sp, -16
    sw   ra, 12(sp)
    
    # Print bootloader menu
    call print_bootloader_menu
    
    # Check for application in flash
    call check_application_signature
    beqz a0, bootloader_menu       # No valid app, show menu
    
    # Check for boot delay (user can interrupt)
    call boot_delay_check
    bnez a0, bootloader_menu       # User interrupted, show menu
    
    # Boot application
    call boot_application
    
bootloader_menu:
    # Interactive bootloader menu
    call handle_bootloader_menu
    
    # Should not return
    j    halt_loop

# =============================================================================
# BOOT DELAY WITH USER INTERRUPT CHECK
# =============================================================================
.global boot_delay_check
.type boot_delay_check, @function

boot_delay_check:
    addi sp, sp, -16
    sw   ra, 12(sp)
    sw   s0, 8(sp)
    
    # Print countdown message
    la   a0, boot_countdown_msg
    call uart_print_string
    
    # 3-second countdown
    li   s0, 3                     # Countdown from 3
    
countdown_loop:
    # Print current count
    mv   a0, s0
    call uart_print_digit
    
    la   a0, countdown_dot_msg
    call uart_print_string
    
    # Delay 1 second while checking for input
    li   t0, 1000                  # 1000ms delay
delay_loop:
    call uart_check_input
    bnez a0, user_interrupt        # User pressed key
    
    call delay_1ms
    addi t0, t0, -1
    bnez t0, delay_loop
    
    # Decrement counter
    addi s0, s0, -1
    bnez s0, countdown_loop
    
    # No user interrupt - return 0
    li   a0, 0
    j    boot_delay_exit

user_interrupt:
    # User interrupted - return 1
    li   a0, 1
    
    la   a0, interrupt_msg
    call uart_print_string

boot_delay_exit:
    lw   s0, 8(sp)
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

# =============================================================================
# CHECK APPLICATION SIGNATURE
# =============================================================================
.global check_application_signature
.type check_application_signature, @function

check_application_signature:
    # Check for valid application at 0x10000 (64KB offset in flash)
    li   t0, 0x00010000            # Application start address
    
    # Load application signature (should be 0x32584D43 = "MCU2")
    lw   t1, 0(t0)                 # Load signature
    li   t2, 0x32584D43            # Expected signature
    
    beq  t1, t2, valid_signature   # Signature matches
    
    # Invalid signature
    li   a0, 0
    ret

valid_signature:
    # Valid signature found
    li   a0, 1
    ret

# =============================================================================
# BOOT APPLICATION
# =============================================================================
.global boot_application
.type boot_application, @function

boot_application:
    # Print boot message
    la   a0, booting_app_msg
    call uart_print_string
    
    # Disable interrupts
    csrci mstatus, 0x8
    
    # Load application entry point
    li   t0, 0x00010000            # Application base
    lw   t1, 4(t0)                 # Load entry point from offset 4
    
    # Set up application environment
    li   sp, 0x20010000            # Reset stack pointer
    
    # Jump to application
    jr   t1

# =============================================================================
# BOOTLOADER MENU HANDLER
# =============================================================================
.global handle_bootloader_menu
.type handle_bootloader_menu, @function

handle_bootloader_menu:
    addi sp, sp, -16
    sw   ra, 12(sp)

menu_loop:
    # Print menu prompt
    la   a0, menu_prompt_msg
    call uart_print_string
    
    # Get user input
    call uart_read_char
    
    # Process menu selection
    li   t0, '1'
    beq  a0, t0, menu_boot_app
    
    li   t0, '2'
    beq  a0, t0, menu_load_app
    
    li   t0, '3'
    beq  a0, t0, menu_system_info
    
    li   t0, '4'
    beq  a0, t0, menu_memory_test
    
    li   t0, '5'
    beq  a0, t0, menu_reset_system
    
    # Invalid selection
    la   a0, invalid_selection_msg
    call uart_print_string
    j    menu_loop

menu_boot_app:
    call boot_application
    j    menu_loop

menu_load_app:
    call load_application_uart
    j    menu_loop

menu_system_info:
    call print_system_info
    j    menu_loop

menu_memory_test:
    call comprehensive_memory_test
    j    menu_loop

menu_reset_system:
    # Software reset
    li   t0, 0x40001000            # Reset controller
    li   t1, 0x12345678            # Reset magic number
    sw   t1, 0x10(t0)              # Trigger reset

    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

# =============================================================================
# UART UTILITY FUNCTIONS
# =============================================================================
.global uart_print_string
.type uart_print_string, @function

uart_print_string:
    # Print null-terminated string in a0
    mv   t0, a0                    # String pointer
    li   t1, 0x40000000            # UART base
    
1:  lb   t2, 0(t0)                # Load character
    beqz t2, 2f                    # End of string
    
    # Wait for TX ready
3:  lw   t3, 0x10(t1)             # Read status register
    andi t3, t3, 0x01              # Check TX ready bit
    beqz t3, 3b                    # Wait while not ready
    
    # Send character
    sw   t2, 0x14(t1)              # Write to TX data register
    
    addi t0, t0, 1                 # Next character
    j    1b                        # Continue
    
2:  ret

.global uart_read_char
.type uart_read_char, @function

uart_read_char:
    li   t0, 0x40000000            # UART base
    
    # Wait for RX data available
1:  lw   t1, 0x10(t0)             # Read status register
    andi t1, t1, 0x02              # Check RX ready bit
    beqz t1, 1b                    # Wait while no data
    
    # Read character
    lw   a0, 0x18(t0)              # Read RX data register
    andi a0, a0, 0xFF              # Mask to 8 bits
    
    ret

.global uart_check_input
.type uart_check_input, @function

uart_check_input:
    li   t0, 0x40000000            # UART base
    
    # Check if RX data is available (non-blocking)
    lw   t1, 0x10(t0)              # Read status register
    andi a0, t1, 0x02              # Return RX ready bit status
    
    ret

.global uart_print_digit
.type uart_print_digit, @function

uart_print_digit:
    # Print single digit (0-9) in a0
    addi a0, a0, '0'               # Convert to ASCII
    
    li   t0, 0x40000000            # UART base
    
    # Wait for TX ready
1:  lw   t1, 0x10(t0)             # Read status register
    andi t1, t1, 0x01              # Check TX ready bit
    beqz t1, 1b                    # Wait while not ready
    
    # Send digit
    sw   a0, 0x14(t0)              # Write to TX data register
    
    ret

# =============================================================================
# DELAY FUNCTIONS
# =============================================================================
.global delay_1ms
.type delay_1ms, @function

delay_1ms:
    # Delay approximately 1ms at 100MHz (100000 cycles)
    li   t0, 100000                # Cycle count for 1ms
    
1:  addi t0, t0, -1               # Decrement counter
    bnez t0, 1b                    # Loop until zero
    
    ret

# =============================================================================
# INTERRUPT HANDLERS
# =============================================================================

# Default interrupt handler
default_handler:
nmi_handler:
hardfault_handler:
memfault_handler:
busfault_handler:
usagefault_handler:
reserved_handler:
svcall_handler:
pendsv_handler:
uart0_handler:
uart1_handler:
spi0_handler:
spi1_handler:
i2c0_handler:
i2c1_handler:
gpio_handler:
timer0_handler:
timer1_handler:
timer2_handler:
dma_handler:
adc_handler:
rtc_handler:
wdt_handler:
flash_handler:
ecc_handler:
    # Save context
    addi sp, sp, -32
    sw   ra, 28(sp)
    sw   t0, 24(sp)
    sw   t1, 20(sp)
    sw   t2, 16(sp)
    sw   a0, 12(sp)
    sw   a1, 8(sp)
    sw   a2, 4(sp)
    sw   a3, 0(sp)
    
    # Handle interrupt (placeholder)
    # Real implementation would dispatch to specific handlers
    
    # Restore context
    lw   a3, 0(sp)
    lw   a2, 4(sp)
    lw   a1, 8(sp)
    lw   a0, 12(sp)
    lw   t2, 16(sp)
    lw   t1, 20(sp)
    lw   t0, 24(sp)
    lw   ra, 28(sp)
    addi sp, sp, 32
    
    mret

# System tick handler (1ms timer)
systick_handler:
    # Increment system tick counter
    # This would be implemented for real-time OS support
    mret

# =============================================================================
# SYSTEM INFORMATION AND DIAGNOSTICS
# =============================================================================
.global print_boot_banner
.type print_boot_banner, @function

print_boot_banner:
    addi sp, sp, -16
    sw   ra, 12(sp)
    
    la   a0, boot_banner_msg
    call uart_print_string
    
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

.global print_bootloader_menu
.type print_bootloader_menu, @function

print_bootloader_menu:
    addi sp, sp, -16
    sw   ra, 12(sp)
    
    la   a0, bootloader_menu_msg
    call uart_print_string
    
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

.global print_system_info
.type print_system_info, @function

print_system_info:
    addi sp, sp, -16
    sw   ra, 12(sp)
    
    la   a0, system_info_msg
    call uart_print_string
    
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

.global comprehensive_memory_test
.type comprehensive_memory_test, @function

comprehensive_memory_test:
    addi sp, sp, -16
    sw   ra, 12(sp)
    
    la   a0, memory_test_msg
    call uart_print_string
    
    # Test SRAM
    call test_sram
    
    # Test DRAM
    call dram_test
    
    la   a0, memory_test_complete_msg
    call uart_print_string
    
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

.global test_sram
.type test_sram, @function

test_sram:
    # Test internal SRAM (avoid stack area)
    li   t0, 0x20000000            # SRAM start
    li   t1, 0x20008000            # Test first 32KB (leave room for stack)
    
    # Walking 1s test
    li   t2, 0x00000001            # Walking 1 pattern
    
1:  sw   t2, 0(t0)                # Write pattern
    lw   t3, 0(t0)                # Read back
    bne  t2, t3, sram_error       # Check for mismatch
    
    slli t2, t2, 1                 # Shift pattern left
    addi t0, t0, 4                 # Next address
    bltu t0, t1, 1b               # Continue test
    
    ret

sram_error:
    la   a0, sram_error_msg
    call uart_print_string
    ret

.global load_application_uart
.type load_application_uart, @function

load_application_uart:
    # Placeholder for UART-based application loading
    # Would implement Intel HEX or binary loading protocol
    addi sp, sp, -16
    sw   ra, 12(sp)
    
    la   a0, load_app_msg
    call uart_print_string
    
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

# =============================================================================
# STRING CONSTANTS
# =============================================================================
.section .rodata

boot_banner_msg:
    .asciz "\r\n\r\n============================================\r\n"
    .asciz "MCU-32X Bootstrap Loader v1.0\r\n"
    .asciz "Copyright (C) 1999 MCU-32X Technologies\r\n"
    .asciz "RISC-V RV32I @ 100MHz, 64KB SRAM, 32KB Cache\r\n"
    .asciz "============================================\r\n\r\n"

bootloader_menu_msg:
    .asciz "\r\nMCU-32X Bootloader Menu:\r\n"
    .asciz "1. Boot Application\r\n"
    .asciz "2. Load Application via UART\r\n"
    .asciz "3. System Information\r\n"
    .asciz "4. Memory Test\r\n"
    .asciz "5. Reset System\r\n"

menu_prompt_msg:
    .asciz "\r\nSelect option (1-5): "

boot_countdown_msg:
    .asciz "Booting application in "

countdown_dot_msg:
    .asciz "."

interrupt_msg:
    .asciz " [INTERRUPTED]\r\n"

booting_app_msg:
    .asciz "Booting application...\r\n"

dram_ok_msg:
    .asciz "DRAM test passed\r\n"

dram_error_msg:
    .asciz "DRAM test FAILED!\r\n"

sram_error_msg:
    .asciz "SRAM test FAILED!\r\n"

system_info_msg:
    .asciz "\r\nMCU-32X System Information:\r\n"
    .asciz "CPU: RISC-V RV32I @ 100MHz\r\n"
    .asciz "Memory: 64KB SRAM + 16MB DRAM\r\n"
    .asciz "Cache: 32KB I-Cache + 32KB D-Cache\r\n"
    .asciz "Flash: 512KB Boot ROM\r\n\r\n"

memory_test_msg:
    .asciz "\r\nRunning comprehensive memory test...\r\n"

memory_test_complete_msg:
    .asciz "Memory test completed\r\n"

load_app_msg:
    .asciz "\r\nReady to receive application via UART...\r\n"
    .asciz "Send Intel HEX file now.\r\n"

invalid_selection_msg:
    .asciz "\r\nInvalid selection. Please try again.\r\n"

.end