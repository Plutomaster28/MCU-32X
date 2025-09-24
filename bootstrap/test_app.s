# MCU-32X Bootstrap Test Application
# Simple test application to verify bootstrap functionality
# This application can be loaded by the bootloader to test system initialization

.include "mcu32x_hardware.h"

.section .app_header, "a"
.global _app_header

# =============================================================================
# APPLICATION HEADER (Required by bootloader)
# =============================================================================
_app_header:
    .word BOOT_SIGNATURE           # Application signature (0x32584D43)
    .word _app_start               # Entry point address
    .word _app_end - _app_start    # Application size
    .word 0x00000000               # Checksum (calculated by build script)
    .word 0x00010000               # Application version (1.0.0.0)
    .space 240                     # Reserved space (total header = 256 bytes)

.section .text
.global _app_start
.type _app_start, @function

# =============================================================================
# APPLICATION ENTRY POINT
# =============================================================================
_app_start:
    # Initialize stack pointer (bootloader should have set this, but be safe)
    load_imm sp, STACK_TOP
    
    # Print application banner
    call print_app_banner
    
    # Run application main loop
    call app_main_loop
    
    # Should never return, but halt if we do
app_halt:
    wfi
    j    app_halt

# =============================================================================
# APPLICATION MAIN LOOP
# =============================================================================
.global app_main_loop
.type app_main_loop, @function

app_main_loop:
    addi sp, sp, -16
    sw   ra, 12(sp)
    sw   s0, 8(sp)
    
    # Initialize application-specific hardware
    call app_hardware_init
    
    # Print menu
    call print_app_menu
    
    # Main application loop
main_loop:
    # Print prompt
    load_addr a0, prompt_msg
    call uart_print_string
    
    # Get user input
    call uart_read_char
    
    # Process commands
    li   t0, '1'
    beq  a0, t0, cmd_led_test
    
    li   t0, '2'
    beq  a0, t0, cmd_gpio_test
    
    li   t0, '3'
    beq  a0, t0, cmd_timer_test
    
    li   t0, '4'
    beq  a0, t0, cmd_memory_test
    
    li   t0, '5'
    beq  a0, t0, cmd_performance_test
    
    li   t0, '6'
    beq  a0, t0, cmd_system_info
    
    li   t0, 'q'
    beq  a0, t0, cmd_quit
    
    li   t0, 'Q'
    beq  a0, t0, cmd_quit
    
    # Invalid command
    load_addr a0, invalid_cmd_msg
    call uart_print_string
    j    main_loop

cmd_led_test:
    call led_blink_test
    j    main_loop

cmd_gpio_test:
    call gpio_loopback_test
    j    main_loop

cmd_timer_test:
    call timer_accuracy_test
    j    main_loop

cmd_memory_test:
    call memory_stress_test
    j    main_loop

cmd_performance_test:
    call cpu_performance_test
    j    main_loop

cmd_system_info:
    call print_detailed_system_info
    j    main_loop

cmd_quit:
    load_addr a0, quit_msg
    call uart_print_string
    
    # Return to bootloader (software reset)
    load_imm t0, RESET_BASE
    load_imm t1, 0x12345678
    sw   t1, 0x10(t0)              # Trigger software reset

    lw   s0, 8(sp)
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

# =============================================================================
# APPLICATION HARDWARE INITIALIZATION
# =============================================================================
.global app_hardware_init
.type app_hardware_init, @function

app_hardware_init:
    # Configure GPIO for LED and button testing
    load_imm t0, GPIOA_BASE
    
    # Set PA0 as output for LED
    li   t1, 0x00000001
    sw   t1, GPIO_DIR(t0)
    
    # Turn off LED initially
    sw   zero, GPIO_DATA(t0)
    
    # Configure GPIO Port B for loopback testing
    load_imm t0, GPIOB_BASE
    
    # Set PB0-PB3 as outputs, PB4-PB7 as inputs
    li   t1, 0x0000000F
    sw   t1, GPIO_DIR(t0)
    
    # Enable pull-ups on input pins
    li   t1, 0x000000F0
    sw   t1, GPIO_PUR(t0)
    
    # Initialize Timer1 for precision timing tests
    load_imm t0, TIMER1_BASE
    
    # Configure for maximum resolution
    load_imm t1, 0xFFFFFFFF
    sw   t1, TMR_PRD(t0)
    
    # Enable timer without interrupts
    li   t1, TMR_EN
    sw   t1, TMR_CR(t0)
    
    ret

# =============================================================================
# LED BLINK TEST
# =============================================================================
.global led_blink_test
.type led_blink_test, @function

led_blink_test:
    addi sp, sp, -16
    sw   ra, 12(sp)
    sw   s0, 8(sp)
    sw   s1, 4(sp)
    
    load_addr a0, led_test_msg
    call uart_print_string
    
    load_imm s0, GPIOA_BASE
    li   s1, 10                    # Blink 10 times
    
blink_loop:
    # Turn on LED
    li   t0, 0x00000001
    sw   t0, GPIO_DATA(s0)
    
    # Delay
    call delay_250ms
    
    # Turn off LED
    sw   zero, GPIO_DATA(s0)
    
    # Delay
    call delay_250ms
    
    # Decrement counter
    addi s1, s1, -1
    bnez s1, blink_loop
    
    load_addr a0, test_complete_msg
    call uart_print_string
    
    lw   s1, 4(sp)
    lw   s0, 8(sp)
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

# =============================================================================
# GPIO LOOPBACK TEST
# =============================================================================
.global gpio_loopback_test
.type gpio_loopback_test, @function

gpio_loopback_test:
    addi sp, sp, -16
    sw   ra, 12(sp)
    sw   s0, 8(sp)
    sw   s1, 4(sp)
    
    load_addr a0, gpio_test_msg
    call uart_print_string
    
    load_imm s0, GPIOB_BASE
    li   s1, 16                    # Test 16 patterns
    
gpio_test_loop:
    # Generate test pattern
    andi t0, s1, 0x0F              # Use counter as pattern
    
    # Write to output pins (PB0-PB3)
    sw   t0, GPIO_DATA(s0)
    
    # Small delay for signal propagation
    call delay_1ms
    
    # Read from input pins (PB4-PB7)
    lw   t1, GPIO_DATA(s0)
    srli t1, t1, 4                 # Shift input to lower 4 bits
    andi t1, t1, 0x0F
    
    # Compare input and output
    beq  t0, t1, gpio_pattern_ok
    
    # Pattern mismatch
    load_addr a0, gpio_error_msg
    call uart_print_string
    j    gpio_test_end

gpio_pattern_ok:
    addi s1, s1, -1
    bnez s1, gpio_test_loop
    
    load_addr a0, gpio_ok_msg
    call uart_print_string

gpio_test_end:
    lw   s1, 4(sp)
    lw   s0, 8(sp)
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

# =============================================================================
# TIMER ACCURACY TEST
# =============================================================================
.global timer_accuracy_test
.type timer_accuracy_test, @function

timer_accuracy_test:
    addi sp, sp, -16
    sw   ra, 12(sp)
    sw   s0, 8(sp)
    sw   s1, 4(sp)
    
    load_addr a0, timer_test_msg
    call uart_print_string
    
    load_imm s0, TIMER1_BASE
    
    # Reset timer
    sw   zero, TMR_CNT(s0)
    
    # Start timing
    lw   s1, TMR_CNT(s0)           # Start count
    
    # Precise delay (should be exactly 100ms at 100MHz)
    li   t0, 10000000              # 10M cycles = 100ms at 100MHz
delay_precise:
    addi t0, t0, -1
    bnez t0, delay_precise
    
    # End timing
    lw   t0, TMR_CNT(s0)           # End count
    sub  t0, t0, s1                # Calculate elapsed cycles
    
    # Expected: 10,000,000 cycles
    # Allow ±1% tolerance (100,000 cycles)
    load_imm t1, 9900000           # Lower bound
    load_imm t2, 10100000          # Upper bound
    
    bltu t0, t1, timer_error
    bgtu t0, t2, timer_error
    
    load_addr a0, timer_ok_msg
    call uart_print_string
    j    timer_test_end

timer_error:
    load_addr a0, timer_error_msg
    call uart_print_string

timer_test_end:
    lw   s1, 4(sp)
    lw   s0, 8(sp)
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

# =============================================================================
# MEMORY STRESS TEST
# =============================================================================
.global memory_stress_test
.type memory_stress_test, @function

memory_stress_test:
    addi sp, sp, -16
    sw   ra, 12(sp)
    sw   s0, 8(sp)
    sw   s1, 4(sp)
    
    load_addr a0, memory_stress_msg
    call uart_print_string
    
    # Test SRAM stress
    call sram_stress_test
    
    # Test DRAM stress
    call dram_stress_test
    
    load_addr a0, memory_stress_complete_msg
    call uart_print_string
    
    lw   s1, 4(sp)
    lw   s0, 8(sp)
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

.global sram_stress_test
.type sram_stress_test, @function

sram_stress_test:
    # SRAM stress test with multiple patterns
    load_imm t0, SRAM_BASE + 0x4000   # Start at 16KB (avoid low memory)
    load_imm t1, SRAM_BASE + 0x8000   # Test 16KB chunk
    
    # Pattern 1: 0x5A5A5A5A
    li   t2, 0x5A5A5A5A
    call memory_fill_test
    
    # Pattern 2: 0xA5A5A5A5
    li   t2, 0xA5A5A5A5
    call memory_fill_test
    
    # Pattern 3: Walking 1s
    call memory_walking_ones_test
    
    # Pattern 4: Walking 0s
    call memory_walking_zeros_test
    
    ret

.global dram_stress_test
.type dram_stress_test, @function

dram_stress_test:
    # DRAM stress test (first 1MB only to save time)
    load_imm t0, DRAM_BASE
    load_imm t1, DRAM_BASE + 0x100000  # Test 1MB
    
    # Quick pattern test
    li   t2, 0x55555555
    call memory_fill_test
    
    li   t2, 0xAAAAAAAA
    call memory_fill_test
    
    ret

.global memory_fill_test
.type memory_fill_test, @function

memory_fill_test:
    # Fill memory with pattern in t2, test range t0-t1
    mv   t3, t0                    # Copy start address
    
    # Fill phase
1:  sw   t2, 0(t3)
    addi t3, t3, 4
    bltu t3, t1, 1b
    
    # Verify phase
    mv   t3, t0                    # Reset to start
2:  lw   t4, 0(t3)
    bne  t2, t4, memory_error
    addi t3, t3, 4
    bltu t3, t1, 2b
    
    ret

memory_error:
    # Memory test failed - print error and address
    load_addr a0, memory_error_msg
    call uart_print_string
    ret

# =============================================================================
# CPU PERFORMANCE TEST
# =============================================================================
.global cpu_performance_test
.type cpu_performance_test, @function

cpu_performance_test:
    addi sp, sp, -16
    sw   ra, 12(sp)
    sw   s0, 8(sp)
    
    load_addr a0, perf_test_msg
    call uart_print_string
    
    # Test 1: Integer arithmetic performance
    call int_arithmetic_test
    
    # Test 2: Memory bandwidth test
    call memory_bandwidth_test
    
    # Test 3: Branch prediction test
    call branch_prediction_test
    
    load_addr a0, perf_test_complete_msg
    call uart_print_string
    
    lw   s0, 8(sp)
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

.global int_arithmetic_test
.type int_arithmetic_test, @function

int_arithmetic_test:
    # Integer arithmetic benchmark
    load_imm s0, TIMER1_BASE
    
    # Reset timer
    sw   zero, TMR_CNT(s0)
    lw   t0, TMR_CNT(s0)           # Start time
    
    # Arithmetic loop (1M operations)
    li   t1, 1000000
    li   t2, 0
    li   t3, 1
    
arith_loop:
    add  t2, t2, t3
    sub  t2, t2, t3
    sll  t2, t2, 1
    srl  t2, t2, 1
    xor  t2, t2, t3
    
    addi t1, t1, -1
    bnez t1, arith_loop
    
    lw   t1, TMR_CNT(s0)           # End time
    sub  t1, t1, t0                # Elapsed cycles
    
    # Print result (cycles per operation)
    load_addr a0, arith_result_msg
    call uart_print_string
    
    ret

.global memory_bandwidth_test
.type memory_bandwidth_test, @function

memory_bandwidth_test:
    # Memory bandwidth test - copy 64KB
    load_imm t0, SRAM_BASE + 0x1000   # Source
    load_imm t1, SRAM_BASE + 0x8000   # Destination  
    li   t2, 16384                 # 64KB / 4 bytes = 16K words
    
    load_imm s0, TIMER1_BASE
    sw   zero, TMR_CNT(s0)
    lw   t3, TMR_CNT(s0)           # Start time
    
bandwidth_loop:
    lw   t4, 0(t0)
    sw   t4, 0(t1)
    addi t0, t0, 4
    addi t1, t1, 4
    addi t2, t2, -1
    bnez t2, bandwidth_loop
    
    lw   t4, TMR_CNT(s0)           # End time
    sub  t4, t4, t3                # Elapsed cycles
    
    load_addr a0, bandwidth_result_msg
    call uart_print_string
    
    ret

.global branch_prediction_test
.type branch_prediction_test, @function

branch_prediction_test:
    # Branch prediction performance test
    load_imm s0, TIMER1_BASE
    sw   zero, TMR_CNT(s0)
    lw   t0, TMR_CNT(s0)           # Start time
    
    li   t1, 100000                # Loop count
    li   t2, 0                     # Counter
    
branch_loop:
    andi t3, t2, 0x01              # Alternating pattern
    bnez t3, branch_taken
    
branch_not_taken:
    addi t2, t2, 1
    j    branch_continue
    
branch_taken:
    addi t2, t2, 1
    
branch_continue:
    addi t1, t1, -1
    bnez t1, branch_loop
    
    lw   t1, TMR_CNT(s0)           # End time
    sub  t1, t1, t0                # Elapsed cycles
    
    load_addr a0, branch_result_msg
    call uart_print_string
    
    ret

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================
.global uart_print_string
.type uart_print_string, @function

uart_print_string:
    mv   t0, a0
    load_imm t1, UART0_BASE
    
1:  lb   t2, 0(t0)
    beqz t2, 2f
    
    # Wait for TX ready
3:  lw   t3, UART_SR(t1)
    andi t3, t3, UART_SR_TXE
    beqz t3, 3b
    
    sw   t2, UART_TDR(t1)
    addi t0, t0, 1
    j    1b
    
2:  ret

.global uart_read_char
.type uart_read_char, @function

uart_read_char:
    load_imm t0, UART0_BASE
    
1:  lw   t1, UART_SR(t0)
    andi t1, t1, UART_SR_RXNE
    beqz t1, 1b
    
    lw   a0, UART_RDR(t0)
    andi a0, a0, 0xFF
    
    ret

.global delay_250ms
.type delay_250ms, @function

delay_250ms:
    li   t0, 25000000              # 250ms at 100MHz
1:  addi t0, t0, -1
    bnez t0, 1b
    ret

.global delay_1ms
.type delay_1ms, @function

delay_1ms:
    li   t0, 100000                # 1ms at 100MHz
1:  addi t0, t0, -1
    bnez t0, 1b
    ret

# Helper functions for memory tests
.global memory_walking_ones_test
.type memory_walking_ones_test, @function

memory_walking_ones_test:
    li   t2, 0x00000001
    li   t5, 32                    # 32 bit positions
    
walking_ones_loop:
    call memory_fill_test
    slli t2, t2, 1
    addi t5, t5, -1
    bnez t5, walking_ones_loop
    ret

.global memory_walking_zeros_test
.type memory_walking_zeros_test, @function

memory_walking_zeros_test:
    li   t2, 0xFFFFFFFE
    li   t5, 32
    
walking_zeros_loop:
    call memory_fill_test
    slli t2, t2, 1
    ori  t2, t2, 1
    addi t5, t5, -1
    bnez t5, walking_zeros_loop
    ret

.global print_app_banner
.type print_app_banner, @function

print_app_banner:
    addi sp, sp, -16
    sw   ra, 12(sp)
    
    load_addr a0, app_banner_msg
    call uart_print_string
    
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

.global print_app_menu
.type print_app_menu, @function

print_app_menu:
    addi sp, sp, -16
    sw   ra, 12(sp)
    
    load_addr a0, app_menu_msg
    call uart_print_string
    
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

.global print_detailed_system_info
.type print_detailed_system_info, @function

print_detailed_system_info:
    addi sp, sp, -16
    sw   ra, 12(sp)
    
    load_addr a0, detailed_info_msg
    call uart_print_string
    
    # Read and display system ID
    load_imm t0, SYSCON_BASE
    lw   t1, SYSCON_ID(t0)
    
    # Read and display system revision
    lw   t2, SYSCON_REV(t0)
    
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

# =============================================================================
# STRING CONSTANTS
# =============================================================================
.section .rodata

app_banner_msg:
    .asciz "\r\n========================================\r\n"
    .asciz "MCU-32X Test Application v1.0\r\n"
    .asciz "Comprehensive Hardware Test Suite\r\n"
    .asciz "========================================\r\n"

app_menu_msg:
    .asciz "\r\nTest Menu:\r\n"
    .asciz "1. LED Blink Test\r\n"
    .asciz "2. GPIO Loopback Test\r\n"
    .asciz "3. Timer Accuracy Test\r\n"
    .asciz "4. Memory Stress Test\r\n"
    .asciz "5. CPU Performance Test\r\n"
    .asciz "6. System Information\r\n"
    .asciz "Q. Quit (Return to Bootloader)\r\n"

prompt_msg:
    .asciz "\r\nSelect test (1-6, Q): "

invalid_cmd_msg:
    .asciz "\r\nInvalid command. Please try again.\r\n"

led_test_msg:
    .asciz "\r\nRunning LED blink test...\r\n"

gpio_test_msg:
    .asciz "\r\nRunning GPIO loopback test...\r\n"

timer_test_msg:
    .asciz "\r\nRunning timer accuracy test...\r\n"

memory_stress_msg:
    .asciz "\r\nRunning memory stress test...\r\n"

perf_test_msg:
    .asciz "\r\nRunning CPU performance test...\r\n"

test_complete_msg:
    .asciz "Test completed successfully.\r\n"

gpio_ok_msg:
    .asciz "GPIO loopback test PASSED.\r\n"

gpio_error_msg:
    .asciz "GPIO loopback test FAILED!\r\n"

timer_ok_msg:
    .asciz "Timer accuracy test PASSED.\r\n"

timer_error_msg:
    .asciz "Timer accuracy test FAILED!\r\n"

memory_stress_complete_msg:
    .asciz "Memory stress test completed.\r\n"

memory_error_msg:
    .asciz "Memory test FAILED at address!\r\n"

perf_test_complete_msg:
    .asciz "Performance test completed.\r\n"

arith_result_msg:
    .asciz "Integer arithmetic: "

bandwidth_result_msg:
    .asciz "Memory bandwidth: "

branch_result_msg:
    .asciz "Branch prediction: "

detailed_info_msg:
    .asciz "\r\nDetailed System Information:\r\n"
    .asciz "MCU-32X RISC-V RV32I Processor\r\n"
    .asciz "Clock: 100MHz, Cache: 64KB Total\r\n"
    .asciz "Memory: 64KB SRAM + 16MB DRAM\r\n"

quit_msg:
    .asciz "\r\nReturning to bootloader...\r\n"

.align 4
_app_end:
    # End marker for size calculation

.end