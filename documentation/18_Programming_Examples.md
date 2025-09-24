# Chapter 18: Programming Examples
## MCU-32X Technical Reference Manual

---

## 18.1 Basic Programming Examples

This chapter provides comprehensive programming examples demonstrating the capabilities of the MCU-32X processor. These examples showcase both fundamental programming techniques and advanced optimization strategies suitable for desktop computing applications.

### 18.1.1 Hello World Program

```assembly
# Hello World - MCU-32X Assembly Language
# Demonstrates basic I/O and string handling

.section .data
hello_msg:
    .string "Hello, World from MCU-32X!\n"
    .equ msg_len, 29

.section .text
.globl _start

_start:
    # Initialize stack pointer
    lui  x2, %hi(stack_top)
    addi x2, x2, %lo(stack_top)
    
    # Print hello message
    jal  x1, print_string
    
    # Exit program  
    jal  x1, exit_program

print_string:
    # Load message address
    lui  x10, %hi(hello_msg)
    addi x10, x10, %lo(hello_msg)
    
    # Load message length
    addi x11, x0, msg_len
    
    # Call system print function
    addi x17, x0, 4        # System call number for write
    addi x10, x0, 1        # File descriptor (stdout)
    # x11 already contains buffer address
    # x12 already contains length
    ecall                  # System call
    
    jalr x0, 0(x1)        # Return

exit_program:
    addi x17, x0, 1       # System call number for exit
    addi x10, x0, 0       # Exit status
    ecall                 # System call
    # Should not return

.section .bss
.align 4
stack_base:
    .space 4096           # 4KB stack
stack_top:
```

### 18.1.2 Fibonacci Sequence Calculator

```assembly
# Fibonacci Sequence Generator
# Demonstrates recursive and iterative algorithms

.section .data
fib_count: .word 20       # Calculate 20 Fibonacci numbers
results:   .space 80      # Storage for results (20 * 4 bytes)

.section .text
.globl _start

_start:
    # Initialize
    lui  x2, %hi(stack_top)
    addi x2, x2, %lo(stack_top)
    
    # Calculate Fibonacci sequence
    jal  x1, fibonacci_sequence
    
    # Print results
    jal  x1, print_results
    
    # Exit
    jal  x1, exit_program

# Iterative Fibonacci calculation (optimized)
fibonacci_sequence:
    # Save return address
    addi x2, x2, -16      # Allocate stack frame
    sw   x1, 12(x2)       # Save return address
    sw   x8, 8(x2)        # Save frame pointer
    sw   x9, 4(x2)        # Save s1
    sw   x18, 0(x2)       # Save s2
    
    # Setup frame pointer
    addi x8, x2, 16
    
    # Load count and results array
    lw   x9, fib_count     # x9 = count
    lui  x18, %hi(results)
    addi x18, x18, %lo(results)  # x18 = results array
    
    # Handle base cases
    beq  x9, x0, fib_done  # If count == 0, done
    
    # F(0) = 0
    sw   x0, 0(x18)        # results[0] = 0
    addi x10, x0, 1        # index = 1
    beq  x9, x10, fib_done # If count == 1, done
    
    # F(1) = 1  
    addi x11, x0, 1
    sw   x11, 4(x18)       # results[1] = 1
    addi x10, x0, 2        # index = 2
    
    # Initialize for loop
    add  x12, x0, x0       # prev_prev = F(0) = 0
    addi x13, x0, 1        # prev = F(1) = 1
    
fib_loop:
    bge  x10, x9, fib_done # if index >= count, done
    
    # Calculate F(n) = F(n-1) + F(n-2)
    add  x14, x13, x12     # current = prev + prev_prev
    
    # Store result
    slli x15, x10, 2       # offset = index * 4
    add  x16, x18, x15     # address = results + offset
    sw   x14, 0(x16)       # results[index] = current
    
    # Update for next iteration
    add  x12, x13, x0      # prev_prev = prev
    add  x13, x14, x0      # prev = current
    addi x10, x10, 1       # index++
    
    jal  x0, fib_loop      # Continue loop

fib_done:
    # Restore registers
    lw   x18, 0(x2)        # Restore s2
    lw   x9, 4(x2)         # Restore s1  
    lw   x8, 8(x2)         # Restore frame pointer
    lw   x1, 12(x2)        # Restore return address
    addi x2, x2, 16        # Deallocate stack frame
    
    jalr x0, 0(x1)         # Return

print_results:
    # Print Fibonacci sequence results
    # Implementation would depend on system I/O
    jalr x0, 0(x1)         # Return (placeholder)

exit_program:
    addi x17, x0, 1        # Exit system call
    addi x10, x0, 0        # Exit status
    ecall
```

## 18.2 Advanced Programming Examples

### 18.2.1 Quick Sort Algorithm

```assembly
# QuickSort Implementation
# Demonstrates advanced algorithm implementation and optimization

.section .data
array_size: .word 1000
test_array: .space 4000    # 1000 integers * 4 bytes

.section .text
.globl quicksort

# QuickSort(array, low, high)
# x10 = array pointer, x11 = low index, x12 = high index
quicksort:
    # Save registers
    addi x2, x2, -32       # Allocate stack frame
    sw   x1, 28(x2)        # Return address
    sw   x8, 24(x2)        # Frame pointer
    sw   x9, 20(x2)        # s1
    sw   x18, 16(x2)       # s2  
    sw   x19, 12(x2)       # s3
    sw   x20, 8(x2)        # s4
    sw   x21, 4(x2)        # s5
    sw   x22, 0(x2)        # s6
    
    addi x8, x2, 32        # Set frame pointer
    
    # Save parameters
    add  x9, x10, x0       # s1 = array
    add  x18, x11, x0      # s2 = low
    add  x19, x12, x0      # s3 = high
    
    # Check if low < high
    bge  x18, x19, qs_done # if low >= high, done
    
    # Partition the array
    add  x10, x9, x0       # array
    add  x11, x18, x0      # low
    add  x12, x19, x0      # high
    jal  x1, partition     # Call partition
    add  x20, x10, x0      # s4 = pivot index
    
    # Recursively sort left partition
    add  x10, x9, x0       # array
    add  x11, x18, x0      # low
    addi x12, x20, -1      # pivot - 1
    jal  x1, quicksort     # Recursive call
    
    # Recursively sort right partition  
    add  x10, x9, x0       # array
    addi x11, x20, 1       # pivot + 1
    add  x12, x19, x0      # high
    jal  x1, quicksort     # Recursive call

qs_done:
    # Restore registers
    lw   x22, 0(x2)
    lw   x21, 4(x2)
    lw   x20, 8(x2)
    lw   x19, 12(x2)
    lw   x18, 16(x2)
    lw   x9, 20(x2)
    lw   x8, 24(x2)
    lw   x1, 28(x2)
    addi x2, x2, 32
    
    jalr x0, 0(x1)         # Return

# Partition function for quicksort
# Returns pivot index in x10
partition:
    # Save registers
    addi x2, x2, -24
    sw   x8, 20(x2)
    sw   x9, 16(x2)
    sw   x18, 12(x2)
    sw   x19, 8(x2)
    sw   x20, 4(x2)
    sw   x21, 0(x2)
    
    # Parameters: x10=array, x11=low, x12=high
    add  x9, x10, x0       # s1 = array
    add  x18, x11, x0      # s2 = low (i)
    add  x19, x12, x0      # s3 = high (j)
    
    # Load pivot (last element)
    slli x20, x19, 2       # offset = high * 4
    add  x21, x9, x20      # address = array + offset
    lw   x20, 0(x21)       # pivot = array[high]
    
    addi x21, x18, -1      # i = low - 1

partition_loop:
    bge  x18, x19, partition_end  # if j >= high, done
    
    # Load array[j]
    slli x22, x18, 2       # offset = j * 4
    add  x23, x9, x22      # address = array + offset  
    lw   x24, 0(x23)       # array[j]
    
    # Compare with pivot
    bge  x24, x20, skip_swap  # if array[j] >= pivot, skip
    
    # Increment i and swap
    addi x21, x21, 1       # i++
    
    # Swap array[i] and array[j]
    slli x25, x21, 2       # offset_i = i * 4
    add  x26, x9, x25      # addr_i = array + offset_i
    lw   x27, 0(x26)       # temp = array[i]
    sw   x24, 0(x26)       # array[i] = array[j]
    sw   x27, 0(x23)       # array[j] = temp

skip_swap:
    addi x18, x18, 1       # j++
    jal  x0, partition_loop

partition_end:
    # Place pivot in correct position
    addi x21, x21, 1       # i++
    slli x25, x21, 2       # offset = i * 4
    add  x26, x9, x25      # addr = array + offset
    lw   x27, 0(x26)       # temp = array[i]
    
    slli x28, x19, 2       # offset = high * 4
    add  x29, x9, x28      # addr = array + offset
    sw   x20, 0(x26)       # array[i] = pivot
    sw   x27, 0(x29)       # array[high] = temp
    
    add  x10, x21, x0      # Return pivot index
    
    # Restore registers
    lw   x21, 0(x2)
    lw   x20, 4(x2)
    lw   x19, 8(x2)
    lw   x18, 12(x2)
    lw   x9, 16(x2)
    lw   x8, 20(x2)
    addi x2, x2, 24
    
    jalr x0, 0(x1)         # Return
```

### 18.2.2 32-bit Integer Multiply (Software Implementation)

```assembly
# 32-bit Integer Multiplication
# Demonstrates bit manipulation and optimization techniques
# Result: 64-bit product in (x11:x10) = x10 * x11

multiply_32x32:
    # Save registers
    addi x2, x2, -16
    sw   x8, 12(x2)
    sw   x9, 8(x2)
    sw   x18, 4(x2)
    sw   x19, 0(x2)
    
    # Initialize
    add  x8, x0, x0        # High part of result
    add  x9, x0, x0        # Low part of result
    add  x18, x10, x0      # Multiplicand
    add  x19, x11, x0      # Multiplier
    addi x12, x0, 32       # Loop counter

multiply_loop:
    beq  x12, x0, multiply_done  # If counter == 0, done
    
    # Check LSB of multiplier
    andi x13, x19, 1       # x13 = multiplier & 1
    beq  x13, x0, skip_add # If LSB == 0, skip addition
    
    # Add multiplicand to result
    add  x9, x9, x18       # Low += multiplicand
    sltu x14, x9, x18      # Carry = (low < multiplicand)
    add  x8, x8, x14       # High += carry

skip_add:
    # Shift multiplicand left, multiplier right
    slli x18, x18, 1       # multiplicand <<= 1
    srli x19, x19, 1       # multiplier >>= 1
    addi x12, x12, -1      # counter--
    
    jal  x0, multiply_loop

multiply_done:
    # Result in x8 (high) and x9 (low)
    add  x10, x9, x0       # Return low part
    add  x11, x8, x0       # Return high part
    
    # Restore registers
    lw   x19, 0(x2)
    lw   x18, 4(x2)
    lw   x9, 8(x2)
    lw   x8, 12(x2)
    addi x2, x2, 16
    
    jalr x0, 0(x1)         # Return
```

## 18.3 System Programming Examples

### 18.3.1 Interrupt Service Routine Template

```assembly
# Interrupt Service Routine Template
# Demonstrates proper context saving and interrupt handling

.section .data
.align 4
context_save_area:
    .space 128             # Space for all registers

.section .text
.globl interrupt_handler

# Main interrupt entry point
interrupt_handler:
    # Save minimal context immediately
    sw   x1, -4(x2)        # Save return address
    sw   x5, -8(x2)        # Save t0 (temporary)
    sw   x6, -12(x2)       # Save t1 (temporary)
    addi x2, x2, -12       # Adjust stack pointer
    
    # Determine interrupt source
    lui  x5, %hi(0xF0000000) # Interrupt status register base
    lw   x6, 0x100(x5)      # Read interrupt status
    
    # Check interrupt priorities (IRQ0 highest)
    andi x1, x6, 0x01       # Check IRQ0
    bne  x1, x0, handle_irq0
    
    andi x1, x6, 0x02       # Check IRQ1
    bne  x1, x0, handle_irq1
    
    andi x1, x6, 0x04       # Check IRQ2
    bne  x1, x0, handle_irq2
    
    # Handle remaining interrupts...
    jal  x0, interrupt_exit

handle_irq0:
    # Critical interrupt - minimal latency required
    jal  x1, save_minimal_context
    jal  x1, process_critical_interrupt
    jal  x1, restore_minimal_context
    jal  x0, interrupt_exit

handle_irq1:
    # Timer interrupt - save full context
    jal  x1, save_full_context
    jal  x1, process_timer_interrupt  
    jal  x1, restore_full_context
    jal  x0, interrupt_exit

handle_irq2:
    # I/O interrupt
    jal  x1, save_full_context
    jal  x1, process_io_interrupt
    jal  x1, restore_full_context
    jal  x0, interrupt_exit

save_full_context:
    # Save all registers to context save area
    lui  x5, %hi(context_save_area)
    addi x5, x5, %lo(context_save_area)
    
    # Save general-purpose registers
    sw   x0, 0(x5)         # x0 (always zero)
    sw   x1, 4(x5)         # x1 (ra)
    sw   x2, 8(x5)         # x2 (sp)
    sw   x3, 12(x5)        # x3 (gp)
    sw   x4, 16(x5)        # x4 (tp)
    # Continue for all 32 registers...
    
    jalr x0, 0(x1)         # Return

restore_full_context:
    # Restore all registers from context save area
    lui  x5, %hi(context_save_area)
    addi x5, x5, %lo(context_save_area)
    
    # Restore in reverse order
    # Skip x0 (hardwired zero)
    lw   x1, 4(x5)         # x1 (ra)
    # Continue for all registers...
    
    jalr x0, 0(x1)         # Return

interrupt_exit:
    # Acknowledge interrupt
    lui  x5, %hi(0xF0000000)
    addi x6, x0, 1
    sw   x6, 0x104(x5)     # Write to interrupt ack register
    
    # Restore minimal context
    lw   x6, 0(x2)         # Restore t1
    lw   x5, 4(x2)         # Restore t0
    lw   x1, 8(x2)         # Restore return address
    addi x2, x2, 12        # Restore stack pointer
    
    # Return from interrupt
    jalr x0, 0(x1)         # Actually would use special return instruction
```

### 18.3.2 Memory Management Routines

```assembly
# Simple Memory Allocator
# Demonstrates heap management and pointer arithmetic

.section .data
.align 4
heap_start:    .word heap_base
heap_current:  .word heap_base
heap_end:      .word heap_base + heap_size
.equ heap_size, 65536      # 64KB heap

.section .bss
.align 4
heap_base:
    .space 65536           # 64KB heap space

.section .text
.globl malloc
.globl free

# Simple malloc implementation
# Input: x10 = size in bytes
# Output: x10 = pointer to allocated memory (or 0 if failed)
malloc:
    # Align size to 4-byte boundary
    addi x10, x10, 3       # Add 3 for alignment
    andi x10, x10, 0xFFFFFFFC  # Clear lower 2 bits
    
    # Load current heap pointer
    lw   x11, heap_current
    
    # Calculate new heap pointer
    add  x12, x11, x10     # new_ptr = current + size
    
    # Check if allocation exceeds heap
    lw   x13, heap_end
    bgt  x12, x13, malloc_failed
    
    # Update heap pointer
    sw   x12, heap_current
    
    # Return allocated pointer
    add  x10, x11, x0      # Return old heap_current
    jalr x0, 0(x1)

malloc_failed:
    add  x10, x0, x0       # Return NULL
    jalr x0, 0(x1)

# Simple free implementation (no-op in this simple allocator)
free:
    # In a real implementation, this would add to free list
    jalr x0, 0(x1)         # Return (no-op)

# Memory copy routine
# memcpy(dest, src, count)
# x10 = destination, x11 = source, x12 = byte count
memcpy:
    beq  x12, x0, memcpy_done  # If count == 0, done
    
    # Check alignment for word copying optimization
    or   x13, x10, x11     # Check if both pointers aligned
    andi x13, x13, 3       # Check lower 2 bits
    bne  x13, x0, byte_copy # If not aligned, use byte copy
    
    # Check if count is multiple of 4
    andi x13, x12, 3
    bne  x13, x0, byte_copy # If not word-aligned count, use byte copy

word_copy:
    # Copy words (4 bytes at a time)
    srli x12, x12, 2       # Convert byte count to word count

word_copy_loop:
    beq  x12, x0, memcpy_done
    lw   x13, 0(x11)       # Load word from source
    sw   x13, 0(x10)       # Store word to destination
    addi x11, x11, 4       # Advance source pointer
    addi x10, x10, 4       # Advance dest pointer
    addi x12, x12, -1      # Decrement count
    jal  x0, word_copy_loop

byte_copy:
    # Copy bytes one at a time
byte_copy_loop:
    beq  x12, x0, memcpy_done
    lb   x13, 0(x11)       # Load byte from source
    sb   x13, 0(x10)       # Store byte to destination
    addi x11, x11, 1       # Advance source pointer
    addi x10, x10, 1       # Advance dest pointer
    addi x12, x12, -1      # Decrement count
    jal  x0, byte_copy_loop

memcpy_done:
    jalr x0, 0(x1)         # Return
```

## 18.4 Performance-Critical Programming

### 18.4.1 Optimized String Operations

```assembly
# High-performance string length calculation
# Demonstrates loop unrolling and optimization techniques

# strlen - optimized string length
# Input: x10 = string pointer
# Output: x10 = string length
strlen:
    add  x11, x10, x0      # Save original pointer
    
    # Align pointer to word boundary for faster access
    andi x12, x10, 3       # Check alignment
    beq  x12, x0, aligned_strlen  # If aligned, skip byte loop
    
    # Handle unaligned bytes at start
unaligned_loop:
    lb   x13, 0(x10)       # Load byte
    beq  x13, x0, strlen_done  # If null terminator, done
    addi x10, x10, 1       # Advance pointer
    andi x12, x10, 3       # Check alignment
    bne  x12, x0, unaligned_loop  # Continue until aligned

aligned_strlen:
    # Now process 4 bytes at a time
word_strlen_loop:
    lw   x13, 0(x10)       # Load 4 bytes
    
    # Check for zero bytes using bit manipulation tricks
    lui  x14, 0x80808     
    ori  x14, x14, 0x080   # x14 = 0x80808080
    lui  x15, 0x01010
    ori  x15, x15, 0x101   # x15 = 0x01010101
    
    sub  x16, x13, x15     # temp = word - 0x01010101
    not  x17, x13          # ~word
    and  x16, x16, x17     # temp & ~word
    and  x16, x16, x14     # temp & ~word & 0x80808080
    
    bne  x16, x0, found_zero_byte  # If non-zero, found zero byte
    addi x10, x10, 4       # Advance by 4 bytes
    jal  x0, word_strlen_loop

found_zero_byte:
    # Check each byte to find exact position
    lb   x13, 0(x10)
    beq  x13, x0, strlen_done
    addi x10, x10, 1
    
    lb   x13, 0(x10)
    beq  x13, x0, strlen_done
    addi x10, x10, 1
    
    lb   x13, 0(x10)
    beq  x13, x0, strlen_done
    addi x10, x10, 1

strlen_done:
    sub  x10, x10, x11     # Length = current - original
    jalr x0, 0(x1)         # Return
```

### 18.4.2 Fixed-Point Arithmetic Library

```assembly
# Fixed-Point Arithmetic (16.16 format)
# Demonstrates mathematical operations without FPU

.section .text

# Fixed-point multiply: (a * b) >> 16
# Input: x10 = a (16.16), x11 = b (16.16)  
# Output: x10 = result (16.16)
fixmul:
    # Perform 32x32 -> 64 bit multiply
    jal  x1, multiply_32x32  # Result in x11:x10
    
    # Shift right by 16 to get 16.16 result
    slli x12, x11, 16        # High part << 16
    srli x13, x10, 16        # Low part >> 16
    or   x10, x12, x13       # Combine parts
    
    jalr x0, 0(x1)          # Return

# Fixed-point divide: (a << 16) / b
# Input: x10 = a (16.16), x11 = b (16.16)
# Output: x10 = result (16.16)  
fixdiv:
    # Shift dividend left by 16
    add  x12, x10, x0       # Save original dividend
    slli x10, x10, 16       # dividend <<= 16
    
    # Perform division (simplified - would use full algorithm)
    # This is a placeholder for complete division algorithm
    div  x10, x10, x11      # Divide (assuming hardware support)
    
    jalr x0, 0(x1)          # Return

# Fixed-point sine approximation using Taylor series
# Input: x10 = angle in radians (16.16 format)
# Output: x10 = sin(angle) (16.16 format)
fixsin:
    addi x2, x2, -32        # Allocate stack space
    sw   x1, 28(x2)         # Save return address
    sw   x8, 24(x2)         # Save registers
    sw   x9, 20(x2)
    sw   x18, 16(x2)
    sw   x19, 12(x2)
    sw   x20, 8(x2)
    sw   x21, 4(x2)
    sw   x22, 0(x2)
    
    add  x8, x10, x0        # x8 = x (input angle)
    add  x9, x10, x0        # x9 = result (start with x)
    add  x18, x10, x0       # x18 = term (current term)
    addi x19, x0, 1         # x19 = n (term number)
    
    # Taylor series: sin(x) = x - x³/3! + x⁵/5! - x⁷/7! + ...
sine_loop:
    addi x19, x19, 2        # n += 2 (next odd power)
    
    # Calculate x^n term
    add  x10, x18, x0       # Current term
    add  x11, x8, x0        # x
    jal  x1, fixmul         # term *= x
    add  x18, x10, x0
    
    add  x10, x18, x0       # Current term  
    add  x11, x8, x0        # x
    jal  x1, fixmul         # term *= x (now x^n)
    add  x18, x10, x0
    
    # Divide by n!
    # Simplified factorial calculation
    add  x10, x18, x0       # term
    mul  x11, x19, x19      # n * n (approximate n!)
    addi x11, x19, -1       # n-1
    mul  x11, x11, x19      # n * (n-1)
    jal  x1, fixdiv         # term /= factorial
    add  x18, x10, x0
    
    # Add/subtract term based on series sign
    andi x20, x19, 4        # Check if n/4 is odd
    beq  x20, x0, add_term
    sub  x9, x9, x18        # Subtract term
    jal  x0, check_continue
    
add_term:
    add  x9, x9, x18        # Add term

check_continue:
    # Check if term is small enough to stop
    lui  x21, 0x0001        # Small threshold (16.16 format)
    blt  x18, x21, sine_done
    addi x22, x0, 10        # Maximum 10 terms
    blt  x19, x22, sine_loop

sine_done:
    add  x10, x9, x0        # Return result
    
    # Restore registers
    lw   x22, 0(x2)
    lw   x21, 4(x2)
    lw   x20, 8(x2)
    lw   x19, 12(x2)
    lw   x18, 16(x2)
    lw   x9, 20(x2)
    lw   x8, 24(x2)
    lw   x1, 28(x2)
    addi x2, x2, 32
    
    jalr x0, 0(x1)          # Return
```

---

*This chapter provided comprehensive programming examples demonstrating the full capabilities of the MCU-32X processor for desktop computing applications. These examples showcase the processor's suitability for complex algorithms, system programming, and performance-critical applications that were typical of desktop computing in 1999.*