# Chapter 8: Assembly Language Programming
## MCU-32X Technical Reference Manual

---

## 8.1 Assembly Language Overview

The MCU-32X uses standard RISC-V assembly syntax with MCU-specific extensions for system programming and optimization.

### 8.1.1 Assembly Language Syntax

**Basic Instruction Format:**
```assembly
[label:]  mnemonic  operand1, operand2, operand3  [# comment]
```

**Register Naming:**
```assembly
# Numeric register names (x0-x31)
add  x1, x2, x3         # x1 = x2 + x3

# ABI register names (preferred)  
add  ra, sp, gp         # ra = sp + gp

# Mixed naming (valid but discouraged)
add  x1, sp, x3         # x1 = sp + x3
```

**Immediate Values:**
```assembly
# Decimal immediates
addi x1, x0, 100        # x1 = 0 + 100
addi x2, x1, -50        # x2 = x1 - 50

# Hexadecimal immediates  
ori  x3, x0, 0xFF       # x3 = 0 | 0xFF
andi x4, x3, 0x0F       # x4 = x3 & 0x0F

# Binary immediates
xori x5, x4, 0b1010     # x5 = x4 ^ 0b1010
```

### 8.1.2 Assembler Directives

**Section Directives:**
```assembly
.section .text          # Code section  
.section .data          # Initialized data
.section .bss           # Uninitialized data
.section .rodata        # Read-only data

# Shorthand directives
.text                   # Equivalent to .section .text
.data                   # Equivalent to .section .data
```

**Symbol Directives:**
```assembly
.global symbol_name     # Make symbol globally visible
.globl  symbol_name     # Alternative spelling
.local  symbol_name     # Symbol only visible in this file
.weak   symbol_name     # Weak symbol (can be overridden)

.equ    CONSTANT, 42    # Define assembler constant
.set    VARIABLE, 100   # Set symbol value
```

**Data Directives:**
```assembly
.byte   0x12, 0x34      # 8-bit values
.half   0x1234          # 16-bit values  
.word   0x12345678      # 32-bit values
.dword  0x123456789ABCDEF0  # 64-bit values

.ascii  "Hello"         # String without null terminator
.asciz  "World"         # String with null terminator
.string "Test"          # Same as .asciz

.space  100             # Reserve 100 bytes (uninitialized)
.zero   50              # Reserve 50 bytes (zero-initialized)
```

**Alignment Directives:**
```assembly
.align  2               # Align to 2^2 = 4 byte boundary
.align  4               # Align to 2^4 = 16 byte boundary
.balign 8               # Align to 8 byte boundary (absolute)
```

---

## 8.2 Instruction Categories and Usage

### 8.2.1 Arithmetic Instructions

**Basic Arithmetic:**
```assembly
# Addition and subtraction
add  t0, t1, t2         # t0 = t1 + t2
sub  t0, t1, t2         # t0 = t1 - t2
addi t0, t1, 100        # t0 = t1 + 100

# Comparison (sets result to 0 or 1)
slt  t0, t1, t2         # t0 = (t1 < t2) ? 1 : 0 (signed)
sltu t0, t1, t2         # t0 = (t1 < t2) ? 1 : 0 (unsigned)
slti t0, t1, 100        # t0 = (t1 < 100) ? 1 : 0 (signed)
```

**Advanced Arithmetic Examples:**
```assembly
# Absolute value
abs_value:
    srai t1, a0, 31         # t1 = sign bits (all 0 or all 1)
    xor  t0, a0, t1         # Flip bits if negative  
    sub  a0, t0, t1         # Subtract -1 if was negative
    ret

# Integer division by constant (multiply by reciprocal)
divide_by_3:
    lui  t0, 0x55555        # Load upper bits of 1/3 approximation
    ori  t0, t0, 0x556      # t0 = 0x55555556
    mulh t1, a0, t0         # High 32 bits of a0 * (1/3)
    srai t1, t1, 0          # Adjust for sign
    srai t2, a0, 31         # Get sign of original
    sub  a0, t1, t2         # Final result
    ret

# Fast multiply by 10
multiply_by_10:
    slli t0, a0, 3          # t0 = a0 * 8
    slli t1, a0, 1          # t1 = a0 * 2  
    add  a0, t0, t1         # a0 = 8*a0 + 2*a0 = 10*a0
    ret
```

### 8.2.2 Logical and Bitwise Instructions

**Basic Logical Operations:**
```assembly
# Bitwise operations
and  t0, t1, t2         # t0 = t1 & t2
or   t0, t1, t2         # t0 = t1 | t2
xor  t0, t1, t2         # t0 = t1 ^ t2
andi t0, t1, 0xFF       # t0 = t1 & 0xFF (mask lower byte)
ori  t0, t1, 0x80       # t0 = t1 | 0x80 (set bit 7)
xori t0, t1, -1         # t0 = ~t1 (bitwise NOT)
```

**Bit Manipulation Examples:**
```assembly
# Set bit n in register
set_bit:
    # a0 = value, a1 = bit number
    li   t0, 1              # Create bit mask
    sll  t0, t0, a1         # Shift to position
    or   a0, a0, t0         # Set the bit
    ret

# Clear bit n in register  
clear_bit:
    # a0 = value, a1 = bit number
    li   t0, 1              # Create bit mask
    sll  t0, t0, a1         # Shift to position
    not  t1, t0             # Invert mask
    and  a0, a0, t1         # Clear the bit
    ret

# Test bit n in register
test_bit:
    # a0 = value, a1 = bit number  
    srl  t0, a0, a1         # Shift bit to LSB
    andi a0, t0, 1          # Mask all but LSB
    ret

# Count leading zeros (software implementation)
count_leading_zeros:
    beqz a0, all_zeros      # Handle zero case
    li   t0, 0              # Counter
    li   t1, 31             # Bit position
clz_loop:
    srl  t2, a0, t1         # Shift to check MSB
    bnez t2, clz_done       # Found first 1 bit
    addi t0, t0, 1          # Increment count
    addi t1, t1, -1         # Next bit position
    bgez t1, clz_loop       # Continue if more bits
all_zeros:
    li   a0, 32             # All bits are zero
    ret
clz_done:
    mv   a0, t0             # Return count
    ret
```

### 8.2.3 Shift Instructions

**Basic Shifts:**
```assembly
# Logical shifts (fill with zeros)
sll  t0, t1, t2         # t0 = t1 << t2 (shift left)
srl  t0, t1, t2         # t0 = t1 >> t2 (logical right)
slli t0, t1, 4          # t0 = t1 << 4 (immediate)
srli t0, t1, 8          # t0 = t1 >> 8 (immediate)

# Arithmetic shifts (preserve sign)
sra  t0, t1, t2         # t0 = t1 >>> t2 (arithmetic right)
srai t0, t1, 16         # t0 = t1 >>> 16 (immediate)
```

**Shift Applications:**
```assembly
# Fast multiplication/division by powers of 2
multiply_by_16:
    slli a0, a0, 4          # a0 = a0 * 16
    ret

divide_by_8:
    srai a0, a0, 3          # a0 = a0 / 8 (signed)
    ret

# Extract bit field  
extract_bits:
    # a0 = value, a1 = start_bit, a2 = width
    sll  t0, a0, a1         # Shift to align field at MSB
    li   t1, 32
    sub  t1, t1, a2         # Calculate shift amount  
    srl  a0, t0, t1         # Shift to extract field
    ret

# Rotate left (using shifts and OR)
rotate_left:
    # a0 = value, a1 = rotate_amount
    andi a1, a1, 31         # Limit to 0-31
    sll  t0, a0, a1         # Left portion
    li   t1, 32
    sub  t1, t1, a1         # Right shift amount
    srl  t2, a0, t1         # Right portion  
    or   a0, t0, t2         # Combine parts
    ret
```

---

## 8.3 Memory Access Programming

### 8.3.1 Load and Store Instructions

**Basic Memory Access:**
```assembly
# Word operations (32-bit)
lw   t0, 0(t1)          # Load word from address in t1
lw   t0, 100(t1)        # Load from t1 + 100
sw   t0, 0(t1)          # Store word to address in t1
sw   t0, -4(t1)         # Store to t1 - 4

# Halfword operations (16-bit)
lh   t0, 0(t1)          # Load signed halfword
lhu  t0, 0(t1)          # Load unsigned halfword  
sh   t0, 0(t1)          # Store halfword

# Byte operations (8-bit)
lb   t0, 0(t1)          # Load signed byte
lbu  t0, 0(t1)          # Load unsigned byte
sb   t0, 0(t1)          # Store byte
```

**Memory Access Patterns:**
```assembly
# Array indexing
array_access:
    # a0 = array_base, a1 = index, a2 = element_size
    mul  t0, a1, a2         # Calculate offset (software multiply)
    add  t0, a0, t0         # Calculate address
    lw   a0, 0(t0)          # Load element
    ret

# Structure member access
struct_access:
    # a0 = struct_pointer
    lw   t0, 0(a0)          # member at offset 0
    lw   t1, 4(a0)          # member at offset 4  
    lh   t2, 8(a0)          # member at offset 8 (16-bit)
    lb   t3, 10(a0)         # member at offset 10 (8-bit)
    ret

# Unaligned access (handle in software)  
load_unaligned_word:
    # a0 = address (potentially unaligned)
    lbu  t0, 0(a0)          # Load byte 0
    lbu  t1, 1(a0)          # Load byte 1
    lbu  t2, 2(a0)          # Load byte 2
    lbu  t3, 3(a0)          # Load byte 3
    
    slli t1, t1, 8          # Shift byte 1
    slli t2, t2, 16         # Shift byte 2  
    slli t3, t3, 24         # Shift byte 3
    
    or   t0, t0, t1         # Combine bytes
    or   t0, t0, t2
    or   a0, t0, t3         # Result in a0
    ret
```

### 8.3.2 Advanced Memory Operations

**Memory Copy Routines:**
```assembly
# Fast word-aligned memory copy
memcpy_fast:
    # a0 = dest, a1 = src, a2 = count (bytes)
    beqz a2, copy_done      # Handle zero count
    
    # Check word alignment
    or   t0, a0, a1
    andi t0, t0, 3
    bnez t0, byte_copy      # Fall back to byte copy
    
    # Word copy loop
    srli t1, a2, 2          # Word count = byte_count / 4
    beqz t1, remainder
    
word_loop:
    lw   t0, 0(a1)          # Load source word
    sw   t0, 0(a0)          # Store to destination
    addi a0, a0, 4          # Advance dest pointer
    addi a1, a1, 4          # Advance src pointer
    addi t1, t1, -1         # Decrement count
    bnez t1, word_loop      # Continue if more words
    
remainder:
    andi a2, a2, 3          # Remaining bytes
    beqz a2, copy_done
    
byte_copy:
    lbu  t0, 0(a1)          # Load source byte
    sb   t0, 0(a0)          # Store to destination
    addi a0, a0, 1          # Advance dest pointer  
    addi a1, a1, 1          # Advance src pointer
    addi a2, a2, -1         # Decrement count
    bnez a2, byte_copy      # Continue if more bytes
    
copy_done:
    ret

# Memory set routine
memset_fast:
    # a0 = dest, a1 = value (byte), a2 = count
    beqz a2, set_done
    
    # Replicate byte to word
    andi a1, a1, 0xFF       # Ensure byte value
    slli t0, a1, 8          # Shift to byte 1
    or   a1, a1, t0         # Combine bytes 0,1
    slli t0, a1, 16         # Shift to bytes 2,3
    or   t6, a1, t0         # t6 = replicated word
    
    # Check alignment and use word stores if possible
    andi t0, a0, 3
    bnez t0, byte_set       # Use byte stores if unaligned
    
    srli t1, a2, 2          # Word count
    beqz t1, set_remainder
    
word_set_loop:
    sw   t6, 0(a0)          # Store word
    addi a0, a0, 4          # Advance pointer
    addi t1, t1, -1         # Decrement count
    bnez t1, word_set_loop
    
set_remainder:
    andi a2, a2, 3          # Remaining bytes
    beqz a2, set_done
    
byte_set:
    sb   a1, 0(a0)          # Store byte
    addi a0, a0, 1          # Advance pointer
    addi a2, a2, -1         # Decrement count
    bnez a2, byte_set
    
set_done:
    ret
```

---

## 8.4 Control Flow Programming

### 8.4.1 Conditional Branches

**Branch Instruction Usage:**
```assembly
# Basic comparisons
beq  t0, t1, equal      # Branch if t0 == t1
bne  t0, t1, not_equal  # Branch if t0 != t1  
blt  t0, t1, less       # Branch if t0 < t1 (signed)
bge  t0, t1, greater_eq # Branch if t0 >= t1 (signed)
bltu t0, t1, less_u     # Branch if t0 < t1 (unsigned)
bgeu t0, t1, greater_eq_u # Branch if t0 >= t1 (unsigned)

# Compare with zero (common patterns)
beqz t0, is_zero        # Branch if t0 == 0
bnez t0, not_zero       # Branch if t0 != 0
blez t0, less_eq_zero   # Branch if t0 <= 0 (signed)
bgtz t0, greater_zero   # Branch if t0 > 0 (signed)
bltz t0, negative       # Branch if t0 < 0 (signed)  
bgez t0, non_negative   # Branch if t0 >= 0 (signed)
```

**Conditional Execution Patterns:**
```assembly
# if-then-else structure
if_then_else:
    # if (a0 > 100)
    li   t0, 100
    ble  a0, t0, else_part
    
    # then part
    addi a0, a0, 1          # a0++
    j    end_if
    
else_part:
    # else part  
    addi a0, a0, -1         # a0--
    
end_if:
    ret

# Multi-way branch (switch statement)
switch_statement:
    # Switch on a0 (0-3)
    li   t0, 3
    bltu a0, t0, case_table # Valid case
    j    default_case       # Invalid case
    
case_table:
    slli t0, a0, 2          # Calculate offset (case * 4)
    la   t1, jump_table
    add  t0, t1, t0         # Calculate jump address
    lw   t0, 0(t0)          # Load jump target
    jr   t0                 # Jump to case
    
jump_table:
    .word case_0, case_1, case_2, case_3
    
case_0:
    li   a0, 10
    ret
case_1: 
    li   a0, 20
    ret
case_2:
    li   a0, 30
    ret
case_3:
    li   a0, 40  
    ret
default_case:
    li   a0, -1
    ret
```

### 8.4.2 Loops

**Basic Loop Structures:**
```assembly
# for loop: for (i = 0; i < count; i++)
for_loop:
    # a0 = count
    li   t0, 0              # i = 0
    
for_condition:
    bge  t0, a0, for_end    # if (i >= count) break
    
    # Loop body
    # ... process using t0 as index ...
    
    addi t0, t0, 1          # i++
    j    for_condition
    
for_end:
    ret

# while loop: while (condition)  
while_loop:
    # a0 = pointer to condition variable
    
while_condition:
    lw   t0, 0(a0)          # Load condition
    beqz t0, while_end      # if (!condition) break
    
    # Loop body
    # ... modify condition ...
    
    j    while_condition
    
while_end:
    ret

# do-while loop: do { ... } while (condition)
do_while_loop:
    
do_body:
    # Loop body (always executes at least once)
    # ...
    
    lw   t0, 0(a0)          # Load condition  
    bnez t0, do_body        # if (condition) continue
    
    ret

# Optimized loop with loop unrolling
unrolled_loop:
    # Process array in chunks of 4
    # a0 = array, a1 = count
    srli t1, a1, 2          # Chunk count = count / 4
    beqz t1, remainder_loop
    
unroll_loop:
    lw   t0, 0(a0)          # Process element 0
    # ... process t0 ...
    lw   t0, 4(a0)          # Process element 1  
    # ... process t0 ...
    lw   t0, 8(a0)          # Process element 2
    # ... process t0 ...
    lw   t0, 12(a0)         # Process element 3
    # ... process t0 ...
    
    addi a0, a0, 16         # Advance by 4 elements
    addi t1, t1, -1         # Decrement chunk count
    bnez t1, unroll_loop
    
remainder_loop:
    andi a1, a1, 3          # Remaining elements  
    beqz a1, loop_done
    
remainder:
    lw   t0, 0(a0)          # Process remaining element
    # ... process t0 ...
    addi a0, a0, 4          # Advance pointer
    addi a1, a1, -1         # Decrement count
    bnez a1, remainder
    
loop_done:
    ret
```

### 8.4.3 Function Calls

**Function Call Mechanics:**
```assembly
# Simple function call
caller:
    # Set up arguments
    li   a0, 42             # First argument
    li   a1, 84             # Second argument
    
    # Call function
    jal  ra, function       # Call and save return address
    
    # Result now in a0
    # Continue with caller code...
    ret

# Function with stack frame
function_with_frame:
    # Prologue: set up stack frame
    addi sp, sp, -16        # Allocate stack space
    sw   ra, 12(sp)         # Save return address
    sw   s0, 8(sp)          # Save frame pointer
    sw   s1, 4(sp)          # Save callee-saved register
    addi s0, sp, 16         # Set up frame pointer
    
    # Function body
    mv   s1, a0             # Save argument in callee-saved reg
    # ... function logic using s1 ...
    mv   a0, s1             # Return value
    
    # Epilogue: tear down stack frame  
    lw   s1, 4(sp)          # Restore callee-saved register
    lw   s0, 8(sp)          # Restore frame pointer
    lw   ra, 12(sp)         # Restore return address
    addi sp, sp, 16         # Deallocate stack space
    ret                     # Return to caller

# Leaf function (no calls to other functions)
leaf_function:
    # No need to save ra (not modified)
    # Can use temporary registers freely
    add  t0, a0, a1         # Use arguments
    slli t0, t0, 1          # Process data
    mv   a0, t0             # Return result
    ret                     # Simple return
```

---

## 8.5 Advanced Assembly Techniques

### 8.5.1 Optimized String Operations

**String Length:**
```assembly
# Optimized strlen using word operations  
strlen_fast:
    mv   t6, a0             # Save original pointer
    
    # Handle unaligned start
    andi t0, a0, 3          # Check alignment
    beqz t0, word_scan      # Skip if aligned
    
byte_start:
    lbu  t1, 0(a0)          # Load byte
    beqz t1, found_end      # Found null terminator
    addi a0, a0, 1          # Advance pointer
    andi t0, a0, 3          # Check alignment  
    bnez t0, byte_start     # Continue until aligned
    
word_scan:
    lw   t1, 0(a0)          # Load 4 bytes
    
    # Check for null using: (word - 0x01010101) & ~word & 0x80808080
    lui  t2, 0x01010        # 0x01010000
    ori  t2, t2, 0x101       # 0x01010101
    sub  t3, t1, t2          # word - 0x01010101
    not  t4, t1              # ~word
    and  t3, t3, t4          # (word - 0x01010101) & ~word
    lui  t4, 0x80808        # 0x80808000
    ori  t4, t4, 0x080       # 0x80808080  
    and  t3, t3, t4          # Check for zero bytes
    bnez t3, null_in_word    # Found null byte
    
    addi a0, a0, 4           # Advance by word
    j    word_scan
    
null_in_word:
    # Find exact position of null byte
byte_end:
    lbu  t1, 0(a0)           # Load byte  
    beqz t1, found_end       # Found null
    addi a0, a0, 1           # Advance
    j    byte_end
    
found_end:
    sub  a0, a0, t6          # Calculate length
    ret
```

**String Copy:**
```assembly
# Optimized strcpy
strcpy_fast:
    mv   t6, a0             # Save dest for return
    
    # Check alignment
    or   t0, a0, a1         # Check if both aligned  
    andi t0, t0, 3
    bnez t0, byte_copy      # Use byte copy if misaligned
    
word_copy:
    lw   t1, 0(a1)          # Load source word
    
    # Check for null byte in word (same technique as strlen)
    lui  t2, 0x01010
    ori  t2, t2, 0x101
    sub  t3, t1, t2
    not  t4, t1  
    and  t3, t3, t4
    lui  t4, 0x80808
    ori  t4, t4, 0x080
    and  t3, t3, t4
    bnez t3, null_in_copy_word
    
    sw   t1, 0(a0)          # Store word
    addi a0, a0, 4          # Advance dest
    addi a1, a1, 4          # Advance src
    j    word_copy
    
null_in_copy_word:
    # Copy remaining bytes including null
byte_copy:
    lbu  t1, 0(a1)          # Load source byte
    sb   t1, 0(a0)          # Store byte
    beqz t1, copy_done      # Stop at null
    addi a0, a0, 1          # Advance dest
    addi a1, a1, 1          # Advance src
    j    byte_copy
    
copy_done:
    mv   a0, t6             # Return original dest
    ret
```

### 8.5.2 Floating-Point Emulation

**Software Floating-Point Operations:**
```assembly
# IEEE 754 single-precision addition (simplified)
fadd_soft:
    # a0 = float1, a1 = float2  
    # Extract sign, exponent, mantissa from both operands
    
    # Float1 components
    srai t0, a0, 31         # Sign bit of float1
    slli t1, a0, 1          # Remove sign bit
    srli t1, t1, 24         # Extract exponent (8 bits)
    slli t2, a0, 9          # Extract mantissa
    srli t2, t2, 9          # Mantissa (23 bits)
    
    # Float2 components  
    srai t3, a1, 31         # Sign bit of float2
    slli t4, a1, 1          # Remove sign bit
    srli t4, t4, 24         # Extract exponent
    slli t5, a1, 9          # Extract mantissa
    srli t5, t5, 9          # Mantissa (23 bits)
    
    # Add implicit leading 1 for normalized numbers
    ori  t2, t2, 0x800000   # Add implicit bit to mantissa1
    ori  t5, t5, 0x800000   # Add implicit bit to mantissa2
    
    # Align mantissas by shifting smaller exponent
    sub  t6, t1, t4         # Exponent difference
    bgez t6, align_float2   # float1 has larger exponent
    
    # float2 has larger exponent
    neg  t6, t6             # Make positive
    srl  t2, t2, t6         # Shift mantissa1 right
    mv   t1, t4             # Use larger exponent
    j    add_mantissas
    
align_float2:
    srl  t5, t5, t6         # Shift mantissa2 right
    
add_mantissas:
    # Add or subtract based on signs
    xor  s0, t0, t3         # Different signs?
    beqz s0, same_sign
    
    # Different signs - subtract
    sub  s1, t2, t5         # mantissa1 - mantissa2
    j    normalize
    
same_sign:
    # Same signs - add  
    add  s1, t2, t5         # mantissa1 + mantissa2
    
normalize:
    # Normalize result (simplified)
    # ... normalization logic ...
    
    # Pack result
    slli a0, t0, 31         # Sign bit
    slli t1, t1, 23         # Position exponent
    or   a0, a0, t1         # Add exponent
    or   a0, a0, s1         # Add mantissa
    ret
```

### 8.5.3 Fixed-Point Arithmetic

**Fixed-Point Math Operations:**
```assembly
# 16.16 fixed-point multiplication
fixmul_16_16:
    # a0 = fixed1 (16.16), a1 = fixed2 (16.16)
    # Result: (a0 * a1) >> 16
    
    # Software 64-bit multiplication
    # Split into high and low parts
    srai t0, a0, 16         # High part of a0
    andi t1, a0, 0xFFFF     # Low part of a0
    srai t2, a1, 16         # High part of a1  
    andi t3, a1, 0xFFFF     # Low part of a1
    
    # Multiply parts: (h1*h2)<<32 + (h1*l2)<<16 + (l1*h2)<<16 + (l1*l2)
    mul  t4, t0, t2         # h1 * h2
    mul  t5, t0, t3         # h1 * l2
    mul  t6, t1, t2         # l1 * h2  
    mul  s0, t1, t3         # l1 * l2
    
    # Combine results  
    add  t5, t5, t6         # (h1*l2) + (l1*h2)
    srli s0, s0, 16         # (l1*l2) >> 16
    add  s0, s0, t5         # Low part of result
    slli t4, t4, 16         # (h1*h2) << 16  
    add  a0, s0, t4         # Final result
    ret

# 16.16 fixed-point division
fixdiv_16_16:
    # a0 = dividend (16.16), a1 = divisor (16.16)  
    # Result: (a0 << 16) / a1
    
    # Check for division by zero
    beqz a1, div_by_zero
    
    # Handle signs
    xor  s2, a0, a1         # Result sign
    bgez a0, pos_dividend
    neg  a0, a0             # Make positive
pos_dividend:
    bgez a1, pos_divisor  
    neg  a1, a1             # Make positive
pos_divisor:
    
    # Shift dividend left by 16 (simulate 64-bit)
    # This is simplified - real implementation needs 64-bit arithmetic
    slli s0, a0, 16         # Shifted dividend
    
    # Software division loop
    li   s1, 0              # Quotient
    li   t0, 32             # Loop counter
    
div_loop:
    slli s1, s1, 1          # Shift quotient left
    slli s0, s0, 1          # Shift remainder left  
    blt  s0, a1, no_subtract # Compare with divisor
    sub  s0, s0, a1         # Subtract divisor
    ori  s1, s1, 1          # Set quotient bit
no_subtract:
    addi t0, t0, -1         # Decrement counter
    bnez t0, div_loop       # Continue loop
    
    # Apply sign to result
    bgez s2, div_positive
    neg  s1, s1             # Negate if needed
div_positive:
    mv   a0, s1             # Return quotient
    ret

div_by_zero:
    li   a0, 0x7FFFFFFF     # Return max positive value
    ret
```

---

## 8.6 System Programming

### 8.6.1 Interrupt Handlers

**Interrupt Service Routine Template:**
```assembly
# Machine mode interrupt handler
machine_interrupt_handler:
    # Save all registers (full context)
    addi sp, sp, -128       # Allocate space for 32 registers
    
    # Save general-purpose registers
    sw   x1, 0(sp)          # ra
    sw   x2, 4(sp)          # sp (original value needed for restoration)
    sw   x3, 8(sp)          # gp  
    sw   x4, 12(sp)         # tp
    sw   x5, 16(sp)         # t0
    sw   x6, 20(sp)         # t1
    sw   x7, 24(sp)         # t2
    sw   x8, 28(sp)         # s0
    sw   x9, 32(sp)         # s1
    sw   x10, 36(sp)        # a0
    sw   x11, 40(sp)        # a1
    sw   x12, 44(sp)        # a2
    sw   x13, 48(sp)        # a3
    sw   x14, 52(sp)        # a4
    sw   x15, 56(sp)        # a5
    sw   x16, 60(sp)        # a6
    sw   x17, 64(sp)        # a7
    sw   x18, 68(sp)        # s2
    sw   x19, 72(sp)        # s3
    sw   x20, 76(sp)        # s4
    sw   x21, 80(sp)        # s5
    sw   x22, 84(sp)        # s6
    sw   x23, 88(sp)        # s7
    sw   x24, 92(sp)        # s8
    sw   x25, 96(sp)        # s9
    sw   x26, 100(sp)       # s10
    sw   x27, 104(sp)       # s11
    sw   x28, 108(sp)       # t3
    sw   x29, 112(sp)       # t4
    sw   x30, 116(sp)       # t5
    sw   x31, 120(sp)       # t6
    
    # Read interrupt cause
    csrr t0, mcause         # Get cause register
    blt  t0, x0, handle_interrupt # MSB set = interrupt
    
    # Handle exception  
    jal  ra, exception_handler
    j    restore_context
    
handle_interrupt:
    # Mask off interrupt bit
    slli t0, t0, 1          # Clear MSB
    srli t0, t0, 1
    
    # Dispatch to specific handler
    li   t1, 3
    beq  t0, t1, software_interrupt
    li   t1, 7  
    beq  t0, t1, timer_interrupt
    li   t1, 11
    beq  t0, t1, external_interrupt
    
    # Unknown interrupt
    j    restore_context
    
software_interrupt:
    jal  ra, handle_software_irq
    j    restore_context
    
timer_interrupt:
    jal  ra, handle_timer_irq  
    j    restore_context
    
external_interrupt:
    jal  ra, handle_external_irq
    j    restore_context
    
restore_context:
    # Restore all registers
    lw   x31, 120(sp)       # t6
    lw   x30, 116(sp)       # t5
    lw   x29, 112(sp)       # t4
    lw   x28, 108(sp)       # t3
    lw   x27, 104(sp)       # s11
    lw   x26, 100(sp)       # s10
    lw   x25, 96(sp)        # s9
    lw   x24, 92(sp)        # s8
    lw   x23, 88(sp)        # s7
    lw   x22, 84(sp)        # s6
    lw   x21, 80(sp)        # s5
    lw   x20, 76(sp)        # s4
    lw   x19, 72(sp)        # s3
    lw   x18, 68(sp)        # s2
    lw   x17, 64(sp)        # a7
    lw   x16, 60(sp)        # a6
    lw   x15, 56(sp)        # a5
    lw   x14, 52(sp)        # a4
    lw   x13, 48(sp)        # a3
    lw   x12, 44(sp)        # a2
    lw   x11, 40(sp)        # a1
    lw   x10, 36(sp)        # a0
    lw   x9, 32(sp)         # s1
    lw   x8, 28(sp)         # s0
    lw   x7, 24(sp)         # t2
    lw   x6, 20(sp)         # t1
    lw   x5, 16(sp)         # t0
    lw   x4, 12(sp)         # tp
    lw   x3, 8(sp)          # gp
    # Skip x2 (sp) - restored by stack adjustment
    lw   x1, 0(sp)          # ra
    
    addi sp, sp, 128        # Restore stack pointer
    mret                    # Return from interrupt
```

### 8.6.2 System Calls

**System Call Interface:**
```assembly
# System call wrapper
syscall_wrapper:
    # a0-a6 = arguments, a7 = syscall number
    ecall                   # Trigger system call
    ret                     # Return to caller
    
# Individual system call wrappers
sys_write:
    # write(fd, buffer, count)  
    # a0=fd, a1=buffer, a2=count
    li   a7, 4              # System call number for write
    ecall
    ret
    
sys_read:
    # read(fd, buffer, count)
    # a0=fd, a1=buffer, a2=count  
    li   a7, 3              # System call number for read
    ecall
    ret
    
sys_exit:
    # exit(status)
    # a0=exit_status
    li   a7, 1              # System call number for exit
    ecall
    # Should not return

# System call handler (kernel side)
handle_syscall:
    # Save user context
    csrw mscratch, sp       # Save user stack pointer
    la   sp, kernel_stack_top # Switch to kernel stack
    
    # Save user registers on kernel stack
    addi sp, sp, -32
    sw   t0, 0(sp)
    sw   t1, 4(sp)  
    sw   t2, 8(sp)
    sw   a0, 12(sp)
    sw   a1, 16(sp)
    sw   a2, 20(sp)
    sw   a3, 24(sp)
    sw   a7, 28(sp)
    
    # Dispatch system call based on a7
    li   t0, 1
    beq  a7, t0, sys_exit_impl
    li   t0, 3
    beq  a7, t0, sys_read_impl  
    li   t0, 4
    beq  a7, t0, sys_write_impl
    
    # Unknown system call
    li   a0, -1             # Error return
    j    syscall_return
    
sys_write_impl:
    # Implement write system call
    jal  ra, kernel_write
    j    syscall_return
    
sys_read_impl:
    # Implement read system call  
    jal  ra, kernel_read
    j    syscall_return
    
sys_exit_impl:
    # Implement exit system call
    jal  ra, kernel_exit
    # Does not return
    
syscall_return:
    # Restore user context
    lw   a7, 28(sp)
    lw   a3, 24(sp)
    lw   a2, 20(sp)
    lw   a1, 16(sp)
    # a0 contains return value - don't restore
    lw   t2, 8(sp)
    lw   t1, 4(sp)
    lw   t0, 0(sp)
    addi sp, sp, 32
    
    csrr sp, mscratch       # Restore user stack pointer
    mret                    # Return to user mode
```

---

*This chapter provided comprehensive coverage of MCU-32X assembly language programming, from basic syntax to advanced system programming techniques, enabling efficient low-level software development.*