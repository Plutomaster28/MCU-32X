# Appendix B: Register Reference
## MCU-32X Technical Reference Manual

---

## B.1 General Purpose Registers

The MCU-32X implements the RISC-V RV32I register set with 32 general-purpose registers, each 32 bits wide.

### B.1.1 Integer Register Set

| Register | ABI Name | Description | Caller/Callee Saved |
|----------|----------|-------------|---------------------|
| x0 | zero | Hard-wired zero | - |
| x1 | ra | Return address | Caller |
| x2 | sp | Stack pointer | Callee |
| x3 | gp | Global pointer | - |
| x4 | tp | Thread pointer | - |
| x5 | t0 | Temporary register 0 | Caller |
| x6 | t1 | Temporary register 1 | Caller |
| x7 | t2 | Temporary register 2 | Caller |
| x8 | s0/fp | Saved register 0 / Frame pointer | Callee |
| x9 | s1 | Saved register 1 | Callee |
| x10 | a0 | Function argument 0 / Return value 0 | Caller |
| x11 | a1 | Function argument 1 / Return value 1 | Caller |
| x12 | a2 | Function argument 2 | Caller |
| x13 | a3 | Function argument 3 | Caller |
| x14 | a4 | Function argument 4 | Caller |
| x15 | a5 | Function argument 5 | Caller |
| x16 | a6 | Function argument 6 | Caller |
| x17 | a7 | Function argument 7 | Caller |
| x18 | s2 | Saved register 2 | Callee |
| x19 | s3 | Saved register 3 | Callee |
| x20 | s4 | Saved register 4 | Callee |
| x21 | s5 | Saved register 5 | Callee |
| x22 | s6 | Saved register 6 | Callee |
| x23 | s7 | Saved register 7 | Callee |
| x24 | s8 | Saved register 8 | Callee |
| x25 | s9 | Saved register 9 | Callee |
| x26 | s10 | Saved register 10 | Callee |
| x27 | s11 | Saved register 11 | Callee |
| x28 | t3 | Temporary register 3 | Caller |
| x29 | t4 | Temporary register 4 | Caller |
| x30 | t5 | Temporary register 5 | Caller |
| x31 | t6 | Temporary register 6 | Caller |

**Register Usage Conventions:**
- **x0 (zero)**: Always contains the value 0, writes are ignored
- **x1 (ra)**: Contains return address for function calls
- **x2 (sp)**: Stack pointer, points to top of current stack frame
- **x3 (gp)**: Global pointer for accessing global variables (optional)
- **x4 (tp)**: Thread pointer for thread-local storage (optional)
- **x5-x7, x28-x31 (t0-t2, t3-t6)**: Temporary registers, not preserved across calls
- **x8-x9, x18-x27 (s0-s1, s2-s11)**: Saved registers, preserved across calls
- **x10-x17 (a0-a7)**: Function arguments and return values

### B.1.2 Assembly Language Examples

**Basic Register Operations:**
```assembly
# Load immediate into register
li    t0, 0x12345678        # Load immediate value
lui   t1, 0x12345           # Load upper immediate
addi  t1, t1, 0x678         # Add immediate (equivalent to above)

# Register-to-register operations  
add   t2, t0, t1            # t2 = t0 + t1
sub   t3, t2, t0            # t3 = t2 - t0
and   t4, t0, t1            # t4 = t0 & t1
or    t5, t0, t1            # t5 = t0 | t1
xor   t6, t0, t1            # t6 = t0 ^ t1

# Shift operations
sll   a0, t0, 4             # Logical left shift by 4
srl   a1, t0, 4             # Logical right shift by 4  
sra   a2, t0, 4             # Arithmetic right shift by 4

# Comparison operations
slt   a3, t0, t1            # Set if t0 < t1 (signed)
sltu  a4, t0, t1            # Set if t0 < t1 (unsigned)
```

**Function Call Convention:**
```assembly
# Calling function example
caller_function:
    # Save caller-saved registers if needed
    addi  sp, sp, -16
    sw    ra, 12(sp)
    sw    t0, 8(sp)
    sw    t1, 4(sp)
    sw    t2, 0(sp)
    
    # Setup function arguments
    li    a0, 10              # First argument
    li    a1, 20              # Second argument  
    li    a2, 30              # Third argument
    
    # Call function
    jal   ra, target_function
    
    # Use return values
    mv    t0, a0              # Copy return value
    
    # Restore caller-saved registers
    lw    t2, 0(sp)
    lw    t1, 4(sp)
    lw    t0, 8(sp)
    lw    ra, 12(sp)
    addi  sp, sp, 16
    
    ret

# Called function example  
target_function:
    # Save callee-saved registers
    addi  sp, sp, -16
    sw    s0, 12(sp)
    sw    s1, 8(sp)
    sw    s2, 4(sp)
    sw    ra, 0(sp)
    
    # Function body using arguments a0, a1, a2
    add   s0, a0, a1          # s0 = a0 + a1
    add   s1, s0, a2          # s1 = s0 + a2
    
    # Prepare return value
    mv    a0, s1              # Return value in a0
    
    # Restore callee-saved registers
    lw    ra, 0(sp)
    lw    s2, 4(sp)
    lw    s1, 8(sp)
    lw    s0, 12(sp)
    addi  sp, sp, 16
    
    ret
```

---

## B.2 Control and Status Registers (CSRs)

The MCU-32X implements machine-mode CSRs for system control and monitoring.

### B.2.1 Machine Information Registers

| CSR Address | Name | Access | Description |
|-------------|------|---------|-------------|
| 0xF11 | mvendorid | RO | Vendor ID |
| 0xF12 | marchid | RO | Architecture ID |  
| 0xF13 | mimpid | RO | Implementation ID |
| 0xF14 | mhartid | RO | Hardware thread ID |

**Machine Information Register Details:**

**mvendorid (0xF11) - Vendor ID Register**
```
Bits [31:0]: Vendor ID = 0x32580000 (MCU-32X vendor identifier)
```

**marchid (0xF12) - Architecture ID Register**
```  
Bits [31:0]: Architecture ID = 0x00000001 (RISC-V RV32I implementation)
```

**mimpid (0xF13) - Implementation ID Register**
```
Bits [31:0]: Implementation ID = 0x01000000 (MCU-32X revision 1.0)
```

**mhartid (0xF14) - Hardware Thread ID Register**
```
Bits [31:0]: Hart ID = 0x00000000 (Single-hart implementation)
```

### B.2.2 Machine Status Registers

| CSR Address | Name | Access | Description |
|-------------|------|---------|-------------|
| 0x300 | mstatus | RW | Machine status register |
| 0x301 | misa | RW | ISA and extensions register |
| 0x302 | medeleg | RW | Machine exception delegation |
| 0x303 | mideleg | RW | Machine interrupt delegation |
| 0x304 | mie | RW | Machine interrupt enable |
| 0x305 | mtvec | RW | Machine trap vector base |
| 0x306 | mcounteren | RW | Machine counter enable |

**mstatus (0x300) - Machine Status Register**
```
Bit 31    30:23  22   21   20   19   18   17   16:15  14:13  12:11  10:8   7     6:4   3     2:0
+------+-------+----+----+----+----+----+----+------+------+------+------+-----+-----+-----+-----+
|  SD  |   0   | TSR| TW | TVM| MXR|SUM |MPRV|  XS  | FS  | MPP  |  0   | MPIE|  0  | MIE |  0  |
+------+-------+----+----+----+----+----+----+------+------+------+------+-----+-----+-----+-----+
```

| Field | Bits | Access | Description |
|-------|------|---------|-------------|
| SD | 31 | RO | State dirty (XS or FS dirty) |
| TSR | 22 | RW | Trap SRET (not applicable in M-mode only) |
| TW | 21 | RW | Timeout wait (not applicable in M-mode only) |
| TVM | 20 | RW | Trap virtual memory (not applicable) |
| MXR | 19 | RW | Make executable readable |
| SUM | 18 | RW | Supervisor user memory access |
| MPRV | 17 | RW | Modify privilege |
| XS | 16:15 | RO | Extension context status |
| FS | 14:13 | RW | Floating-point context status |
| MPP | 12:11 | RW | Machine previous privilege |
| MPIE | 7 | RW | Machine previous interrupt enable |
| MIE | 3 | RW | Machine interrupt enable |

**misa (0x301) - ISA and Extensions Register**
```
Bit 31:30  29:26  25:0
+------+-------+-------------------------+
| MXL  |   0   |      Extensions         |
+------+-------+-------------------------+
```

| Field | Bits | Value | Description |
|-------|------|-------|-------------|
| MXL | 31:30 | 01 | Machine XLEN (32-bit) |
| Extensions | 25:0 | 0x00000100 | I = Base integer instruction set |

**mie (0x304) - Machine Interrupt Enable Register**
```
Bit 31:16  15:12  11   10:8   7    6:4   3    2:0
+------+-------+----+------+----+-----+----+-----+
|Local |   0   |MEIE|  0   |MTIE|  0  |MSIE|  0  |
+------+-------+----+------+----+-----+----+-----+
```

| Field | Bits | Description |
|-------|------|-------------|
| Local | 31:16 | Local interrupt enables |
| MEIE | 11 | Machine external interrupt enable |
| MTIE | 7 | Machine timer interrupt enable |
| MSIE | 3 | Machine software interrupt enable |

**mtvec (0x305) - Machine Trap Vector Base Address Register**
```
Bit 31:2   1:0
+-------+-----+
| BASE  |MODE |
+-------+-----+
```

| Field | Bits | Description |
|-------|------|-------------|
| BASE | 31:2 | Vector base address (4-byte aligned) |
| MODE | 1:0 | Vector mode: 0=Direct, 1=Vectored |

### B.2.3 Machine Protection and Translation Registers

| CSR Address | Name | Access | Description |
|-------------|------|---------|-------------|
| 0x3A0 | pmpcfg0 | RW | Physical memory protection config 0 |
| 0x3A1 | pmpcfg1 | RW | Physical memory protection config 1 |
| 0x3A2 | pmpcfg2 | RW | Physical memory protection config 2 |
| 0x3A3 | pmpcfg3 | RW | Physical memory protection config 3 |
| 0x3B0-0x3BF | pmpaddr0-15 | RW | Physical memory protection addresses |

**pmpcfg0-3 (0x3A0-0x3A3) - PMP Configuration Registers**
```
Each register contains 4 PMP entries (8 bits each):

Bit 31:24  23:16  15:8   7:0
+------+------+-----+-----+
| PMP3 | PMP2 | PMP1| PMP0|
+------+------+-----+-----+

Each PMP entry format:
Bit 7    6:5    4:3    2    1    0
+----+------+------+----+----+----+
| L  | RES  |  A   | X  | W  | R  |
+----+------+------+----+----+----+
```

| Field | Bits | Description |
|-------|------|-------------|
| L | 7 | Lock bit |
| A | 4:3 | Address matching mode (00=OFF, 01=TOR, 10=NA4, 11=NAPOT) |
| X | 2 | Execute permission |
| W | 1 | Write permission |
| R | 0 | Read permission |

### B.2.4 Machine Counter/Timer Registers

| CSR Address | Name | Access | Description |
|-------------|------|---------|-------------|
| 0xB00 | mcycle | RW | Machine cycle counter |
| 0xB02 | minstret | RW | Machine instruction retired counter |
| 0xB03-0xB1F | mhpmcounter3-31 | RW | Machine performance counters |
| 0xB80 | mcycleh | RW | Upper 32 bits of mcycle |
| 0xB82 | minstreth | RW | Upper 32 bits of minstret |
| 0xB83-0xB9F | mhpmcounter3h-31h | RW | Upper 32 bits of performance counters |

**Performance Counter Configuration:**
```assembly
# Read 64-bit cycle count
read_cycle_count:
1:  csrr  t1, mcycleh        # Read upper half
    csrr  t0, mcycle         # Read lower half
    csrr  t2, mcycleh        # Re-read upper half
    bne   t1, t2, 1b         # Retry if upper changed
    # t1:t0 now contains 64-bit cycle count
    ret

# Read instruction count
read_instruction_count:
1:  csrr  a1, minstreth      # Read upper half
    csrr  a0, minstret       # Read lower half  
    csrr  t0, minstreth      # Re-read upper half
    bne   a1, t0, 1b         # Retry if upper changed
    # a1:a0 now contains 64-bit instruction count
    ret
```

### B.2.5 Machine Exception Registers

| CSR Address | Name | Access | Description |
|-------------|------|---------|-------------|
| 0x340 | mscratch | RW | Machine scratch register |
| 0x341 | mepc | RW | Machine exception program counter |
| 0x342 | mcause | RW | Machine exception cause |
| 0x343 | mtval | RW | Machine trap value |
| 0x344 | mip | RW | Machine interrupt pending |

**mcause (0x342) - Machine Cause Register**
```
Bit 31   30:0
+----+--------+
|INT |  CODE  |
+----+--------+
```

| Interrupt (INT=1) | Code | Description |
|------------------|------|-------------|
| 1 | 3 | Machine software interrupt |
| 1 | 7 | Machine timer interrupt |  
| 1 | 11 | Machine external interrupt |
| 1 | 16-31 | Local interrupts |

| Exception (INT=0) | Code | Description |
|------------------|------|-------------|
| 0 | 0 | Instruction address misaligned |
| 0 | 1 | Instruction access fault |
| 0 | 2 | Illegal instruction |
| 0 | 3 | Breakpoint |
| 0 | 4 | Load address misaligned |
| 0 | 5 | Load access fault |
| 0 | 6 | Store/AMO address misaligned |
| 0 | 7 | Store/AMO access fault |
| 0 | 8 | Environment call from M-mode |
| 0 | 11 | Environment call from M-mode |

**mip (0x344) - Machine Interrupt Pending Register**
```
Same format as mie register - shows pending interrupts
```

### B.2.6 CSR Access Instructions

**CSR Manipulation Instructions:**
```assembly
# CSR read/write operations
csrr    t0, mstatus         # Read CSR to register (csrrs t0, mstatus, x0)
csrw    mstatus, t0         # Write register to CSR (csrrw x0, mstatus, t0)

# CSR bit manipulation
csrs    mstatus, t0         # Set bits (OR operation)
csrc    mstatus, t0         # Clear bits (AND with complement)

# CSR immediate operations  
csrwi   mstatus, 8          # Write immediate to CSR
csrsi   mstatus, 8          # Set bits with immediate
csrci   mstatus, 8          # Clear bits with immediate

# Atomic CSR read-modify-write
csrrw   t0, mstatus, t1     # Atomically swap t1 with mstatus, old value to t0
csrrs   t0, mstatus, t1     # Atomically OR t1 with mstatus, old value to t0  
csrrc   t0, mstatus, t1     # Atomically clear bits in mstatus using t1 mask

# Example: Disable interrupts and save state
csrci   mstatus, 8          # Clear MIE bit (disable interrupts)
# or
csrrci  t0, mstatus, 8      # Clear MIE and save old mstatus in t0
```

---

## B.3 Special Purpose Registers

### B.3.1 Program Counter (PC)

The Program Counter is not directly accessible as a CSR, but can be read through:
- **mepc**: During exception handling
- **auipc**: Add upper immediate to PC instruction
- **jal/jalr**: Jump and link instructions save PC+4

**PC Manipulation Examples:**
```assembly
# Get current PC value (PC of auipc instruction)
auipc t0, 0                 # t0 = current PC

# Get PC of next instruction
auipc t0, 0                 # t0 = PC of this instruction
addi  t0, t0, 4             # t0 = PC of next instruction

# PC-relative addressing
auipc t0, %pcrel_hi(symbol) # Load upper PC-relative address
addi  t0, t0, %pcrel_lo(symbol) # Add lower PC-relative offset

# Function return address handling
jal   ra, function          # Save PC+4 in ra, jump to function
# ... function executes ...
jr    ra                    # Return using saved address
# or
jalr  x0, ra, 0             # Equivalent return
```

### B.3.2 Stack Pointer (SP)

The stack pointer (x2/sp) is a general-purpose register with special significance:

**Stack Management:**
```assembly
# Function prologue - allocate stack frame
function_entry:
    addi  sp, sp, -32       # Allocate 32-byte stack frame
    sw    ra, 28(sp)        # Save return address
    sw    s0, 24(sp)        # Save frame pointer
    sw    s1, 20(sp)        # Save callee-saved register
    addi  s0, sp, 32        # Set up frame pointer
    
    # Function body
    # ...
    
# Function epilogue - deallocate stack frame  
function_exit:
    lw    s1, 20(sp)        # Restore callee-saved register
    lw    s0, 24(sp)        # Restore frame pointer
    lw    ra, 28(sp)        # Restore return address
    addi  sp, sp, 32        # Deallocate stack frame
    ret

# Stack pointer alignment
# Stack should be 16-byte aligned for ABI compliance
    andi  t0, sp, 15        # Check alignment
    beqz  t0, aligned       # Branch if already aligned
    andi  sp, sp, -16       # Force 16-byte alignment
aligned:
```

### B.3.3 Global Pointer (GP)

The global pointer (x3/gp) provides efficient access to global variables:

**Global Pointer Setup:**
```assembly
# Initialize global pointer (done once at startup)
.option push
.option norelax
    la   gp, __global_pointer$    # Load global pointer address  
.option pop

# Access global variables using GP-relative addressing
global_var_access:
    lw   t0, %gprel(global_var)(gp)    # Load from global variable
    addi t0, t0, 1                     # Increment value
    sw   t0, %gprel(global_var)(gp)    # Store back to global variable
```

### B.3.4 Thread Pointer (TP)

The thread pointer (x4/tp) supports thread-local storage:

**Thread-Local Storage:**
```assembly
# Thread pointer setup (per thread)
setup_thread_pointer:
    la   t0, thread_local_base    # Get thread-local base address
    add  t0, t0, thread_id        # Add thread-specific offset
    mv   tp, t0                   # Set thread pointer
    
# Access thread-local variables
access_thread_local:
    lw   t0, %tprel(tls_var)(tp)  # Load from thread-local variable
    addi t0, t0, 1                # Modify value
    sw   t0, %tprel(tls_var)(tp)  # Store back
```

---

## B.4 Register Usage in C Calling Convention

### B.4.1 Function Call ABI

**Register Allocation by GCC:**

| Usage Class | Registers | Description |
|-------------|-----------|-------------|
| Argument/Return | a0-a7 (x10-x17) | Function arguments and return values |
| Temporary | t0-t2 (x5-x7) | Temporary registers (caller-saved) |
| Temporary | t3-t6 (x28-x31) | Additional temporary registers |
| Saved | s0-s11 (x8-x9, x18-x27) | Callee-saved registers |
| Special | ra (x1) | Return address |
| Special | sp (x2) | Stack pointer |
| Special | gp (x3) | Global pointer |
| Special | tp (x4) | Thread pointer |
| Zero | zero (x0) | Constant zero |

### B.4.2 C Function Examples

**C Function with Register Usage:**
```c
// C function prototype
int calculate_sum(int a, int b, int c, int d, int e);

// Assembly implementation showing register usage
```

```assembly
calculate_sum:
    # Arguments: a0=a, a1=b, a2=c, a3=d, a4=e
    # Return value in a0
    
    # Save callee-saved registers if needed
    addi  sp, sp, -16
    sw    s0, 12(sp)
    sw    s1, 8(sp)
    
    # Perform calculations using temporary registers
    add   t0, a0, a1        # t0 = a + b
    add   t1, a2, a3        # t1 = c + d  
    add   s0, t0, t1        # s0 = (a+b) + (c+d)
    add   a0, s0, a4        # return (a+b+c+d) + e
    
    # Restore callee-saved registers
    lw    s1, 8(sp)
    lw    s0, 12(sp)
    addi  sp, sp, 16
    
    ret
```

**Structure Passing:**
```c
typedef struct {
    int x, y, z, w;
} point4d_t;

int process_point(point4d_t *pt, int scale);
```

```assembly
process_point:
    # Arguments: a0=pt (pointer), a1=scale
    
    # Load structure members
    lw   t0, 0(a0)          # t0 = pt->x
    lw   t1, 4(a0)          # t1 = pt->y
    lw   t2, 8(a0)          # t2 = pt->z
    lw   t3, 12(a0)         # t3 = pt->w
    
    # Scale all components
    mul  t0, t0, a1         # x *= scale
    mul  t1, t1, a1         # y *= scale  
    mul  t2, t2, a1         # z *= scale
    mul  t3, t3, a1         # w *= scale
    
    # Store back to structure
    sw   t0, 0(a0)          # pt->x = scaled x
    sw   t1, 4(a0)          # pt->y = scaled y
    sw   t2, 8(a0)          # pt->z = scaled z
    sw   t3, 12(a0)         # pt->w = scaled w
    
    # Return sum as result
    add  a0, t0, t1         # a0 = x + y
    add  t4, t2, t3         # t4 = z + w
    add  a0, a0, t4         # return x + y + z + w
    
    ret
```

---

## B.5 Register Save/Restore Patterns

### B.5.1 Minimal Save/Restore

**Simple Leaf Function:**
```assembly
simple_leaf_function:
    # No need to save registers for leaf function
    # that only uses temporary registers
    
    add  t0, a0, a1         # Use arguments
    sll  t1, t0, 2          # Shift left by 2
    add  a0, t1, a0         # Calculate result
    
    ret                     # Return (no register restore needed)
```

### B.5.2 Standard Save/Restore

**Function Using Callee-Saved Registers:**
```assembly
standard_function:
    # Prologue - save used callee-saved registers
    addi sp, sp, -32        # Allocate stack space
    sw   ra, 28(sp)         # Save return address
    sw   s0, 24(sp)         # Save s0 (frame pointer)
    sw   s1, 20(sp)         # Save s1
    sw   s2, 16(sp)         # Save s2
    
    # Set up frame pointer
    addi s0, sp, 32         # s0 points to original sp
    
    # Function body using s1, s2 for local variables
    mv   s1, a0             # Save first argument
    mv   s2, a1             # Save second argument
    
    # ... function logic ...
    
    # Epilogue - restore registers
    lw   s2, 16(sp)         # Restore s2
    lw   s1, 20(sp)         # Restore s1  
    lw   s0, 24(sp)         # Restore frame pointer
    lw   ra, 28(sp)         # Restore return address
    addi sp, sp, 32         # Deallocate stack space
    
    ret
```

### B.5.3 Context Switch Save/Restore

**Full Context Save for Task Switching:**
```assembly
save_context:
    # Save all registers to context structure
    # a0 points to context save area
    
    sw   x1, 4(a0)          # Save ra
    sw   x2, 8(a0)          # Save sp  
    sw   x3, 12(a0)         # Save gp
    sw   x4, 16(a0)         # Save tp
    sw   x5, 20(a0)         # Save t0
    sw   x6, 24(a0)         # Save t1
    sw   x7, 28(a0)         # Save t2
    sw   x8, 32(a0)         # Save s0/fp
    sw   x9, 36(a0)         # Save s1
    # ... continue for all registers ...
    sw   x31, 124(a0)       # Save t6
    
    # Save CSRs
    csrr t0, mstatus
    sw   t0, 128(a0)        # Save mstatus
    csrr t0, mepc  
    sw   t0, 132(a0)        # Save mepc
    
    ret

restore_context:
    # Restore all registers from context structure
    # a0 points to context save area
    
    # Restore CSRs first
    lw   t0, 128(a0)        # Load mstatus
    csrw mstatus, t0
    lw   t0, 132(a0)        # Load mepc
    csrw mepc, t0
    
    # Restore general registers
    lw   x1, 4(a0)          # Restore ra
    lw   x2, 8(a0)          # Restore sp
    lw   x3, 12(a0)         # Restore gp  
    lw   x4, 16(a0)         # Restore tp
    lw   x5, 20(a0)         # Restore t0
    # ... continue for all registers ...
    lw   x31, 124(a0)       # Restore t6
    
    # Note: a0 restored last since it contains context pointer
    lw   x10, 40(a0)        # Restore original a0
    
    ret
```

---

*This appendix provides complete reference information for all MCU-32X registers, enabling efficient programming and debugging of applications across embedded and desktop computing environments.*