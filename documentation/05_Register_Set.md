# Chapter 5: Register Set
## MCU-32X Technical Reference Manual

---

## 5.1 General-Purpose Registers

The MCU-32X implements the standard RISC-V RV32I register set, providing 32 general-purpose registers for computation and data storage.

### 5.1.1 Register Architecture

**Register File Specifications:**
- **Quantity**: 32 registers (x0-x31)
- **Width**: 32 bits each
- **Total Storage**: 1024 bits (128 bytes)
- **Read Ports**: 2 simultaneous
- **Write Ports**: 1 per cycle
- **Reset Value**: All registers except x0 contain undefined values

### 5.1.2 Register Naming Conventions

The RISC-V ABI defines both numeric (x0-x31) and symbolic names for registers:

| Register | ABI Name | Description | Preserved | Usage |
|----------|----------|-------------|-----------|--------|
| x0 | zero | Hard-wired zero | N/A | Always reads 0, writes ignored |
| x1 | ra | Return address | No | Link register for calls |
| x2 | sp | Stack pointer | Yes | Points to stack top |
| x3 | gp | Global pointer | N/A | Points to global data |
| x4 | tp | Thread pointer | N/A | Thread-local storage |
| x5-x7 | t0-t2 | Temporaries | No | Scratch registers |
| x8 | s0/fp | Saved/Frame pointer | Yes | Frame pointer or saved reg |
| x9 | s1 | Saved register | Yes | Preserved across calls |
| x10-x11 | a0-a1 | Function args/return | No | First args/return values |
| x12-x17 | a2-a7 | Function arguments | No | Additional function args |
| x18-x27 | s2-s11 | Saved registers | Yes | Preserved across calls |
| x28-x31 | t3-t6 | Temporaries | No | Additional scratch regs |

---

### 5.1.3 Register x0 (zero) Behavior

**Special Properties:**
- Always returns 0 when read
- Write operations are ignored (no-op)
- Can be used as immediate 0 source
- Optimizes many instruction patterns

**Hardware Implementation:**
```verilog
// Register file read logic
always_comb begin
    if (rs1_addr == 5'b00000)
        rs1_data = 32'h00000000;  // x0 always reads 0
    else
        rs1_data = registers[rs1_addr];
        
    if (rs2_addr == 5'b00000)  
        rs2_data = 32'h00000000;  // x0 always reads 0
    else
        rs2_data = registers[rs2_addr];
end

// Register file write logic
always_ff @(posedge clk) begin
    if (reset) begin
        // x0 doesn't need reset - always reads 0
        // Other registers have undefined reset values
    end
    else if (write_enable && (rd_addr != 5'b00000)) begin
        registers[rd_addr] <= write_data;  // Don't write to x0
    end
end
```

---

### 5.1.4 Stack Pointer (x2/sp) Convention

**Stack Management:**
- Grows downward (decreasing addresses)
- Always points to last used stack location
- Must maintain alignment (4-byte minimum)
- Used for local variables and function calls

**Stack Frame Layout:**
```
Higher Addresses
┌─────────────────┐
│ Previous Frame  │
├─────────────────┤ ← Previous SP
│ Return Address  │ (saved ra)
├─────────────────┤
│ Saved Registers │ (s0-s11 as needed)
├─────────────────┤
│ Local Variables │
├─────────────────┤
│ Arguments >8    │ (if any)
├─────────────────┤ ← Current SP
│ Next Frame      │
└─────────────────┘
Lower Addresses
```

**Stack Operations:**
```assembly
# Function prologue example
function_entry:
    addi sp, sp, -16        # Allocate 16 bytes
    sw   ra, 12(sp)         # Save return address
    sw   s0, 8(sp)          # Save frame pointer
    addi s0, sp, 16         # Set frame pointer

# Function epilogue example  
function_exit:
    lw   ra, 12(sp)         # Restore return address
    lw   s0, 8(sp)          # Restore frame pointer
    addi sp, sp, 16         # Deallocate stack frame
    ret                     # Return to caller
```

---

### 5.1.5 Global Pointer (x3/gp) Usage

**Purpose:**
- Base address for global data access
- Enables 12-bit immediate addressing of globals
- Reduces code size and improves performance

**Address Range:**
- Can address ±2KB from gp base
- Typically points to middle of .sdata section
- Set up by runtime system during initialization

**Usage Example:**
```assembly
# Without global pointer (6 bytes)
lui  t0, %hi(global_var)
lw   t1, %lo(global_var)(t0)

# With global pointer (4 bytes)  
lw   t1, %gp_rel(global_var)(gp)
```

---

## 5.2 Control and Status Registers (CSRs)

The MCU-32X implements essential machine-mode CSRs for system control and status monitoring.

### 5.2.1 Machine Information Registers

#### 5.2.1.1 mvendorid (0xF11) - Vendor ID
**Access**: Read-only  
**Reset Value**: 0x4D435532 ("MCU2")  

| Bits | Name | Description |
|------|------|-------------|
| 31:0 | VENDORID | MCU Technologies vendor identifier |

#### 5.2.1.2 marchid (0xF12) - Architecture ID  
**Access**: Read-only  
**Reset Value**: 0x00000032  

| Bits | Name | Description |
|------|------|-------------|
| 31:0 | ARCHID | MCU-32X architecture identifier (50 decimal) |

#### 5.2.1.3 mimpid (0xF13) - Implementation ID
**Access**: Read-only  
**Reset Value**: 0x00010000  

| Bits | Name | Description |
|------|------|-------------|
| 31:0 | IMPID | Implementation version (1.0) |

#### 5.2.1.4 mhartid (0xF14) - Hardware Thread ID
**Access**: Read-only  
**Reset Value**: 0x00000000  

| Bits | Name | Description |
|------|------|-------------|
| 31:0 | HARTID | Hardware thread ID (always 0 for single-hart) |

---

### 5.2.2 Machine Trap Setup

#### 5.2.2.1 mstatus (0x300) - Machine Status
**Access**: Read/Write  
**Reset Value**: 0x00000000  

| Bits | Name | Access | Description |
|------|------|--------|-------------|
| 31:13 | Reserved | RO | Reserved (reads 0) |
| 12:11 | MPP | RW | Previous privilege mode |
| 10:8 | Reserved | RO | Reserved (reads 0) |
| 7 | MPIE | RW | Previous interrupt enable |
| 6:4 | Reserved | RO | Reserved (reads 0) |
| 3 | MIE | RW | Machine interrupt enable |
| 2:0 | Reserved | RO | Reserved (reads 0) |

**Bit Field Details:**
- **MPP (12:11)**: Always 11 (machine mode) - no other modes supported
- **MPIE (7)**: Saves MIE value when trap taken
- **MIE (3)**: Global interrupt enable for machine mode

#### 5.2.2.2 mie (0x304) - Machine Interrupt Enable
**Access**: Read/Write  
**Reset Value**: 0x00000000  

| Bits | Name | Access | Description |
|------|------|--------|-------------|
| 31:12 | Reserved | RO | Reserved (reads 0) |
| 11 | MEIE | RW | Machine external interrupt enable |
| 10:8 | Reserved | RO | Reserved (reads 0) |  
| 7 | MTIE | RW | Machine timer interrupt enable |
| 6:4 | Reserved | RO | Reserved (reads 0) |
| 3 | MSIE | RW | Machine software interrupt enable |
| 2:0 | Reserved | RO | Reserved (reads 0) |

#### 5.2.2.3 mtvec (0x305) - Machine Trap Vector
**Access**: Read/Write  
**Reset Value**: 0x00000000  

| Bits | Name | Access | Description |
|------|------|--------|-------------|
| 31:2 | BASE | RW | Vector base address |
| 1:0 | MODE | RW | Vector mode (0=Direct, 1=Vectored) |

**Vector Modes:**
- **Direct (0)**: All traps jump to BASE address
- **Vectored (1)**: Interrupts jump to BASE + 4×cause

---

### 5.2.3 Machine Trap Handling

#### 5.2.3.1 mscratch (0x340) - Machine Scratch
**Access**: Read/Write  
**Reset Value**: 0x00000000  

General-purpose register for machine mode software use. Typically used to save a register before trap handler entry.

#### 5.2.3.2 mepc (0x341) - Machine Exception PC
**Access**: Read/Write  
**Reset Value**: 0x00000000  

| Bits | Name | Access | Description |
|------|------|--------|-------------|
| 31:2 | EPC | RW | Exception program counter |
| 1:0 | Reserved | RO | Always 0 (halfword aligned) |

Holds the address to return to after trap handling.

#### 5.2.3.3 mcause (0x342) - Machine Cause
**Access**: Read/Write  
**Reset Value**: 0x00000000  

| Bits | Name | Access | Description |
|------|------|--------|-------------|
| 31 | INT | RW | Interrupt flag (1=interrupt, 0=exception) |
| 30:5 | Reserved | RO | Reserved (reads 0) |
| 4:0 | ECODE | RW | Exception/interrupt code |

**Exception Codes (INT=0):**
- 0: Instruction address misaligned
- 1: Instruction access fault
- 2: Illegal instruction
- 3: Breakpoint
- 4: Load address misaligned
- 5: Load access fault
- 6: Store address misaligned
- 7: Store access fault
- 8: Environment call
- 11: Machine external interrupt

**Interrupt Codes (INT=1):**
- 3: Machine software interrupt
- 7: Machine timer interrupt  
- 11: Machine external interrupt

#### 5.2.3.4 mtval (0x343) - Machine Trap Value
**Access**: Read/Write  
**Reset Value**: 0x00000000  

Provides additional information about the trap:
- **Address exceptions**: Faulting address
- **Illegal instruction**: Instruction bits
- **Other exceptions**: 0

---

### 5.2.4 Machine Memory Protection

The MCU-32X implements a simplified memory protection scheme suitable for embedded applications.

#### 5.2.4.1 Memory Protection Overview

**Protection Regions:**
- 8 configurable memory regions
- Each region has base address and size
- Permissions: Read, Write, Execute
- Machine mode can access all regions

#### 5.2.4.2 Protection Configuration

**Region Configuration Format:**
```c
typedef struct {
    uint32_t base_addr;     /* Region base address */
    uint32_t size_mask;     /* Size = ~mask + 1 */  
    uint32_t attributes;    /* Access permissions */
} mpu_region_t;

/* Attributes bit fields */
#define MPU_READ    (1 << 0)
#define MPU_WRITE   (1 << 1)  
#define MPU_EXEC    (1 << 2)
#define MPU_VALID   (1 << 7)
```

---

### 5.2.5 Performance Monitoring

#### 5.2.5.1 mcycle (0xB00) - Machine Cycle Counter
**Access**: Read/Write  
**Reset Value**: 0x00000000  

Counts processor clock cycles. Can be written for initialization.

#### 5.2.5.2 minstret (0xB02) - Machine Instructions Retired
**Access**: Read/Write  
**Reset Value**: 0x00000000  

Counts completed instructions (not including those that caused exceptions).

#### 5.2.5.3 mcycleh (0xB80) - Upper 32 bits of mcycle
**Access**: Read/Write  
**Reset Value**: 0x00000000  

Upper half of 64-bit cycle counter.

#### 5.2.5.4 minstreth (0xB82) - Upper 32 bits of minstret  
**Access**: Read/Write  
**Reset Value**: 0x00000000  

Upper half of 64-bit instruction counter.

---

### 5.2.6 Machine Counter Setup

#### 5.2.6.1 mcountinhibit (0x320) - Counter Inhibit
**Access**: Read/Write  
**Reset Value**: 0x00000000  

| Bits | Name | Access | Description |
|------|------|--------|-------------|
| 31:3 | Reserved | RO | Reserved (reads 0) |
| 2 | IR | RW | Inhibit minstret increment |
| 1 | Reserved | RO | Reserved (reads 0) |
| 0 | CY | RW | Inhibit mcycle increment |

Setting a bit stops the corresponding counter from incrementing.

---

## 5.3 Register Access Instructions

### 5.3.1 CSR Access Instructions

**CSRRW - CSR Read/Write:**
```assembly
csrrw rd, csr, rs1      # rd = csr; csr = rs1
csrrw x0, csr, rs1      # csr = rs1 (no read)
```

**CSRRS - CSR Read/Set:**
```assembly
csrrs rd, csr, rs1      # rd = csr; csr |= rs1  
csrrs rd, csr, x0       # rd = csr (read only)
```

**CSRRC - CSR Read/Clear:**
```assembly
csrrc rd, csr, rs1      # rd = csr; csr &= ~rs1
```

**Immediate Variants:**
```assembly
csrrwi rd, csr, imm     # rd = csr; csr = zero_extend(imm)
csrrsi rd, csr, imm     # rd = csr; csr |= zero_extend(imm)
csrrci rd, csr, imm     # rd = csr; csr &= ~zero_extend(imm)
```

### 5.3.2 Common CSR Usage Patterns

**Reading Status:**
```assembly
csrr t0, mstatus        # Read machine status
csrr t1, mcause         # Read trap cause
csrr t2, mepc           # Read exception PC
```

**Interrupt Control:**
```assembly
csrci mstatus, 0x8      # Disable interrupts (clear MIE)
csrsi mstatus, 0x8      # Enable interrupts (set MIE)
```

**Performance Monitoring:**
```assembly
csrr t0, mcycle         # Read cycle counter
csrr t1, minstret       # Read instruction counter
```

---

## 5.4 Register Usage Guidelines

### 5.4.1 Function Calling Convention

**Caller-Saved Registers:**
- Must be saved by calling function if needed across calls
- Registers: ra, t0-t6, a0-a7
- Used for temporary values and arguments

**Callee-Saved Registers:**  
- Must be preserved by called function
- Registers: sp, s0-s11
- Used for variables that live across function calls

**Function Call Example:**
```assembly
caller:
    # Save caller-saved registers if needed
    addi sp, sp, -8
    sw   t0, 4(sp)
    sw   ra, 0(sp)
    
    # Set up arguments
    li   a0, 100            # First argument
    li   a1, 200            # Second argument
    
    # Call function
    jal  ra, function
    
    # Result in a0, restore registers
    lw   ra, 0(sp)
    lw   t0, 4(sp)
    addi sp, sp, 8
    ret

function:
    # Save callee-saved registers  
    addi sp, sp, -4
    sw   s0, 0(sp)
    
    # Function body - can use s0 safely
    add  s0, a0, a1         # s0 = a0 + a1
    # ... more computation ...
    mv   a0, s0             # Return value
    
    # Restore and return
    lw   s0, 0(sp)
    addi sp, sp, 4
    ret
```

### 5.4.2 Register Allocation Strategy

**Priority Order for Allocation:**
1. **t0-t6**: Temporary computations, no preservation needed
2. **s2-s11**: Long-lived variables, preserved across calls
3. **a0-a7**: Arguments and return values, short-lived
4. **s0**: Frame pointer or additional saved register
5. **s1**: Additional saved register

**Optimization Tips:**
- Use temporaries for short computations
- Reserve saved registers for loop variables
- Minimize register spills through careful allocation
- Use x0 as source for immediate 0 values

---

*This chapter provided complete documentation of the MCU-32X register architecture, enabling efficient programming and system software development.*