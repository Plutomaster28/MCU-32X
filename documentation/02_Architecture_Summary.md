# Chapter 2: Architecture Summary
## MCU-32X Technical Reference Manual

---

## 2.1 Processor Architecture Overview

The MCU-32X implements a classic 32-bit RISC architecture based on the RISC-V RV32I instruction set specification. The processor features a Harvard architecture with separate instruction and data paths, enabling simultaneous instruction fetch and data access operations for maximum throughput.

### 2.1.1 Block Diagram

```
                    MCU-32X Processor Core
    ┌─────────────────────────────────────────────────────────────┐
    │                                                             │
    │  ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐    │
    │  │ Fetch   │──→│ Decode  │──→│Execute  │──→│ Memory  │    │
    │  │ Stage   │   │ Stage   │   │ Stage   │   │ Stage   │    │
    │  │   IF    │   │   ID    │   │   EX    │   │   MEM   │    │
    │  └─────────┘   └─────────┘   └─────────┘   └─────────┘    │
    │       │             │             │             │         │
    │       │             │             │             │         │
    │  ┌─────────────────────────────────────────────────────────┤
    │  │                Writeback Stage (WB)                    │
    │  └─────────────────────────────────────────────────────────┤
    │                                                             │
    │  ┌─────────────────────────────────────────────────────────┤
    │  │              Register File (32 x 32-bit)                │
    │  │                   (x0 hardwired to 0)                   │
    │  └─────────────────────────────────────────────────────────┤
    │                                                             │
    │  ┌─────────────────┐  ┌─────────────────┐                 │
    │  │ Arithmetic      │  │ Interrupt       │                 │
    │  │ Logic Unit      │  │ Controller      │                 │
    │  │ (32-bit)        │  │                 │                 │
    │  └─────────────────┘  └─────────────────┘                 │
    │                                                             │
    └─────────────────────────────────────────────────────────────┘
              │                               │
              ▼                               ▼
    ┌─────────────────┐              ┌─────────────────┐
    │ Instruction     │              │ Data Memory     │
    │ Memory          │              │ Interface       │
    │ Interface       │              │                 │
    │ (32-bit addr)   │              │ (32-bit addr)   │
    │ (32-bit data)   │              │ (32-bit data)   │
    └─────────────────┘              └─────────────────┘
```

### 2.1.2 Pipeline Architecture

The MCU-32X employs a classical five-stage RISC pipeline optimized for single-cycle throughput:

| Stage | Abbreviation | Function | Duration |
|-------|--------------|----------|----------|
| **Instruction Fetch** | IF | Fetch instruction from memory | 1 cycle |
| **Instruction Decode** | ID | Decode instruction and read registers | 1 cycle |
| **Execute** | EX | ALU operations and address calculation | 1 cycle |
| **Memory Access** | MEM | Load/store operations | 1 cycle |
| **Writeback** | WB | Write results to register file | 1 cycle |

## 2.2 Instruction Set Architecture

### 2.2.1 RISC-V RV32I Base Integer Instruction Set

The MCU-32X implements the complete RV32I base integer instruction set, providing a comprehensive foundation for general-purpose computing:

**Instruction Categories:**
- **Arithmetic Instructions**: 10 operations (ADD, SUB, SLT, SLTU, AND, OR, XOR, SLL, SRL, SRA)
- **Immediate Instructions**: 9 operations (ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI)
- **Load Instructions**: 5 operations (LB, LH, LW, LBU, LHU)
- **Store Instructions**: 3 operations (SB, SH, SW)
- **Branch Instructions**: 6 operations (BEQ, BNE, BLT, BGE, BLTU, BGEU)
- **Jump Instructions**: 2 operations (JAL, JALR)
- **Upper Immediate**: 2 operations (LUI, AUIPC)
- **System Instructions**: FENCE, ECALL, EBREAK (framework implemented)

**Total: 37 instructions** providing complete computational capability

### 2.2.2 Instruction Encoding Formats

The MCU-32X supports all six RISC-V instruction formats:

```
R-Type (Register-Register):
31    25 24  20 19  15 14  12 11   7 6    0
├─funct7─┤├─rs2─┤├─rs1─┤├funct3┤├─rd──┤├opcode┤

I-Type (Immediate):
31        20 19  15 14  12 11   7 6    0  
├─────imm[11:0]────┤├─rs1─┤├funct3┤├─rd──┤├opcode┤

S-Type (Store):
31    25 24  20 19  15 14  12 11     7 6    0
├─imm[11:5]─┤├─rs2─┤├─rs1─┤├funct3┤├imm[4:0]┤├opcode┤

B-Type (Branch):
31 30    25 24  20 19  15 14  12 11    8 7 6    0
├i├─imm[10:5]─┤├─rs2─┤├─rs1─┤├funct3┤├imm[4:1]┤├i├opcode┤

U-Type (Upper Immediate):
31            12 11   7 6    0
├──────imm[31:12]──────┤├─rd──┤├opcode┤

J-Type (Jump):  
31 30      21 20 19      12 11   7 6    0
├i├─imm[10:1]─┤├i├─imm[19:12]─┤├─rd──┤├opcode┤
```

### 2.2.3 Addressing Modes

The MCU-32X supports the following addressing modes:

- **Register Direct**: Operands from registers (`add x1, x2, x3`)
- **Immediate**: One operand from instruction (`addi x1, x2, 100`)  
- **Base + Displacement**: Memory access (`lw x1, 8(x2)`)
- **PC-Relative**: Branch targets (`beq x1, x2, loop`)
- **Absolute**: Jump targets (`jal x1, function`)

## 2.3 Register Architecture

### 2.3.1 General-Purpose Registers

The MCU-32X provides 32 general-purpose registers, each 32 bits wide:

| Register | ABI Name | Description | Caller/Callee |
|----------|----------|-------------|---------------|
| x0 | zero | Hardware zero | - |
| x1 | ra | Return address | Caller |
| x2 | sp | Stack pointer | Callee |
| x3 | gp | Global pointer | - |
| x4 | tp | Thread pointer | - |
| x5-x7 | t0-t2 | Temporary registers | Caller |
| x8 | s0/fp | Saved register/Frame pointer | Callee |
| x9 | s1 | Saved register | Callee |
| x10-x11 | a0-a1 | Function arguments/Return values | Caller |
| x12-x17 | a2-a7 | Function arguments | Caller |
| x18-x27 | s2-s11 | Saved registers | Callee |
| x28-x31 | t3-t6 | Temporary registers | Caller |

### 2.3.2 Special Register Behavior

- **x0 (zero)**: Always reads as 0, writes are ignored (hardware enforced)
- **x1 (ra)**: Used by JAL/JALR for return addresses
- **x2 (sp)**: Software stack pointer by convention
- **All others**: General-purpose with no hardware restrictions

### 2.3.3 Register File Implementation

```
Register File Specifications:
• 32 registers × 32 bits = 1024 bits total storage
• Dual-read, single-write ports
• Single-cycle read/write operations  
• x0 hardwired to zero (hardware implementation)
• Bypass network for pipeline hazard resolution
```

## 2.4 Memory Architecture

### 2.4.1 Address Space Organization

The MCU-32X provides a flat 32-bit address space with the following organization:

```
Memory Map:
0xFFFF_FFFF ┌──────────────────┐
            │ System Registers │  (256MB)
0xF000_0000 ├──────────────────┤
            │ Peripheral Space │  (256MB)
0xE000_0000 ├──────────────────┤
            │ Reserved         │
            │                  │
0x8000_0000 ├──────────────────┤
            │                  │
            │ Main Memory      │  (2GB)
            │ (RAM/ROM)        │
            │                  │
0x0000_0000 └──────────────────┘
```

### 2.4.2 Memory Interface

**Instruction Memory Interface:**
- 32-bit address bus (4GB address space)
- 32-bit data bus (word-aligned accesses)
- Single-cycle access with ready signaling
- Harvard architecture (separate from data)

**Data Memory Interface:**  
- 32-bit address bus (4GB address space)
- 32-bit data bus with byte enable strobes
- Byte, halfword, and word access modes
- Load/store architecture (no memory-to-memory operations)

### 2.4.3 Memory Access Types

| Access Type | Size | Alignment | Byte Enables | Cycles |
|-------------|------|-----------|--------------|--------|
| Byte (LB/SB) | 8-bit | Any | 1 of 4 | 1-2 |
| Halfword (LH/SH) | 16-bit | Even | 2 of 4 | 1-2 |  
| Word (LW/SW) | 32-bit | Word | All 4 | 1-2 |

## 2.5 Execution Units

### 2.5.1 Arithmetic Logic Unit (ALU)

The MCU-32X ALU supports all RV32I arithmetic and logical operations:

**Arithmetic Operations:**
- Addition/Subtraction (ADD/SUB/ADDI)
- Comparison (SLT/SLTU/SLTI/SLTIU)

**Logical Operations:**
- Bitwise AND/OR/XOR (AND/OR/XOR/ANDI/ORI/XORI)
- Shift operations (SLL/SRL/SRA/SLLI/SRLI/SRAI)

**ALU Features:**
- 32-bit datapath width
- Single-cycle operation for most instructions  
- Integrated zero detection for branch conditions
- Overflow detection (available for software use)

### 2.5.2 Branch Unit

**Branch Types Supported:**
- Conditional branches (BEQ, BNE, BLT, BGE, BLTU, BGEU)
- Unconditional jumps (JAL, JALR)
- PC-relative addressing for branches
- Absolute addressing for jumps

**Branch Performance:**
- Single-cycle branch detection
- Static branch prediction (backward taken, forward not taken)
- 2-cycle branch penalty on misprediction
- Return address stack for function calls (software managed)

### 2.5.3 Load/Store Unit

**Supported Operations:**
- Load operations: LB, LH, LW, LBU, LHU
- Store operations: SB, SH, SW
- Base + immediate addressing mode
- Automatic sign/zero extension for sub-word loads

**Memory Interface Features:**  
- Single-cycle address generation
- Configurable memory timing (1-2 cycles)
- Byte-level write enables for sub-word stores
- Unaligned access support (software emulated)

## 2.6 Pipeline Control and Hazards

### 2.6.1 Pipeline Hazards

**Data Hazards:**
- Read-after-write (RAW) dependencies
- Hardware forwarding/bypassing implemented
- Automatic stall insertion when forwarding impossible

**Control Hazards:**
- Branch instructions cause pipeline flush on misprediction
- Static branch prediction reduces average penalty
- Jump instructions always cause 1-cycle delay

**Structural Hazards:**
- Avoided through Harvard architecture design
- Separate instruction and data memory interfaces
- Dedicated functional units prevent resource conflicts

### 2.6.2 Pipeline Performance

**Ideal Performance:**
- 1 instruction per cycle throughput
- 100 MIPS at 100 MHz operation
- CPI (Cycles Per Instruction) = 1.0 for straight-line code

**Real-World Performance:**
- Branch penalty: ~1.2 average cycles per branch
- Load-use penalty: ~0.1 average cycles (with forwarding)
- Overall CPI: ~1.15 for typical code mixes

---

*This chapter provided an architectural overview of the MCU-32X processor. The following chapters will detail specific subsystems and programming models.*