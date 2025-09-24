# Chapter 7: Instruction Set Architecture
## MCU-32X Technical Reference Manual

---

## 7.1 Instruction Set Overview

The MCU-32X implements the complete RISC-V RV32I base integer instruction set, providing a comprehensive foundation for general-purpose computing. The instruction set consists of 37 instructions organized into six categories, each optimized for single-cycle execution in the MCU-32X pipeline.

### 7.1.1 Instruction Set Philosophy

**RISC Design Principles:**
- **Fixed 32-bit instruction length** for simplified decode and fetch
- **Load/store architecture** with no memory-to-memory operations  
- **Large register set** (32 registers) to minimize memory traffic
- **Orthogonal operations** enabling efficient compiler code generation
- **Simple addressing modes** reducing instruction complexity

**Performance Optimization:**
- **Single-cycle execution** for arithmetic and logical operations
- **Minimal instruction variants** reducing decode complexity
- **Regular instruction encoding** enabling parallel decode paths
- **Predictable execution timing** for real-time applications

### 7.1.2 Instruction Categories

| Category | Instructions | Count | Description |
|----------|-------------|-------|-------------|
| **Arithmetic** | ADD, SUB, SLT, SLTU, etc. | 10 | Register-register operations |
| **Immediate** | ADDI, SLTI, ANDI, etc. | 9 | Register-immediate operations |
| **Load** | LB, LH, LW, LBU, LHU | 5 | Memory to register transfers |
| **Store** | SB, SH, SW | 3 | Register to memory transfers |
| **Branch** | BEQ, BNE, BLT, BGE, etc. | 6 | Conditional control flow |
| **Jump** | JAL, JALR | 2 | Unconditional control flow |
| **Upper Immediate** | LUI, AUIPC | 2 | 20-bit immediate operations |
| **Total** | | **37** | **Complete RV32I implementation** |

## 7.2 Instruction Formats

The MCU-32X supports all six RISC-V instruction formats, each optimized for specific operation types:

### 7.2.1 R-Type Instructions (Register-Register)

```
31    25 24  20 19  15 14  12 11   7 6    0
├─funct7─┤├─rs2─┤├─rs1─┤├funct3┤├─rd──┤├opcode┤
 7 bits   5 bits 5 bits 3 bits  5 bits 7 bits
```

**Field Descriptions:**
- **opcode[6:0]**: Operation code (0110011 for R-type)
- **rd[11:7]**: Destination register (x0-x31)
- **funct3[14:12]**: Function code (operation selector)  
- **rs1[19:15]**: Source register 1 (x0-x31)
- **rs2[24:20]**: Source register 2 (x0-x31)
- **funct7[31:25]**: Extended function code (operation modifier)

**Usage Examples:**
```assembly
add  x1, x2, x3    # x1 = x2 + x3
sub  x4, x5, x6    # x4 = x5 - x6  
and  x7, x8, x9    # x7 = x8 & x9
or   x10, x11, x12 # x10 = x11 | x12
```

### 7.2.2 I-Type Instructions (Immediate)

```
31        20 19  15 14  12 11   7 6    0
├─────imm[11:0]────┤├─rs1─┤├funct3┤├─rd──┤├opcode┤
    12 bits         5 bits 3 bits  5 bits 7 bits
```

**Field Descriptions:**
- **opcode[6:0]**: Operation code (varies by instruction type)
- **rd[11:7]**: Destination register
- **funct3[14:12]**: Function code
- **rs1[19:15]**: Source register 1  
- **imm[31:20]**: 12-bit signed immediate (sign-extended to 32 bits)

**Usage Examples:**
```assembly
addi x1, x2, 100   # x1 = x2 + 100
slti x3, x4, -50   # x3 = (x4 < -50) ? 1 : 0
andi x5, x6, 0xFF  # x5 = x6 & 0xFF
lw   x7, 8(x8)     # x7 = MEM[x8 + 8]
```

### 7.2.3 S-Type Instructions (Store)

```
31    25 24  20 19  15 14  12 11     7 6    0
├─imm[11:5]─┤├─rs2─┤├─rs1─┤├funct3┤├imm[4:0]┤├opcode┤
   7 bits    5 bits 5 bits 3 bits    5 bits   7 bits
```

**Field Descriptions:**
- **opcode[6:0]**: Store operation code (0100011)
- **imm[11:0]**: 12-bit signed offset (split across instruction)
- **funct3[14:12]**: Store size (000=byte, 001=halfword, 010=word)
- **rs1[19:15]**: Base address register
- **rs2[24:20]**: Source data register

**Usage Examples:**
```assembly
sw x1, 0(x2)       # MEM[x2] = x1 (word store)
sh x3, 4(x4)       # MEM[x4+4] = x3[15:0] (halfword store)  
sb x5, -1(x6)      # MEM[x6-1] = x5[7:0] (byte store)
```

### 7.2.4 B-Type Instructions (Branch)

```
31 30    25 24  20 19  15 14  12 11    8 7 6    0
├i├─imm[10:5]─┤├─rs2─┤├─rs1─┤├funct3┤├imm[4:1]┤├i├opcode┤
1b    6 bits   5 bits 5 bits 3 bits   4 bits   1b 7 bits
```

**Field Descriptions:**
- **opcode[6:0]**: Branch operation code (1100011)
- **imm[12:1]**: 12-bit signed branch offset (split and encoded)
- **funct3[14:12]**: Branch condition code
- **rs1[19:15]**: Source register 1 (comparison)
- **rs2[24:20]**: Source register 2 (comparison)

**Usage Examples:**
```assembly
beq x1, x2, loop   # if (x1 == x2) PC += offset
bne x3, x4, end    # if (x3 != x4) PC += offset
blt x5, x6, skip   # if (x5 < x6) PC += offset (signed)
bgeu x7, x8, next  # if (x7 >= x8) PC += offset (unsigned)
```

### 7.2.5 U-Type Instructions (Upper Immediate)

```
31            12 11   7 6    0
├──────imm[31:12]──────┤├─rd──┤├opcode┤
        20 bits         5 bits 7 bits
```

**Field Descriptions:**
- **opcode[6:0]**: Upper immediate operation code
- **rd[11:7]**: Destination register
- **imm[31:12]**: 20-bit immediate (loaded into upper bits)

**Usage Examples:**
```assembly
lui   x1, 0x12345   # x1 = 0x12345000  
auipc x2, 0x1000    # x2 = PC + 0x1000000
```

### 7.2.6 J-Type Instructions (Jump)

```
31 30      21 20 19      12 11   7 6    0
├i├─imm[10:1]─┤├i├─imm[19:12]─┤├─rd──┤├opcode┤
1b   10 bits  1b    8 bits     5 bits 7 bits
```

**Field Descriptions:**
- **opcode[6:0]**: Jump operation code (1101111)
- **rd[11:7]**: Destination register (return address)
- **imm[20:1]**: 20-bit signed jump offset (PC-relative)

**Usage Examples:**
```assembly
jal  x1, function   # x1 = PC+4; PC += offset
jalr x1, 0(x2)      # x1 = PC+4; PC = x2 + 0
```

## 7.3 Arithmetic Instructions

### 7.3.1 Integer Arithmetic Operations

**Addition and Subtraction:**

| Instruction | Encoding | Operation | Overflow |
|-------------|----------|-----------|----------|
| **ADD rd, rs1, rs2** | R-type, funct3=000, funct7=0000000 | rd = rs1 + rs2 | Ignored |
| **SUB rd, rs1, rs2** | R-type, funct3=000, funct7=0100000 | rd = rs1 - rs2 | Ignored |
| **ADDI rd, rs1, imm** | I-type, funct3=000 | rd = rs1 + sign_ext(imm) | Ignored |

**Implementation Notes:**
- All arithmetic is performed in 32-bit two's complement
- Overflow conditions are ignored (no exceptions generated)
- ADDI with rd=x0 creates NOP instruction (common idiom)

**Performance Characteristics:**
- **Execution Time**: 1 cycle
- **Pipeline Stage**: Execute (EX)
- **Forwarding**: Full bypass support from EX and MEM stages

### 7.3.2 Comparison Instructions

**Set Less Than Operations:**

| Instruction | Encoding | Operation | Notes |
|-------------|----------|-----------|-------|
| **SLT rd, rs1, rs2** | R-type, funct3=010 | rd = (rs1 < rs2) ? 1 : 0 | Signed comparison |
| **SLTU rd, rs1, rs2** | R-type, funct3=011 | rd = (rs1 < rs2) ? 1 : 0 | Unsigned comparison |
| **SLTI rd, rs1, imm** | I-type, funct3=010 | rd = (rs1 < imm) ? 1 : 0 | Signed immediate |
| **SLTIU rd, rs1, imm** | I-type, funct3=011 | rd = (rs1 < imm) ? 1 : 0 | Unsigned immediate |

**Comparison Examples:**
```assembly
slt  x1, x2, x3      # x1 = 1 if x2 < x3 (signed)
sltu x4, x5, x6      # x4 = 1 if x5 < x6 (unsigned)
slti x7, x8, -100    # x7 = 1 if x8 < -100
sltiu x9, x10, 0x80000000  # Compare with large unsigned value
```

## 7.4 Logical Instructions

### 7.4.1 Bitwise Operations

**Logical Operations:**

| Instruction | Encoding | Operation | Description |
|-------------|----------|-----------|-------------|
| **AND rd, rs1, rs2** | R-type, funct3=111 | rd = rs1 & rs2 | Bitwise AND |
| **OR rd, rs1, rs2** | R-type, funct3=110 | rd = rs1 \| rs2 | Bitwise OR |
| **XOR rd, rs1, rs2** | R-type, funct3=100 | rd = rs1 ^ rs2 | Bitwise XOR |
| **ANDI rd, rs1, imm** | I-type, funct3=111 | rd = rs1 & sign_ext(imm) | AND immediate |
| **ORI rd, rs1, imm** | I-type, funct3=110 | rd = rs1 \| sign_ext(imm) | OR immediate |
| **XORI rd, rs1, imm** | I-type, funct3=100 | rd = rs1 ^ sign_ext(imm) | XOR immediate |

**Common Logical Patterns:**
```assembly
# Clear bits
andi x1, x2, 0x0F   # Clear upper 28 bits, keep lower 4

# Set bits  
ori  x3, x4, 0x80   # Set bit 7

# Toggle bits
xori x5, x6, 0xFF   # Invert lower 8 bits

# Test for zero
or   x7, x8, x0     # Copy x8 to x7 (test for zero)
```

### 7.4.2 Shift Operations

**Shift Instructions:**

| Instruction | Encoding | Operation | Description |
|-------------|----------|-----------|-------------|
| **SLL rd, rs1, rs2** | R-type, funct3=001, funct7=0000000 | rd = rs1 << rs2[4:0] | Shift left logical |
| **SRL rd, rs1, rs2** | R-type, funct3=101, funct7=0000000 | rd = rs1 >> rs2[4:0] | Shift right logical |
| **SRA rd, rs1, rs2** | R-type, funct3=101, funct7=0100000 | rd = rs1 >>> rs2[4:0] | Shift right arithmetic |
| **SLLI rd, rs1, shamt** | I-type, funct3=001 | rd = rs1 << shamt | Shift left immediate |
| **SRLI rd, rs1, shamt** | I-type, funct3=101, imm[11:5]=0000000 | rd = rs1 >> shamt | Shift right immediate |
| **SRAI rd, rs1, shamt** | I-type, funct3=101, imm[11:5]=0100000 | rd = rs1 >>> shamt | Shift right arithmetic immediate |

**Shift Operation Details:**
- **Shift Amount**: Only lower 5 bits used (0-31 bit positions)
- **Left Shifts**: Fill with zeros from right
- **Right Logical**: Fill with zeros from left  
- **Right Arithmetic**: Fill with sign bit from left

**Shift Examples:**
```assembly
slli x1, x2, 4       # Multiply x2 by 16 (left shift 4)
srli x3, x4, 8       # Divide x4 by 256 unsigned (right shift 8)
srai x5, x6, 2       # Divide x6 by 4 signed (arithmetic right shift)
sll  x7, x8, x9      # Variable shift left by x9[4:0] amount
```

## 7.5 Memory Access Instructions

### 7.5.1 Load Instructions

**Load Operations:**

| Instruction | Encoding | Operation | Sign Extension |
|-------------|----------|-----------|----------------|
| **LB rd, imm(rs1)** | I-type, funct3=000 | rd = sign_ext(MEM[rs1+imm][7:0]) | Sign-extended byte |
| **LH rd, imm(rs1)** | I-type, funct3=001 | rd = sign_ext(MEM[rs1+imm][15:0]) | Sign-extended halfword |
| **LW rd, imm(rs1)** | I-type, funct3=010 | rd = MEM[rs1+imm] | Full 32-bit word |
| **LBU rd, imm(rs1)** | I-type, funct3=100 | rd = zero_ext(MEM[rs1+imm][7:0]) | Zero-extended byte |
| **LHU rd, imm(rs1)** | I-type, funct3=101 | rd = zero_ext(MEM[rs1+imm][15:0]) | Zero-extended halfword |

**Load Addressing:**
- **Base + Displacement**: Effective address = rs1 + sign_ext(imm)
- **Address Range**: 12-bit signed offset (-2048 to +2047)
- **Alignment**: Natural alignment recommended for performance
- **Byte Order**: Little-endian (LSB at lower address)

**Load Examples:**
```assembly
lw  x1, 0(x2)        # Load word from address in x2
lh  x3, 4(x4)        # Load signed halfword from x4+4
lbu x5, -1(x6)       # Load unsigned byte from x6-1
lb  x7, 100(x8)      # Load signed byte from x8+100
```

### 7.5.2 Store Instructions

**Store Operations:**

| Instruction | Encoding | Operation | Data Size |
|-------------|----------|-----------|-----------|
| **SB rs2, imm(rs1)** | S-type, funct3=000 | MEM[rs1+imm][7:0] = rs2[7:0] | 8 bits |
| **SH rs2, imm(rs1)** | S-type, funct3=001 | MEM[rs1+imm][15:0] = rs2[15:0] | 16 bits |
| **SW rs2, imm(rs1)** | S-type, funct3=010 | MEM[rs1+imm] = rs2 | 32 bits |

**Store Addressing:**
- **Base + Displacement**: Effective address = rs1 + sign_ext(imm)
- **Address Range**: 12-bit signed offset (-2048 to +2047)
- **Partial Stores**: Only specified bytes are modified
- **Byte Enables**: Hardware generates appropriate strobes

**Store Examples:**
```assembly
sw  x1, 0(x2)        # Store full word x1 to address x2
sh  x3, 8(x4)        # Store lower halfword x3 to x4+8  
sb  x5, -4(x6)       # Store lower byte x5 to x6-4
```

### 7.5.3 Memory Access Performance

**Load Performance:**
- **Cache Hit**: 1 cycle (with future cache implementation)
- **Memory Access**: 2 cycles (current zero-wait-state SRAM)
- **Load-Use Hazard**: 1 cycle stall if result used immediately

**Store Performance:**
- **Write Buffer**: 1 cycle (posted write)
- **Memory Update**: 1-2 cycles (overlapped with next instruction)
- **No Hazard**: Store-load forwarding implemented

## 7.6 Control Flow Instructions

### 7.6.1 Conditional Branch Instructions

**Branch Operations:**

| Instruction | Encoding | Condition | Description |
|-------------|----------|-----------|-------------|
| **BEQ rs1, rs2, imm** | B-type, funct3=000 | rs1 == rs2 | Branch if equal |
| **BNE rs1, rs2, imm** | B-type, funct3=001 | rs1 != rs2 | Branch if not equal |
| **BLT rs1, rs2, imm** | B-type, funct3=100 | rs1 < rs2 | Branch if less than (signed) |
| **BGE rs1, rs2, imm** | B-type, funct3=101 | rs1 >= rs2 | Branch if greater or equal (signed) |
| **BLTU rs1, rs2, imm** | B-type, funct3=110 | rs1 < rs2 | Branch if less than (unsigned) |
| **BGEU rs1, rs2, imm** | B-type, funct3=111 | rs1 >= rs2 | Branch if greater or equal (unsigned) |

**Branch Addressing:**
- **PC-Relative**: Target = PC + sign_ext(imm)
- **Address Range**: ±4KB (12-bit signed offset × 2)
- **Alignment**: Target must be halfword-aligned  
- **Encoding**: Immediate is stored in multiple fields

**Branch Examples:**
```assembly
beq  x1, x0, zero_handler    # Branch if x1 is zero
bne  x2, x3, not_equal       # Branch if x2 != x3
blt  x4, x5, less_than       # Signed comparison branch
bgeu x6, x7, unsigned_ge     # Unsigned comparison branch
```

### 7.6.2 Unconditional Jump Instructions

**Jump Operations:**

| Instruction | Encoding | Operation | Return Address |
|-------------|----------|-----------|----------------|
| **JAL rd, imm** | J-type | rd = PC+4; PC += sign_ext(imm) | Saved in rd |
| **JALR rd, imm(rs1)** | I-type, funct3=000 | rd = PC+4; PC = (rs1+imm) & ~1 | Saved in rd |

**Jump Addressing:**
- **JAL**: PC-relative addressing (±1MB range)
- **JALR**: Register + immediate addressing (full 32-bit range)
- **Return Address**: Always PC+4 (next instruction)
- **Alignment**: Target address LSB cleared (halfword alignment)

**Jump Examples:**
```assembly
jal  x1, function      # Call function, return address in x1
jalr x0, 0(x1)         # Return from function (x0 discards return address)
jal  x0, loop          # Unconditional jump (discard return address)
jalr x1, -4(x2)        # Jump to x2-4, save return address
```

### 7.6.3 Control Flow Performance

**Branch Performance:**
- **Prediction**: Static (backward taken, forward not taken)
- **Correct Prediction**: 1 cycle (no penalty)
- **Misprediction**: 3 cycles (pipeline flush + refetch)
- **Branch Frequency**: ~20% of instructions in typical code

**Jump Performance:**  
- **Unconditional**: Always 3 cycles (known target)
- **Register Indirect**: 3 cycles (computed target)
- **Function Calls**: 3 cycles + function prologue
- **Returns**: 3 cycles (could be optimized with return stack)

---

*This chapter provided comprehensive coverage of the MCU-32X instruction set architecture. The next chapter will focus on assembly language programming techniques and optimization strategies.*