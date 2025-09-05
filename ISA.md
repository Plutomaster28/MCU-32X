# MCU-32X RISC-V RV32I Instruction Set Architecture

## Overview
MCU-32X is a 32-bit RISC-V RV32I processor with a 5-stage pipeline, 32 general-purpose registers, and standard RISC-V instruction encoding. It supports the complete RV32I base instruction set with arithmetic, logic, memory, and control flow operations.

---

## Register File
- 32 general-purpose registers (`x0` to `x31`), each 32 bits wide.
- `x0` is hardwired to zero (cannot be written).
- Register addresses are 5 bits (`[4:0]`).
- Standard RISC-V ABI names can be used (ra, sp, gp, tp, etc.).

---

## Instruction Format
- All instructions are 32 bits wide.
- RISC-V uses 7-bit opcodes (`[6:0]`).
- Five main instruction formats: R, I, S, B, U, J.

### RISC-V Instruction Formats
```
R-type: [31:25] funct7 | [24:20] rs2 | [19:15] rs1 | [14:12] funct3 | [11:7] rd | [6:0] opcode
I-type: [31:20] imm[11:0] | [19:15] rs1 | [14:12] funct3 | [11:7] rd | [6:0] opcode
S-type: [31:25] imm[11:5] | [24:20] rs2 | [19:15] rs1 | [14:12] funct3 | [11:7] imm[4:0] | [6:0] opcode
B-type: [31] imm[12] | [30:25] imm[10:5] | [24:20] rs2 | [19:15] rs1 | [14:12] funct3 | [11:8] imm[4:1] | [7] imm[11] | [6:0] opcode
U-type: [31:12] imm[31:12] | [11:7] rd | [6:0] opcode
J-type: [31] imm[20] | [30:21] imm[10:1] | [20] imm[11] | [19:12] imm[19:12] | [11:7] rd | [6:0] opcode
```

### Field Descriptions
- `opcode` — 7-bit operation code
- `rd` — 5-bit destination register
- `rs1` — 5-bit source register 1
- `rs2` — 5-bit source register 2
- `funct3` — 3-bit function modifier
- `funct7` — 7-bit function modifier
- `imm` — Immediate value (various widths and positions)

---

## RISC-V RV32I Base Instructions

### R-Type Instructions (opcode: 0110011)
| Instruction | funct7  | funct3 | Operation         | Description |
|-------------|---------|--------|-------------------|-------------|
| ADD         | 0000000 | 000    | rd = rs1 + rs2    | Add |
| SUB         | 0100000 | 000    | rd = rs1 - rs2    | Subtract |
| SLL         | 0000000 | 001    | rd = rs1 << rs2   | Shift left logical |
| SLT         | 0000000 | 010    | rd = (rs1 < rs2)  | Set less than |
| SLTU        | 0000000 | 011    | rd = (rs1 < rs2)  | Set less than unsigned |
| XOR         | 0000000 | 100    | rd = rs1 ^ rs2    | Exclusive OR |
| SRL         | 0000000 | 101    | rd = rs1 >> rs2   | Shift right logical |
| SRA         | 0100000 | 101    | rd = rs1 >>> rs2  | Shift right arithmetic |
| OR          | 0000000 | 110    | rd = rs1 | rs2    | Bitwise OR |
| AND         | 0000000 | 111    | rd = rs1 & rs2    | Bitwise AND |

### I-Type Instructions
| Instruction | Opcode  | funct3 | Operation                | Description |
|-------------|---------|--------|--------------------------|-------------|
| ADDI        | 0010011 | 000    | rd = rs1 + imm           | Add immediate |
| SLTI        | 0010011 | 010    | rd = (rs1 < imm)         | Set less than immediate |
| SLTIU       | 0010011 | 011    | rd = (rs1 < imm)         | Set less than immediate unsigned |
| XORI        | 0010011 | 100    | rd = rs1 ^ imm           | XOR immediate |
| ORI         | 0010011 | 110    | rd = rs1 | imm           | OR immediate |
| ANDI        | 0010011 | 111    | rd = rs1 & imm           | AND immediate |
| SLLI        | 0010011 | 001    | rd = rs1 << imm          | Shift left logical immediate |
| SRLI        | 0010011 | 101    | rd = rs1 >> imm          | Shift right logical immediate |
| SRAI        | 0010011 | 101    | rd = rs1 >>> imm         | Shift right arithmetic immediate |

### Load Instructions (opcode: 0000011)
| Instruction | funct3 | Operation                | Description |
|-------------|--------|--------------------------|-------------|
| LB          | 000    | rd = MEM[rs1 + imm][7:0] | Load byte |
| LH          | 001    | rd = MEM[rs1 + imm][15:0]| Load halfword |
| LW          | 010    | rd = MEM[rs1 + imm]      | Load word |
| LBU         | 100    | rd = MEM[rs1 + imm][7:0] | Load byte unsigned |
| LHU         | 101    | rd = MEM[rs1 + imm][15:0]| Load halfword unsigned |

### Store Instructions (opcode: 0100011)
| Instruction | funct3 | Operation                | Description |
|-------------|--------|--------------------------|-------------|
| SB          | 000    | MEM[rs1 + imm][7:0] = rs2| Store byte |
| SH          | 001    | MEM[rs1 + imm][15:0] = rs2| Store halfword |
| SW          | 010    | MEM[rs1 + imm] = rs2     | Store word |

### Branch Instructions (opcode: 1100011)
| Instruction | funct3 | Operation                | Description |
|-------------|--------|--------------------------|-------------|
| BEQ         | 000    | if (rs1 == rs2) PC += imm| Branch if equal |
| BNE         | 001    | if (rs1 != rs2) PC += imm| Branch if not equal |
| BLT         | 100    | if (rs1 < rs2) PC += imm | Branch if less than |
| BGE         | 101    | if (rs1 >= rs2) PC += imm| Branch if greater or equal |
| BLTU        | 110    | if (rs1 < rs2) PC += imm | Branch if less than unsigned |
| BGEU        | 111    | if (rs1 >= rs2) PC += imm| Branch if greater or equal unsigned |

### Jump Instructions
| Instruction | Opcode  | Operation                | Description |
|-------------|---------|--------------------------|-------------|
| JAL         | 1101111 | rd = PC+4; PC += imm     | Jump and link |
| JALR        | 1100111 | rd = PC+4; PC = rs1+imm  | Jump and link register |

### Upper Immediate Instructions
| Instruction | Opcode  | Operation                | Description |
|-------------|---------|--------------------------|-------------|
| LUI         | 0110111 | rd = imm << 12           | Load upper immediate |
| AUIPC       | 0010111 | rd = PC + (imm << 12)    | Add upper immediate to PC |

---

## Example Assembly (RISC-V)
```asm
# RISC-V assembly examples
addi x1, x0, 1          # x1 = 1
addi x2, x0, 2          # x2 = 2  
add  x3, x1, x2         # x3 = x1 + x2 = 3
sub  x4, x1, x2         # x4 = x1 - x2 = -1
and  x5, x1, x2         # x5 = x1 & x2 = 0
or   x6, x1, x2         # x6 = x1 | x2 = 3
xor  x7, x1, x2         # x7 = x1 ^ x2 = 3

# Memory operations
sw   x3, 0(x0)          # MEM[0] = x3
lw   x8, 0(x0)          # x8 = MEM[0]

# Control flow
beq  x1, x2, end        # if x1 == x2, branch to end
jal  x1, function       # call function, return address in x1
jalr x0, 0(x1)          # return from function

# Upper immediate
lui  x9, 0x12345        # x9 = 0x12345000
auipc x10, 0x1000       # x10 = PC + 0x1000000

end:
    nop                 # no operation (addi x0, x0, 0)
```

---

## Programming Model
- Use standard RISC-V assembly syntax and mnemonics
- x0 is always zero and cannot be modified
- All instructions are 32 bits and word-aligned
- Memory is byte-addressable with little-endian ordering
- PC (program counter) increments by 4 for each instruction

---

## RISC-V Assembler
A RISC-V assembler script is provided in `tools/mcu32x_assembler.py`.

Usage:
```bash
python tools/mcu32x_assembler.py program.s > program.hex
```

The assembler supports:
- Standard RISC-V RV32I instruction set
- Register names (x0-x31) and ABI names (ra, sp, gp, etc.)
- Labels and branch/jump targets
- Immediate values in decimal and hexadecimal
- Comments with # or //

---

## Current Implementation Status
### ✅ Implemented
- Complete RV32I base instruction set
- 32-register file with x0 hardwired to zero
- 5-stage pipeline (Fetch, Decode, Execute, Memory, Writeback)
- ALU with all RISC-V operations
- Memory interface for loads and stores
- Branch and jump instructions
- Immediate operations

### 🚧 In Progress
- Exception handling (ECALL, EBREAK)
- CSR (Control and Status Register) instructions
- Interrupt controller integration
- Cache subsystem

### 📋 Future Extensions
- RV32M (Multiply/Divide extension)
- RV32F (Single-precision floating-point extension)
- RV32C (Compressed instruction extension)
- Custom instruction extensions
