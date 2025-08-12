# MCU-32X Instruction Set Architecture (ISA)

## Overview
MCU-32X is a 32-bit RISC-style processor with a 5-stage pipeline, 32 general-purpose registers, and a MIPS-like instruction encoding. It supports basic arithmetic, logic, memory, and control flow operations.

---

## Register File
- 32 general-purpose registers (`x0` to `x31`), each 32 bits wide.
- Register addresses are 5 bits (`[4:0]`).

---

## Instruction Format
- All instructions are 32 bits.
- The top 6 bits (`[31:26]`) are the opcode.
- R-type instructions use the lower 4 bits (`[3:0]`) as the ALU function code.

### Field Mapping
- `instruction[31:26]` — Opcode
- `instruction[24:20]` — Source register 2 (rt)
- `instruction[19:15]` — Source register 1 (rs)
- `instruction[4:0]` — Destination register (rd) (for R-type)
- `instruction[3:0]` — ALU function (for R-type)
- Immediate field: lower 16 bits (for I-type)

---

## Supported Instructions

### R-Type (opcode: `000000`)
| Name | Funct (3:0) | Operation         |
|------|-------------|-------------------|
| ADD  | 0000        | rd = rs + rt      |
| SUB  | 0001        | rd = rs - rt      |
| AND  | 0010        | rd = rs & rt      |
| OR   | 0011        | rd = rs | rt      |
| XOR  | 0100        | rd = rs ^ rt      |
| NOR  | 0101        | rd = ~(rs | rt)   |
| SLL  | 0110        | rd = rs << rt     |
| SRL  | 0111        | rd = rs >> rt     |
| SRA  | 1000        | rd = rs >>> rt    |
| MUL  | 1001        | rd = rs * rt      |
| DIV  | 1010        | rd = rs / rt      |

### I-Type
| Name | Opcode   | Operation                |
|------|----------|--------------------------|
| LW   | 100011   | rt = MEM[rs + imm]       |
| SW   | 101011   | MEM[rs + imm] = rt       |
| BEQ  | 000100   | if (rs == rt) PC += imm  |

### J-Type
| Name | Opcode   | Operation                |
|------|----------|--------------------------|
| J    | 000010   | PC = address             |

---

## Example Assembly
```
# add x1, x2, x3
ADD x1, x2, x3
# lw x4, 8(x2)
LW x4, 8, x2
# sw x5, 12(x2)
SW x5, 12, x2
# beq x2, x3, 16
BEQ x2, x3, 16
# j 1024
J 1024
```

---

## Programming Model
- Write 32-bit machine code instructions using the above encoding.
- Load them into instruction memory.
- Use Wishbone, GPIO, or logic analyzer for I/O (see Caravel integration).

---

## Assembler
A minimal assembler script is provided in `tools/mcu32x_assembler.py`.

Usage:
```
python tools/mcu32x_assembler.py program.s > program.hex
```

---

## Limitations / TODO
- Immediate field usage and sign extension are minimal.
- No assembler or loader provided for advanced features.
- No support for interrupts, exceptions, or system calls.
- Only a minimal set of instructions is implemented.
