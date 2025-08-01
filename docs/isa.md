# Instruction Set Architecture (ISA) for MCU-32X

## Overview
The MCU-32X processor utilizes a custom RISC-like Instruction Set Architecture (ISA) designed to optimize performance for integer workloads, suitable for running a homebrew operating system or game kernel. This document outlines the instruction formats, available instructions, and their functionalities.

## Instruction Formats
The MCU-32X ISA supports several instruction formats:

1. **R-Type (Register Type)**
   - Format: `opcode | rs1 | rs2 | rd | funct3 | funct7`
   - Used for arithmetic and logical operations.

2. **I-Type (Immediate Type)**
   - Format: `opcode | rs1 | rd | funct3 | immediate`
   - Used for operations that require an immediate value, such as loading data.

3. **S-Type (Store Type)**
   - Format: `opcode | rs1 | rs2 | funct3 | immediate`
   - Used for storing data from registers to memory.

4. **B-Type (Branch Type)**
   - Format: `opcode | rs1 | rs2 | funct3 | immediate`
   - Used for branch instructions that alter the flow of control.

5. **U-Type (Upper Immediate Type)**
   - Format: `opcode | rd | immediate`
   - Used for loading upper immediate values.

6. **J-Type (Jump Type)**
   - Format: `opcode | rd | immediate`
   - Used for jump instructions.

## Available Instructions
### Arithmetic Instructions
- `ADD rd, rs1, rs2`: Adds two registers.
- `SUB rd, rs1, rs2`: Subtracts one register from another.
- `MUL rd, rs1, rs2`: Multiplies two registers (if FPU is implemented).

### Logical Instructions
- `AND rd, rs1, rs2`: Bitwise AND.
- `OR rd, rs1, rs2`: Bitwise OR.
- `XOR rd, rs1, rs2`: Bitwise XOR.

### Load/Store Instructions
- `LW rd, offset(rs1)`: Load word from memory.
- `SW rs2, offset(rs1)`: Store word to memory.

### Control Flow Instructions
- `BEQ rs1, rs2, offset`: Branch if equal.
- `BNE rs1, rs2, offset`: Branch if not equal.
- `JAL rd, offset`: Jump and link.

### Immediate Instructions
- `ADDI rd, rs1, immediate`: Add immediate to a register.
- `ANDI rd, rs1, immediate`: Bitwise AND with immediate.

## Special Features
- **Hardware Multiply/Divide**: The MCU-32X includes hardware support for multiplication and division operations.
- **Interrupt Handling**: The ISA includes instructions for managing interrupts, allowing for responsive system behavior.
- **DMA Support**: Instructions for initiating DMA transfers to optimize data movement to and from peripherals.

## Conclusion
The MCU-32X ISA is designed to provide a balance between simplicity and performance, making it suitable for a variety of applications while maintaining a retro feel. Further details on specific instructions and their encoding can be found in the accompanying documentation.