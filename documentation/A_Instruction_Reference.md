# Appendix A: Complete Instruction Reference
## MCU-32X Technical Reference Manual

---

## A.1 Instruction Reference Format

Each instruction entry provides complete information for programming and implementation:

**Format Legend:**
- **Mnemonic**: Assembly language instruction name
- **Format**: Instruction syntax with operands
- **Encoding**: 32-bit binary encoding with bit fields
- **Operation**: Pseudo-code describing instruction behavior  
- **Flags**: Status flags affected (if any)
- **Cycles**: Execution time in clock cycles
- **Example**: Sample assembly code usage

---

## A.2 Arithmetic Instructions

### A.2.1 ADD - Add Registers

**Format:** `ADD rd, rs1, rs2`  
**Encoding:** R-type, opcode=0110011, funct3=000, funct7=0000000  
**Operation:** `rd = rs1 + rs2`  
**Flags:** None (overflow ignored)  
**Cycles:** 1  
**Example:**
```assembly
add x3, x1, x2    # x3 = x1 + x2
add x5, x4, x0    # x5 = x4 + 0 (copy x4 to x5)
```

**Detailed Encoding:**
```
31    25 24  20 19  15 14  12 11   7 6    0
0000000  rs2   rs1   000   rd    0110011
```

**Implementation Notes:**
- Two's complement 32-bit addition
- Overflow detection available but not trapped
- Result always written to destination register
- Source registers read in parallel during decode

---

### A.2.2 ADDI - Add Immediate

**Format:** `ADDI rd, rs1, immediate`  
**Encoding:** I-type, opcode=0010011, funct3=000  
**Operation:** `rd = rs1 + sign_extend(immediate)`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
addi x1, x0, 100     # x1 = 0 + 100 (load constant)
addi x2, x2, 1       # x2 = x2 + 1 (increment)
addi x3, x1, -50     # x3 = x1 - 50 (subtract constant)
```

**Detailed Encoding:**
```
31        20 19  15 14  12 11   7 6    0
   imm[11:0]   rs1   000   rd   0010011
```

**Special Cases:**
- `ADDI x0, x0, 0` creates NOP instruction
- Immediate range: -2048 to +2047 (12-bit signed)
- Sign extension fills upper 20 bits of immediate

---

### A.2.3 SUB - Subtract Registers

**Format:** `SUB rd, rs1, rs2`  
**Encoding:** R-type, opcode=0110011, funct3=000, funct7=0100000  
**Operation:** `rd = rs1 - rs2`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
sub x3, x1, x2       # x3 = x1 - x2
sub x4, x0, x1       # x4 = 0 - x1 (negate x1)
```

**Implementation Notes:**
- Implemented as addition with two's complement of rs2
- No subtract immediate instruction (use ADDI with negative immediate)
- Underflow ignored (wraps around)

---

### A.2.4 SLT - Set Less Than

**Format:** `SLT rd, rs1, rs2`  
**Encoding:** R-type, opcode=0110011, funct3=010, funct7=0000000  
**Operation:** `rd = (rs1 < rs2) ? 1 : 0` (signed comparison)  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
slt x3, x1, x2       # x3 = 1 if x1 < x2 (signed)
slt x4, x1, x0       # x4 = 1 if x1 < 0 (test negative)
```

---

### A.2.5 SLTI - Set Less Than Immediate

**Format:** `SLTI rd, rs1, immediate`  
**Encoding:** I-type, opcode=0010011, funct3=010  
**Operation:** `rd = (rs1 < sign_extend(immediate)) ? 1 : 0`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
slti x2, x1, 100     # x2 = 1 if x1 < 100
slti x3, x1, -1      # x3 = 1 if x1 < -1
```

---

### A.2.6 SLTU - Set Less Than Unsigned

**Format:** `SLTU rd, rs1, rs2`  
**Encoding:** R-type, opcode=0110011, funct3=011, funct7=0000000  
**Operation:** `rd = (rs1 < rs2) ? 1 : 0` (unsigned comparison)  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
sltu x3, x1, x2      # x3 = 1 if x1 < x2 (unsigned)
sltu x4, x1, x0      # x4 = 0 (nothing < 0 unsigned)
```

---

### A.2.7 SLTIU - Set Less Than Immediate Unsigned

**Format:** `SLTIU rd, rs1, immediate`  
**Encoding:** I-type, opcode=0010011, funct3=011  
**Operation:** `rd = (rs1 < zero_extend(immediate)) ? 1 : 0`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
sltiu x2, x1, 1      # x2 = 1 if x1 == 0 (test for zero)
sltiu x3, x1, 0x800  # x3 = 1 if x1 < 2048 (unsigned)
```

---

## A.3 Logical Instructions

### A.3.1 AND - Bitwise AND

**Format:** `AND rd, rs1, rs2`  
**Encoding:** R-type, opcode=0110011, funct3=111, funct7=0000000  
**Operation:** `rd = rs1 & rs2`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
and x3, x1, x2       # x3 = x1 & x2
and x4, x1, x1       # x4 = x1 (copy register)
```

---

### A.3.2 ANDI - Bitwise AND Immediate

**Format:** `ANDI rd, rs1, immediate`  
**Encoding:** I-type, opcode=0010011, funct3=111  
**Operation:** `rd = rs1 & sign_extend(immediate)`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
andi x2, x1, 0xFF    # x2 = x1 & 0xFF (mask lower byte)
andi x3, x1, -2      # x3 = x1 & 0xFFFFFFFE (clear LSB)
```

**Common Use Cases:**
- Bit masking and isolation
- Clearing specific bits
- Testing bit patterns

---

### A.3.3 OR - Bitwise OR

**Format:** `OR rd, rs1, rs2`  
**Encoding:** R-type, opcode=0110011, funct3=110, funct7=0000000  
**Operation:** `rd = rs1 | rs2`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
or x3, x1, x2        # x3 = x1 | x2  
or x4, x1, x0        # x4 = x1 | 0 (copy x1, test for zero)
```

---

### A.3.4 ORI - Bitwise OR Immediate

**Format:** `ORI rd, rs1, immediate`  
**Encoding:** I-type, opcode=0010011, funct3=110  
**Operation:** `rd = rs1 | sign_extend(immediate)`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
ori x2, x1, 0x80     # x2 = x1 | 0x80 (set bit 7)
ori x3, x0, 0x123    # x3 = 0x123 (load small constant)
```

---

### A.3.5 XOR - Bitwise Exclusive OR

**Format:** `XOR rd, rs1, rs2`  
**Encoding:** R-type, opcode=0110011, funct3=100, funct7=0000000  
**Operation:** `rd = rs1 ^ rs2`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
xor x3, x1, x2       # x3 = x1 ^ x2
xor x4, x1, x1       # x4 = 0 (clear register)
```

---

### A.3.6 XORI - Bitwise XOR Immediate

**Format:** `XORI rd, rs1, immediate`  
**Encoding:** I-type, opcode=0010011, funct3=100  
**Operation:** `rd = rs1 ^ sign_extend(immediate)`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
xori x2, x1, -1      # x2 = ~x1 (bitwise NOT)
xori x3, x1, 0xFF    # x3 = x1 ^ 0xFF (toggle lower byte)
```

---

## A.4 Shift Instructions

### A.4.1 SLL - Shift Left Logical

**Format:** `SLL rd, rs1, rs2`  
**Encoding:** R-type, opcode=0110011, funct3=001, funct7=0000000  
**Operation:** `rd = rs1 << (rs2 & 0x1F)`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
sll x3, x1, x2       # x3 = x1 << (x2 & 31)
sll x4, x1, x0       # x4 = x1 << 0 (copy register)
```

**Implementation Notes:**
- Only lower 5 bits of rs2 used (shift amount 0-31)
- Fills with zeros from the right
- Equivalent to multiplication by 2^n

---

### A.4.2 SLLI - Shift Left Logical Immediate

**Format:** `SLLI rd, rs1, shamt`  
**Encoding:** I-type, opcode=0010011, funct3=001, imm[11:5]=0000000  
**Operation:** `rd = rs1 << shamt`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
slli x2, x1, 4       # x2 = x1 << 4 (multiply by 16)
slli x3, x1, 1       # x3 = x1 << 1 (multiply by 2)
```

**Detailed Encoding:**
```
31     25 24  20 19  15 14  12 11   7 6    0
0000000  shamt  rs1   001   rd   0010011
```

---

### A.4.3 SRL - Shift Right Logical

**Format:** `SRL rd, rs1, rs2`  
**Encoding:** R-type, opcode=0110011, funct3=101, funct7=0000000  
**Operation:** `rd = rs1 >> (rs2 & 0x1F)`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
srl x3, x1, x2       # x3 = x1 >> (x2 & 31)
srl x4, x1, x0       # x4 = x1 >> 0 (copy register)
```

**Implementation Notes:**
- Logical right shift (fills with zeros from left)
- Only lower 5 bits of rs2 used
- Equivalent to unsigned division by 2^n

---

### A.4.4 SRLI - Shift Right Logical Immediate

**Format:** `SRLI rd, rs1, shamt`  
**Encoding:** I-type, opcode=0010011, funct3=101, imm[11:5]=0000000  
**Operation:** `rd = rs1 >> shamt`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
srli x2, x1, 8       # x2 = x1 >> 8 (unsigned divide by 256)
srli x3, x1, 1       # x3 = x1 >> 1 (unsigned divide by 2)
```

---

### A.4.5 SRA - Shift Right Arithmetic

**Format:** `SRA rd, rs1, rs2`  
**Encoding:** R-type, opcode=0110011, funct3=101, funct7=0100000  
**Operation:** `rd = rs1 >>> (rs2 & 0x1F)`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
sra x3, x1, x2       # x3 = x1 >>> (x2 & 31) (signed shift)
```

**Implementation Notes:**
- Arithmetic right shift (fills with sign bit)
- Preserves sign for negative numbers
- Equivalent to signed division by 2^n (rounded toward negative infinity)

---

### A.4.6 SRAI - Shift Right Arithmetic Immediate

**Format:** `SRAI rd, rs1, shamt`  
**Encoding:** I-type, opcode=0010011, funct3=101, imm[11:5]=0100000  
**Operation:** `rd = rs1 >>> shamt`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
srai x2, x1, 4       # x2 = x1 >>> 4 (signed divide by 16)
srai x3, x1, 31      # x3 = sign bit of x1 (all 0s or all 1s)
```

---

## A.5 Memory Access Instructions

### A.5.1 LW - Load Word

**Format:** `LW rd, offset(rs1)`  
**Encoding:** I-type, opcode=0000011, funct3=010  
**Operation:** `rd = memory[rs1 + sign_extend(offset)]`  
**Flags:** None  
**Cycles:** 2 (typical)  
**Example:**
```assembly
lw x2, 0(x1)         # x2 = memory[x1]
lw x3, 100(x1)       # x3 = memory[x1 + 100]
lw x4, -4(x2)        # x4 = memory[x2 - 4]
```

**Address Calculation:**
- Effective Address = rs1 + sign_extend(offset)
- Offset range: -2048 to +2047 bytes
- Word alignment recommended for performance
- Little-endian byte ordering

---

### A.5.2 LH - Load Halfword

**Format:** `LH rd, offset(rs1)`  
**Encoding:** I-type, opcode=0000011, funct3=001  
**Operation:** `rd = sign_extend(memory[rs1 + sign_extend(offset)][15:0])`  
**Flags:** None  
**Cycles:** 2 (typical)  
**Example:**
```assembly
lh x2, 0(x1)         # Load signed 16-bit value
lh x3, 2(x1)         # Load next halfword
```

**Implementation Notes:**
- Loads 16 bits, sign-extends to 32 bits
- Halfword alignment recommended
- Negative values properly sign-extended

---

### A.5.3 LHU - Load Halfword Unsigned

**Format:** `LHU rd, offset(rs1)`  
**Encoding:** I-type, opcode=0000011, funct3=101  
**Operation:** `rd = zero_extend(memory[rs1 + sign_extend(offset)][15:0])`  
**Flags:** None  
**Cycles:** 2 (typical)  
**Example:**
```assembly
lhu x2, 0(x1)        # Load unsigned 16-bit value
lhu x3, 2(x1)        # Upper 16 bits will be zero
```

---

### A.5.4 LB - Load Byte

**Format:** `LB rd, offset(rs1)`  
**Encoding:** I-type, opcode=0000011, funct3=000  
**Operation:** `rd = sign_extend(memory[rs1 + sign_extend(offset)][7:0])`  
**Flags:** None  
**Cycles:** 2 (typical)  
**Example:**
```assembly
lb x2, 0(x1)         # Load signed byte
lb x3, 1(x1)         # Load next byte
```

**Implementation Notes:**
- Loads 8 bits, sign-extends to 32 bits
- Can access any byte address
- Useful for character and packed data

---

### A.5.5 LBU - Load Byte Unsigned

**Format:** `LBU rd, offset(rs1)`  
**Encoding:** I-type, opcode=0000011, funct3=100  
**Operation:** `rd = zero_extend(memory[rs1 + sign_extend(offset)][7:0])`  
**Flags:** None  
**Cycles:** 2 (typical)  
**Example:**
```assembly
lbu x2, 0(x1)        # Load unsigned byte
lbu x3, 1(x1)        # Upper 24 bits will be zero
```

---

### A.5.6 SW - Store Word

**Format:** `SW rs2, offset(rs1)`  
**Encoding:** S-type, opcode=0100011, funct3=010  
**Operation:** `memory[rs1 + sign_extend(offset)] = rs2`  
**Flags:** None  
**Cycles:** 2 (typical)  
**Example:**
```assembly
sw x2, 0(x1)         # Store x2 to memory[x1]
sw x3, 100(x1)       # Store x3 to memory[x1 + 100]
sw x0, -4(x2)        # Store 0 to memory[x2 - 4]
```

**Detailed Encoding:**
```
31    25 24  20 19  15 14  12 11     7 6    0
imm[11:5] rs2   rs1   010  imm[4:0] 0100011
```

---

### A.5.7 SH - Store Halfword

**Format:** `SH rs2, offset(rs1)`  
**Encoding:** S-type, opcode=0100011, funct3=001  
**Operation:** `memory[rs1 + sign_extend(offset)][15:0] = rs2[15:0]`  
**Flags:** None  
**Cycles:** 2 (typical)  
**Example:**
```assembly
sh x2, 0(x1)         # Store lower 16 bits of x2
sh x3, 2(x1)         # Store to next halfword
```

---

### A.5.8 SB - Store Byte

**Format:** `SB rs2, offset(rs1)`  
**Encoding:** S-type, opcode=0100011, funct3=000  
**Operation:** `memory[rs1 + sign_extend(offset)][7:0] = rs2[7:0]`  
**Flags:** None  
**Cycles:** 2 (typical)  
**Example:**
```assembly
sb x2, 0(x1)         # Store lower 8 bits of x2
sb x3, 1(x1)         # Store to next byte
```

---

## A.6 Control Flow Instructions

### A.6.1 BEQ - Branch if Equal

**Format:** `BEQ rs1, rs2, offset`  
**Encoding:** B-type, opcode=1100011, funct3=000  
**Operation:** `if (rs1 == rs2) PC = PC + sign_extend(offset)`  
**Flags:** None  
**Cycles:** 1 (not taken), 3 (taken)  
**Example:**
```assembly
beq x1, x2, loop     # Branch if x1 == x2
beq x1, x0, is_zero  # Branch if x1 == 0
```

**Address Calculation:**
- Target = PC + sign_extend(offset)  
- Offset is in multiples of 2 bytes
- Range: ±4KB from current PC

---

### A.6.2 BNE - Branch if Not Equal

**Format:** `BNE rs1, rs2, offset`  
**Encoding:** B-type, opcode=1100011, funct3=001  
**Operation:** `if (rs1 != rs2) PC = PC + sign_extend(offset)`  
**Flags:** None  
**Cycles:** 1 (not taken), 3 (taken)  
**Example:**
```assembly
bne x1, x2, not_equal  # Branch if x1 != x2
bne x1, x0, not_zero   # Branch if x1 != 0
```

---

### A.6.3 BLT - Branch if Less Than

**Format:** `BLT rs1, rs2, offset`  
**Encoding:** B-type, opcode=1100011, funct3=100  
**Operation:** `if (rs1 < rs2) PC = PC + sign_extend(offset)` (signed)  
**Flags:** None  
**Cycles:** 1 (not taken), 3 (taken)  
**Example:**
```assembly
blt x1, x2, less_than    # Branch if x1 < x2 (signed)
blt x1, x0, negative     # Branch if x1 < 0
```

---

### A.6.4 BGE - Branch if Greater or Equal

**Format:** `BGE rs1, rs2, offset`  
**Encoding:** B-type, opcode=1100011, funct3=101  
**Operation:** `if (rs1 >= rs2) PC = PC + sign_extend(offset)` (signed)  
**Flags:** None  
**Cycles:** 1 (not taken), 3 (taken)  
**Example:**
```assembly
bge x1, x2, greater_equal  # Branch if x1 >= x2 (signed)
bge x1, x0, non_negative   # Branch if x1 >= 0
```

---

### A.6.5 BLTU - Branch if Less Than Unsigned

**Format:** `BLTU rs1, rs2, offset`  
**Encoding:** B-type, opcode=1100011, funct3=110  
**Operation:** `if (rs1 < rs2) PC = PC + sign_extend(offset)` (unsigned)  
**Flags:** None  
**Cycles:** 1 (not taken), 3 (taken)  
**Example:**
```assembly
bltu x1, x2, less_unsigned   # Branch if x1 < x2 (unsigned)
bltu x1, x0, never          # Never branches (nothing < 0 unsigned)
```

---

### A.6.6 BGEU - Branch if Greater or Equal Unsigned

**Format:** `BGEU rs1, rs2, offset`  
**Encoding:** B-type, opcode=1100011, funct3=111  
**Operation:** `if (rs1 >= rs2) PC = PC + sign_extend(offset)` (unsigned)  
**Flags:** None  
**Cycles:** 1 (not taken), 3 (taken)  
**Example:**
```assembly
bgeu x1, x2, greater_equal_u  # Branch if x1 >= x2 (unsigned)
bgeu x1, x0, always          # Always branches (everything >= 0 unsigned)
```

---

### A.6.7 JAL - Jump and Link

**Format:** `JAL rd, offset`  
**Encoding:** J-type, opcode=1101111  
**Operation:** `rd = PC + 4; PC = PC + sign_extend(offset)`  
**Flags:** None  
**Cycles:** 3  
**Example:**
```assembly
jal x1, function      # Call function, return address in x1
jal x0, label         # Unconditional jump (discard return address)
```

**Detailed Encoding:**
```
31 30      21 20 19      12 11   7 6    0
i  imm[10:1]  i  imm[19:12] rd   1101111
```

**Address Calculation:**
- Return address = PC + 4
- Target = PC + sign_extend(offset)
- Range: ±1MB from current PC

---

### A.6.8 JALR - Jump and Link Register

**Format:** `JALR rd, offset(rs1)`  
**Encoding:** I-type, opcode=1100111, funct3=000  
**Operation:** `rd = PC + 4; PC = (rs1 + sign_extend(offset)) & ~1`  
**Flags:** None  
**Cycles:** 3  
**Example:**
```assembly
jalr x1, 0(x2)        # Jump to address in x2, return in x1
jalr x0, 0(x1)        # Return from function (jump to x1)
jalr x3, 100(x4)      # Jump to x4+100, return in x3
```

**Implementation Notes:**
- LSB of target address is cleared (halfword alignment)
- Can jump to any 32-bit address
- Commonly used for function returns and indirect calls

---

## A.7 Upper Immediate Instructions

### A.7.1 LUI - Load Upper Immediate

**Format:** `LUI rd, immediate`  
**Encoding:** U-type, opcode=0110111  
**Operation:** `rd = immediate << 12`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
lui x1, 0x12345       # x1 = 0x12345000
lui x2, 0x80000       # x2 = 0x80000000
```

**Detailed Encoding:**
```
31            12 11   7 6    0
   imm[31:12]    rd   0110111
```

**Usage Patterns:**
- Loading large constants (combined with ORI/ADDI)
- Setting up base addresses  
- Creating bit masks

---

### A.7.2 AUIPC - Add Upper Immediate to PC

**Format:** `AUIPC rd, immediate`  
**Encoding:** U-type, opcode=0010111  
**Operation:** `rd = PC + (immediate << 12)`  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
auipc x1, 0x1000      # x1 = PC + 0x1000000
auipc x2, 0           # x2 = PC (get current address)
```

**Usage Patterns:**
- Position-independent code
- Calculating addresses relative to PC
- Accessing global data with PC-relative addressing

---

## A.8 System Instructions

### A.8.1 ECALL - Environment Call

**Format:** `ECALL`  
**Encoding:** I-type, opcode=1110011, funct3=000, imm=000000000000  
**Operation:** Trigger environment call exception  
**Flags:** None  
**Cycles:** Variable (depends on system)  
**Example:**
```assembly
ecall                 # System call (implementation defined)
```

**Implementation Notes:**
- Transfers control to operating system or monitor
- Register conventions for system calls are software-defined
- Typically used for I/O, memory management, etc.

---

### A.8.2 EBREAK - Environment Break

**Format:** `EBREAK`  
**Encoding:** I-type, opcode=1110011, funct3=000, imm=000000000001  
**Operation:** Trigger breakpoint exception  
**Flags:** None  
**Cycles:** Variable  
**Example:**
```assembly
ebreak                # Breakpoint for debugger
```

**Usage:**
- Software breakpoints for debugging
- Transfers control to debug monitor
- Can be used for runtime assertions

---

### A.8.3 FENCE - Memory Fence

**Format:** `FENCE pred, succ`  
**Encoding:** I-type, opcode=0001111, funct3=000  
**Operation:** Memory ordering fence  
**Flags:** None  
**Cycles:** 1  
**Example:**
```assembly
fence                 # Full memory fence
```

**Implementation Notes:**
- Ensures memory operation ordering
- Implementation specific behavior
- May be NOP on simple systems with coherent memory

---

## A.9 Pseudo-Instructions

These are assembler conveniences that translate to real instructions:

### A.9.1 Common Pseudo-Instructions

| Pseudo-Instruction | Real Instruction | Description |
|-------------------|------------------|-------------|
| `NOP` | `ADDI x0, x0, 0` | No operation |
| `MV rd, rs` | `ADDI rd, rs, 0` | Copy register |
| `NOT rd, rs` | `XORI rd, rs, -1` | Bitwise NOT |
| `NEG rd, rs` | `SUB rd, x0, rs` | Negate |
| `J offset` | `JAL x0, offset` | Unconditional jump |
| `JR rs` | `JALR x0, 0(rs)` | Jump register |
| `RET` | `JALR x0, 0(x1)` | Return from function |
| `LI rd, imm` | `ADDI rd, x0, imm` | Load immediate (small) |

### A.9.2 Multi-Instruction Pseudo-Instructions

**Loading Large Constants:**
```assembly
# li x1, 0x12345678 expands to:
lui  x1, 0x12346      # Load upper bits (rounded up)
addi x1, x1, 0x678    # Add lower bits
```

**Loading Addresses:**
```assembly
# la x1, symbol expands to:
auipc x1, %pcrel_hi(symbol)
addi  x1, x1, %pcrel_lo(symbol)
```

---

*This appendix provided complete reference information for all MCU-32X instructions, enabling efficient programming and system development.*