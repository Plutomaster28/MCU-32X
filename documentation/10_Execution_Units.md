# Chapter 10: Execution Units
## MCU-32X Technical Reference Manual

---

## 10.1 Execution Unit Overview

The MCU-32X implements a high-performance execution engine designed to maximize instruction throughput while maintaining the simplicity and determinism required for embedded applications. The execution units are optimized for the RISC-V RV32I instruction set with careful attention to 1990s manufacturing constraints and performance targets.

### 10.1.1 Execution Unit Architecture

**Execution Unit Organization:**
- **Integer Execution Unit (IEU)**: Primary arithmetic and logic operations
- **Load/Store Unit (LSU)**: Memory access operations
- **Branch Execution Unit (BEU)**: Branch and jump operations
- **Multiply/Divide Unit (MDU)**: Extended arithmetic operations
- **Floating-Point Unit (FPU)**: IEEE 754 single-precision operations

**Performance Characteristics:**
```
Integer Operations:    1 cycle (most operations)
Load Operations:       2-3 cycles (cache hit)
Store Operations:      1 cycle (write-through)
Branch Operations:     1 cycle (predicted correctly)
Multiply:              3 cycles (32x32→64 bit)
Divide:                34 cycles (32÷32→32 bit)
Floating-Point:        4-8 cycles (operation dependent)
```

### 10.1.2 Execution Pipeline Integration

**Pipeline Stage Mapping:**
```
Stage    | Function                    | Execution Units Involved
---------|-----------------------------|-----------------------
IF       | Instruction Fetch           | -
ID       | Instruction Decode          | All units (dispatch)
EX       | Execute                     | IEU, LSU, BEU, MDU, FPU
MEM      | Memory Access               | LSU (data), IEU (results)
WB       | Write Back                  | All units (completion)
```

**Execution Unit Dispatch Logic:**
```verilog
module execution_dispatch (
    input wire clk,
    input wire reset,
    
    // Decoded instruction interface
    input wire [31:0] instruction,
    input wire [6:0] opcode,
    input wire [2:0] funct3,
    input wire [6:0] funct7,
    input wire [4:0] rs1, rs2, rd,
    input wire [31:0] rs1_data, rs2_data,
    input wire [31:0] immediate,
    
    // Execution unit control
    output wire ieu_enable,
    output wire lsu_enable,
    output wire beu_enable,
    output wire mdu_enable,
    output wire fpu_enable,
    
    // Execution unit inputs
    output wire [31:0] alu_op1, alu_op2,
    output wire [3:0] alu_control,
    output wire [31:0] mem_addr,
    output wire [31:0] mem_data,
    output wire mem_read, mem_write,
    output wire [31:0] branch_target,
    output wire branch_taken
);

// Instruction type decoding
wire is_arithmetic = (opcode == 7'b0110011) || (opcode == 7'b0010011);
wire is_load      = (opcode == 7'b0000011);
wire is_store     = (opcode == 7'b0100011);
wire is_branch    = (opcode == 7'b1100011);
wire is_jal       = (opcode == 7'b1101111);
wire is_jalr      = (opcode == 7'b1100111);
wire is_lui       = (opcode == 7'b0110111);
wire is_auipc     = (opcode == 7'b0010111);

// Execution unit enables
assign ieu_enable = is_arithmetic || is_lui || is_auipc;
assign lsu_enable = is_load || is_store;
assign beu_enable = is_branch || is_jal || is_jalr;
assign mdu_enable = is_arithmetic && (funct7 == 7'b0000001); // RV32M extension
assign fpu_enable = (opcode == 7'b1010011); // Floating-point operations

// ALU operation encoding
always_comb begin
    case ({opcode, funct3, funct7[5]})
        {7'b0110011, 3'b000, 1'b0}: alu_control = 4'b0000; // ADD
        {7'b0110011, 3'b000, 1'b1}: alu_control = 4'b0001; // SUB
        {7'b0110011, 3'b001, 1'b0}: alu_control = 4'b0010; // SLL
        {7'b0110011, 3'b010, 1'b0}: alu_control = 4'b0011; // SLT
        {7'b0110011, 3'b011, 1'b0}: alu_control = 4'b0100; // SLTU
        {7'b0110011, 3'b100, 1'b0}: alu_control = 4'b0101; // XOR
        {7'b0110011, 3'b101, 1'b0}: alu_control = 4'b0110; // SRL
        {7'b0110011, 3'b101, 1'b1}: alu_control = 4'b0111; // SRA
        {7'b0110011, 3'b110, 1'b0}: alu_control = 4'b1000; // OR
        {7'b0110011, 3'b111, 1'b0}: alu_control = 4'b1001; // AND
        default: alu_control = 4'b0000;
    endcase
end

// Operand selection
assign alu_op1 = (opcode == 7'b0010111) ? pc : rs1_data; // AUIPC uses PC
assign alu_op2 = (opcode[5] == 1'b0) ? immediate : rs2_data; // Immediate vs register

endmodule
```

---

## 10.2 Integer Execution Unit (IEU)

### 10.2.1 Arithmetic Logic Unit (ALU)

**ALU Architecture:**
The MCU-32X ALU implements all RISC-V integer arithmetic and logical operations with optimized critical paths for 100MHz operation.

```verilog
module integer_alu (
    input wire [31:0] operand_a,
    input wire [31:0] operand_b,
    input wire [3:0] alu_control,
    output wire [31:0] result,
    output wire zero_flag,
    output wire negative_flag,
    output wire carry_flag,
    output wire overflow_flag
);

// ALU operation definitions
parameter ALU_ADD  = 4'b0000;
parameter ALU_SUB  = 4'b0001;
parameter ALU_SLL  = 4'b0010;
parameter ALU_SLT  = 4'b0011;
parameter ALU_SLTU = 4'b0100;
parameter ALU_XOR  = 4'b0101;
parameter ALU_SRL  = 4'b0110;
parameter ALU_SRA  = 4'b0111;
parameter ALU_OR   = 4'b1000;
parameter ALU_AND  = 4'b1001;

wire [32:0] add_result;
wire [31:0] shift_result;
wire [4:0] shift_amount;

// Adder/Subtractor (shared for ADD/SUB/comparison)
assign add_result = (alu_control == ALU_SUB) ? 
                   {1'b0, operand_a} - {1'b0, operand_b} :
                   {1'b0, operand_a} + {1'b0, operand_b};

// Shift amount (lower 5 bits of operand_b)
assign shift_amount = operand_b[4:0];

// Barrel shifter for shift operations
barrel_shifter shifter (
    .data_in(operand_a),
    .shift_amount(shift_amount),
    .shift_right(alu_control == ALU_SRL || alu_control == ALU_SRA),
    .arithmetic(alu_control == ALU_SRA),
    .data_out(shift_result)
);

// ALU result multiplexer
always_comb begin
    case (alu_control)
        ALU_ADD, ALU_SUB:
            result = add_result[31:0];
        ALU_SLL, ALU_SRL, ALU_SRA:
            result = shift_result;
        ALU_SLT:
            result = ($signed(operand_a) < $signed(operand_b)) ? 32'h1 : 32'h0;
        ALU_SLTU:
            result = (operand_a < operand_b) ? 32'h1 : 32'h0;
        ALU_XOR:
            result = operand_a ^ operand_b;
        ALU_OR:
            result = operand_a | operand_b;
        ALU_AND:
            result = operand_a & operand_b;
        default:
            result = 32'h0;
    endcase
end

// Flag generation
assign zero_flag = (result == 32'h0);
assign negative_flag = result[31];
assign carry_flag = add_result[32];
assign overflow_flag = (alu_control == ALU_ADD) ? 
    (operand_a[31] == operand_b[31]) && (result[31] != operand_a[31]) :
    (alu_control == ALU_SUB) ?
    (operand_a[31] != operand_b[31]) && (result[31] != operand_a[31]) : 1'b0;

endmodule
```

### 10.2.2 Barrel Shifter Implementation

**High-Performance Shift Operations:**
```verilog
module barrel_shifter (
    input wire [31:0] data_in,
    input wire [4:0] shift_amount,
    input wire shift_right,
    input wire arithmetic,
    output wire [31:0] data_out
);

// Multi-stage barrel shifter for single-cycle operation
wire [31:0] stage0, stage1, stage2, stage3, stage4;
wire fill_bit;

// Fill bit for arithmetic right shifts
assign fill_bit = arithmetic ? data_in[31] : 1'b0;

// Stage 0: Shift by 1 bit
assign stage0 = shift_amount[0] ? 
    (shift_right ? {fill_bit, data_in[31:1]} : {data_in[30:0], 1'b0}) : 
    data_in;

// Stage 1: Shift by 2 bits
assign stage1 = shift_amount[1] ? 
    (shift_right ? {{2{fill_bit}}, stage0[31:2]} : {stage0[29:0], 2'b00}) : 
    stage0;

// Stage 2: Shift by 4 bits
assign stage2 = shift_amount[2] ? 
    (shift_right ? {{4{fill_bit}}, stage1[31:4]} : {stage1[27:0], 4'b0000}) : 
    stage1;

// Stage 3: Shift by 8 bits
assign stage3 = shift_amount[3] ? 
    (shift_right ? {{8{fill_bit}}, stage2[31:8]} : {stage2[23:0], 8'b00000000}) : 
    stage2;

// Stage 4: Shift by 16 bits
assign stage4 = shift_amount[4] ? 
    (shift_right ? {{16{fill_bit}}, stage3[31:16]} : {stage3[15:0], 16'h0000}) : 
    stage3;

assign data_out = stage4;

endmodule
```

### 10.2.3 Integer Execution Performance

**Execution Timing:**
```assembly
# Single-cycle integer operations
add   t0, t1, t2    # 1 cycle: t0 = t1 + t2
sub   t0, t1, t2    # 1 cycle: t0 = t1 - t2
and   t0, t1, t2    # 1 cycle: t0 = t1 & t2
or    t0, t1, t2    # 1 cycle: t0 = t1 | t2
xor   t0, t1, t2    # 1 cycle: t0 = t1 ^ t2
sll   t0, t1, t2    # 1 cycle: t0 = t1 << t2[4:0]
srl   t0, t1, t2    # 1 cycle: t0 = t1 >> t2[4:0] (logical)
sra   t0, t1, t2    # 1 cycle: t0 = t1 >> t2[4:0] (arithmetic)

# Immediate operations (also single cycle)
addi  t0, t1, 100   # 1 cycle: t0 = t1 + 100
andi  t0, t1, 0xFF  # 1 cycle: t0 = t1 & 0xFF
ori   t0, t1, 0x80  # 1 cycle: t0 = t1 | 0x80
xori  t0, t1, -1    # 1 cycle: t0 = t1 ^ 0xFFFFFFFF (NOT operation)
slli  t0, t1, 4     # 1 cycle: t0 = t1 << 4
srli  t0, t1, 8     # 1 cycle: t0 = t1 >> 8 (logical)
srai  t0, t1, 12    # 1 cycle: t0 = t1 >> 12 (arithmetic)

# Comparison operations
slt   t0, t1, t2    # 1 cycle: t0 = (t1 < t2) ? 1 : 0 (signed)
sltu  t0, t1, t2    # 1 cycle: t0 = (t1 < t2) ? 1 : 0 (unsigned)
slti  t0, t1, 100   # 1 cycle: t0 = (t1 < 100) ? 1 : 0 (signed)
sltiu t0, t1, 200   # 1 cycle: t0 = (t1 < 200) ? 1 : 0 (unsigned)
```

---

## 10.3 Load/Store Unit (LSU)

### 10.3.1 Memory Access Architecture

**LSU Design Features:**
- **Address Generation**: Single-cycle effective address calculation
- **Data Alignment**: Hardware support for byte, halfword, and word accesses
- **Cache Interface**: Optimized for Harvard cache architecture
- **Store Buffer**: Write-through with store buffer for performance

```verilog
module load_store_unit (
    input wire clk,
    input wire reset,
    
    // Pipeline interface
    input wire lsu_enable,
    input wire [31:0] base_addr,
    input wire [31:0] offset,
    input wire [31:0] store_data,
    input wire [2:0] funct3,        // Load/store type
    input wire mem_read,
    input wire mem_write,
    
    // Cache interface
    output wire [31:0] cache_addr,
    output wire [31:0] cache_write_data,
    input wire [31:0] cache_read_data,
    output wire [3:0] cache_byte_enable,
    output wire cache_read_req,
    output wire cache_write_req,
    input wire cache_ready,
    
    // Pipeline outputs
    output wire [31:0] load_result,
    output wire lsu_ready,
    output wire lsu_exception,
    output wire [3:0] exception_cause
);

// Address generation
wire [31:0] effective_addr;
assign effective_addr = base_addr + offset;

// Memory access type decoding
wire is_byte = (funct3[1:0] == 2'b00);     // LB/LBU/SB
wire is_half = (funct3[1:0] == 2'b01);     // LH/LHU/SH  
wire is_word = (funct3[1:0] == 2'b10);     // LW/SW
wire is_unsigned = funct3[2];               // Unsigned load (LBU/LHU)

// Address alignment check
wire addr_misaligned = (is_half && effective_addr[0]) || 
                      (is_word && effective_addr[1:0] != 2'b00);

// Exception detection
assign lsu_exception = addr_misaligned;
assign exception_cause = mem_read ? 4'h4 : 4'h6; // Load/store address misaligned

// Cache interface signals
assign cache_addr = {effective_addr[31:2], 2'b00}; // Word-aligned address
assign cache_read_req = mem_read && !lsu_exception;
assign cache_write_req = mem_write && !lsu_exception;

// Byte enable generation for stores
always_comb begin
    if (is_byte) begin
        case (effective_addr[1:0])
            2'b00: cache_byte_enable = 4'b0001;
            2'b01: cache_byte_enable = 4'b0010;
            2'b10: cache_byte_enable = 4'b0100;
            2'b11: cache_byte_enable = 4'b1000;
        endcase
    end else if (is_half) begin
        case (effective_addr[1])
            1'b0: cache_byte_enable = 4'b0011;
            1'b1: cache_byte_enable = 4'b1100;
        endcase
    end else begin // word access
        cache_byte_enable = 4'b1111;
    end
end

// Store data alignment
always_comb begin
    if (is_byte) begin
        case (effective_addr[1:0])
            2'b00: cache_write_data = {24'b0, store_data[7:0]};
            2'b01: cache_write_data = {16'b0, store_data[7:0], 8'b0};
            2'b10: cache_write_data = {8'b0, store_data[7:0], 16'b0};
            2'b11: cache_write_data = {store_data[7:0], 24'b0};
        endcase
    end else if (is_half) begin
        case (effective_addr[1])
            1'b0: cache_write_data = {16'b0, store_data[15:0]};
            1'b1: cache_write_data = {store_data[15:0], 16'b0};
        endcase
    end else begin
        cache_write_data = store_data;
    end
end

// Load data extraction and sign extension
reg [31:0] extracted_data;
always_comb begin
    if (is_byte) begin
        case (effective_addr[1:0])
            2'b00: extracted_data = cache_read_data[7:0];
            2'b01: extracted_data = cache_read_data[15:8];
            2'b10: extracted_data = cache_read_data[23:16];
            2'b11: extracted_data = cache_read_data[31:24];
        endcase
    end else if (is_half) begin
        case (effective_addr[1])
            1'b0: extracted_data = cache_read_data[15:0];
            1'b1: extracted_data = cache_read_data[31:16];
        endcase
    end else begin
        extracted_data = cache_read_data;
    end
end

// Sign extension for loads
assign load_result = is_unsigned ? extracted_data :
                    is_byte ? {{24{extracted_data[7]}}, extracted_data[7:0]} :
                    is_half ? {{16{extracted_data[15]}}, extracted_data[15:0]} :
                    extracted_data;

assign lsu_ready = cache_ready || lsu_exception;

endmodule
```

### 10.3.2 Store Buffer Implementation

**Write-Through Store Buffer:**
```verilog
module store_buffer (
    input wire clk,
    input wire reset,
    
    // Store requests from LSU
    input wire store_request,
    input wire [31:0] store_addr,
    input wire [31:0] store_data,
    input wire [3:0] store_byte_enable,
    
    // Memory interface
    output wire [31:0] mem_addr,
    output wire [31:0] mem_data,
    output wire [3:0] mem_byte_enable,
    output wire mem_write,
    input wire mem_ready,
    
    // Buffer status
    output wire buffer_full,
    output wire buffer_empty
);

// Store buffer parameters
parameter BUFFER_DEPTH = 4;    // 4-entry store buffer

// Store buffer entry structure
typedef struct packed {
    logic [31:0] addr;
    logic [31:0] data;
    logic [3:0] byte_enable;
    logic valid;
} store_buffer_entry_t;

store_buffer_entry_t store_buffer [BUFFER_DEPTH];
reg [1:0] write_ptr, read_ptr;
reg [$clog2(BUFFER_DEPTH+1)-1:0] entry_count;

// Buffer status
assign buffer_full = (entry_count == BUFFER_DEPTH);
assign buffer_empty = (entry_count == 0);

// Store buffer write (from LSU)
always_ff @(posedge clk) begin
    if (reset) begin
        write_ptr <= 0;
        for (int i = 0; i < BUFFER_DEPTH; i++) begin
            store_buffer[i].valid <= 1'b0;
        end
    end else if (store_request && !buffer_full) begin
        store_buffer[write_ptr].addr <= store_addr;
        store_buffer[write_ptr].data <= store_data;
        store_buffer[write_ptr].byte_enable <= store_byte_enable;
        store_buffer[write_ptr].valid <= 1'b1;
        write_ptr <= write_ptr + 1;
    end
end

// Store buffer read (to memory)
always_ff @(posedge clk) begin
    if (reset) begin
        read_ptr <= 0;
    end else if (!buffer_empty && mem_ready) begin
        store_buffer[read_ptr].valid <= 1'b0;
        read_ptr <= read_ptr + 1;
    end
end

// Entry count management
always_ff @(posedge clk) begin
    if (reset) begin
        entry_count <= 0;
    end else begin
        case ({store_request && !buffer_full, !buffer_empty && mem_ready})
            2'b10: entry_count <= entry_count + 1; // Write only
            2'b01: entry_count <= entry_count - 1; // Read only
            2'b11: entry_count <= entry_count;     // Read and write
            2'b00: entry_count <= entry_count;     // No change
        endcase
    end
end

// Output assignments
assign mem_addr = store_buffer[read_ptr].addr;
assign mem_data = store_buffer[read_ptr].data;
assign mem_byte_enable = store_buffer[read_ptr].byte_enable;
assign mem_write = !buffer_empty;

endmodule
```

### 10.3.3 Memory Access Examples

**Load/Store Operation Examples:**
```assembly
# Byte operations
lb    t0, 0(t1)         # Load signed byte from [t1+0]
lbu   t0, 1(t1)         # Load unsigned byte from [t1+1]
sb    t0, 2(t1)         # Store byte to [t1+2]

# Halfword operations  
lh    t0, 0(t2)         # Load signed halfword from [t2+0]
lhu   t0, 2(t2)         # Load unsigned halfword from [t2+2]
sh    t0, 4(t2)         # Store halfword to [t2+4]

# Word operations
lw    t0, 0(t3)         # Load word from [t3+0]
sw    t0, 4(t3)         # Store word to [t3+4]

# Address calculation examples
la    t0, data_array    # Load address of data_array
lw    t1, 0(t0)         # Load first element
lw    t2, 4(t0)         # Load second element
lw    t3, 8(t0)         # Load third element

# Array access with offset
li    t4, 10            # Array index
slli  t4, t4, 2         # Convert to byte offset (index * 4)
add   t5, t0, t4        # Calculate element address
lw    t6, 0(t5)         # Load array[10]
```

---

## 10.4 Branch Execution Unit (BEU)

### 10.4.1 Branch Processing Architecture

**Branch Unit Features:**
- **Single-Cycle Branches**: Most branches complete in one cycle
- **Branch Prediction**: Static prediction with dynamic override
- **Target Calculation**: Parallel branch target address generation
- **Return Stack**: Hardware return address stack for function calls

```verilog
module branch_execution_unit (
    input wire clk,
    input wire reset,
    
    // Pipeline interface
    input wire beu_enable,
    input wire [31:0] pc,
    input wire [31:0] rs1_data,
    input wire [31:0] rs2_data,
    input wire [31:0] immediate,
    input wire [2:0] funct3,
    input wire [6:0] opcode,
    
    // Branch prediction interface
    input wire prediction,
    input wire [31:0] predicted_target,
    
    // Pipeline control
    output wire branch_taken,
    output wire [31:0] branch_target,
    output wire prediction_correct,
    output wire pipeline_flush_req
);

// Branch type decoding
wire is_beq  = (funct3 == 3'b000);
wire is_bne  = (funct3 == 3'b001);
wire is_blt  = (funct3 == 3'b100);
wire is_bge  = (funct3 == 3'b101);
wire is_bltu = (funct3 == 3'b110);
wire is_bgeu = (funct3 == 3'b111);
wire is_jal  = (opcode == 7'b1101111);
wire is_jalr = (opcode == 7'b1100111);

// Comparison logic
wire equal = (rs1_data == rs2_data);
wire less_than_signed = ($signed(rs1_data) < $signed(rs2_data));
wire less_than_unsigned = (rs1_data < rs2_data);

// Branch condition evaluation
wire branch_condition;
always_comb begin
    case (1'b1)
        is_beq:  branch_condition = equal;
        is_bne:  branch_condition = !equal;
        is_blt:  branch_condition = less_than_signed;
        is_bge:  branch_condition = !less_than_signed;
        is_bltu: branch_condition = less_than_unsigned;
        is_bgeu: branch_condition = !less_than_unsigned;
        is_jal:  branch_condition = 1'b1;  // Unconditional
        is_jalr: branch_condition = 1'b1;  // Unconditional
        default: branch_condition = 1'b0;
    endcase
end

// Branch target calculation
always_comb begin
    if (is_jalr) begin
        branch_target = (rs1_data + immediate) & ~32'h1; // Clear LSB
    end else begin
        branch_target = pc + immediate; // PC-relative
    end
end

assign branch_taken = beu_enable && branch_condition;

// Branch prediction evaluation
assign prediction_correct = (prediction == branch_taken) && 
                           (predicted_target == branch_target);
assign pipeline_flush_req = branch_taken && !prediction_correct;

endmodule
```

### 10.4.2 Return Address Stack

**Hardware Return Stack Implementation:**
```verilog
module return_address_stack (
    input wire clk,
    input wire reset,
    
    // Function call/return detection
    input wire is_call,          // JAL/JALR with rd=x1
    input wire is_return,        // JALR with rs1=x1, rd=x0
    input wire [31:0] return_addr,
    
    // Prediction interface
    output wire [31:0] predicted_return,
    output wire prediction_valid
);

parameter STACK_DEPTH = 8;

reg [31:0] ras_stack [STACK_DEPTH];
reg [$clog2(STACK_DEPTH)-1:0] stack_ptr;
reg [STACK_DEPTH-1:0] valid_entries;

// Stack operations
always_ff @(posedge clk) begin
    if (reset) begin
        stack_ptr <= 0;
        valid_entries <= 0;
    end else begin
        if (is_call) begin
            // Push return address
            ras_stack[stack_ptr] <= return_addr;
            valid_entries[stack_ptr] <= 1'b1;
            stack_ptr <= stack_ptr + 1;
        end else if (is_return && valid_entries[stack_ptr - 1]) begin
            // Pop return address
            valid_entries[stack_ptr - 1] <= 1'b0;
            stack_ptr <= stack_ptr - 1;
        end
    end
end

// Prediction output
assign predicted_return = ras_stack[stack_ptr - 1];
assign prediction_valid = is_return && valid_entries[stack_ptr - 1];

endmodule
```

### 10.4.3 Branch Performance Optimization

**Branch Execution Examples:**
```assembly
# Conditional branches (1 cycle when predicted correctly)
beq   t0, t1, target    # Branch if t0 == t1
bne   t0, t1, target    # Branch if t0 != t1
blt   t0, t1, target    # Branch if t0 < t1 (signed)
bge   t0, t1, target    # Branch if t0 >= t1 (signed)
bltu  t0, t1, target    # Branch if t0 < t1 (unsigned)
bgeu  t0, t1, target    # Branch if t0 >= t1 (unsigned)

# Unconditional jumps (1 cycle)
jal   ra, function      # Jump and link (call function)
jalr  x0, ra, 0         # Jump register (return from function)

# Optimized branch sequences
# Compare and branch
li    t0, 100
blt   a0, t0, less_than_100   # Branch if a0 < 100

# Loop constructs
li    t0, 10            # Loop counter
loop:
    # ... loop body ...
    addi t0, t0, -1     # Decrement counter
    bnez t0, loop       # Branch if not zero

# Function call with return address stack optimization
jal   ra, subroutine    # Hardware predicts return address
# ... main code continues ...

subroutine:
    # ... function body ...
    ret                 # Predicted by return address stack
```

---

## 10.5 Multiply/Divide Unit (MDU)

### 10.5.1 Multiplication Implementation

**32-bit × 32-bit Multiplier:**
```verilog
module multiply_unit (
    input wire clk,
    input wire reset,
    
    // Control signals
    input wire multiply_start,
    input wire [1:0] multiply_type, // 00=MUL, 01=MULH, 10=MULHSU, 11=MULHU
    
    // Operands
    input wire [31:0] multiplicand,
    input wire [31:0] multiplier,
    
    // Results
    output wire [31:0] result_low,
    output wire [31:0] result_high,
    output wire multiply_done,
    output wire [2:0] cycles_remaining
);

// Multiplication types
wire mul_signed   = (multiply_type == 2'b01); // MULH
wire mul_su       = (multiply_type == 2'b10); // MULHSU
wire mul_unsigned = (multiply_type == 2'b11); // MULHU
wire mul_low      = (multiply_type == 2'b00); // MUL

// Extended operands for signed multiplication
wire [32:0] ext_multiplicand = mul_unsigned ? {1'b0, multiplicand} : 
                              {multiplicand[31], multiplicand};
wire [32:0] ext_multiplier = (mul_unsigned || mul_su) ? {1'b0, multiplier} : 
                            {multiplier[31], multiplier};

// 3-cycle pipelined multiplier
reg [65:0] partial_product_stage1;
reg [65:0] partial_product_stage2;
reg [65:0] final_product;
reg [2:0] pipeline_valid;

always_ff @(posedge clk) begin
    if (reset) begin
        pipeline_valid <= 3'b000;
        partial_product_stage1 <= 0;
        partial_product_stage2 <= 0;
        final_product <= 0;
    end else begin
        // Stage 1: Partial product generation
        if (multiply_start) begin
            partial_product_stage1 <= ext_multiplicand * ext_multiplier[16:0];
            pipeline_valid[0] <= 1'b1;
        end else begin
            pipeline_valid[0] <= 1'b0;
        end
        
        // Stage 2: Accumulation
        if (pipeline_valid[0]) begin
            partial_product_stage2 <= partial_product_stage1 + 
                                    (ext_multiplicand * ext_multiplier[32:17] << 17);
            pipeline_valid[1] <= 1'b1;
        end else begin
            pipeline_valid[1] <= 1'b0;
        end
        
        // Stage 3: Final result
        if (pipeline_valid[1]) begin
            final_product <= partial_product_stage2;
            pipeline_valid[2] <= 1'b1;
        end else begin
            pipeline_valid[2] <= 1'b0;
        end
    end
end

assign result_low = final_product[31:0];
assign result_high = final_product[63:32];
assign multiply_done = pipeline_valid[2];
assign cycles_remaining = pipeline_valid[2] ? 3'b000 :
                         pipeline_valid[1] ? 3'b001 :
                         pipeline_valid[0] ? 3'b010 : 3'b011;

endmodule
```

### 10.5.2 Division Implementation

**Non-Restoring Division Algorithm:**
```verilog
module divide_unit (
    input wire clk,
    input wire reset,
    
    // Control signals
    input wire divide_start,
    input wire divide_signed,
    
    // Operands
    input wire [31:0] dividend,
    input wire [31:0] divisor,
    
    // Results
    output wire [31:0] quotient,
    output wire [31:0] remainder,
    output wire divide_done,
    output wire divide_by_zero,
    output wire [5:0] cycles_remaining
);

// Division state machine
typedef enum {
    DIV_IDLE,
    DIV_SETUP,
    DIV_ITERATE,
    DIV_NORMALIZE,
    DIV_DONE
} div_state_t;

div_state_t div_state;
reg [5:0] iteration_count;
reg [63:0] working_dividend;
reg [31:0] working_divisor;
reg [31:0] working_quotient;
reg dividend_negative, divisor_negative;
reg result_negative_quotient, result_negative_remainder;

// Division control
always_ff @(posedge clk) begin
    if (reset) begin
        div_state <= DIV_IDLE;
        iteration_count <= 0;
    end else begin
        case (div_state)
            DIV_IDLE: begin
                if (divide_start) begin
                    div_state <= DIV_SETUP;
                    iteration_count <= 32;
                end
            end
            
            DIV_SETUP: begin
                // Setup operands and sign tracking
                dividend_negative <= divide_signed && dividend[31];
                divisor_negative <= divide_signed && divisor[31];
                result_negative_quotient <= divide_signed && (dividend[31] ^ divisor[31]);
                result_negative_remainder <= divide_signed && dividend[31];
                
                // Convert to positive values for division
                working_dividend <= {32'h0, divide_signed && dividend[31] ? -dividend : dividend};
                working_divisor <= divide_signed && divisor[31] ? -divisor : divisor;
                working_quotient <= 32'h0;
                
                div_state <= DIV_ITERATE;
            end
            
            DIV_ITERATE: begin
                // Non-restoring division step
                if (working_dividend[63:32] >= working_divisor) begin
                    working_dividend <= {working_dividend[62:0], 1'b0} - 
                                      {working_divisor, 32'h0};
                    working_quotient <= {working_quotient[30:0], 1'b1};
                end else begin
                    working_dividend <= {working_dividend[62:0], 1'b0};
                    working_quotient <= {working_quotient[30:0], 1'b0};
                end
                
                iteration_count <= iteration_count - 1;
                if (iteration_count == 1) begin
                    div_state <= DIV_NORMALIZE;
                end
            end
            
            DIV_NORMALIZE: begin
                // Apply signs to results
                div_state <= DIV_DONE;
            end
            
            DIV_DONE: begin
                div_state <= DIV_IDLE;
            end
        endcase
    end
end

// Result assignment with sign correction
assign quotient = (div_state == DIV_DONE) ? 
    (result_negative_quotient ? -working_quotient : working_quotient) : 32'h0;
assign remainder = (div_state == DIV_DONE) ? 
    (result_negative_remainder ? -working_dividend[63:32] : working_dividend[63:32]) : 32'h0;
assign divide_done = (div_state == DIV_DONE);
assign divide_by_zero = (divisor == 32'h0);
assign cycles_remaining = (div_state == DIV_ITERATE) ? iteration_count : 6'h0;

endmodule
```

### 10.5.3 Extended Arithmetic Examples

**RV32M Extension Instructions:**
```assembly
# Multiplication instructions (3 cycles)
mul   t0, t1, t2        # t0 = (t1 * t2)[31:0]
mulh  t0, t1, t2        # t0 = (t1 * t2)[63:32] (signed × signed)
mulhsu t0, t1, t2       # t0 = (t1 * t2)[63:32] (signed × unsigned)
mulhu t0, t1, t2        # t0 = (t1 * t2)[63:32] (unsigned × unsigned)

# Division instructions (34 cycles)
div   t0, t1, t2        # t0 = t1 / t2 (signed)
divu  t0, t1, t2        # t0 = t1 / t2 (unsigned)
rem   t0, t1, t2        # t0 = t1 % t2 (signed remainder)
remu  t0, t1, t2        # t0 = t1 % t2 (unsigned remainder)

# 64-bit multiplication example
li    t1, 0x12345678
li    t2, 0x9ABCDEF0
mul   t0, t1, t2        # Low 32 bits of product
mulhu t3, t1, t2        # High 32 bits of product
# t3:t0 now contains 64-bit product

# Division with remainder
li    t1, 1000          # Dividend
li    t2, 7             # Divisor
div   t0, t1, t2        # t0 = 1000 / 7 = 142
rem   t3, t1, t2        # t3 = 1000 % 7 = 6
```

---

*This chapter provided detailed coverage of all MCU-32X execution units, enabling optimal code generation and performance analysis for both embedded and desktop computing applications.*