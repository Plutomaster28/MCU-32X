# Chapter 9: Pipeline Architecture
## MCU-32X Technical Reference Manual

---

## 9.1 Pipeline Overview

The MCU-32X implements a classic 5-stage RISC pipeline optimized for single-issue execution with predictable performance characteristics.

### 9.1.1 Pipeline Stages

**Five-Stage Pipeline:**
1. **IF (Instruction Fetch)**: Fetch instruction from memory
2. **ID (Instruction Decode)**: Decode instruction and read registers  
3. **EX (Execute)**: Perform ALU operations and address calculation
4. **MEM (Memory Access)**: Access data memory for loads/stores
5. **WB (Write Back)**: Write results back to register file

**Pipeline Diagram:**
```
Cycle:    1    2    3    4    5    6    7    8
Inst 1:  IF   ID   EX  MEM   WB
Inst 2:       IF   ID   EX  MEM   WB  
Inst 3:            IF   ID   EX  MEM   WB
Inst 4:                 IF   ID   EX  MEM   WB
```

### 9.1.2 Pipeline Characteristics

**Performance Metrics:**
- **Throughput**: 1 instruction per cycle (ideal)
- **Latency**: 5 cycles for single instruction
- **CPI**: 1.0 cycles per instruction (no hazards)
- **Branch Penalty**: 2 cycles (mispredicted branches)

**Design Philosophy:**
- Simple, deterministic execution
- Minimal hardware complexity
- Predictable timing for real-time applications
- Low power consumption
- Easy to verify and test

---

## 9.2 Instruction Fetch Stage (IF)

### 9.2.1 Fetch Unit Design

**Program Counter Management:**
```verilog
// Program counter logic
always_ff @(posedge clk) begin
    if (reset) begin
        pc <= RESET_VECTOR;
    end
    else if (branch_taken) begin
        pc <= branch_target;        // Branch target from EX stage
    end
    else if (!stall) begin
        pc <= pc + 4;              // Normal increment
    end
    // else pc remains same (stall condition)
end

// Next PC calculation  
always_comb begin
    case (pc_sel)
        PC_PLUS_4:   next_pc = pc + 4;
        PC_BRANCH:   next_pc = pc + branch_offset;  
        PC_JUMP:     next_pc = jump_target;
        PC_JALR:     next_pc = (rs1_data + imm) & ~1;
        default:     next_pc = pc + 4;
    endcase
end
```

**Instruction Cache Interface:**
- **Cache Size**: 32KB, 4-way set associative
- **Line Size**: 32 bytes (8 instructions)
- **Hit Time**: 1 cycle
- **Miss Penalty**: 12 cycles to external memory

**Fetch Bandwidth:**
- Peak: 4 instructions per cycle (cache line)
- Sustained: 1 instruction per cycle
- Typical: 0.85 instructions per cycle (with branches)

### 9.2.2 Branch Prediction

**Two-Bit Saturating Counter:**
```verilog
// Branch prediction table (256 entries)
typedef enum logic [1:0] {
    STRONGLY_NOT_TAKEN = 2'b00,
    WEAKLY_NOT_TAKEN   = 2'b01,
    WEAKLY_TAKEN       = 2'b10,
    STRONGLY_TAKEN     = 2'b11
} branch_state_t;

branch_state_t bht[256];   // Branch history table

// Prediction logic
wire [7:0] bht_index = pc[9:2];  // Use PC bits for index
wire branch_predicted = (bht[bht_index] >= WEAKLY_TAKEN);

// Update on branch resolution
always_ff @(posedge clk) begin
    if (branch_resolve) begin
        case (bht[resolved_index])
            STRONGLY_NOT_TAKEN: 
                bht[resolved_index] <= branch_taken ? WEAKLY_NOT_TAKEN : STRONGLY_NOT_TAKEN;
            WEAKLY_NOT_TAKEN:
                bht[resolved_index] <= branch_taken ? WEAKLY_TAKEN : STRONGLY_NOT_TAKEN;
            WEAKLY_TAKEN:
                bht[resolved_index] <= branch_taken ? STRONGLY_TAKEN : WEAKLY_NOT_TAKEN;
            STRONGLY_TAKEN:
                bht[resolved_index] <= branch_taken ? STRONGLY_TAKEN : WEAKLY_TAKEN;
        endcase
    end
end
```

**Prediction Performance:**
- **Accuracy**: 78-85% typical workloads
- **Simple loops**: >95% accuracy  
- **Complex control flow**: 65-75% accuracy
- **Cold start**: 50% accuracy (no history)

### 9.2.3 Instruction Fetch Buffer

**Fetch Buffer Design:**
```verilog  
// 4-entry instruction buffer
typedef struct {
    logic [31:0] instruction;
    logic [31:0] pc;
    logic        valid;
} fetch_buffer_entry_t;

fetch_buffer_entry_t fetch_buffer[4];
logic [1:0] buffer_head, buffer_tail;
wire buffer_full = (buffer_tail + 1) == buffer_head;
wire buffer_empty = (buffer_tail == buffer_head);

// Buffer management
always_ff @(posedge clk) begin
    // Fill buffer from cache
    if (icache_valid && !buffer_full) begin
        fetch_buffer[buffer_tail] <= '{
            instruction: icache_data,
            pc: fetch_pc,  
            valid: 1'b1
        };
        buffer_tail <= buffer_tail + 1;
    end
    
    // Drain buffer to decode stage
    if (decode_ready && !buffer_empty) begin
        if_id_instruction <= fetch_buffer[buffer_head].instruction;
        if_id_pc <= fetch_buffer[buffer_head].pc;
        buffer_head <= buffer_head + 1;
    end
end
```

---

## 9.3 Instruction Decode Stage (ID)

### 9.3.1 Instruction Decoder

**Decode Logic Implementation:**
```verilog
// Instruction field extraction
wire [6:0]  opcode    = instruction[6:0];
wire [4:0]  rd        = instruction[11:7];
wire [2:0]  funct3    = instruction[14:12];
wire [4:0]  rs1       = instruction[19:15];
wire [4:0]  rs2       = instruction[24:20];
wire [6:0]  funct7    = instruction[31:25];

// Immediate generation
always_comb begin
    case (opcode)
        OP_IMM, OP_LOAD, OP_JALR: begin
            // I-type immediate
            immediate = {{20{instruction[31]}}, instruction[31:20]};
        end
        OP_STORE: begin
            // S-type immediate  
            immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        end
        OP_BRANCH: begin
            // B-type immediate
            immediate = {{19{instruction[31]}}, instruction[31], instruction[7], 
                        instruction[30:25], instruction[11:8], 1'b0};
        end
        OP_LUI, OP_AUIPC: begin
            // U-type immediate
            immediate = {instruction[31:12], 12'b0};
        end
        OP_JAL: begin
            // J-type immediate
            immediate = {{11{instruction[31]}}, instruction[31], instruction[19:12],
                        instruction[20], instruction[30:21], 1'b0};
        end
        default: immediate = 32'b0;
    endcase
end

// Control signal generation
always_comb begin
    // Default values
    alu_op = ALU_ADD;
    alu_src = ALU_SRC_REG;
    mem_read = 1'b0;
    mem_write = 1'b0;
    reg_write = 1'b0;
    branch = 1'b0;
    jump = 1'b0;
    
    case (opcode)
        OP_REG: begin  // R-type instructions
            alu_src = ALU_SRC_REG;
            reg_write = 1'b1;
            case ({funct7, funct3})
                {7'h00, 3'h0}: alu_op = ALU_ADD;    // ADD
                {7'h20, 3'h0}: alu_op = ALU_SUB;    // SUB
                {7'h00, 3'h1}: alu_op = ALU_SLL;    // SLL
                {7'h00, 3'h2}: alu_op = ALU_SLT;    // SLT
                {7'h00, 3'h3}: alu_op = ALU_SLTU;   // SLTU
                {7'h00, 3'h4}: alu_op = ALU_XOR;    // XOR
                {7'h00, 3'h5}: alu_op = ALU_SRL;    // SRL
                {7'h20, 3'h5}: alu_op = ALU_SRA;    // SRA
                {7'h00, 3'h6}: alu_op = ALU_OR;     // OR
                {7'h00, 3'h7}: alu_op = ALU_AND;    // AND
                default: ; // Invalid instruction
            endcase
        end
        
        OP_IMM: begin  // I-type ALU instructions
            alu_src = ALU_SRC_IMM;
            reg_write = 1'b1;
            case (funct3)
                3'h0: alu_op = ALU_ADD;     // ADDI
                3'h2: alu_op = ALU_SLT;     // SLTI  
                3'h3: alu_op = ALU_SLTU;    // SLTIU
                3'h4: alu_op = ALU_XOR;     // XORI
                3'h6: alu_op = ALU_OR;      // ORI
                3'h7: alu_op = ALU_AND;     // ANDI
                3'h1: alu_op = ALU_SLL;     // SLLI
                3'h5: begin
                    if (funct7 == 7'h00)
                        alu_op = ALU_SRL;   // SRLI
                    else  
                        alu_op = ALU_SRA;   // SRAI
                end
                default: ; // Invalid
            endcase
        end
        
        OP_LOAD: begin
            alu_op = ALU_ADD;
            alu_src = ALU_SRC_IMM;
            mem_read = 1'b1;
            reg_write = 1'b1;
        end
        
        OP_STORE: begin
            alu_op = ALU_ADD;
            alu_src = ALU_SRC_IMM;
            mem_write = 1'b1;
        end
        
        OP_BRANCH: begin
            alu_op = ALU_SUB;  // For comparison
            branch = 1'b1;
        end
        
        OP_JAL: begin
            reg_write = 1'b1;
            jump = 1'b1;
        end
        
        OP_JALR: begin
            alu_op = ALU_ADD;
            alu_src = ALU_SRC_IMM;
            reg_write = 1'b1;
            jump = 1'b1;
        end
        
        default: ; // Invalid instruction
    endcase
end
```

### 9.3.2 Register File Access

**Dual-Port Register File:**
```verilog
// Register file with 2 read ports, 1 write port
logic [31:0] registers[32];

// Read ports (combinational)
always_comb begin
    rs1_data = (rs1 == 5'b0) ? 32'b0 : registers[rs1];
    rs2_data = (rs2 == 5'b0) ? 32'b0 : registers[rs2];
end

// Write port (synchronous)
always_ff @(posedge clk) begin
    if (reg_write_enable && (rd != 5'b0)) begin
        registers[rd] <= write_data;
    end
end

// Bypass/forwarding detection
wire rs1_hazard_ex  = (rs1 != 0) && (rs1 == ex_rd) && ex_reg_write;
wire rs1_hazard_mem = (rs1 != 0) && (rs1 == mem_rd) && mem_reg_write;
wire rs2_hazard_ex  = (rs2 != 0) && (rs2 == ex_rd) && ex_reg_write;
wire rs2_hazard_mem = (rs2 != 0) && (rs2 == mem_rd) && mem_reg_write;
```

### 9.3.3 Hazard Detection

**Data Hazard Detection:**
```verilog
// Load-use hazard detection
wire load_use_hazard = ex_mem_read && ((ex_rd == rs1) || (ex_rd == rs2));

// Control hazard detection  
wire control_hazard = branch || jump;

// Stall and flush control
assign stall_if = load_use_hazard;
assign stall_id = load_use_hazard;
assign flush_ex = control_hazard || load_use_hazard;

// Pipeline control
always_ff @(posedge clk) begin
    if (reset || flush_ex) begin
        // Clear pipeline register
        ex_instruction <= NOP_INSTRUCTION;
        ex_valid <= 1'b0;
    end
    else if (!stall_id) begin
        // Normal pipeline advance
        ex_instruction <= id_instruction;
        ex_rs1_data <= forwarded_rs1_data;
        ex_rs2_data <= forwarded_rs2_data;
        ex_immediate <= immediate;
        ex_control <= control_signals;
        ex_valid <= id_valid;
    end
end
```

---

## 9.4 Execute Stage (EX)

### 9.4.1 Arithmetic Logic Unit (ALU)

**ALU Implementation:**
```verilog
// ALU operations
typedef enum logic [3:0] {
    ALU_ADD  = 4'b0000,
    ALU_SUB  = 4'b0001,  
    ALU_SLL  = 4'b0010,
    ALU_SLT  = 4'b0011,
    ALU_SLTU = 4'b0100,
    ALU_XOR  = 4'b0101,
    ALU_SRL  = 4'b0110,
    ALU_SRA  = 4'b0111,
    ALU_OR   = 4'b1000,
    ALU_AND  = 4'b1001
} alu_op_t;

// ALU logic
always_comb begin
    case (alu_op)
        ALU_ADD:  alu_result = operand_a + operand_b;
        ALU_SUB:  alu_result = operand_a - operand_b;
        ALU_SLL:  alu_result = operand_a << operand_b[4:0];
        ALU_SLT:  alu_result = ($signed(operand_a) < $signed(operand_b)) ? 1 : 0;
        ALU_SLTU: alu_result = (operand_a < operand_b) ? 1 : 0;
        ALU_XOR:  alu_result = operand_a ^ operand_b;
        ALU_SRL:  alu_result = operand_a >> operand_b[4:0];
        ALU_SRA:  alu_result = $signed(operand_a) >>> operand_b[4:0];
        ALU_OR:   alu_result = operand_a | operand_b;
        ALU_AND:  alu_result = operand_a & operand_b;
        default:  alu_result = 32'b0;
    endcase
    
    // Zero flag for branch conditions
    zero_flag = (alu_result == 0);
    
    // Less than flag for branch conditions
    less_than_flag = alu_result[0];  // From SLT operation
end
```

### 9.4.2 Branch Resolution

**Branch Target Calculation:**
```verilog
// Branch and jump target calculation
always_comb begin
    // Default values
    branch_taken = 1'b0;
    branch_target = pc + 4;
    
    if (is_branch) begin
        case (funct3)
            3'b000: branch_taken = zero_flag;           // BEQ
            3'b001: branch_taken = !zero_flag;          // BNE  
            3'b100: branch_taken = less_than_flag;      // BLT
            3'b101: branch_taken = !less_than_flag;     // BGE
            3'b110: branch_taken = less_than_flag;      // BLTU (unsigned)
            3'b111: branch_taken = !less_than_flag;     // BGEU (unsigned)
            default: branch_taken = 1'b0;
        endcase
        
        if (branch_taken) begin
            branch_target = pc + immediate;
        end
    end
    else if (is_jal) begin
        branch_taken = 1'b1;
        branch_target = pc + immediate;
    end
    else if (is_jalr) begin
        branch_taken = 1'b1;
        branch_target = (rs1_data + immediate) & ~1;  // Clear LSB
    end
end

// Return address calculation for JAL/JALR
assign return_address = pc + 4;
```

### 9.4.3 Data Forwarding

**Forwarding Unit:**
```verilog
// Data forwarding/bypass logic
always_comb begin
    // Default: use register file data
    forwarded_rs1 = id_rs1_data;
    forwarded_rs2 = id_rs2_data;
    
    // Forward from EX stage (ALU result)
    if (ex_reg_write && (ex_rd != 0) && (ex_rd == id_rs1)) begin
        forwarded_rs1 = ex_alu_result;
    end
    if (ex_reg_write && (ex_rd != 0) && (ex_rd == id_rs2)) begin
        forwarded_rs2 = ex_alu_result;  
    end
    
    // Forward from MEM stage (memory data or ALU result)
    if (mem_reg_write && (mem_rd != 0) && (mem_rd == id_rs1)) begin
        if (mem_mem_read) begin
            forwarded_rs1 = mem_read_data;  // Load result
        end else begin
            forwarded_rs1 = mem_alu_result; // ALU result
        end
    end
    if (mem_reg_write && (mem_rd != 0) && (mem_rd == id_rs2)) begin
        if (mem_mem_read) begin
            forwarded_rs2 = mem_read_data;
        end else begin
            forwarded_rs2 = mem_alu_result;
        end
    end
    
    // Priority: MEM stage has higher priority than EX stage
    // (handled by order of if statements above)
end
```

---

## 9.5 Memory Access Stage (MEM)

### 9.5.1 Data Cache Interface

**Cache Access Logic:**
```verilog
// Data cache interface
always_comb begin
    // Address calculation complete from EX stage
    dcache_addr = mem_alu_result;
    dcache_write_data = mem_rs2_data;  // Store data
    dcache_read = mem_mem_read;
    dcache_write = mem_mem_write;
    
    // Byte enable generation for sub-word operations
    case (mem_funct3)
        3'b000: dcache_byte_en = 4'b0001 << dcache_addr[1:0];  // SB
        3'b001: dcache_byte_en = 4'b0011 << dcache_addr[1:0];  // SH
        3'b010: dcache_byte_en = 4'b1111;                      // SW
        default: dcache_byte_en = 4'b1111;
    endcase
end

// Load data processing
always_comb begin
    case (mem_funct3)
        3'b000: begin  // LB (load byte, sign extend)
            case (mem_alu_result[1:0])
                2'b00: load_data = {{24{dcache_read_data[7]}}, dcache_read_data[7:0]};
                2'b01: load_data = {{24{dcache_read_data[15]}}, dcache_read_data[15:8]};
                2'b10: load_data = {{24{dcache_read_data[23]}}, dcache_read_data[23:16]};
                2'b11: load_data = {{24{dcache_read_data[31]}}, dcache_read_data[31:24]};
            endcase
        end
        3'b001: begin  // LH (load halfword, sign extend)
            case (mem_alu_result[1])
                1'b0: load_data = {{16{dcache_read_data[15]}}, dcache_read_data[15:0]};
                1'b1: load_data = {{16{dcache_read_data[31]}}, dcache_read_data[31:16]};
            endcase
        end
        3'b010: begin  // LW (load word)
            load_data = dcache_read_data;
        end
        3'b100: begin  // LBU (load byte unsigned)
            case (mem_alu_result[1:0])
                2'b00: load_data = {24'b0, dcache_read_data[7:0]};
                2'b01: load_data = {24'b0, dcache_read_data[15:8]};
                2'b10: load_data = {24'b0, dcache_read_data[23:16]};
                2'b11: load_data = {24'b0, dcache_read_data[31:24]};
            endcase
        end
        3'b101: begin  // LHU (load halfword unsigned)
            case (mem_alu_result[1])
                1'b0: load_data = {16'b0, dcache_read_data[15:0]};
                1'b1: load_data = {16'b0, dcache_read_data[31:16]};
            endcase
        end
        default: load_data = dcache_read_data;
    endcase
end
```

### 9.5.2 Memory Stall Handling

**Cache Miss Management:**
```verilog
// Memory access state machine
typedef enum logic [1:0] {
    MEM_IDLE,
    MEM_WAIT,
    MEM_FILL
} mem_state_t;

mem_state_t mem_state;

always_ff @(posedge clk) begin
    if (reset) begin
        mem_state <= MEM_IDLE;
        mem_stall <= 1'b0;
    end
    else begin
        case (mem_state)
            MEM_IDLE: begin
                if ((dcache_read || dcache_write) && !dcache_hit) begin
                    mem_state <= MEM_WAIT;
                    mem_stall <= 1'b1;
                end
                else begin
                    mem_stall <= 1'b0;
                end
            end
            
            MEM_WAIT: begin
                if (external_mem_ready) begin
                    mem_state <= MEM_FILL;
                end
            end
            
            MEM_FILL: begin
                mem_state <= MEM_IDLE;
                mem_stall <= 1'b0;
            end
        endcase
    end
end

// Stall pipeline on memory miss
assign pipeline_stall = mem_stall;
```

---

## 9.6 Write Back Stage (WB)

### 9.6.1 Result Selection

**Write-Back Multiplexer:**
```verilog
// Select data to write back to register file
always_comb begin
    case (wb_control.wb_src)
        WB_ALU:  write_back_data = wb_alu_result;
        WB_MEM:  write_back_data = wb_mem_data;
        WB_PC4:  write_back_data = wb_pc + 4;      // For JAL/JALR
        default: write_back_data = wb_alu_result;
    endcase
end

// Register write control
assign reg_write_enable = wb_control.reg_write && wb_valid;
assign reg_write_addr = wb_rd;
assign reg_write_data = write_back_data;
```

### 9.6.2 Pipeline Completion

**Instruction Retirement:**
```verilog
// Instruction completion tracking
always_ff @(posedge clk) begin
    if (reset) begin
        instructions_retired <= 64'b0;
        cycles_count <= 64'b0;
    end
    else begin
        cycles_count <= cycles_count + 1;
        
        if (wb_valid && !exception) begin
            instructions_retired <= instructions_retired + 1;
        end
    end
end

// Performance monitoring
assign cpi = cycles_count / instructions_retired;  // Cycles per instruction
assign ipc = instructions_retired / cycles_count;  // Instructions per cycle
```

---

## 9.7 Pipeline Control and Hazards

### 9.7.1 Hazard Summary

**Types of Hazards:**

1. **Structural Hazards**: 
   - Resource conflicts (none in MCU-32X due to separate I/D caches)
   
2. **Data Hazards**:
   - RAW (Read After Write): Forwarding resolves most cases
   - Load-use hazard: Requires 1-cycle stall
   
3. **Control Hazards**:
   - Branch misprediction: 2-cycle penalty
   - Unconditional jumps: 2-cycle penalty

### 9.7.2 Performance Impact Analysis

**Hazard Frequency (typical workloads):**
```
Hazard Type           Frequency    Penalty    Impact
Load-use hazard       12%         1 cycle    +12% CPI
Branch misprediction  5%          2 cycles   +10% CPI  
Jump instructions     3%          2 cycles   +6% CPI
Cache miss (I)        2%          12 cycles  +24% CPI
Cache miss (D)        4%          12 cycles  +48% CPI

Total CPI impact: ~2.0 cycles per instruction (including cache misses)
```

**Optimization Strategies:**
- Compiler scheduling to avoid load-use hazards
- Loop unrolling to reduce branch frequency  
- Better branch prediction for complex code
- Larger caches to reduce miss rates
- Prefetching for predictable access patterns

### 9.7.3 Pipeline Visualization

**Pipeline State Diagram:**
```
Normal Operation (no hazards):
T1: [IF] [ID] [EX] [MEM] [WB]
T2:      [IF] [ID] [EX]  [MEM] [WB]  
T3:           [IF] [ID]  [EX]  [MEM] [WB]

Load-Use Hazard (1 stall):
T1: [IF] [ID] [EX] [MEM] [WB]     # Load instruction
T2:      [IF] [ID] [--]  [EX]  [MEM] [WB]  # Dependent instruction stalled  
T3:           [--] [ID]  [EX]  [MEM] [WB]  # Bubble inserted
T4:                [EX]  [MEM] [WB]        # Dependent instruction continues

Branch Misprediction (2 flushes):  
T1: [IF] [ID] [EX] [MEM] [WB]     # Branch instruction
T2:      [IF] [ID] [EX]  [--]  [MEM] [WB]  # Wrong path instruction 1
T3:           [IF] [ID]  [--]  [--]  [--]  # Wrong path instruction 2  
T4:                [--]  [--]  [--]  [--]  # Flush pipeline
T5:                [IF]  [--]  [--]  [--]  # Correct path starts
```

---

*This chapter provided detailed coverage of the MCU-32X pipeline architecture, enabling understanding of performance characteristics and optimization opportunities for both hardware designers and software developers.*