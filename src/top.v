module MCU32X (
    // Clock and Reset
    input wire clk,
    input wire reset,
    
    // External Memory Interface (32-bit)
    output wire [31:0] mem_addr,
    output wire [31:0] mem_wdata,
    input wire [31:0] mem_rdata,
    output wire mem_read,
    output wire mem_write,
    output wire [3:0] mem_strb,      // Byte enable strobes
    input wire mem_ready,
    
    // External Instruction Memory Interface
    output wire [31:0] imem_addr,
    input wire [31:0] imem_rdata,
    output wire imem_read,
    input wire imem_ready,
    
    // GPIO Interface (for demo purposes)
    output wire [31:0] gpio_out,
    input wire [31:0] gpio_in,
    output wire [31:0] gpio_dir,     // GPIO direction control
    
    // Debug/Status Interface
    output wire [31:0] pc_out,       // Current program counter
    output wire [31:0] reg_debug,    // Debug register output
    output wire [4:0] reg_debug_addr, // Debug register address
    output wire cpu_halted,          // CPU halt status
    
    // Interrupt Interface
    input wire [7:0] irq_lines,      // External interrupt lines
    output wire irq_ack,             // Interrupt acknowledge
    
    // Performance Counters
    output wire [31:0] cycle_count,
    output wire [31:0] instr_count
);

    // Internal signals for RISC-V pipeline
    wire [31:0] pc; // Program counter
    wire [31:0] instruction; // Instruction from fetch stage
    wire [31:0] alu_result; // ALU result
    wire [31:0] read_data; // Data read from memory
    wire [31:0] write_data; // Data to write to memory
    wire [31:0] reg_data1, reg_data2; // Data from register file
    wire [31:0] fpu_result; // Result from FPU
    wire [31:0] fetch_address; // Address for fetching instructions
    
    // RISC-V specific control signals
    wire [6:0] opcode;           // RISC-V 7-bit opcode
    wire [2:0] funct3;           // 3-bit function code
    wire [6:0] funct7;           // 7-bit function code
    wire [4:0] rs1, rs2, rd;     // Source and destination registers
    wire [31:0] immediate;       // Immediate value
    wire [4:0] alu_control;      // ALU operation control (expanded to 5 bits)
    wire reg_write;              // Register write enable
    wire branch;                 // Branch signal
    wire jump;                   // Jump signal
    wire alu_zero;               // ALU zero flag
    wire mem_ready_int;          // Internal memory ready signal
    wire fpu_valid;              // FPU valid signal
    
    // Additional RISC-V control signals
    wire mem_to_reg;             // Memory to register mux control
    wire alu_src;                // ALU source mux control
    wire pc_src;                 // PC source mux control
    wire [1:0] wb_sel;           // Writeback selection
    
    // Performance counter registers
    reg [31:0] cycle_counter;
    reg [31:0] instruction_counter;
    
    // GPIO registers
    reg [31:0] gpio_out_reg;
    reg [31:0] gpio_dir_reg;
    
    // Program counter register with proper RISC-V behavior
    reg [31:0] pc_reg;
    assign pc = pc_reg;
    
    // Performance counters
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            cycle_counter <= 32'h00000000;
            instruction_counter <= 32'h00000000;
        end else begin
            cycle_counter <= cycle_counter + 1;
            if (!cpu_halted && mem_ready) begin
                instruction_counter <= instruction_counter + 1;
            end
        end
    end
    
    // GPIO control
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            gpio_out_reg <= 32'h00000000;
            gpio_dir_reg <= 32'h00000000;
        end else begin
            // GPIO can be controlled via memory-mapped writes
            // Implementation would depend on memory map
        end
    end
    
    // Simple PC increment logic (to be enhanced with proper RISC-V PC control)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_reg <= 32'h00000000;
        end else if (jump) begin
            pc_reg <= alu_result;  // Jump to target address
        end else if (branch && alu_zero) begin
            pc_reg <= pc + immediate;  // Branch offset
        end else begin
            pc_reg <= pc_reg + 4; // Increment by 4 (word-aligned)
        end
    end

    // Output assignments for expanded interface
    assign mem_addr = alu_result;
    assign mem_wdata = reg_data2;
    assign mem_strb = 4'b1111;  // Full word access for now
    assign imem_addr = pc;
    assign imem_read = 1'b1;     // Always reading instructions
    assign pc_out = pc;
    assign reg_debug = reg_data1;
    assign reg_debug_addr = rs1;
    assign gpio_out = gpio_out_reg;
    assign gpio_dir = gpio_dir_reg;
    assign cpu_halted = 1'b0;    // Never halted in this simple implementation
    assign irq_ack = 1'b0;       // No interrupt handling yet
    assign cycle_count = cycle_counter;
    assign instr_count = instruction_counter;

    // Instantiate the RISC-V decoder
    decode decoder (
        .instruction(instruction),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7),
        .immediate(immediate)
    );

    // Instantiate the RISC-V control unit
    control_unit cu (
        .instruction(instruction),
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7),
        .clk(clk),
        .reset(reset),
        .alu_control(alu_control),
        .reg_write(reg_write),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .branch(branch),
        .jump(jump),
        .mem_to_reg(mem_to_reg),
        .alu_src(alu_src),
        .pc_src(pc_src),
        .wb_sel(wb_sel)
    );

    // RISC-V register file (x0 is hardwired to 0)
    register_file rf (
        .clk(clk),
        .reset(reset),
        .read_addr1(rs1),
        .read_addr2(rs2),
        .write_addr(rd),
        .write_data(mem_to_reg ? read_data : alu_result),
        .we(reg_write),
        .read_data1(reg_data1),
        .read_data2(reg_data2)
    );

    // RISC-V ALU with expanded operations
    alu alu_inst (
        .A(reg_data1),
        .B(alu_src ? immediate : reg_data2),
        .ALUOp(alu_control),
        .Result(alu_result),
        .Zero(alu_zero)
    );

    // Instruction fetch stage
    fetch fetch_stage (
        .clk(clk),
        .reset(reset),
        .pc_in(pc),
        .instruction_out(instruction),
        .pc_out(),  // Not used in this simplified version
        .mem_addr(imem_addr),
        .mem_data(imem_rdata),
        .mem_ready(imem_ready)
    );

    // Memory interface (simplified, can be expanded)
    assign read_data = mem_rdata;

    // Optional FPU instantiation (placeholder for RISC-V F extension)
    fpu fpu_inst (
        .clk(clk),
        .rst(reset),
        .operand_a(reg_data1),
        .operand_b(reg_data2),
        .operation(funct3), // Use funct3 for FP operation selection
        .result(fpu_result),
        .valid(fpu_valid)
    );

endmodule