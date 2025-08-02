module MCU32X (
    input wire clk,
    input wire reset,
    output wire [7:0] result_low,    // Only expose lower 8 bits of result for initial test
    output wire [7:0] address_low,   // Only expose lower 8 bits of address for initial test
    output wire mem_read,
    output wire mem_write
);

    // Internal signals
    wire [31:0] pc; // Program counter
    wire [31:0] instruction; // Instruction from fetch stage
    wire [31:0] alu_result; // ALU result
    wire [31:0] read_data; // Data read from memory
    wire [31:0] write_data; // Data to write to memory
    wire [31:0] reg_data; // Data from register file
    wire [31:0] fpu_result; // Result from FPU
    wire [31:0] fetch_address; // Address for fetching instructions
    
    // Program counter register
    reg [31:0] pc_reg;
    assign pc = pc_reg;
    
    // Simple PC increment logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_reg <= 32'h00000000;
        end else begin
            pc_reg <= pc_reg + 4; // Increment by 4 (word-aligned)
        end
    end

    // Assign outputs
    assign result_low = alu_result[7:0];    // Only output lower 8 bits
    assign address_low = alu_result[7:0];   // Only output lower 8 bits

    // Instantiate the components
    control_unit cu (
        .instruction(instruction),
        .clk(clk),
        .reset(reset),
        .alu_control(), // Leave unconnected for now
        .reg_write(),   // Leave unconnected for now
        .mem_read(mem_read),
        .mem_write(mem_write),
        .branch(),      // Leave unconnected for now
        .jump()         // Leave unconnected for now
    );

    register_file rf (
        .clk(clk),
        .read_addr1(5'b00000), // Default to register 0
        .read_addr2(5'b00001), // Default to register 1
        .write_addr(5'b00000), // Default to register 0
        .write_data(alu_result),
        .we(1'b0), // Default write disabled
        .read_data1(reg_data),
        .read_data2(write_data) // Connect to write_data instead of leaving unconnected
    );

    alu alu_inst (
        .A(reg_data),
        .B(write_data),
        .ALUOp(4'b0000), // Default to ADD operation
        .Result(alu_result),
        .Zero() // Leave unconnected for now
    );

    fetch fetch_stage (
        .clk(clk),
        .reset(reset),
        .pc_in(pc),
        .instruction_out(instruction),
        .pc_out() // Leave unconnected for now
    );

    memory mem (
        .clk(clk),
        .reset(reset),
        .address(alu_result),      // Use full internal address
        .write_data(write_data),
        .read_data(read_data),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .mem_ready() // Leave unconnected for now
    );

    // Optional FPU instantiation
    fpu fpu_inst (
        .clk(clk),
        .rst(reset),
        .operand_a(reg_data),
        .operand_b(write_data),
        .operation(3'b000), // Default to addition
        .result(fpu_result),
        .valid() // Leave unconnected for now
    );

    // Additional logic for connecting components can be added here

endmodule