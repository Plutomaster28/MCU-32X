module fetch(
    input wire clk,
    input wire reset,
    input wire [31:0] pc_in,
    output reg [31:0] instruction_out,
    output reg [31:0] pc_out,
    // External memory interface
    output wire [31:0] mem_addr,
    input wire [31:0] mem_data,
    input wire mem_ready
);

    // Use external memory interface for instruction fetch
    assign mem_addr = pc_in;
    
    // Instruction memory (for simulation/fallback purposes)
    reg [31:0] instruction_memory [0:1023]; // 1KB of instruction memory
    
    // Initialize instruction memory with RISC-V NOPs
    initial begin
        integer i;
        for (i = 0; i < 1024; i = i + 1) begin
            instruction_memory[i] = 32'h00000013; // RISC-V NOP instruction (addi x0, x0, 0)
        end
        
        // Add some sample RISC-V instructions for testing
        instruction_memory[0] = 32'h00100093;  // addi x1, x0, 1     (x1 = 1)
        instruction_memory[1] = 32'h00200113;  // addi x2, x0, 2     (x2 = 2)
        instruction_memory[2] = 32'h002081b3;  // add  x3, x1, x2    (x3 = x1 + x2)
        instruction_memory[3] = 32'h40208233;  // sub  x4, x1, x2    (x4 = x1 - x2)
        instruction_memory[4] = 32'h0020f2b3;  // and  x5, x1, x2    (x5 = x1 & x2)
        instruction_memory[5] = 32'h0020e333;  // or   x6, x1, x2    (x6 = x1 | x2)
        instruction_memory[6] = 32'h0020c3b3;  // xor  x7, x1, x2    (x7 = x1 ^ x2)
        instruction_memory[7] = 32'h00000013;  // nop
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_out <= 32'b0; // Reset program counter
            instruction_out <= 32'h00000013; // Reset to NOP instruction
        end else if (mem_ready) begin
            pc_out <= pc_in; // Update program counter
            instruction_out <= mem_data; // Use external memory data
        end else begin
            // Fallback to internal memory if external not ready
            pc_out <= pc_in;
            instruction_out <= instruction_memory[pc_in[11:2]]; // Fetch instruction
        end
    end

endmodule
