module fetch(
    input wire clk,
    input wire reset,
    input wire [31:0] pc_in,
    output reg [31:0] instruction_out,
    output reg [31:0] pc_out
);

    // Instruction memory (for simulation purposes)
    reg [31:0] instruction_memory [0:1023]; // 1KB of instruction memory
    
    // Initialize instruction memory with NOPs
    initial begin
        integer i;
        for (i = 0; i < 1024; i = i + 1) begin
            instruction_memory[i] = 32'h00000013; // NOP instruction (addi x0, x0, 0)
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_out <= 32'b0; // Reset program counter
            instruction_out <= 32'b0; // Reset instruction output
        end else begin
            pc_out <= pc_in; // Update program counter
            instruction_out <= instruction_memory[pc_in[11:2]]; // Fetch instruction
        end
    end

endmodule
