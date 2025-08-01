module rom (
    input wire [31:0] address,
    output reg [31:0] data,
    input wire enable,
    input wire clk
);

    // ROM memory array
    reg [31:0] memory [0:16383]; // 16KB ROM

    initial begin
        // Load ROM with initial values or instructions
        // This is where you can initialize your ROM contents
        // For example:
        // memory[0] = 32'h00000000; // Instruction at address 0
        // memory[1] = 32'h00000001; // Instruction at address 1
        // Add more initializations as needed
    end

    always @(posedge clk) begin
        if (enable) begin
            data <= memory[address[13:2]]; // Accessing 4-byte aligned addresses
        end
    end

endmodule