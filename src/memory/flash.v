module flash_memory (
    input wire clk,
    input wire rst,
    input wire [31:0] address,
    input wire [31:0] data_in,
    input wire write_enable,
    output reg [31:0] data_out,
    output reg ready
);

    // Flash memory size: 16KB
    localparam MEMORY_SIZE = 16384; // 16KB
    reg [31:0] memory [0:MEMORY_SIZE-1];

    integer i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Initialize memory or handle reset
            for (i = 0; i < MEMORY_SIZE; i = i + 1) begin
                memory[i] <= 32'b0; // Clear memory on reset
            end
            ready <= 1'b0;
        end else begin
            if (write_enable) begin
                // Write data to flash memory
                memory[address[13:2]] <= data_in; // Addressing 4-byte aligned
                ready <= 1'b1;
            end else begin
                // Read data from flash memory
                data_out <= memory[address[13:2]]; // Addressing 4-byte aligned
                ready <= 1'b1;
            end
        end
    end

endmodule