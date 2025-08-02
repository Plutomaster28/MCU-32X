module memory (
    input wire clk,
    input wire reset,
    input wire mem_read,
    input wire mem_write,
    input wire [31:0] address,
    input wire [31:0] write_data,
    output reg [31:0] read_data,
    output reg mem_ready
);

    // Memory array (for simplicity, using a small size)
    reg [31:0] memory [0:1023]; // 4KB of memory

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_ready <= 0;
            read_data <= 32'b0;
        end else begin
            mem_ready <= 1; // Indicate memory operation is in progress

            if (mem_read) begin
                read_data <= memory[address[11:2]]; // Read from memory
            end

            if (mem_write) begin
                memory[address[11:2]] <= write_data; // Write to memory
            end
        end
    end

endmodule
