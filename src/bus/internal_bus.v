module internal_bus (
    input wire [31:0] data_in,
    input wire [31:0] address,
    input wire read_enable,
    input wire write_enable,
    output reg [31:0] data_out,
    output reg ready
);

    // Internal memory for the bus
    reg [31:0] memory [0:255]; // 256 words of memory

    always @(*) begin
        ready = 1'b0; // Default to not ready
        if (read_enable) begin
            data_out = memory[address[7:0]]; // Read from memory
            ready = 1'b1; // Indicate ready after read
        end else if (write_enable) begin
            memory[address[7:0]] = data_in; // Write to memory
            ready = 1'b1; // Indicate ready after write
        end
    end

endmodule
