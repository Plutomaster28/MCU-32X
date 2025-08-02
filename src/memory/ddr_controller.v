module ddr_controller (
    input wire clk,
    input wire reset,
    input wire [31:0] address,
    input wire [31:0] write_data,
    input wire read_enable,
    input wire write_enable,
    output reg [31:0] read_data,
    output reg ready
);

    // Internal signals
    reg [31:0] memory [0:1023]; // Simple memory array for simulation
    reg [9:0] addr; // Address for memory access

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ready <= 0;
            read_data <= 32'b0;
        end else begin
            addr <= address[11:2]; // Assuming 4-byte aligned addresses
            ready <= 0;

            if (read_enable) begin
                read_data <= memory[addr];
                ready <= 1; // Indicate read is ready
            end else if (write_enable) begin
                memory[addr] <= write_data;
                ready <= 1; // Indicate write is ready
            end
        end
    end

endmodule
