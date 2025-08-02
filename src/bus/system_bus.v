module system_bus (
    input wire clk,
    input wire reset,
    input wire [31:0] addr,
    input wire [31:0] data_in,
    output reg [31:0] data_out,
    input wire read_enable,
    input wire write_enable,
    output reg ready
);

    // Internal signals
    reg [31:0] memory [0:1023]; // Simple memory array for demonstration

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ready <= 1'b0;
            data_out <= 32'b0;
        end else begin
            if (read_enable) begin
                data_out <= memory[addr[11:2]]; // Read from memory
                ready <= 1'b1;
            end else if (write_enable) begin
                memory[addr[11:2]] <= data_in; // Write to memory
                ready <= 1'b1;
            end else begin
                ready <= 1'b0; // No operation
            end
        end
    end

endmodule
