module writeback(
    input wire clk,
    input wire rst,
    input wire [31:0] alu_result,
    input wire [4:0] write_reg,
    input wire reg_write_enable
);

    // Register file should be internal, not exposed as a port
    reg [31:0] reg_file [0:31];
    integer i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset all registers
            for (i = 0; i < 32; i = i + 1) begin
                reg_file[i] <= 32'b0;
            end
        end else if (reg_write_enable) begin
            reg_file[write_reg] <= alu_result;
        end
    end

endmodule
