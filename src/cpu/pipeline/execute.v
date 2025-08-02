module execute (
    input wire clk,
    input wire reset,
    input wire [31:0] alu_input_a,
    input wire [31:0] alu_input_b,
    input wire [3:0] alu_control,
    output reg [31:0] alu_result,
    output reg zero_flag
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            alu_result <= 32'b0;
            zero_flag <= 1'b0;
        end else begin
            case (alu_control)
                4'b0000: alu_result <= alu_input_a & alu_input_b; // AND
                4'b0001: alu_result <= alu_input_a | alu_input_b; // OR
                4'b0010: alu_result <= alu_input_a + alu_input_b; // ADD
                4'b0110: alu_result <= alu_input_a - alu_input_b; // SUB
                4'b0111: alu_result <= alu_input_a < alu_input_b ? 32'b1 : 32'b0; // SLT
                // Additional ALU operations can be added here
                default: alu_result <= 32'b0;
            endcase
            
            zero_flag <= (alu_result == 32'b0) ? 1'b1 : 1'b0;
        end
    end

endmodule
