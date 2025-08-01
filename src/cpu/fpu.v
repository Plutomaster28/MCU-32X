module fpu (
    input wire clk,
    input wire rst,
    input wire [31:0] operand_a,
    input wire [31:0] operand_b,
    input wire [2:0] operation, // 0: add, 1: sub, 2: mul, 3: div
    output reg [31:0] result,
    output reg valid
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            result <= 32'b0;
            valid <= 1'b0;
        end else begin
            valid <= 1'b1;
            case (operation)
                3'b000: result <= operand_a + operand_b; // Addition
                3'b001: result <= operand_a - operand_b; // Subtraction
                3'b010: result <= operand_a * operand_b; // Multiplication
                3'b011: begin
                    if (operand_b != 32'b0) 
                        result <= operand_a / operand_b; // Division
                    else 
                        result <= 32'b0; // Handle division by zero
                end
                default: result <= 32'b0; // Default case
            endcase
        end
    end

endmodule