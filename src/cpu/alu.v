module alu (
    input [31:0] A,
    input [31:0] B,
    input [3:0] ALUOp,
    output reg [31:0] Result,
    output reg Zero
);

    always @(*) begin
        case (ALUOp)
            4'b0000: Result = A + B; // ADD
            4'b0001: Result = A - B; // SUB
            4'b0010: Result = A & B; // AND
            4'b0011: Result = A | B; // OR
            4'b0100: Result = A ^ B; // XOR
            4'b0101: Result = ~(A | B); // NOR
            4'b0110: Result = A << B; // SLL
            4'b0111: Result = A >> B; // SRL
            4'b1000: Result = $signed(A) >>> B; // SRA
            4'b1001: Result = A * B; // MUL (if FPU is included)
            4'b1010: Result = A / B; // DIV (if FPU is included)
            default: Result = 32'b0; // Default case
        endcase
        
        Zero = (Result == 32'b0) ? 1'b1 : 1'b0; // Set Zero flag
    end
endmodule