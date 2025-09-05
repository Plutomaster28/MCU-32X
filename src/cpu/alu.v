module alu (
    input [31:0] A,
    input [31:0] B,
    input [4:0] ALUOp,  // Expanded to 5 bits for RISC-V
    output reg [31:0] Result,
    output reg Zero
);

    // RISC-V ALU operation codes
    parameter ALU_ADD   = 5'b00000;
    parameter ALU_SUB   = 5'b00001;
    parameter ALU_SLL   = 5'b00010;
    parameter ALU_SLT   = 5'b00011;
    parameter ALU_SLTU  = 5'b00100;
    parameter ALU_XOR   = 5'b00101;
    parameter ALU_SRL   = 5'b00110;
    parameter ALU_SRA   = 5'b00111;
    parameter ALU_OR    = 5'b01000;
    parameter ALU_AND   = 5'b01001;
    parameter ALU_MUL   = 5'b01010;  // For M extension
    parameter ALU_DIV   = 5'b01011;  // For M extension

    always @(*) begin
        case (ALUOp)
            ALU_ADD:  Result = A + B;                    // ADD/ADDI
            ALU_SUB:  Result = A - B;                    // SUB
            ALU_SLL:  Result = A << B[4:0];              // SLL/SLLI (shift left logical)
            ALU_SLT:  Result = ($signed(A) < $signed(B)) ? 32'h1 : 32'h0; // SLT/SLTI
            ALU_SLTU: Result = (A < B) ? 32'h1 : 32'h0;  // SLTU/SLTIU
            ALU_XOR:  Result = A ^ B;                    // XOR/XORI
            ALU_SRL:  Result = A >> B[4:0];              // SRL/SRLI (shift right logical)
            ALU_SRA:  Result = $signed(A) >>> B[4:0];    // SRA/SRAI (shift right arithmetic)
            ALU_OR:   Result = A | B;                    // OR/ORI
            ALU_AND:  Result = A & B;                    // AND/ANDI
            ALU_MUL:  Result = A * B;                    // MUL (if M extension)
            ALU_DIV:  Result = (B != 0) ? $signed(A) / $signed(B) : 32'hFFFFFFFF; // DIV
            default:  Result = 32'b0;                    // Default case
        endcase
        
        Zero = (Result == 32'b0) ? 1'b1 : 1'b0;         // Set Zero flag
    end
endmodule
