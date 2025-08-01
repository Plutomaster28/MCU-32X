module decode (
    input wire [31:0] instruction,
    output reg [4:0] rs1,
    output reg [4:0] rs2,
    output reg [4:0] rd,
    output reg [6:0] opcode,
    output reg [2:0] funct3,
    output reg [6:0] funct7,
    output reg [31:0] immediate
);

    always @(*) begin
        // Default values
        rs1 = instruction[19:15];
        rs2 = instruction[24:20];
        rd = instruction[11:7];
        opcode = instruction[6:0];
        funct3 = instruction[14:12];
        funct7 = instruction[31:25];

        // Immediate extraction based on opcode
        case (opcode)
            7'b0000011: // Load
                immediate = {{20{instruction[31]}}, instruction[31:20]};
            7'b0010011: // Immediate Arithmetic
                immediate = {{20{instruction[31]}}, instruction[31:20]};
            7'b0100011: // Store
                immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            7'b1100011: // Branch
                immediate = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            default:
                immediate = 32'b0; // Default to zero for unsupported opcodes
        endcase
    end
endmodule