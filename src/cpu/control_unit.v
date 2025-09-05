module control_unit (
    input wire [31:0] instruction,
    input wire [6:0] opcode,
    input wire [2:0] funct3,
    input wire [6:0] funct7,
    input wire clk,
    input wire reset,
    output reg [4:0] alu_control,
    output reg reg_write,
    output reg mem_read,
    output reg mem_write,
    output reg branch,
    output reg jump,
    output reg mem_to_reg,
    output reg alu_src,
    output reg pc_src,
    output reg [1:0] wb_sel
);

    // RISC-V RV32I Base Instruction Set opcodes
    parameter OP_LUI    = 7'b0110111;  // Load Upper Immediate
    parameter OP_AUIPC  = 7'b0010111;  // Add Upper Immediate to PC
    parameter OP_JAL    = 7'b1101111;  // Jump and Link
    parameter OP_JALR   = 7'b1100111;  // Jump and Link Register
    parameter OP_BRANCH = 7'b1100011;  // Branch operations
    parameter OP_LOAD   = 7'b0000011;  // Load operations
    parameter OP_STORE  = 7'b0100011;  // Store operations
    parameter OP_IMM    = 7'b0010011;  // Immediate arithmetic
    parameter OP_REG    = 7'b0110011;  // Register-register operations
    parameter OP_FENCE  = 7'b0001111;  // Memory fence
    parameter OP_SYSTEM = 7'b1110011;  // System calls

    // ALU operation codes (expanded for RISC-V)
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

    // RISC-V control signal generation (combinational)
    always @(*) begin
        // Default control signals
        alu_control = ALU_ADD;
        reg_write = 1'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;
        branch = 1'b0;
        jump = 1'b0;
        mem_to_reg = 1'b0;
        alu_src = 1'b0;
        pc_src = 1'b0;
        wb_sel = 2'b00;

        case (opcode)
            OP_LUI: begin // Load Upper Immediate
                reg_write = 1'b1;
                alu_control = ALU_ADD;  // Pass immediate through ALU
                alu_src = 1'b1;         // Use immediate
                wb_sel = 2'b01;         // Write immediate to register
            end
            
            OP_AUIPC: begin // Add Upper Immediate to PC
                reg_write = 1'b1;
                alu_control = ALU_ADD;
                alu_src = 1'b1;
                wb_sel = 2'b00;         // ALU result
            end
            
            OP_JAL: begin // Jump and Link
                reg_write = 1'b1;
                jump = 1'b1;
                wb_sel = 2'b10;         // PC + 4
            end
            
            OP_JALR: begin // Jump and Link Register
                reg_write = 1'b1;
                jump = 1'b1;
                alu_src = 1'b1;
                wb_sel = 2'b10;         // PC + 4
            end
            
            OP_BRANCH: begin // Branch operations
                branch = 1'b1;
                case (funct3)
                    3'b000: alu_control = ALU_SUB; // BEQ
                    3'b001: alu_control = ALU_SUB; // BNE
                    3'b100: alu_control = ALU_SLT; // BLT
                    3'b101: alu_control = ALU_SLT; // BGE
                    3'b110: alu_control = ALU_SLTU; // BLTU
                    3'b111: alu_control = ALU_SLTU; // BGEU
                endcase
            end
            
            OP_LOAD: begin // Load operations
                reg_write = 1'b1;
                mem_read = 1'b1;
                alu_control = ALU_ADD;
                alu_src = 1'b1;         // Use immediate offset
                mem_to_reg = 1'b1;      // Write memory data to register
            end
            
            OP_STORE: begin // Store operations
                mem_write = 1'b1;
                alu_control = ALU_ADD;
                alu_src = 1'b1;         // Use immediate offset
            end
            
            OP_IMM: begin // Immediate arithmetic
                reg_write = 1'b1;
                alu_src = 1'b1;         // Use immediate
                case (funct3)
                    3'b000: alu_control = ALU_ADD;  // ADDI
                    3'b010: alu_control = ALU_SLT;  // SLTI
                    3'b011: alu_control = ALU_SLTU; // SLTIU
                    3'b100: alu_control = ALU_XOR;  // XORI
                    3'b110: alu_control = ALU_OR;   // ORI
                    3'b111: alu_control = ALU_AND;  // ANDI
                    3'b001: alu_control = ALU_SLL;  // SLLI
                    3'b101: begin
                        if (funct7[5]) 
                            alu_control = ALU_SRA;  // SRAI
                        else 
                            alu_control = ALU_SRL;  // SRLI
                    end
                endcase
            end
            
            OP_REG: begin // Register-register operations
                reg_write = 1'b1;
                case (funct3)
                    3'b000: begin
                        if (funct7[5]) 
                            alu_control = ALU_SUB;  // SUB
                        else 
                            alu_control = ALU_ADD;  // ADD
                    end
                    3'b001: alu_control = ALU_SLL;  // SLL
                    3'b010: alu_control = ALU_SLT;  // SLT
                    3'b011: alu_control = ALU_SLTU; // SLTU
                    3'b100: alu_control = ALU_XOR;  // XOR
                    3'b101: begin
                        if (funct7[5]) 
                            alu_control = ALU_SRA;  // SRA
                        else 
                            alu_control = ALU_SRL;  // SRL
                    end
                    3'b110: alu_control = ALU_OR;   // OR
                    3'b111: alu_control = ALU_AND;  // AND
                endcase
            end
            
            OP_FENCE: begin // Memory fence (NOP for now)
                // No operation
            end
            
            OP_SYSTEM: begin // System calls (NOP for now)
                // ECALL, EBREAK, CSR instructions would go here
            end
            
            default: begin
                // Default case for unimplemented opcodes
                // All control signals remain at their default values
            end
        endcase
    end

endmodule
