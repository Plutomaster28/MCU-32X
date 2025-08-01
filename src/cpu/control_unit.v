module control_unit (
    input wire [31:0] instruction,
    input wire clk,
    input wire reset,
    output reg [3:0] alu_control,
    output reg reg_write,
    output reg mem_read,
    output reg mem_write,
    output reg branch,
    output reg jump
);

    // State encoding
    parameter FETCH = 3'b000;
    parameter DECODE = 3'b001;
    parameter EXECUTE = 3'b010;
    parameter MEMORY = 3'b011;
    parameter WRITEBACK = 3'b100;

    reg [2:0] state, next_state;

    // State transition
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= FETCH;
        else
            state <= next_state;
    end

    // Next state logic
    always @(*) begin
        case (state)
            FETCH: next_state = DECODE;
            DECODE: next_state = EXECUTE;
            EXECUTE: next_state = (branch || jump) ? FETCH : MEMORY;
            MEMORY: next_state = WRITEBACK;
            WRITEBACK: next_state = FETCH;
            default: next_state = FETCH;
        endcase
    end

    // Control signal generation
    always @(*) begin
        // Default control signals
        alu_control = 4'b0000;
        reg_write = 0;
        mem_read = 0;
        mem_write = 0;
        branch = 0;
        jump = 0;

        case (state)
            DECODE: begin
                // Decode instruction and set control signals
                case (instruction[31:26]) // opcode
                    6'b000000: begin // R-type
                        alu_control = instruction[3:0]; // funct (lower 4 bits)
                        reg_write = 1;
                    end
                    6'b100011: begin // lw
                        mem_read = 1;
                        reg_write = 1;
                    end
                    6'b101011: begin // sw
                        mem_write = 1;
                    end
                    6'b000100: begin // beq
                        branch = 1;
                    end
                    6'b000010: begin // j
                        jump = 1;
                    end
                endcase
            end
        endcase
    end

endmodule