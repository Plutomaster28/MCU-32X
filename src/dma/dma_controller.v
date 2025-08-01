module dma_controller (
    input wire clk,
    input wire reset,
    input wire start,
    input wire [31:0] src_addr,
    input wire [31:0] dest_addr,
    input wire [31:0] transfer_size,
    output reg busy,
    output reg done
);

    // State definitions
    localparam IDLE = 2'b00;
    localparam TRANSFER = 2'b01;
    localparam DONE = 2'b10;

    reg [1:0] current_state, next_state;

    // State transition
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    // Next state logic
    always @(*) begin
        case (current_state)
            IDLE: begin
                if (start) begin
                    next_state = TRANSFER;
                end else begin
                    next_state = IDLE;
                end
            end
            TRANSFER: begin
                // Implement data transfer logic here
                // For simplicity, we assume the transfer is instantaneous
                next_state = DONE;
            end
            DONE: begin
                next_state = IDLE;
            end
            default: begin
                next_state = IDLE;
            end
        endcase
    end

    // Output logic
    always @(*) begin
        case (current_state)
            IDLE: begin
                busy = 0;
                done = 0;
            end
            TRANSFER: begin
                busy = 1;
                done = 0;
            end
            DONE: begin
                busy = 0;
                done = 1;
            end
            default: begin
                busy = 0;
                done = 0;
            end
        endcase
    end

endmodule