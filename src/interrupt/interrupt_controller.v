module interrupt_controller (
    input wire clk,
    input wire reset,
    input wire [31:0] interrupt_request,
    output reg interrupt_ack,
    output reg [31:0] interrupt_vector
);

    // Internal state
    reg [31:0] pending_interrupts;

    // Interrupt handling logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pending_interrupts <= 32'b0;
            interrupt_ack <= 1'b0;
            interrupt_vector <= 32'b0;
        end else begin
            // Update pending interrupts
            pending_interrupts <= pending_interrupts | interrupt_request;

            // Acknowledge the highest priority interrupt
            if (pending_interrupts != 32'b0) begin
                interrupt_ack <= 1'b1;
                interrupt_vector <= pending_interrupts; // Simplified for demonstration
                pending_interrupts <= 32'b0; // Clear pending interrupts after acknowledgment
            end else begin
                interrupt_ack <= 1'b0;
            end
        end
    end

endmodule