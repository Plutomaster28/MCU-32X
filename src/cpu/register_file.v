module register_file (
    input wire clk,
    input wire reset,
    input wire [4:0] read_addr1,
    input wire [4:0] read_addr2,
    input wire [4:0] write_addr,
    input wire [31:0] write_data,
    input wire we, // write enable
    output reg [31:0] read_data1,
    output reg [31:0] read_data2
);
    // 32 general-purpose registers (x0-x31)
    // x0 is hardwired to zero in RISC-V
    reg [31:0] registers [0:31];
    
    integer i;

    // Initialize registers
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'h00000000;
            end
        end else if (we && write_addr != 5'b00000) begin
            // x0 (register 0) is hardwired to zero in RISC-V
            registers[write_addr] <= write_data;
        end
    end

    // Combinational read with x0 hardwired to zero
    always @(*) begin
        if (read_addr1 == 5'b00000)
            read_data1 = 32'h00000000;  // x0 always reads as zero
        else
            read_data1 = registers[read_addr1];
            
        if (read_addr2 == 5'b00000)
            read_data2 = 32'h00000000;  // x0 always reads as zero
        else
            read_data2 = registers[read_addr2];
    end
endmodule
