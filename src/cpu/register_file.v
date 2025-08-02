module register_file (
    input wire clk,
    input wire [4:0] read_addr1,
    input wire [4:0] read_addr2,
    input wire [4:0] write_addr,
    input wire [31:0] write_data,
    input wire we, // write enable
    output reg [31:0] read_data1,
    output reg [31:0] read_data2
);
    // 32 general-purpose registers
    reg [31:0] registers [0:31];

    always @(posedge clk) begin
        if (we) begin
            registers[write_addr] <= write_data;
        end
    end

    always @(*) begin
        read_data1 = registers[read_addr1];
        read_data2 = registers[read_addr2];
    end
endmodule
