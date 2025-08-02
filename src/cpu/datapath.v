module datapath (
    input wire clk,
    input wire reset,
    input wire [31:0] instruction,
    input wire [31:0] data_in,
    output wire [31:0] data_out,
    output wire [31:0] address,
    output wire mem_read,
    output wire mem_write
);

    // Internal signals
    wire [31:0] alu_result;
    wire [31:0] read_data;
    wire [31:0] reg_data1, reg_data2;
    wire [3:0] alu_control; // ALU control signals
    wire alu_zero; // ALU zero flag
    wire [4:0] write_reg;
    wire alu_src, reg_write, mem_to_reg, branch, jump;

    // Instantiate the control unit
    control_unit ctrl (
        .instruction(instruction),
        .clk(clk),
        .reset(reset),
        .alu_control(alu_control),
        .reg_write(reg_write),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .branch(branch),
        .jump(jump)
    );

    // Instantiate the register file
    register_file reg_file (
        .clk(clk),
        .read_addr1(instruction[19:15]),
        .read_addr2(instruction[24:20]),
        .write_addr(write_reg),
        .write_data(alu_result),
        .we(reg_write),
        .read_data1(reg_data1),
        .read_data2(reg_data2)
    );

    // Instantiate the ALU
    alu alu_unit (
        .A(reg_data1),
        .B(alu_src ? instruction[24:20] : reg_data2),
        .ALUOp(alu_control),
        .Result(alu_result),
        .Zero(alu_zero)
    );

    // Memory access logic
    assign address = alu_result;
    assign data_out = mem_to_reg ? read_data : alu_result;

    // Instantiate the memory module (not shown here)
    // memory mem (
    //     .address(address),
    //     .data_in(reg_data2),
    //     .data_out(read_data),
    //     .mem_read(mem_read),
    //     .mem_write(mem_write)
    // );

endmodule
