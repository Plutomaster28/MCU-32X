// Create: test/tb_mcu32x.v
module tb_mcu32x;
    reg clk, reset;
    reg [31:0] instruction;
    wire [31:0] result, address;
    wire mem_read, mem_write;
    
    MCU32X dut(
        .clk(clk),
        .reset(reset),
        .instruction(instruction),
        .result(result),
        .address(address),
        .mem_read(mem_read),
        .mem_write(mem_write)
    );
    
    // Clock generation
    always #5 clk = ~clk;
    
    initial begin
        $dumpfile("mcu32x.vcd");
        $dumpvars(0, tb_mcu32x);
        
        clk = 0; reset = 1;
        #10 reset = 0;
        
        // Test some basic instructions
        instruction = 32'h20010001; // addi $1, $0, 1
        #10;
        instruction = 32'h20020002; // addi $2, $0, 2  
        #10;
        instruction = 32'h00221820; // add $3, $1, $2
        #50;
        
        $finish;
    end
endmodule