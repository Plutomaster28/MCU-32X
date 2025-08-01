// Testbench for MCU32X
module tb_mcu32x;
    reg clk, reset;
    wire [31:0] result, address;
    wire mem_read, mem_write;
    
    // Instantiate the MCU32X (instruction is now internal)
    MCU32X dut(
        .clk(clk),
        .reset(reset),
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
        
        // Since instructions come from fetch stage internally,
        // we just let the CPU run and observe the outputs
        $display("Starting CPU simulation...");
        
        #100; // Run for 100 time units
        
        $display("Simulation completed");
        $finish;
    end
    
    // Monitor the CPU outputs
    always @(posedge clk) begin
        if (!reset) begin
            $display("Time: %t | Result: %h | Address: %h | MemRead: %b | MemWrite: %b", 
                     $time, result, address, mem_read, mem_write);
        end
    end
endmodule