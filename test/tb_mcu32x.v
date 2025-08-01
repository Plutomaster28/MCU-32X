// Testbench for MCU32X - Verilator compatible
module tb_mcu32x;
    reg clk, reset;
    wire [31:0] result, address;
    wire mem_read, mem_write;
    reg [7:0] cycle_count;
    
    // Instantiate the MCU32X (instruction is now internal)
    MCU32X dut(
        .clk(clk),
        .reset(reset),
        .result(result),
        .address(address),
        .mem_read(mem_read),
        .mem_write(mem_write)
    );
    
    // Clock generation for Verilator
    /* verilator lint_off INFINITELOOP */
    initial begin
        clk = 0;
        forever begin
            clk = ~clk;
        end
    end
    /* verilator lint_on INFINITELOOP */
    
    // Main test procedure
    initial begin
        $dumpfile("mcu32x.vcd");
        $dumpvars(0, tb_mcu32x);
        
        // Initialize
        reset = 1;
        cycle_count = 0;
        
        $display("Starting CPU simulation...");
        $display("Reset asserted");
        
        // Wait a bit then release reset
        // Note: In Verilator no-timing mode, we can't use delays
        // The simulation will be driven by the always blocks
        
        $display("Simulation will run for several cycles");
        // The simulation will be controlled by the cycle counter
    end
    
    // Cycle counter and reset control
    always @(posedge clk) begin
        cycle_count <= cycle_count + 1;
        
        // Release reset after 2 cycles
        if (cycle_count == 2) begin
            reset <= 0;
            $display("Reset released at cycle %d", cycle_count);
        end
        
        // End simulation after 20 cycles
        if (cycle_count >= 22) begin
            $display("Simulation completed after %d cycles", cycle_count);
            $finish;
        end
    end
    
    // Monitor the CPU outputs
    always @(posedge clk) begin
        if (!reset && cycle_count > 2) begin
            $display("Cycle: %d | Result: %h | Address: %h | MemRead: %b | MemWrite: %b", 
                     cycle_count, result, address, mem_read, mem_write);
        end
    end
endmodule