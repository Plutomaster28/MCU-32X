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
            #1 clk = ~clk;
        end
    end
    /* verilator lint_on INFINITELOOP */
    
    // Main test procedure
    initial begin
        // Initialize
        reset = 1;
        cycle_count = 0;
        
        $display("Starting CPU simulation...");
        $display("Reset asserted");
        
        // Wait a few cycles then release reset
        #10;
        reset = 0;
        $display("Reset released");
        
        // Run simulation for a while
        #200;
        
        $display("Simulation completed after sufficient time");
        $finish;
    end
    
    // Monitor the CPU outputs (reduced frequency)
    always @(posedge clk) begin
        cycle_count <= cycle_count + 1;
        
        if (!reset && (cycle_count % 4 == 0)) begin // Print every 4th cycle
            $display("Cycle: %d | Result: %h | Address: %h | MemRead: %b | MemWrite: %b", 
                     cycle_count, result, address, mem_read, mem_write);
        end
    end
endmodule