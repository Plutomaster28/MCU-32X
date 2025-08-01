module cpu_tb;

  // Parameters
  parameter CLK_PERIOD = 10; // Clock period in time units

  // Signals
  reg clk;
  reg reset;
  reg [31:0] instruction;
  wire [31:0] result;
  wire [31:0] data_out;
  
  // Instantiate the CPU
  cpu uut (
    .clk(clk),
    .reset(reset),
    .instruction(instruction),
    .result(result),
    .data_out(data_out)
  );

  // Clock generation
  initial begin
    clk = 0;
    forever #(CLK_PERIOD / 2) clk = ~clk;
  end

  // Test sequence
  initial begin
    // Initialize signals
    reset = 1;
    instruction = 32'h00000000; // Example instruction

    // Release reset
    #(CLK_PERIOD) reset = 0;

    // Apply test instructions
    #(CLK_PERIOD) instruction = 32'h00000001; // Test instruction 1
    #(CLK_PERIOD) instruction = 32'h00000002; // Test instruction 2
    // Add more instructions as needed

    // Wait for some time to observe results
    #(CLK_PERIOD * 10);

    // Finish simulation
    $finish;
  end

  // Monitor results
  initial begin
    $monitor("Time: %0t | Instruction: %h | Result: %h | Data Out: %h", 
             $time, instruction, result, data_out);
  end

endmodule