module memory_tb;

  // Parameters
  parameter ADDR_WIDTH = 32;
  parameter DATA_WIDTH = 32;
  parameter MEM_SIZE = 1024; // Size of memory in words

  // Memory array
  reg [DATA_WIDTH-1:0] memory [0:MEM_SIZE-1];

  // Test signals
  reg [ADDR_WIDTH-1:0] addr;
  reg [DATA_WIDTH-1:0] data_in;
  reg write_enable;
  wire [DATA_WIDTH-1:0] data_out;

  // Instantiate the memory module (replace with actual memory module name)
  // memory_module mem (
  //   .addr(addr),
  //   .data_in(data_in),
  //   .write_enable(write_enable),
  //   .data_out(data_out)
  // );

  initial begin
    // Initialize memory
    integer i;
    for (i = 0; i < MEM_SIZE; i = i + 1) begin
      memory[i] = 32'h0;
    end

    // Test write operation
    addr = 32'h00000000;
    data_in = 32'hDEADBEEF;
    write_enable = 1;
    // mem.write(addr, data_in); // Call write method of memory module

    // Test read operation
    write_enable = 0;
    // data_out = mem.read(addr); // Call read method of memory module

    // Check if the data read is correct
    if (data_out !== data_in) begin
      $display("Error: Data mismatch at address %h. Expected: %h, Got: %h", addr, data_in, data_out);
    end else begin
      $display("Memory test passed at address %h. Data: %h", addr, data_out);
    end

    // Additional tests can be added here

    $finish;
  end

endmodule