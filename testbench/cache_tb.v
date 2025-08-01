module cache_tb;

  // Parameters
  parameter ADDR_WIDTH = 32;
  parameter DATA_WIDTH = 32;
  parameter CACHE_SIZE = 16; // Example cache size in KB

  // Signals
  reg clk;
  reg reset;
  reg [ADDR_WIDTH-1:0] addr;
  reg [DATA_WIDTH-1:0] data_in;
  wire [DATA_WIDTH-1:0] data_out;
  reg read_enable;
  reg write_enable;

  // Instantiate the cache (I-cache and D-cache)
  icache #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .CACHE_SIZE(CACHE_SIZE)) ICache (
    .clk(clk),
    .reset(reset),
    .addr(addr),
    .data_in(data_in),
    .data_out(data_out),
    .read_enable(read_enable),
    .write_enable(write_enable)
  );

  dcache #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .CACHE_SIZE(CACHE_SIZE)) DCache (
    .clk(clk),
    .reset(reset),
    .addr(addr),
    .data_in(data_in),
    .data_out(data_out),
    .read_enable(read_enable),
    .write_enable(write_enable)
  );

  // Clock generation
  initial begin
    clk = 0;
    forever #5 clk = ~clk; // 100 MHz clock
  end

  // Test procedure
  initial begin
    // Initialize signals
    reset = 1;
    read_enable = 0;
    write_enable = 0;
    addr = 0;
    data_in = 0;

    // Release reset
    #10 reset = 0;

    // Test write operation
    addr = 32'h00000000;
    data_in = 32'hDEADBEEF;
    write_enable = 1;
    #10 write_enable = 0;

    // Test read operation
    read_enable = 1;
    #10 read_enable = 0;

    // Add more test cases as needed

    // Finish simulation
    #100 $finish;
  end

endmodule