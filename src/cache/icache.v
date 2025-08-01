module icache (
    input wire clk,
    input wire reset,
    input wire [31:0] addr,
    input wire [31:0] data_in,
    input wire write_enable,
    output reg [31:0] data_out,
    output reg hit
);

    parameter CACHE_SIZE = 16; // 16 entries for the cache
    reg [31:0] cache [0:CACHE_SIZE-1];
    reg [31:0] tags [0:CACHE_SIZE-1];
    reg valid [0:CACHE_SIZE-1];

    integer i;
    integer index;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < CACHE_SIZE; i = i + 1) begin
                valid[i] <= 0;
                tags[i] <= 0;
                cache[i] <= 0;
            end
            data_out <= 0;
            hit <= 0;
        end else begin
            // Cache access logic
            index = addr[3:0]; // Assuming 16-byte cache lines
            if (write_enable) begin
                // Write data to cache
                cache[index] <= data_in;
                tags[index] <= addr[31:4]; // Store tag
                valid[index] <= 1; // Mark as valid
            end else begin
                // Read data from cache
                if (valid[index] && tags[index] == addr[31:4]) begin
                    data_out <= cache[index];
                    hit <= 1; // Cache hit
                end else begin
                    data_out <= 32'b0; // Cache miss
                    hit <= 0;
                end
            end
        end
    end
endmodule