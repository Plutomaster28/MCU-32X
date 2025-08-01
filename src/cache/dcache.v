module dcache (
    input wire clk,
    input wire rst,
    input wire [31:0] addr,
    input wire [31:0] write_data,
    input wire we, // write enable
    output reg [31:0] read_data,
    output reg hit // cache hit indicator
);

    parameter CACHE_SIZE = 16; // 16 entries
    parameter BLOCK_SIZE = 4;   // 4 bytes per block

    reg [31:0] cache_data [0:CACHE_SIZE-1]; // cache data storage
    reg [31:0] cache_tags [0:CACHE_SIZE-1];  // cache tags
    reg valid [0:CACHE_SIZE-1];              // valid bits

    integer i;
    integer index;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < CACHE_SIZE; i = i + 1) begin
                valid[i] <= 0;
                cache_tags[i] <= 0;
                cache_data[i] <= 0;
            end
            read_data <= 0;
            hit <= 0;
        end else begin
            // Cache access logic
            index = (addr[3:0]); // Simple index based on lower bits of address

            if (valid[index] && (cache_tags[index] == addr[31:4])) begin
                // Cache hit
                hit <= 1;
                read_data <= cache_data[index];
                if (we) begin
                    // Write data to cache
                    cache_data[index] <= write_data;
                end
            end else begin
                // Cache miss
                hit <= 0;
                if (we) begin
                    // Load data into cache on write
                    cache_tags[index] <= addr[31:4];
                    cache_data[index] <= write_data;
                    valid[index] <= 1;
                end
            end
        end
    end
endmodule