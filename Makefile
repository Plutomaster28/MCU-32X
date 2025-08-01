CC = iverilog
CFLAGS = -g2012
SRC_DIR = src
TB_DIR = testbench
OUTPUT = mcu32x

all: $(OUTPUT)

$(OUTPUT): $(SRC_DIR)/top.v $(TB_DIR)/cpu_tb.v $(TB_DIR)/cache_tb.v $(TB_DIR)/memory_tb.v
	$(CC) $(CFLAGS) -o $(OUTPUT) $(SRC_DIR)/top.v $(TB_DIR)/cpu_tb.v $(TB_DIR)/cache_tb.v $(TB_DIR)/memory_tb.v

clean:
	rm -f $(OUTPUT) *.vcd

.PHONY: all clean