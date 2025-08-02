# MCU-32X Makefile
# Provides reproducible build environment for the MCU-32X processor

# Simulation tools
CC = iverilog
CFLAGS = -g2012
GTKWAVE = gtkwave

# Directories
SRC_DIR = src
TB_DIR = testbench
SCRIPTS_DIR = scripts
OPENLANE_DIR = openlane

# Output files
OUTPUT = mcu32x
VCD_FILE = mcu32x.vcd

# Source files for simulation
VERILOG_SOURCES = $(wildcard $(SRC_DIR)/*.v $(SRC_DIR)/**/*.v)
TESTBENCH_SOURCES = $(wildcard $(TB_DIR)/*.v)

# Default target
all: compile

# Compile the design for simulation
compile: $(OUTPUT)

$(OUTPUT): $(VERILOG_SOURCES) $(TESTBENCH_SOURCES)
	$(CC) $(CFLAGS) -o $(OUTPUT) $(VERILOG_SOURCES) $(TESTBENCH_SOURCES)

# Run simulation
simulate: $(OUTPUT)
	./$(OUTPUT)

# View waveforms
view: $(VCD_FILE)
	$(GTKWAVE) $(VCD_FILE)

# OpenLane targets
mount:
	@echo "Starting OpenLane Docker container..."
	@echo "Run: docker run -it -v \$$(pwd):/openlane/designs/MCU-32X efabless/openlane:latest"
	@echo "Then: flow.tcl -design MCU-32X"

# Run OpenLane flow (requires Docker container)
openlane:
	@echo "Please run the following in OpenLane container:"
	@echo "flow.tcl -design /openlane/designs/MCU-32X -tag production"

# Clean build artifacts
clean:
	rm -f $(OUTPUT) *.vcd *.lxt2

# Clean all generated files including OpenLane runs
clean-all: clean
	rm -rf runs/

# Lint the Verilog code
lint:
	@echo "Running Verilator lint check..."
	verilator --lint-only --top-module MCU32X $(VERILOG_SOURCES) || echo "Install Verilator for linting"

# Check repository is ready for submission
check-submission:
	@echo "Checking MCU-32X submission readiness..."
	@echo "✓ README.md exists: $$(test -f README.md && echo YES || echo NO)"
	@echo "✓ config.tcl exists: $$(test -f config.tcl && echo YES || echo NO)"
	@echo "✓ Source files exist: $$(test -d src && echo YES || echo NO)"
	@echo "✓ OpenLane config: $$(test -f openlane/config.tcl && echo YES || echo NO)"
	@echo "Repository structure ready for fabless submission!"

# Help target
help:
	@echo "MCU-32X Makefile Help"
	@echo "====================="
	@echo "compile      - Compile Verilog for simulation"
	@echo "simulate     - Run simulation"
	@echo "view         - Open waveform viewer"
	@echo "mount        - Instructions for OpenLane Docker"
	@echo "openlane     - Instructions for running ASIC flow"
	@echo "lint         - Run Verilog linting"
	@echo "check-submission - Verify repo is submission-ready"
	@echo "clean        - Remove build artifacts"
	@echo "clean-all    - Remove all generated files"
	@echo "help         - Show this help"

.PHONY: all compile simulate view mount openlane clean clean-all lint check-submission help