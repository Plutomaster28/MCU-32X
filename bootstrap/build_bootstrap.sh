# MCU-32X Bootstrap Build Script
# Assembles and links the bootstrap loader for MCU-32X processor

#!/bin/bash

# Build configuration
TARGET=riscv32-unknown-elf
AS=${TARGET}-as
LD=${TARGET}-ld
OBJCOPY=${TARGET}-objcopy
OBJDUMP=${TARGET}-objdump
SIZE=${TARGET}-size

# Source files
ASM_SRC=bootstrap.s
LINKER_SCRIPT=bootstrap.ld

# Output files
ELF_OUTPUT=bootstrap.elf
BIN_OUTPUT=bootstrap.bin
HEX_OUTPUT=bootstrap.hex
MAP_OUTPUT=bootstrap.map
LST_OUTPUT=bootstrap.lst

# Assembly flags for RISC-V RV32I
ASFLAGS="-march=rv32i -mabi=ilp32 -32"

# Linker flags
LDFLAGS="-T ${LINKER_SCRIPT} -Map=${MAP_OUTPUT} --gc-sections"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}MCU-32X Bootstrap Build System${NC}"
echo "=================================="

# Check if tools are available
echo -n "Checking RISC-V toolchain... "
if ! command -v ${AS} &> /dev/null; then
    echo -e "${RED}FAILED${NC}"
    echo "Error: RISC-V toolchain not found. Please install riscv32-unknown-elf-gcc"
    echo "On Ubuntu/Debian: sudo apt-get install gcc-riscv64-unknown-elf"
    echo "On macOS with brew: brew install riscv-tools"
    exit 1
fi
echo -e "${GREEN}OK${NC}"

# Clean previous build
echo -n "Cleaning previous build... "
rm -f ${ELF_OUTPUT} ${BIN_OUTPUT} ${HEX_OUTPUT} ${MAP_OUTPUT} ${LST_OUTPUT}
echo -e "${GREEN}OK${NC}"

# Assemble bootstrap
echo -n "Assembling bootstrap.s... "
if ${AS} ${ASFLAGS} -o bootstrap.o ${ASM_SRC} 2>build.log; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
    echo "Assembly errors:"
    cat build.log
    exit 1
fi

# Link bootstrap
echo -n "Linking bootstrap... "
if ${LD} ${LDFLAGS} -o ${ELF_OUTPUT} bootstrap.o 2>>build.log; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
    echo "Linking errors:"
    cat build.log
    exit 1
fi

# Generate binary output
echo -n "Generating binary... "
if ${OBJCOPY} -O binary ${ELF_OUTPUT} ${BIN_OUTPUT}; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
    exit 1
fi

# Generate Intel HEX output
echo -n "Generating Intel HEX... "
if ${OBJCOPY} -O ihex ${ELF_OUTPUT} ${HEX_OUTPUT}; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
    exit 1
fi

# Generate disassembly listing
echo -n "Generating disassembly... "
if ${OBJDUMP} -d -S ${ELF_OUTPUT} > ${LST_OUTPUT}; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
    exit 1
fi

# Show size information
echo ""
echo -e "${YELLOW}Memory Usage:${NC}"
${SIZE} ${ELF_OUTPUT}

# Show section sizes
echo ""
echo -e "${YELLOW}Section Sizes:${NC}"
${SIZE} -A ${ELF_OUTPUT} | grep -E "(\.text|\.data|\.bss|\.vectors|\.rodata)"

# Calculate memory utilization
ROM_SIZE=65536  # 64KB ROM
RAM_SIZE=65536  # 64KB RAM

TEXT_SIZE=$(${SIZE} -A ${ELF_OUTPUT} | grep "\.text" | awk '{print $2}')
DATA_SIZE=$(${SIZE} -A ${ELF_OUTPUT} | grep "\.data" | awk '{print $2}')
BSS_SIZE=$(${SIZE} -A ${ELF_OUTPUT} | grep "\.bss" | awk '{print $2}')
RODATA_SIZE=$(${SIZE} -A ${ELF_OUTPUT} | grep "\.rodata" | awk '{print $2}')
VECTORS_SIZE=$(${SIZE} -A ${ELF_OUTPUT} | grep "\.vectors" | awk '{print $2}')

ROM_USED=$((TEXT_SIZE + RODATA_SIZE + DATA_SIZE + VECTORS_SIZE))
RAM_USED=$((DATA_SIZE + BSS_SIZE + 8192))  # Include 8KB stack

ROM_PERCENT=$((ROM_USED * 100 / ROM_SIZE))
RAM_PERCENT=$((RAM_USED * 100 / RAM_SIZE))

echo ""
echo -e "${YELLOW}Memory Utilization:${NC}"
printf "ROM: %d / %d bytes (%d%%)\n" $ROM_USED $ROM_SIZE $ROM_PERCENT
printf "RAM: %d / %d bytes (%d%%)\n" $RAM_USED $RAM_SIZE $RAM_PERCENT

# Check for warnings
if [ $ROM_PERCENT -gt 80 ]; then
    echo -e "${YELLOW}Warning: ROM usage is high (${ROM_PERCENT}%)${NC}"
fi

if [ $RAM_PERCENT -gt 80 ]; then
    echo -e "${YELLOW}Warning: RAM usage is high (${RAM_PERCENT}%)${NC}"
fi

# Verify critical sections
echo ""
echo -e "${YELLOW}Verification:${NC}"

# Check vector table alignment
VECTOR_ADDR=$(${OBJDUMP} -t ${ELF_OUTPUT} | grep "_vectors" | awk '{print $1}')
if [ ! -z "$VECTOR_ADDR" ]; then
    VECTOR_DECIMAL=$((0x$VECTOR_ADDR))
    VECTOR_ALIGN=$((VECTOR_DECIMAL % 256))
    if [ $VECTOR_ALIGN -eq 0 ]; then
        echo -e "Vector table alignment: ${GREEN}OK${NC} (0x$VECTOR_ADDR)"
    else
        echo -e "Vector table alignment: ${RED}FAILED${NC} (0x$VECTOR_ADDR not 256-byte aligned)"
    fi
fi

# Check entry point
ENTRY_ADDR=$(${OBJDUMP} -f ${ELF_OUTPUT} | grep "start address" | awk '{print $3}')
echo "Entry point: $ENTRY_ADDR"

# Show build output files
echo ""
echo -e "${YELLOW}Generated Files:${NC}"
ls -la ${ELF_OUTPUT} ${BIN_OUTPUT} ${HEX_OUTPUT} ${MAP_OUTPUT} ${LST_OUTPUT} 2>/dev/null

# Generate build report
cat > build_report.txt << EOF
MCU-32X Bootstrap Build Report
==============================
Build Date: $(date)
Toolchain: ${TARGET}

Memory Usage:
  ROM: ${ROM_USED} / ${ROM_SIZE} bytes (${ROM_PERCENT}%)
  RAM: ${RAM_USED} / ${RAM_SIZE} bytes (${RAM_PERCENT}%)

Section Sizes:
  .vectors: ${VECTORS_SIZE} bytes
  .text:    ${TEXT_SIZE} bytes
  .rodata:  ${RODATA_SIZE} bytes
  .data:    ${DATA_SIZE} bytes
  .bss:     ${BSS_SIZE} bytes

Entry Point: ${ENTRY_ADDR}
Vector Table: 0x${VECTOR_ADDR}

Output Files:
  ${ELF_OUTPUT} - ELF executable
  ${BIN_OUTPUT} - Raw binary
  ${HEX_OUTPUT} - Intel HEX format
  ${MAP_OUTPUT} - Linker map
  ${LST_OUTPUT} - Disassembly listing
EOF

echo ""
echo -e "${GREEN}Build completed successfully!${NC}"
echo "Build report saved to build_report.txt"

# Clean up intermediate files
rm -f bootstrap.o build.log

exit 0