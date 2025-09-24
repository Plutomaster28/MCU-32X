# Chapter 17: Development Environment
## MCU-32X Technical Reference Manual

---

## 17.1 Development Environment Overview

The MCU-32X development ecosystem provides comprehensive tools and utilities designed to support both embedded systems development and desktop application programming typical of late-1990s development practices. The toolchain integrates seamlessly with popular development environments while providing MCU-32X specific optimizations and debugging capabilities.

### 17.1.1 Development Toolchain Architecture

**Core Development Components:**
- **MCU-32X GCC Toolchain**: RISC-V cross-compiler with MCU-32X optimizations
- **MCU-32X Assembler**: Native assembler with macro support and optimization
- **MCU-32X Debugger**: JTAG-based hardware debugger with real-time tracing
- **MCU-32X Simulator**: Cycle-accurate processor simulator for software development
- **MCU-32X Profiler**: Performance analysis and optimization tools
- **Development IDE**: Integrated development environment with project management

**Supported Host Platforms:**
```
Windows NT/98/2000:    Primary development platform
Linux (Red Hat 6.x):   Command-line tools and makefiles
Solaris 2.6/7:         Unix workstation support
AIX 4.3:              IBM RS/6000 compatibility
```

### 17.1.2 Toolchain Installation

**Windows Installation:**
```batch
REM MCU-32X Development Suite Installation
REM Extract development suite to C:\MCU32X\

cd C:\
mkdir MCU32X
cd MCU32X

REM Extract toolchain archive
pkunzip mcu32x-tools-v1.0.zip

REM Set environment variables
set MCU32X_ROOT=C:\MCU32X
set PATH=%MCU32X_ROOT%\bin;%PATH%
set MCU32X_INC=%MCU32X_ROOT%\include
set MCU32X_LIB=%MCU32X_ROOT%\lib

REM Add to autoexec.bat for permanent installation
echo SET MCU32X_ROOT=C:\MCU32X >> C:\AUTOEXEC.BAT
echo SET PATH=%MCU32X_ROOT%\bin;%PATH% >> C:\AUTOEXEC.BAT
echo SET MCU32X_INC=%MCU32X_ROOT%\include >> C:\AUTOEXEC.BAT
echo SET MCU32X_LIB=%MCU32X_ROOT%\lib >> C:\AUTOEXEC.BAT

REM Verify installation
mcu32x-gcc --version
mcu32x-as --version
mcu32x-ld --version
```

**Linux Installation:**
```bash
#!/bin/bash
# MCU-32X Development Suite Installation Script

# Extract toolchain
cd /opt
sudo mkdir mcu32x
cd mcu32x
sudo tar -xzf mcu32x-tools-v1.0-linux.tar.gz

# Set permissions
sudo chown -R root:root /opt/mcu32x
sudo chmod -R 755 /opt/mcu32x

# Create environment setup script
cat > /opt/mcu32x/setup-env.sh << 'EOF'
#!/bin/bash
export MCU32X_ROOT=/opt/mcu32x
export PATH=$MCU32X_ROOT/bin:$PATH
export MCU32X_INC=$MCU32X_ROOT/include
export MCU32X_LIB=$MCU32X_ROOT/lib
export LD_LIBRARY_PATH=$MCU32X_ROOT/lib:$LD_LIBRARY_PATH
EOF

chmod +x /opt/mcu32x/setup-env.sh

# Add to system profile (optional)
echo "source /opt/mcu32x/setup-env.sh" >> ~/.bashrc

# Verify installation
source /opt/mcu32x/setup-env.sh
mcu32x-gcc --version
```

---

## 17.2 Cross-Compilation Toolchain

### 17.2.1 GCC Cross-Compiler

**MCU-32X GCC Configuration:**
```c
// Target configuration for MCU-32X
#define MCU32X_TARGET_CONFIG

// Architecture-specific definitions
#define __MCU32X__              1
#define __RISC_V__              1
#define __RV32I__               1
#define __LITTLE_ENDIAN__       1

// MCU-32X specific features
#define MCU32X_HAS_CACHE        1
#define MCU32X_HAS_FPU          0  // No floating point unit
#define MCU32X_HAS_MMU          0  // No memory management unit
#define MCU32X_MAX_FREQ         100000000  // 100MHz maximum

// Compiler optimization flags for MCU-32X
// Optimized for 1999-era development practices
static const char* mcu32x_cflags = 
    "-march=rv32i "              // RISC-V 32-bit integer ISA
    "-mabi=ilp32 "               // Integer long pointer 32-bit ABI
    "-mcmodel=medlow "           // Medium-low code model
    "-mno-relax "                // Disable linker relaxation
    "-fno-pic "                  // No position-independent code
    "-ffunction-sections "       // Separate functions for linking
    "-fdata-sections "           // Separate data sections
    "-fno-exceptions "           // No C++ exceptions
    "-fno-rtti "                 // No C++ runtime type info
    "-fstack-usage "             // Generate stack usage info
    "-Wall -Wextra "             // Enable warnings
    "-std=c99";                  // C99 standard (latest for 1999)
```

**Cross-Compiler Usage:**
```bash
# Basic compilation
mcu32x-gcc -c source.c -o source.o

# Optimized compilation for embedded target
mcu32x-gcc -O2 -Os -c source.c -o source.o

# Compilation with debugging information
mcu32x-gcc -g -O0 -c source.c -o source.o

# Cross-compilation with specific MCU-32X features
mcu32x-gcc -DMCU32X_CLOCK_100MHZ -DMCU32X_CACHE_32K -c source.c -o source.o

# Linking executable for MCU-32X
mcu32x-gcc -T mcu32x.ld -nostartfiles -o program.elf \
    startup.o main.o system.o -lmcu32x

# Generate various output formats
mcu32x-objcopy -O binary program.elf program.bin    # Raw binary
mcu32x-objcopy -O ihex program.elf program.hex      # Intel HEX
mcu32x-objcopy -O srec program.elf program.s19      # Motorola S-record
```

### 17.2.2 Linker Scripts and Memory Maps

**MCU-32X Linker Script (mcu32x.ld):**
```ld
/* MCU-32X Linker Script */
/* Defines memory layout and section placement */

ENTRY(_start)

MEMORY
{
    /* Internal memory regions */
    FLASH (rx)   : ORIGIN = 0x00000000, LENGTH = 512K    /* Boot ROM/Flash */
    SRAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 64K     /* Internal SRAM */
    
    /* Cache memory (not directly addressable) */
    ICACHE       : ORIGIN = 0x10000000, LENGTH = 32K     /* Instruction cache */
    DCACHE       : ORIGIN = 0x10008000, LENGTH = 32K     /* Data cache */
    
    /* External memory interfaces */
    EXTMEM (rwx) : ORIGIN = 0x40000000, LENGTH = 16M     /* External DRAM */
    PERIPH (rw)  : ORIGIN = 0x80000000, LENGTH = 256M    /* Peripheral space */
}

SECTIONS
{
    /* Reset and interrupt vectors */
    .vectors : ALIGN(4)
    {
        _vectors_start = .;
        KEEP(*(.vectors))
        . = ALIGN(256);  /* Align to vector table size */
        _vectors_end = .;
    } > FLASH
    
    /* Program code */
    .text : ALIGN(4)
    {
        _text_start = .;
        
        /* Startup code first */
        *(.startup)
        
        /* Main program code */
        *(.text)
        *(.text.*)
        
        /* Read-only data */
        *(.rodata)
        *(.rodata.*)
        
        /* Constructor/destructor tables */
        . = ALIGN(4);
        _ctors_start = .;
        KEEP(*(.ctors))
        _ctors_end = .;
        
        _dtors_start = .;
        KEEP(*(.dtors))
        _dtors_end = .;
        
        . = ALIGN(4);
        _text_end = .;
    } > FLASH
    
    /* Initialized data (copied from FLASH to SRAM at startup) */
    .data : ALIGN(4)
    {
        _data_start = .;
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        _data_end = .;
    } > SRAM AT > FLASH
    
    /* Location of initialized data in FLASH */
    _data_load_start = LOADADDR(.data);
    
    /* Uninitialized data (cleared at startup) */
    .bss : ALIGN(4)
    {
        _bss_start = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > SRAM
    
    /* Heap space (managed by malloc) */
    .heap : ALIGN(4)
    {
        _heap_start = .;
        . = . + 0x2000;  /* 8KB heap */
        _heap_end = .;
    } > SRAM
    
    /* Stack space (grows downward from end of SRAM) */
    _stack_top = ORIGIN(SRAM) + LENGTH(SRAM);
    _stack_size = 0x1000;  /* 4KB stack */
    _stack_bottom = _stack_top - _stack_size;
    
    /* External memory sections */
    .extdata : ALIGN(4)
    {
        *(.extdata)
        *(.extbss)
    } > EXTMEM
    
    /* Debug information (not loaded) */
    .debug_info     0 : { *(.debug_info) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_ranges   0 : { *(.debug_ranges) }
    
    /* Discard unwanted sections */
    /DISCARD/ :
    {
        *(.note.GNU-stack)
        *(.gnu_debuglink)
        *(.gnu.lto_*)
    }
}

/* Provide symbols for startup code */
PROVIDE(_text_size = _text_end - _text_start);
PROVIDE(_data_size = _data_end - _data_start);
PROVIDE(_bss_size = _bss_end - _bss_start);
```

**Memory Map Verification:**
```c
// Memory map verification utility
#include "mcu32x.h"

void verify_memory_map(void) {
    printf("MCU-32X Memory Map Verification\n");
    printf("===============================\n\n");
    
    // External symbols from linker script
    extern uint32_t _text_start, _text_end, _text_size;
    extern uint32_t _data_start, _data_end, _data_size;
    extern uint32_t _bss_start, _bss_end, _bss_size;
    extern uint32_t _heap_start, _heap_end;
    extern uint32_t _stack_top, _stack_bottom;
    
    printf("Code Section (.text):\n");
    printf("  Start: 0x%08X\n", (uint32_t)&_text_start);
    printf("  End:   0x%08X\n", (uint32_t)&_text_end);
    printf("  Size:  %d bytes\n\n", (uint32_t)&_text_size);
    
    printf("Data Section (.data):\n");
    printf("  Start: 0x%08X\n", (uint32_t)&_data_start);
    printf("  End:   0x%08X\n", (uint32_t)&_data_end);
    printf("  Size:  %d bytes\n\n", (uint32_t)&_data_size);
    
    printf("BSS Section (.bss):\n");
    printf("  Start: 0x%08X\n", (uint32_t)&_bss_start);
    printf("  End:   0x%08X\n", (uint32_t)&_bss_end);
    printf("  Size:  %d bytes\n\n", (uint32_t)&_bss_size);
    
    printf("Heap:\n");
    printf("  Start: 0x%08X\n", (uint32_t)&_heap_start);
    printf("  End:   0x%08X\n", (uint32_t)&_heap_end);
    printf("  Size:  %d bytes\n\n", ((uint32_t)&_heap_end - (uint32_t)&_heap_start));
    
    printf("Stack:\n");
    printf("  Top:    0x%08X\n", (uint32_t)&_stack_top);
    printf("  Bottom: 0x%08X\n", (uint32_t)&_stack_bottom);
    printf("  Size:   %d bytes\n\n", ((uint32_t)&_stack_top - (uint32_t)&_stack_bottom));
    
    // Memory usage summary
    uint32_t total_ram = 65536;  // 64KB internal SRAM
    uint32_t used_ram = (uint32_t)&_data_size + (uint32_t)&_bss_size + 
                       ((uint32_t)&_heap_end - (uint32_t)&_heap_start) +
                       ((uint32_t)&_stack_top - (uint32_t)&_stack_bottom);
    
    printf("Memory Usage Summary:\n");
    printf("  Total RAM:   %d bytes\n", total_ram);
    printf("  Used RAM:    %d bytes\n", used_ram);
    printf("  Free RAM:    %d bytes\n", total_ram - used_ram);
    printf("  Utilization: %d%%\n", (used_ram * 100) / total_ram);
}
```

---

## 17.3 Debugging and Development Tools

### 17.3.1 JTAG Debugger Interface

**JTAG Hardware Configuration:**
```c
// JTAG interface definitions for MCU-32X
#define JTAG_BASE               0x80001000

typedef volatile struct {
    uint32_t control;           /* 0x00: JTAG control register */
    uint32_t status;            /* 0x04: JTAG status register */
    uint32_t instruction;       /* 0x08: JTAG instruction register */
    uint32_t data;              /* 0x0C: JTAG data register */
    uint32_t debug_control;     /* 0x10: Debug control */
    uint32_t debug_status;      /* 0x14: Debug status */
    uint32_t breakpoint[8];     /* 0x18-0x34: Hardware breakpoints */
    uint32_t watchpoint[4];     /* 0x38-0x44: Hardware watchpoints */
    uint32_t trace_control;     /* 0x48: Trace control */
    uint32_t trace_buffer;      /* 0x4C: Trace buffer access */
} jtag_regs_t;

#define JTAG ((jtag_regs_t*)JTAG_BASE)

// JTAG control bits
#define JTAG_ENABLE             (1 << 0)   // Enable JTAG interface
#define JTAG_DEBUG_ENABLE       (1 << 1)   // Enable debug mode
#define JTAG_TRACE_ENABLE       (1 << 2)   // Enable instruction trace
#define JTAG_BREAK_ON_RESET     (1 << 3)   // Break on system reset

// Debug control commands
#define DEBUG_CMD_HALT          0x01       // Halt CPU execution
#define DEBUG_CMD_RESUME        0x02       // Resume CPU execution
#define DEBUG_CMD_STEP          0x03       // Single step execution
#define DEBUG_CMD_RESET         0x04       // Reset CPU core

// JTAG initialization for debugging
void jtag_debug_init(void) {
    // Enable JTAG interface and debug mode
    JTAG->control = JTAG_ENABLE | JTAG_DEBUG_ENABLE | JTAG_TRACE_ENABLE;
    
    // Configure debug features
    JTAG->debug_control = DEBUG_CMD_HALT;  // Start with CPU halted
    
    // Clear all breakpoints and watchpoints
    for (int i = 0; i < 8; i++) {
        JTAG->breakpoint[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        JTAG->watchpoint[i] = 0;
    }
    
    // Configure trace buffer
    JTAG->trace_control = 0x0100;  // Enable trace, 256-entry buffer
}

// Hardware breakpoint management
void set_hardware_breakpoint(int bp_num, uint32_t address) {
    if (bp_num >= 8) return;
    
    // Set breakpoint address with enable bit
    JTAG->breakpoint[bp_num] = address | (1 << 0);
}

void clear_hardware_breakpoint(int bp_num) {
    if (bp_num >= 8) return;
    
    JTAG->breakpoint[bp_num] = 0;
}

// Hardware watchpoint management  
void set_hardware_watchpoint(int wp_num, uint32_t address, int type) {
    if (wp_num >= 4) return;
    
    // Watchpoint types: 1=read, 2=write, 3=read/write
    JTAG->watchpoint[wp_num] = address | (type << 1) | (1 << 0);
}

// Debug command interface
void debug_halt_cpu(void) {
    JTAG->debug_control = DEBUG_CMD_HALT;
    
    // Wait for CPU to halt
    while (!(JTAG->debug_status & (1 << 0))) {
        // CPU still running
    }
}

void debug_resume_cpu(void) {
    JTAG->debug_control = DEBUG_CMD_RESUME;
}

void debug_single_step(void) {
    JTAG->debug_control = DEBUG_CMD_STEP;
    
    // Wait for step to complete
    while (!(JTAG->debug_status & (1 << 1))) {
        // Step in progress
    }
}
```

**GDB Integration:**
```gdb
# MCU-32X GDB Configuration File (.gdbinit)

# Connect to JTAG interface
target remote localhost:3333

# Set architecture
set architecture riscv:rv32

# MCU-32X specific settings
set endian little
set print pretty on
set print array on
set confirm off

# Helper functions for MCU-32X debugging
define mcu32x_reset
    monitor reset halt
    flushregs
end

define mcu32x_load
    load
    monitor reset halt
end

define mcu32x_run
    continue
end

# Display MCU-32X system state
define mcu32x_status
    printf "MCU-32X System Status\n"
    printf "====================\n"
    
    # CPU registers
    printf "CPU Registers:\n"
    info registers
    
    # Cache status
    printf "\nCache Status:\n"
    x/1wx 0x10000000  # Instruction cache control
    x/1wx 0x10008000  # Data cache control
    
    # Power management
    printf "\nPower Management:\n"
    x/1wx 0x40005000  # PMU control register
    
    # Current PC and instruction
    printf "\nCurrent Instruction:\n"
    x/i $pc
end

# Memory dump helpers
define dump_sram
    dump binary memory sram_dump.bin 0x20000000 0x20010000
end

define dump_flash
    dump binary memory flash_dump.bin 0x00000000 0x00080000
end

# Breakpoint helpers for common debugging scenarios
define break_main
    break main
end

define break_interrupt
    break interrupt_handler
end

define break_exception
    break exception_handler
end

# Load symbols and start debugging session
symbol-file program.elf
mcu32x_reset
```

### 17.3.2 Software Simulator

**MCU-32X Simulator Configuration:**
```c
// MCU-32X Cycle-Accurate Simulator
// Simulates processor behavior for software development

#include "mcu32x_sim.h"

// Simulator configuration structure
typedef struct {
    // Processor configuration
    uint32_t cpu_frequency_hz;      // Simulated CPU frequency
    uint32_t memory_latency_cycles; // Memory access latency
    int enable_cache_simulation;    // Simulate cache behavior
    int enable_pipeline_simulation; // Simulate pipeline stalls
    
    // Debugging features
    int enable_instruction_trace;   // Log instruction execution
    int enable_memory_trace;        // Log memory accesses
    int enable_cycle_counting;      // Count execution cycles
    
    // Peripheral simulation
    int simulate_uart;              // Simulate UART I/O
    int simulate_timer;             // Simulate timer peripherals
    int simulate_gpio;              // Simulate GPIO operations
    
    // File I/O redirection
    char *stdin_file;               // Redirect stdin from file
    char *stdout_file;              // Redirect stdout to file
    char *stderr_file;              // Redirect stderr to file
} simulator_config_t;

// Default simulator configuration
static simulator_config_t default_sim_config = {
    .cpu_frequency_hz = 100000000,  // 100MHz simulation
    .memory_latency_cycles = 1,     // 1 cycle memory access
    .enable_cache_simulation = 1,   // Enable cache simulation
    .enable_pipeline_simulation = 1, // Enable pipeline simulation
    .enable_instruction_trace = 0,  // Disabled by default
    .enable_memory_trace = 0,       // Disabled by default  
    .enable_cycle_counting = 1,     // Enable cycle counting
    .simulate_uart = 1,             // Enable UART simulation
    .simulate_timer = 1,            // Enable timer simulation
    .simulate_gpio = 1,             // Enable GPIO simulation
    .stdin_file = NULL,             // Use console stdin
    .stdout_file = NULL,            // Use console stdout
    .stderr_file = NULL             // Use console stderr
};

// Simulator usage example
int main(int argc, char *argv[]) {
    simulator_config_t sim_config = default_sim_config;
    
    // Parse command line arguments
    parse_simulator_args(argc, argv, &sim_config);
    
    // Initialize simulator
    if (mcu32x_sim_init(&sim_config) != 0) {
        fprintf(stderr, "Error: Failed to initialize MCU-32X simulator\n");
        return 1;
    }
    
    // Load program
    if (mcu32x_sim_load_program("program.elf") != 0) {
        fprintf(stderr, "Error: Failed to load program\n");
        return 1;
    }
    
    // Run simulation
    printf("Starting MCU-32X simulation...\n");
    int result = mcu32x_sim_run();
    
    // Display simulation statistics
    mcu32x_sim_statistics_t stats = mcu32x_sim_get_statistics();
    printf("\nSimulation Statistics:\n");
    printf("======================\n");
    printf("Instructions executed: %llu\n", stats.instructions_executed);
    printf("Cycles simulated:      %llu\n", stats.cycles_simulated);
    printf("IPC (average):         %.2f\n", 
           (double)stats.instructions_executed / stats.cycles_simulated);
    printf("Cache hit rate:        %.1f%%\n", stats.cache_hit_rate * 100.0);
    printf("Memory accesses:       %llu\n", stats.memory_accesses);
    printf("Simulation time:       %.2f seconds\n", stats.simulation_time_sec);
    printf("Simulation speed:      %.1f MIPS\n", 
           (stats.instructions_executed / 1000000.0) / stats.simulation_time_sec);
    
    // Cleanup
    mcu32x_sim_cleanup();
    
    return result;
}
```

**Simulator Command-Line Interface:**
```bash
# MCU-32X Simulator Usage Examples

# Basic simulation
mcu32x-sim program.elf

# Simulation with instruction tracing
mcu32x-sim --trace-instructions program.elf

# Simulation with memory tracing  
mcu32x-sim --trace-memory program.elf

# Simulation with I/O redirection
mcu32x-sim --stdin input.txt --stdout output.txt program.elf

# Simulation with specific frequency
mcu32x-sim --frequency 50000000 program.elf  # 50MHz simulation

# Simulation with cache disabled
mcu32x-sim --no-cache program.elf

# Simulation with detailed statistics
mcu32x-sim --stats --cycles program.elf

# Batch simulation with configuration file
mcu32x-sim --config sim_config.txt program.elf

# Interactive debugging mode
mcu32x-sim --debug program.elf
```

---

## 17.4 Integrated Development Environment

### 17.4.1 MCU-32X IDE Features

**IDE Configuration (Windows):**
```ini
; MCU-32X IDE Configuration File (mcu32x.ini)
[General]
WorkspaceDir=C:\MCU32X\Workspace
ProjectTemplatesDir=C:\MCU32X\Templates
ToolchainDir=C:\MCU32X\Tools
DocumentationDir=C:\MCU32X\Docs

[Editor]
TabSize=4
ShowLineNumbers=true
SyntaxHighlighting=true
AutoIndent=true
ShowWhitespace=false
WordWrap=false
Font=Courier New
FontSize=10

[Build]
DefaultToolchain=mcu32x-gcc
DefaultOptimization=-O2
DefaultDebugInfo=true
ShowBuildOutput=true
BuildOnSave=false
ParallelBuild=true

[Debug]
DefaultDebugger=mcu32x-gdb
JTAGInterface=OpenOCD
JTAGPort=3333
JTAGSpeed=1000
AutoLoadSymbols=true
ShowDisassembly=true

[Simulator]
DefaultSimulator=mcu32x-sim
EnableCacheSimulation=true
EnableTracing=false
SimulationFrequency=100000000

[ProjectTypes]
; Available project templates
EmbeddedApplication=Embedded Application
DesktopApplication=Desktop Application (hosted)
BootLoader=Boot Loader
DeviceDriver=Device Driver
StaticLibrary=Static Library
```

**Project Template (Embedded Application):**
```c
// MCU-32X Embedded Application Template
// Generated by MCU-32X IDE v1.0

#include "mcu32x.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Application configuration
#define APP_VERSION         "1.0.0"
#define CPU_FREQUENCY_HZ    100000000
#define UART_BAUD_RATE      115200

// Forward declarations
static void system_init(void);
static void application_main(void);
static void error_handler(const char *message);

// Main entry point
int main(void) {
    // Initialize system
    system_init();
    
    // Print startup banner
    printf("MCU-32X Application v%s\n", APP_VERSION);
    printf("System Clock: %d Hz\n", CPU_FREQUENCY_HZ);
    printf("Build Date: %s %s\n", __DATE__, __TIME__);
    
    // Run main application
    application_main();
    
    // Should not reach here
    error_handler("Application exit");
    return 0;
}

// System initialization
static void system_init(void) {
    // Initialize clock system
    clock_init(CPU_FREQUENCY_HZ);
    
    // Initialize UART for console I/O
    uart_init(0, UART_BAUD_RATE);
    
    // Initialize GPIO
    gpio_init();
    
    // Initialize timers
    timer_init();
    
    // Enable global interrupts
    enable_interrupts();
}

// Main application logic
static void application_main(void) {
    printf("Application started.\n");
    
    // Main application loop
    while (1) {
        // Application-specific code goes here
        
        // Example: Blink LED
        gpio_toggle_pin(0, 0);  // Toggle PA0
        delay_ms(500);
        
        // Example: Print status
        printf("System running...\n");
    }
}

// Error handling
static void error_handler(const char *message) {
    printf("ERROR: %s\n", message);
    
    // Disable interrupts
    disable_interrupts();
    
    // Halt system
    while (1) {
        // Error state - infinite loop
    }
}

// Interrupt handlers (weak symbols - can be overridden)
__attribute__((weak))
void timer_interrupt_handler(void) {
    // Default timer interrupt handler
    clear_timer_interrupt();
}

__attribute__((weak)) 
void uart_interrupt_handler(void) {
    // Default UART interrupt handler
    clear_uart_interrupt();
}

__attribute__((weak))
void gpio_interrupt_handler(void) {
    // Default GPIO interrupt handler
    clear_gpio_interrupt();
}
```

**Makefile Template:**
```makefile
# MCU-32X Application Makefile
# Generated by MCU-32X IDE v1.0

# Project configuration
PROJECT_NAME = mcu32x_app
TARGET_ARCH = mcu32x
BUILD_DIR = build
SRC_DIR = src
INC_DIR = include

# Toolchain configuration
CC = mcu32x-gcc
AS = mcu32x-as
LD = mcu32x-ld
OBJCOPY = mcu32x-objcopy
OBJDUMP = mcu32x-objdump
SIZE = mcu32x-size

# Compiler flags
CFLAGS = -march=rv32i -mabi=ilp32 -mcmodel=medlow
CFLAGS += -O2 -g -Wall -Wextra -std=c99
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -I$(INC_DIR) -I$(MCU32X_INC)

# Assembler flags
ASFLAGS = -march=rv32i -mabi=ilp32

# Linker flags
LDFLAGS = -T mcu32x.ld -nostartfiles
LDFLAGS += -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/$(PROJECT_NAME).map

# Libraries
LIBS = -lmcu32x -lgcc

# Source files
C_SOURCES = $(wildcard $(SRC_DIR)/*.c)
ASM_SOURCES = $(wildcard $(SRC_DIR)/*.s)

# Object files
C_OBJECTS = $(C_SOURCES:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)
ASM_OBJECTS = $(ASM_SOURCES:$(SRC_DIR)/%.s=$(BUILD_DIR)/%.o)
OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

# Default target
all: $(BUILD_DIR)/$(PROJECT_NAME).elf $(BUILD_DIR)/$(PROJECT_NAME).bin $(BUILD_DIR)/$(PROJECT_NAME).hex

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile C source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	@echo "Compiling $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble assembly source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.s | $(BUILD_DIR)
	@echo "Assembling $<"
	$(AS) $(ASFLAGS) -c $< -o $@

# Link executable
$(BUILD_DIR)/$(PROJECT_NAME).elf: $(OBJECTS)
	@echo "Linking $@"
	$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $@
	$(SIZE) $@

# Generate binary image
$(BUILD_DIR)/$(PROJECT_NAME).bin: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo "Generating binary image $@"
	$(OBJCOPY) -O binary $< $@

# Generate Intel HEX image
$(BUILD_DIR)/$(PROJECT_NAME).hex: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo "Generating Intel HEX image $@"
	$(OBJCOPY) -O ihex $< $@

# Generate disassembly listing
disasm: $(BUILD_DIR)/$(PROJECT_NAME).elf
	$(OBJDUMP) -d $< > $(BUILD_DIR)/$(PROJECT_NAME).dis

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)

# Program flash memory (requires programmer)
flash: $(BUILD_DIR)/$(PROJECT_NAME).bin
	mcu32x-flash --device MCU32X --file $<

# Debug with GDB
debug: $(BUILD_DIR)/$(PROJECT_NAME).elf
	mcu32x-gdb $<

# Run in simulator
simulate: $(BUILD_DIR)/$(PROJECT_NAME).elf
	mcu32x-sim $<

# Static analysis
analyze:
	cppcheck --enable=all $(SRC_DIR)

# Generate documentation
docs:
	doxygen Doxyfile

.PHONY: all clean flash debug simulate analyze docs
```

### 17.4.2 Code Templates and Wizards

**Device Driver Template:**
```c
// MCU-32X Device Driver Template
// Generated by MCU-32X IDE Device Driver Wizard

#include "mcu32x.h"
#include "device_driver.h"

// Driver configuration
#define DRIVER_NAME         "example_device"
#define DRIVER_VERSION      "1.0.0"
#define MAX_DEVICES         4

// Device state structure
typedef struct {
    int initialized;
    uint32_t base_address;
    int interrupt_number;
    device_callback_t callback;
    void *user_data;
} device_state_t;

// Driver state
static device_state_t devices[MAX_DEVICES];
static int driver_initialized = 0;

// Driver initialization
int device_driver_init(void) {
    if (driver_initialized) {
        return DEVICE_ERROR_ALREADY_INITIALIZED;
    }
    
    // Clear device state
    memset(devices, 0, sizeof(devices));
    
    // Initialize driver resources
    // TODO: Add driver-specific initialization
    
    driver_initialized = 1;
    return DEVICE_SUCCESS;
}

// Device initialization
int device_init(int device_id, uint32_t base_address, int interrupt_num) {
    if (!driver_initialized) {
        return DEVICE_ERROR_NOT_INITIALIZED;
    }
    
    if (device_id >= MAX_DEVICES) {
        return DEVICE_ERROR_INVALID_ID;
    }
    
    device_state_t *device = &devices[device_id];
    
    if (device->initialized) {
        return DEVICE_ERROR_ALREADY_INITIALIZED;
    }
    
    // Initialize device
    device->base_address = base_address;
    device->interrupt_number = interrupt_num;
    device->callback = NULL;
    device->user_data = NULL;
    
    // TODO: Add device-specific initialization
    
    // Register interrupt handler
    if (interrupt_num >= 0) {
        register_interrupt_handler(interrupt_num, device_interrupt_handler);
    }
    
    device->initialized = 1;
    return DEVICE_SUCCESS;
}

// Device operation functions
int device_read(int device_id, uint32_t offset, void *buffer, size_t size) {
    if (!driver_initialized) {
        return DEVICE_ERROR_NOT_INITIALIZED;
    }
    
    if (device_id >= MAX_DEVICES || !devices[device_id].initialized) {
        return DEVICE_ERROR_INVALID_ID;
    }
    
    device_state_t *device = &devices[device_id];
    
    // TODO: Implement device read operation
    
    return DEVICE_SUCCESS;
}

int device_write(int device_id, uint32_t offset, const void *buffer, size_t size) {
    if (!driver_initialized) {
        return DEVICE_ERROR_NOT_INITIALIZED;
    }
    
    if (device_id >= MAX_DEVICES || !devices[device_id].initialized) {
        return DEVICE_ERROR_INVALID_ID;
    }
    
    device_state_t *device = &devices[device_id];
    
    // TODO: Implement device write operation
    
    return DEVICE_SUCCESS;
}

int device_control(int device_id, int command, void *param) {
    if (!driver_initialized) {
        return DEVICE_ERROR_NOT_INITIALIZED;
    }
    
    if (device_id >= MAX_DEVICES || !devices[device_id].initialized) {
        return DEVICE_ERROR_INVALID_ID;
    }
    
    device_state_t *device = &devices[device_id];
    
    switch (command) {
        case DEVICE_CMD_GET_STATUS:
            // TODO: Return device status
            break;
            
        case DEVICE_CMD_SET_CONFIG:
            // TODO: Set device configuration
            break;
            
        case DEVICE_CMD_RESET:
            // TODO: Reset device
            break;
            
        default:
            return DEVICE_ERROR_INVALID_COMMAND;
    }
    
    return DEVICE_SUCCESS;
}

// Set callback function for asynchronous operations
int device_set_callback(int device_id, device_callback_t callback, void *user_data) {
    if (!driver_initialized) {
        return DEVICE_ERROR_NOT_INITIALIZED;
    }
    
    if (device_id >= MAX_DEVICES || !devices[device_id].initialized) {
        return DEVICE_ERROR_INVALID_ID;
    }
    
    device_state_t *device = &devices[device_id];
    device->callback = callback;
    device->user_data = user_data;
    
    return DEVICE_SUCCESS;
}

// Interrupt handler
static void device_interrupt_handler(int interrupt_num) {
    // Find device with matching interrupt number
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (devices[i].initialized && devices[i].interrupt_number == interrupt_num) {
            device_state_t *device = &devices[i];
            
            // TODO: Handle device-specific interrupt
            
            // Call registered callback if available
            if (device->callback) {
                device->callback(i, DEVICE_EVENT_INTERRUPT, device->user_data);
            }
            
            break;
        }
    }
}

// Driver cleanup
int device_driver_cleanup(void) {
    if (!driver_initialized) {
        return DEVICE_ERROR_NOT_INITIALIZED;
    }
    
    // Cleanup all initialized devices
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (devices[i].initialized) {
            // TODO: Add device-specific cleanup
            
            // Unregister interrupt handler
            if (devices[i].interrupt_number >= 0) {
                unregister_interrupt_handler(devices[i].interrupt_number);
            }
            
            devices[i].initialized = 0;
        }
    }
    
    driver_initialized = 0;
    return DEVICE_SUCCESS;
}
```

---

*This chapter provided comprehensive coverage of the MCU-32X development environment, toolchain, and debugging capabilities, reflecting the state-of-the-art development practices available in 1999 for embedded systems and desktop processor development.*