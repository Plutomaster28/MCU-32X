# Chapter 4: Signal Interface
## MCU-32X Technical Reference Manual

---

## 4.1 Signal Interface Overview

The MCU-32X processor provides a comprehensive 154-pin interface designed for maximum flexibility and performance in desktop computing applications. The pin configuration supports separate instruction and data memory interfaces, extensive GPIO capabilities, and advanced debug features suitable for both production systems and development environments.

### 4.1.1 Pin Configuration Summary

| Interface Category | Pin Count | Description |
|-------------------|-----------|-------------|
| **Clock and Reset** | 2 | System timing and initialization |
| **Instruction Memory** | 68 | Harvard architecture instruction interface |
| **Data Memory** | 69 | Separate data memory interface with byte enables |
| **GPIO Interface** | 96 | High-performance general-purpose I/O |
| **Debug Interface** | 72 | Real-time debugging and performance monitoring |
| **Interrupt Interface** | 9 | External interrupt processing |
| **Performance Counters** | 64 | Hardware performance monitoring |
| **Reserved/Future** | 4 | Expansion capability |
| **Total I/O Pins** | **154** | **Complete system interface** |

## 4.2 Clock and Reset Interface

### 4.2.1 Clock Input (CLK)

**Signal Name:** `clk`  
**Direction:** Input  
**Type:** CMOS  
**Description:** Master system clock input

**Electrical Characteristics:**
- **Voltage Levels**: 0V (low), 3.3V (high)
- **Input Frequency**: 1 MHz to 100 MHz
- **Duty Cycle**: 45% - 55%
- **Rise/Fall Time**: < 2ns (10% - 90%)
- **Clock Skew Tolerance**: ±500ps

**Clock Requirements:**
```
Timing Specifications:
• Minimum Period: 10ns (100 MHz)
• Maximum Period: 1μs (1 MHz)  
• Jitter Tolerance: ±100ps RMS
• Long-term Stability: ±50ppm
```

### 4.2.2 Reset Input (RESET)

**Signal Name:** `reset`  
**Direction:** Input  
**Type:** CMOS with internal pull-down  
**Description:** Asynchronous active-high system reset

**Reset Behavior:**
- **Assertion**: Immediately halts processor operation
- **Deassertion**: Synchronous release on rising clock edge
- **Duration**: Minimum 10 clock cycles for proper reset
- **Power-On Reset**: Automatic reset for 100μs after power stable

**Reset Sequence:**
```
Reset Timeline:
1. RESET asserted (high)
2. All pipeline stages flushed
3. PC reset to 0x00000000  
4. All registers cleared to 0x00000000
5. Control signals reset to safe states
6. RESET deasserted on clock edge
7. Normal operation begins
```

## 4.3 Instruction Memory Interface

The instruction memory interface implements the instruction-side of the Harvard architecture, providing dedicated bandwidth for instruction fetches.

### 4.3.1 Instruction Address Bus

**Signal Name:** `imem_addr[31:0]`  
**Direction:** Output  
**Type:** CMOS  
**Description:** 32-bit instruction memory address

**Address Characteristics:**
- **Address Space**: 4GB (32-bit addressing)
- **Alignment**: Word-aligned (bits [1:0] always 00)
- **Update Rate**: Every clock cycle during fetch
- **Drive Strength**: 4mA per pin

### 4.3.2 Instruction Data Bus

**Signal Name:** `imem_rdata[31:0]`  
**Direction:** Input  
**Type:** CMOS  
**Description:** 32-bit instruction data input

**Data Timing:**
- **Setup Time**: 2ns before clock edge
- **Hold Time**: 1ns after clock edge  
- **Access Time**: 8ns maximum from address valid
- **Data Width**: Always 32 bits (full instruction word)

### 4.3.3 Instruction Control Signals

**Instruction Read Enable (imem_read):**
- **Direction:** Output
- **Function:** Indicates active instruction fetch cycle
- **Timing:** Asserted with address, held for entire cycle
- **Active Level:** High (1 = fetch requested)

**Instruction Ready (imem_ready):**
- **Direction:** Input  
- **Function:** Memory ready signal for instruction data
- **Timing:** Must be stable setup/hold around clock edge
- **Active Level:** High (1 = data valid)

### 4.3.4 Instruction Interface Timing

```
Instruction Fetch Cycle:
     ┌───┐   ┌───┐   ┌───┐   ┌───┐
CLK  │   │   │   │   │   │   │   │
     └───┘   └───┘   └───┘   └───┘
         
ADDR ────┬─────────┬─────────┬─────
         │ ADDR_N  │ ADDR_N+4│     
         └─────────┴─────────┴─────

READ ─────┐                 ┌─────
          └─────────────────┘

RDY  ─────────┐       ┌───────────
              └───────┘

DATA ─────────┬───────┬─────────── 
              │INST_N │INST_N+1
              └───────┴───────────
```

## 4.4 Data Memory Interface

The data memory interface provides full 32-bit data access with byte-level control for optimal memory utilization.

### 4.4.1 Data Address Bus

**Signal Name:** `mem_addr[31:0]`  
**Direction:** Output  
**Type:** CMOS  
**Description:** 32-bit data memory address

**Address Properties:**
- **Address Space**: 4GB (full 32-bit range)
- **Alignment**: Byte, halfword, or word aligned
- **Byte Addressing**: LSB indicates byte within word
- **Address Decode**: External memory controller handles decode

### 4.4.2 Data Bus Signals

**Data Write Bus (mem_wdata[31:0]):**
- **Direction:** Output
- **Function:** Data to be written to memory
- **Width:** 32 bits (word-wide interface)
- **Timing:** Valid entire write cycle

**Data Read Bus (mem_rdata[31:0]):**
- **Direction:** Input
- **Function:** Data read from memory  
- **Width:** 32 bits (word-wide interface)
- **Timing:** Must be valid when mem_ready asserted

### 4.4.3 Data Control Signals

**Memory Read Enable (mem_read):**
- **Direction:** Output
- **Function:** Indicates read operation requested
- **Timing:** Asserted entire read cycle
- **Exclusive:** Cannot be asserted with mem_write

**Memory Write Enable (mem_write):**
- **Direction:** Output  
- **Function:** Indicates write operation requested
- **Timing:** Asserted entire write cycle
- **Exclusive:** Cannot be asserted with mem_read

**Byte Enable Strobes (mem_strb[3:0]):**
- **Direction:** Output
- **Function:** Byte-level write enables for sub-word stores
- **Encoding:** 
  - `mem_strb[0]`: Byte 0 (bits 7:0)
  - `mem_strb[1]`: Byte 1 (bits 15:8)  
  - `mem_strb[2]`: Byte 2 (bits 23:16)
  - `mem_strb[3]`: Byte 3 (bits 31:24)

**Memory Ready (mem_ready):**
- **Direction:** Input
- **Function:** Memory operation complete signal
- **Timing:** Asserted when data valid (read) or accepted (write)
- **Wait States:** Processor stalls until ready asserted

### 4.4.4 Data Access Examples

**Word Store (SW instruction):**
```
Address: 0x1000
Data: 0x12345678
Strobes: 1111 (all bytes enabled)
Result: Memory[0x1000] = 0x12345678
```

**Halfword Store (SH instruction):**  
```
Address: 0x1002  
Data: 0x0000ABCD
Strobes: 1100 (upper 2 bytes enabled)
Result: Memory[0x1002-0x1003] = 0xABCD
```

**Byte Store (SB instruction):**
```
Address: 0x1001
Data: 0x000000EF  
Strobes: 0010 (byte 1 enabled)
Result: Memory[0x1001] = 0xEF
```

## 4.5 GPIO Interface

The MCU-32X provides extensive GPIO capabilities suitable for direct peripheral control and system interfacing.

### 4.5.1 GPIO Data Signals

**GPIO Output (gpio_out[31:0]):**
- **Direction:** Output
- **Function:** GPIO output data values
- **Update Rate:** Single-cycle updates (100 MHz)
- **Drive Strength:** Configurable (2mA, 4mA, 8mA per pin)

**GPIO Input (gpio_in[31:0]):**
- **Direction:** Input  
- **Function:** GPIO input data values
- **Sampling Rate:** Every clock cycle
- **Input Characteristics:** TTL/CMOS compatible

**GPIO Direction Control (gpio_dir[31:0]):**
- **Direction:** Output
- **Function:** Pin direction control (1=output, 0=input)
- **Default State:** All inputs (0x00000000) after reset
- **Update Rate:** Single-cycle updates

### 4.5.2 GPIO Electrical Characteristics

**Output Characteristics:**
- **High Level Output Voltage (VOH)**: 2.4V minimum @ 2mA
- **Low Level Output Voltage (VOL)**: 0.4V maximum @ 2mA  
- **Output Current**: 2/4/8mA selectable drive strength
- **Rise/Fall Time**: < 5ns with 50pF load

**Input Characteristics:**
- **High Level Input Voltage (VIH)**: 2.0V minimum
- **Low Level Input Voltage (VIL)**: 0.8V maximum
- **Input Current**: ±10μA maximum (high impedance)
- **Input Capacitance**: < 10pF per pin

### 4.5.3 GPIO Programming Model

**GPIO Control Registers** (Memory-Mapped):
```
Base Address: 0xF000_0000

Offset  Name        Description
0x000   GPIO_OUT    Output data register
0x004   GPIO_IN     Input data register (read-only)  
0x008   GPIO_DIR    Direction control register
0x00C   GPIO_CFG    Configuration register
```

## 4.6 Debug Interface

The debug interface provides comprehensive visibility into processor operation for development and production testing.

### 4.6.1 Debug Status Signals

**Program Counter Output (pc_out[31:0]):**
- **Direction:** Output
- **Function:** Real-time program counter value
- **Update Rate:** Every instruction completion
- **Use Cases:** Debugging, performance analysis, fault diagnosis

**CPU Halt Status (cpu_halted):**
- **Direction:** Output  
- **Function:** Indicates processor halt condition
- **Active Level:** High (1 = halted, 0 = running)
- **Halt Causes:** Breakpoints, exceptions, debug commands

### 4.6.2 Register Debug Interface

**Debug Register Output (reg_debug[31:0]):**
- **Direction:** Output
- **Function:** Selected register file contents
- **Selection:** Controlled by reg_debug_addr
- **Update Rate:** Combinational (immediate)

**Debug Register Address (reg_debug_addr[4:0]):**
- **Direction:** Output
- **Function:** Selects register for debug output
- **Range:** 0-31 (corresponds to x0-x31)
- **Default:** x0 (always outputs 0x00000000)

### 4.6.3 Debug Interface Applications

**Software Debugging:**
- Real-time PC tracking for execution flow analysis
- Register contents monitoring without CPU intervention
- Non-intrusive debugging capabilities

**Production Testing:**
- Functional verification of register file operation
- Pipeline stage verification through PC monitoring
- Built-in self-test (BIST) support

**Performance Analysis:**
- Execution profiling through PC trace
- Hot spot identification in application code
- Real-time performance monitoring

## 4.7 Interrupt Interface

The interrupt interface provides efficient handling of external events with minimal software overhead.

### 4.7.1 Interrupt Input Signals

**Interrupt Request Lines (irq_lines[7:0]):**
- **Direction:** Input
- **Function:** External interrupt request signals
- **Active Level:** High (1 = interrupt pending)
- **Priority:** Hardware priority encoder (IRQ0 highest priority)
- **Edge/Level:** Software configurable per interrupt

### 4.7.2 Interrupt Control Signal

**Interrupt Acknowledge (irq_ack):**
- **Direction:** Output
- **Function:** Acknowledge interrupt processing
- **Timing:** Asserted during interrupt vector fetch
- **Duration:** Single clock cycle pulse

### 4.7.3 Interrupt Processing

**Interrupt Latency:**
```
Interrupt Response Sequence:
1. Interrupt detected (1 cycle)
2. Current instruction completion (0-4 cycles)
3. Pipeline flush (2 cycles)  
4. Interrupt acknowledge (1 cycle)
5. Vector fetch (1 cycle)
Total: 5-9 cycles (50-90ns worst case)
```

## 4.8 Performance Counter Interface

Hardware performance counters provide real-time system monitoring capabilities.

### 4.8.1 Counter Output Signals

**Cycle Counter (cycle_count[31:0]):**
- **Direction:** Output  
- **Function:** Total clock cycle count since reset
- **Update Rate:** Every clock cycle
- **Overflow:** Wraps to zero at 0xFFFFFFFF
- **Resolution:** Single clock cycle (10ns @ 100MHz)

**Instruction Counter (instr_count[31:0]):**
- **Direction:** Output
- **Function:** Completed instruction count since reset  
- **Update Rate:** Every instruction retirement
- **Accuracy:** Excludes flushed/stalled instructions
- **Use Cases:** IPC calculation, performance monitoring

### 4.8.2 Performance Analysis Applications

**Real-Time IPC Calculation:**
```
IPC = instruction_count / cycle_count
Typical Values: 0.85 - 0.95 for well-optimized code
```

**Workload Characterization:**
- Compute vs. memory intensive code identification
- Branch prediction effectiveness analysis
- Pipeline efficiency monitoring

---

*This chapter provided comprehensive signal interface specifications for the MCU-32X processor. The next chapter will detail the programming model and register architecture.*