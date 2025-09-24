# Chapter 13: Power Management
## MCU-32X Technical Reference Manual

---

## 13.1 Power Management Overview

The MCU-32X implements comprehensive power management features designed for both high-performance desktop applications and power-sensitive embedded systems. The power management architecture supports multiple operating modes, dynamic voltage and frequency scaling, and fine-grained power domain control.

### 13.1.1 Power Management Philosophy

**Design Objectives:**
- **Performance Scalability**: Dynamic adjustment of performance vs. power consumption
- **Battery Life Extension**: Optimized power modes for portable applications
- **Thermal Management**: Prevent thermal runaway in high-performance configurations
- **Compatibility**: Maintain software compatibility across power modes

**Power Consumption Targets (180nm Process, 1.8V Core):**
```
Full Performance Mode:    500 mW @ 100 MHz
Balanced Mode:           250 mW @ 50 MHz  
Power Saver Mode:        100 mW @ 25 MHz
Sleep Mode:               10 mW (clocks gated)
Deep Sleep Mode:          1 mW (minimal retention)
Hibernate Mode:           0.1 mW (external wake only)
```

### 13.1.2 Power Domain Architecture

**Power Domain Organization:**
- **Core Domain**: CPU pipeline, register file, ALU
- **Cache Domain**: Instruction and data caches
- **Memory Interface Domain**: External memory controllers
- **Peripheral Domain**: UART, Timer, GPIO, DMA
- **I/O Domain**: External interface pins and drivers
- **Always-On Domain**: RTC, wake-up logic, minimal SRAM

**Power Domain Control:**
```verilog
module power_management_unit (
    input wire clk,
    input wire reset,
    
    // Power mode control
    input wire [2:0] power_mode,
    input wire power_mode_change_req,
    
    // Clock control
    output wire core_clk_enable,
    output wire cache_clk_enable,
    output wire memory_clk_enable,
    output wire peripheral_clk_enable,
    
    // Voltage control
    output wire [3:0] core_voltage_level,
    output wire [3:0] io_voltage_level,
    
    // Power gating
    output wire core_power_enable,
    output wire cache_power_enable,
    output wire memory_power_enable,
    output wire peripheral_power_enable,
    
    // Status and interrupts
    output wire power_mode_ready,
    output wire thermal_warning,
    output wire power_good
);

// Power modes
parameter PM_PERFORMANCE  = 3'b000;
parameter PM_BALANCED     = 3'b001;
parameter PM_POWER_SAVER  = 3'b010;
parameter PM_SLEEP        = 3'b011;
parameter PM_DEEP_SLEEP   = 3'b100;
parameter PM_HIBERNATE    = 3'b101;

// Current power mode state
reg [2:0] current_power_mode;
reg [2:0] target_power_mode;
reg mode_transition_active;

// Power mode transition state machine
typedef enum {
    PM_STABLE,
    PM_PREPARE_TRANSITION,
    PM_VOLTAGE_SCALING,
    PM_CLOCK_SCALING,
    PM_POWER_GATING,
    PM_TRANSITION_COMPLETE
} pm_state_t;

pm_state_t pm_state;
reg [7:0] transition_delay_counter;

// Power mode control logic
always_ff @(posedge clk) begin
    if (reset) begin
        current_power_mode <= PM_PERFORMANCE;
        target_power_mode <= PM_PERFORMANCE;
        mode_transition_active <= 1'b0;
        pm_state <= PM_STABLE;
    end else begin
        case (pm_state)
            PM_STABLE: begin
                if (power_mode_change_req && power_mode != current_power_mode) begin
                    target_power_mode <= power_mode;
                    mode_transition_active <= 1'b1;
                    pm_state <= PM_PREPARE_TRANSITION;
                end
            end
            
            PM_PREPARE_TRANSITION: begin
                // Save critical state if needed
                transition_delay_counter <= 8'd10; // Preparation delay
                pm_state <= PM_VOLTAGE_SCALING;
            end
            
            PM_VOLTAGE_SCALING: begin
                if (transition_delay_counter > 0) begin
                    transition_delay_counter <= transition_delay_counter - 1;
                end else begin
                    transition_delay_counter <= 8'd5; // Voltage settling time
                    pm_state <= PM_CLOCK_SCALING;
                end
            end
            
            PM_CLOCK_SCALING: begin
                if (transition_delay_counter > 0) begin
                    transition_delay_counter <= transition_delay_counter - 1;
                end else begin
                    pm_state <= PM_POWER_GATING;
                end
            end
            
            PM_POWER_GATING: begin
                transition_delay_counter <= 8'd3; // Power gating delay
                pm_state <= PM_TRANSITION_COMPLETE;
            end
            
            PM_TRANSITION_COMPLETE: begin
                if (transition_delay_counter > 0) begin
                    transition_delay_counter <= transition_delay_counter - 1;
                end else begin
                    current_power_mode <= target_power_mode;
                    mode_transition_active <= 1'b0;
                    pm_state <= PM_STABLE;
                end
            end
        endcase
    end
end

// Clock enable generation
always_comb begin
    case (current_power_mode)
        PM_PERFORMANCE: begin
            core_clk_enable = 1'b1;
            cache_clk_enable = 1'b1;
            memory_clk_enable = 1'b1;
            peripheral_clk_enable = 1'b1;
        end
        PM_BALANCED: begin
            core_clk_enable = 1'b1;
            cache_clk_enable = 1'b1;
            memory_clk_enable = 1'b1;
            peripheral_clk_enable = 1'b1;
        end
        PM_POWER_SAVER: begin
            core_clk_enable = 1'b1;
            cache_clk_enable = 1'b1;
            memory_clk_enable = 1'b1;
            peripheral_clk_enable = 1'b1;
        end
        PM_SLEEP: begin
            core_clk_enable = 1'b0;
            cache_clk_enable = 1'b0;
            memory_clk_enable = 1'b0;
            peripheral_clk_enable = 1'b1; // Keep for wake-up
        end
        PM_DEEP_SLEEP: begin
            core_clk_enable = 1'b0;
            cache_clk_enable = 1'b0;
            memory_clk_enable = 1'b0;
            peripheral_clk_enable = 1'b0;
        end
        PM_HIBERNATE: begin
            core_clk_enable = 1'b0;
            cache_clk_enable = 1'b0;
            memory_clk_enable = 1'b0;
            peripheral_clk_enable = 1'b0;
        end
        default: begin
            core_clk_enable = 1'b1;
            cache_clk_enable = 1'b1;
            memory_clk_enable = 1'b1;
            peripheral_clk_enable = 1'b1;
        end
    endcase
end

// Voltage level generation
always_comb begin
    case (current_power_mode)
        PM_PERFORMANCE:  {core_voltage_level, io_voltage_level} = {4'd15, 4'd15}; // 1.8V
        PM_BALANCED:     {core_voltage_level, io_voltage_level} = {4'd12, 4'd15}; // 1.5V core
        PM_POWER_SAVER:  {core_voltage_level, io_voltage_level} = {4'd10, 4'd15}; // 1.3V core
        PM_SLEEP:        {core_voltage_level, io_voltage_level} = {4'd8,  4'd15}; // 1.1V core
        PM_DEEP_SLEEP:   {core_voltage_level, io_voltage_level} = {4'd6,  4'd12}; // 0.9V both
        PM_HIBERNATE:    {core_voltage_level, io_voltage_level} = {4'd4,  4'd8};  // 0.7V both
        default:         {core_voltage_level, io_voltage_level} = {4'd15, 4'd15};
    endcase
end

assign power_mode_ready = !mode_transition_active;

endmodule
```

---

## 13.2 Dynamic Voltage and Frequency Scaling (DVFS)

### 13.2.1 DVFS Implementation

**Voltage and Frequency Relationships:**
```
Voltage Level | Core Voltage | Max Frequency | Power Scaling
--------------|--------------|---------------|---------------
15 (Max)      | 1.8V         | 100 MHz      | 1.00x
12            | 1.5V         | 75 MHz       | 0.56x
10            | 1.3V         | 50 MHz       | 0.35x
8             | 1.1V         | 30 MHz       | 0.20x
6             | 0.9V         | 15 MHz       | 0.11x
4             | 0.7V         | 8 MHz        | 0.06x
```

**DVFS Controller:**
```verilog
module dvfs_controller (
    input wire clk,
    input wire reset,
    
    // Performance monitoring
    input wire [7:0] cpu_utilization,
    input wire [7:0] cache_miss_rate,
    input wire [7:0] memory_bandwidth_usage,
    
    // Temperature monitoring
    input wire [7:0] core_temperature,
    input wire [7:0] ambient_temperature,
    
    // Manual override
    input wire manual_dvfs_enable,
    input wire [3:0] manual_voltage_level,
    input wire [7:0] manual_frequency_divider,
    
    // DVFS outputs
    output wire [3:0] voltage_level,
    output wire [7:0] frequency_divider,
    output wire dvfs_transition_req,
    output wire [2:0] performance_level
);

// Performance thresholds
parameter CPU_UTIL_HIGH = 8'd220;     // 85% utilization
parameter CPU_UTIL_LOW = 8'd128;      // 50% utilization
parameter TEMP_WARNING = 8'd200;      // 78°C (normalized)
parameter TEMP_CRITICAL = 8'd230;     // 90°C (normalized)

// DVFS algorithm state
reg [3:0] current_voltage_level;
reg [7:0] current_frequency_divider;
reg [7:0] dvfs_hysteresis_counter;
reg [2:0] current_performance_level;

// Performance level definitions
parameter PERF_MAX = 3'd7;
parameter PERF_HIGH = 3'd5;
parameter PERF_MEDIUM = 3'd3;
parameter PERF_LOW = 3'd1;
parameter PERF_MIN = 3'd0;

// DVFS decision logic
always_ff @(posedge clk) begin
    if (reset) begin
        current_voltage_level <= 4'd15;        // Maximum voltage
        current_frequency_divider <= 8'd1;     // No frequency division
        current_performance_level <= PERF_MAX;
        dvfs_hysteresis_counter <= 0;
    end else if (!manual_dvfs_enable) begin
        // Automatic DVFS based on performance and thermal monitoring
        
        // Temperature-based emergency scaling
        if (core_temperature > TEMP_CRITICAL) begin
            // Emergency thermal throttling
            current_performance_level <= PERF_MIN;
            dvfs_hysteresis_counter <= 8'd50; // Long delay before increasing
        end else if (core_temperature > TEMP_WARNING) begin
            // Thermal warning - reduce performance
            if (current_performance_level > PERF_LOW) begin
                current_performance_level <= current_performance_level - 1;
                dvfs_hysteresis_counter <= 8'd20;
            end
        end else begin
            // Normal operation - performance-based scaling
            if (dvfs_hysteresis_counter > 0) begin
                dvfs_hysteresis_counter <= dvfs_hysteresis_counter - 1;
            end else begin
                // Check if we should increase performance
                if (cpu_utilization > CPU_UTIL_HIGH && 
                    current_performance_level < PERF_MAX) begin
                    current_performance_level <= current_performance_level + 1;
                    dvfs_hysteresis_counter <= 8'd10; // Prevent oscillation
                end
                // Check if we should decrease performance
                else if (cpu_utilization < CPU_UTIL_LOW && 
                        current_performance_level > PERF_MIN) begin
                    current_performance_level <= current_performance_level - 1;
                    dvfs_hysteresis_counter <= 8'd15; // Slower decrease
                end
            end
        end
        
        // Convert performance level to voltage and frequency
        case (current_performance_level)
            PERF_MAX:    {current_voltage_level, current_frequency_divider} = {4'd15, 8'd1};  // 1.8V, 100MHz
            PERF_HIGH:   {current_voltage_level, current_frequency_divider} = {4'd12, 8'd1};  // 1.5V, 100MHz
            PERF_MEDIUM: {current_voltage_level, current_frequency_divider} = {4'd12, 8'd2};  // 1.5V, 50MHz
            PERF_LOW:    {current_voltage_level, current_frequency_divider} = {4'd10, 8'd2};  // 1.3V, 50MHz
            PERF_MIN:    {current_voltage_level, current_frequency_divider} = {4'd10, 8'd4};  // 1.3V, 25MHz
            default:     {current_voltage_level, current_frequency_divider} = {4'd15, 8'd1};
        endcase
    end else begin
        // Manual DVFS override
        current_voltage_level <= manual_voltage_level;
        current_frequency_divider <= manual_frequency_divider;
    end
end

assign voltage_level = current_voltage_level;
assign frequency_divider = current_frequency_divider;
assign performance_level = current_performance_level;

// Generate transition request when levels change
reg [3:0] prev_voltage_level;
reg [7:0] prev_frequency_divider;

always_ff @(posedge clk) begin
    prev_voltage_level <= current_voltage_level;
    prev_frequency_divider <= current_frequency_divider;
end

assign dvfs_transition_req = (current_voltage_level != prev_voltage_level) ||
                            (current_frequency_divider != prev_frequency_divider);

endmodule
```

### 13.2.2 Performance Monitoring Unit

**Hardware Performance Counters:**
```verilog
module performance_monitoring_unit (
    input wire clk,
    input wire reset,
    
    // Pipeline monitoring inputs
    input wire instruction_issue,
    input wire instruction_complete,
    input wire pipeline_stall,
    input wire branch_mispredict,
    input wire cache_miss,
    input wire memory_access,
    
    // Performance counter outputs
    output wire [31:0] instructions_issued,
    output wire [31:0] instructions_completed,
    output wire [31:0] pipeline_stalls,
    output wire [31:0] branch_mispredicts,
    output wire [31:0] cache_misses,
    output wire [31:0] memory_accesses,
    
    // Derived metrics
    output wire [7:0] cpu_utilization,        // 0-255 (0-100%)
    output wire [7:0] ipc_metric,             // Instructions per cycle
    output wire [7:0] cache_miss_rate,        // 0-255 (0-100%)
    output wire [7:0] branch_prediction_rate  // 0-255 (0-100%)
);

// Performance counters (32-bit)
reg [31:0] counter_instructions_issued;
reg [31:0] counter_instructions_completed;
reg [31:0] counter_pipeline_stalls;
reg [31:0] counter_branch_mispredicts;
reg [31:0] counter_cache_misses;
reg [31:0] counter_memory_accesses;
reg [31:0] counter_total_cycles;

// Performance counter updates
always_ff @(posedge clk) begin
    if (reset) begin
        counter_instructions_issued <= 0;
        counter_instructions_completed <= 0;
        counter_pipeline_stalls <= 0;
        counter_branch_mispredicts <= 0;
        counter_cache_misses <= 0;
        counter_memory_accesses <= 0;
        counter_total_cycles <= 0;
    end else begin
        counter_total_cycles <= counter_total_cycles + 1;
        
        if (instruction_issue)
            counter_instructions_issued <= counter_instructions_issued + 1;
        if (instruction_complete)
            counter_instructions_completed <= counter_instructions_completed + 1;
        if (pipeline_stall)
            counter_pipeline_stalls <= counter_pipeline_stalls + 1;
        if (branch_mispredict)
            counter_branch_mispredicts <= counter_branch_mispredicts + 1;
        if (cache_miss)
            counter_cache_misses <= counter_cache_misses + 1;
        if (memory_access)
            counter_memory_accesses <= counter_memory_accesses + 1;
    end
end

// Metric calculation (updated every 1024 cycles)
reg [9:0] metric_update_counter;
reg [31:0] sample_window_instructions;
reg [31:0] sample_window_stalls;
reg [31:0] sample_window_cache_accesses;
reg [31:0] sample_window_cache_misses;
reg [31:0] sample_window_branches;
reg [31:0] sample_window_mispredicts;

always_ff @(posedge clk) begin
    if (reset) begin
        metric_update_counter <= 0;
    end else begin
        metric_update_counter <= metric_update_counter + 1;
        
        if (metric_update_counter == 10'h3FF) begin // Every 1024 cycles
            // Sample counters for metric calculation
            sample_window_instructions <= counter_instructions_completed;
            sample_window_stalls <= counter_pipeline_stalls;
            sample_window_cache_accesses <= counter_memory_accesses;
            sample_window_cache_misses <= counter_cache_misses;
            sample_window_branches <= counter_instructions_issued; // Approximation
            sample_window_mispredicts <= counter_branch_mispredicts;
            
            // Reset counters for next window
            counter_instructions_issued <= 0;
            counter_instructions_completed <= 0;
            counter_pipeline_stalls <= 0;
            counter_branch_mispredicts <= 0;
            counter_cache_misses <= 0;
            counter_memory_accesses <= 0;
        end
    end
end

// Calculate derived metrics
assign cpu_utilization = (sample_window_instructions > 1024) ? 8'd255 :
                         (sample_window_instructions * 256) >> 10; // Instructions/1024 * 256

assign ipc_metric = (sample_window_instructions > 1024) ? 8'd255 :
                   (sample_window_instructions * 256) >> 10; // IPC approximation

assign cache_miss_rate = (sample_window_cache_accesses == 0) ? 8'd0 :
                        (sample_window_cache_misses > sample_window_cache_accesses) ? 8'd255 :
                        (sample_window_cache_misses * 256) / sample_window_cache_accesses;

assign branch_prediction_rate = (sample_window_branches == 0) ? 8'd255 :
                               (sample_window_mispredicts > sample_window_branches) ? 8'd0 :
                               255 - ((sample_window_mispredicts * 256) / sample_window_branches);

// Counter outputs
assign instructions_issued = counter_instructions_issued;
assign instructions_completed = counter_instructions_completed;
assign pipeline_stalls = counter_pipeline_stalls;
assign branch_mispredicts = counter_branch_mispredicts;
assign cache_misses = counter_cache_misses;
assign memory_accesses = counter_memory_accesses;

endmodule
```

---

## 13.3 Sleep and Wake-up Management

### 13.3.1 Sleep Mode Implementation

**Sleep Mode Controller:**
```c
#define POWER_MGMT_BASE     0x40009000

typedef volatile struct {
    uint32_t power_mode;        /* 0x00: Power mode control */
    uint32_t wake_sources;      /* 0x04: Wake-up source enable */
    uint32_t wake_status;       /* 0x08: Wake-up status (RO) */
    uint32_t sleep_config;      /* 0x0C: Sleep configuration */
    uint32_t voltage_control;   /* 0x10: Voltage control */
    uint32_t clock_control;     /* 0x14: Clock gating control */
    uint32_t power_status;      /* 0x18: Power domain status */
    uint32_t thermal_control;   /* 0x1C: Thermal management */
} power_mgmt_t;

#define PWR_MGMT ((power_mgmt_t*)POWER_MGMT_BASE)

// Power modes
#define POWER_MODE_PERFORMANCE  0x00
#define POWER_MODE_BALANCED     0x01
#define POWER_MODE_POWER_SAVER  0x02
#define POWER_MODE_SLEEP        0x03
#define POWER_MODE_DEEP_SLEEP   0x04
#define POWER_MODE_HIBERNATE    0x05

// Wake-up sources
#define WAKE_SOURCE_RTC         (1 << 0)
#define WAKE_SOURCE_EXTERNAL    (1 << 1)
#define WAKE_SOURCE_UART        (1 << 2)
#define WAKE_SOURCE_GPIO        (1 << 3)
#define WAKE_SOURCE_TIMER       (1 << 4)
#define WAKE_SOURCE_WDT         (1 << 5)
#define WAKE_SOURCE_USB         (1 << 6)
#define WAKE_SOURCE_ETHERNET    (1 << 7)

// Sleep configuration
#define SLEEP_RETAIN_SRAM       (1 << 0)
#define SLEEP_RETAIN_CACHE      (1 << 1)
#define SLEEP_RETAIN_REGISTERS  (1 << 2)
#define SLEEP_FAST_WAKE         (1 << 3)

// Power management functions
void enter_sleep_mode(uint32_t wake_sources, uint32_t sleep_config) {
    // Save critical processor state
    save_processor_context();
    
    // Configure wake-up sources
    PWR_MGMT->wake_sources = wake_sources;
    
    // Configure sleep parameters
    PWR_MGMT->sleep_config = sleep_config;
    
    // Clear any pending wake events
    PWR_MGMT->wake_status = 0xFFFFFFFF;
    
    // Enter sleep mode
    PWR_MGMT->power_mode = POWER_MODE_SLEEP;
    
    // Execute wait-for-interrupt
    __asm volatile ("wfi");
    
    // Execution resumes here after wake-up
    restore_processor_context();
}

void enter_deep_sleep_mode(uint32_t wake_sources) {
    // Save all processor and system state
    save_system_context();
    
    // Configure minimal wake-up sources
    PWR_MGMT->wake_sources = wake_sources & 
        (WAKE_SOURCE_RTC | WAKE_SOURCE_EXTERNAL | WAKE_SOURCE_WDT);
    
    // Configure for minimal power consumption
    PWR_MGMT->sleep_config = SLEEP_RETAIN_REGISTERS; // Minimal retention
    
    // Clear wake events
    PWR_MGMT->wake_status = 0xFFFFFFFF;
    
    // Enter deep sleep
    PWR_MGMT->power_mode = POWER_MODE_DEEP_SLEEP;
    
    // Wait for interrupt
    __asm volatile ("wfi");
    
    // Full system restore required after deep sleep
    restore_system_context();
    reinitialize_system();
}

void enter_hibernate_mode(void) {
    // Save critical data to non-volatile storage
    save_hibernate_context();
    
    // Configure RTC-only wake-up
    PWR_MGMT->wake_sources = WAKE_SOURCE_RTC | WAKE_SOURCE_EXTERNAL;
    
    // Minimal retention configuration
    PWR_MGMT->sleep_config = 0; // No retention except always-on domain
    
    // Enter hibernate mode
    PWR_MGMT->power_mode = POWER_MODE_HIBERNATE;
    
    // This is essentially a controlled shutdown
    __asm volatile ("wfi");
    
    // System will reset on wake-up from hibernate
    // Execution will not return here
}

// Wake-up handling
void handle_wake_up(void) {
    uint32_t wake_status = PWR_MGMT->wake_status;
    
    // Determine wake-up source and handle accordingly
    if (wake_status & WAKE_SOURCE_RTC) {
        handle_rtc_wake_up();
        PWR_MGMT->wake_status = WAKE_SOURCE_RTC; // Clear status
    }
    
    if (wake_status & WAKE_SOURCE_EXTERNAL) {
        handle_external_wake_up();
        PWR_MGMT->wake_status = WAKE_SOURCE_EXTERNAL;
    }
    
    if (wake_status & WAKE_SOURCE_UART) {
        handle_uart_wake_up();
        PWR_MGMT->wake_status = WAKE_SOURCE_UART;
    }
    
    if (wake_status & WAKE_SOURCE_GPIO) {
        handle_gpio_wake_up();
        PWR_MGMT->wake_status = WAKE_SOURCE_GPIO;
    }
    
    // Return to normal operation
    PWR_MGMT->power_mode = POWER_MODE_PERFORMANCE;
}
```

### 13.3.2 Context Save and Restore

**System Context Management:**
```c
// Context save/restore structure
typedef struct {
    // CPU registers
    uint32_t general_registers[32];
    uint32_t pc;
    uint32_t mstatus;
    uint32_t mie;
    uint32_t mtvec;
    uint32_t mscratch;
    uint32_t mepc;
    uint32_t mcause;
    uint32_t mtval;
    
    // Cache configuration
    uint32_t icache_config;
    uint32_t dcache_config;
    
    // Memory controller state
    uint32_t memory_config[8];
    
    // Peripheral state
    uint32_t uart_config;
    uint32_t timer_config;
    uint32_t gpio_config;
    uint32_t interrupt_config;
    
    // Power management state
    uint32_t power_config;
    uint32_t clock_config;
    
    // Checksum for validation
    uint32_t context_checksum;
} system_context_t;

static system_context_t saved_context __attribute__((section(".retention_ram")));

void save_processor_context(void) {
    // Save general purpose registers
    __asm volatile (
        "sw x1,  %0"  : "=m"(saved_context.general_registers[1])
        : : "memory"
    );
    __asm volatile (
        "sw x2,  %0"  : "=m"(saved_context.general_registers[2])
        : : "memory"
    );
    // ... continue for all registers x1-x31 (x0 is always zero)
    
    // Save CSRs
    __asm volatile (
        "csrr t0, mstatus\n"
        "sw t0, %0"
        : "=m"(saved_context.mstatus)
        : : "t0", "memory"
    );
    
    __asm volatile (
        "csrr t0, mie\n"
        "sw t0, %0"
        : "=m"(saved_context.mie)
        : : "t0", "memory"
    );
    
    // Save other critical CSRs...
    
    // Save cache configuration
    saved_context.icache_config = CACHE_CTRL->icache_config;
    saved_context.dcache_config = CACHE_CTRL->dcache_config;
    
    // Save memory controller configuration
    for (int i = 0; i < 8; i++) {
        saved_context.memory_config[i] = MEM_CONFIG[i];
    }
    
    // Save peripheral configurations
    saved_context.uart_config = UART->control;
    saved_context.timer_config = TIMER->control;
    saved_context.gpio_config = GPIO->direction;
    saved_context.interrupt_config = IRQ_CTRL->irq_enable;
    
    // Calculate and store checksum
    saved_context.context_checksum = calculate_context_checksum(&saved_context);
}

void restore_processor_context(void) {
    // Validate context with checksum
    uint32_t calculated_checksum = calculate_context_checksum(&saved_context);
    if (calculated_checksum != saved_context.context_checksum) {
        // Context corrupted - perform cold boot
        system_cold_boot();
        return;
    }
    
    // Restore peripheral configurations first
    UART->control = saved_context.uart_config;
    TIMER->control = saved_context.timer_config;
    GPIO->direction = saved_context.gpio_config;
    IRQ_CTRL->irq_enable = saved_context.interrupt_config;
    
    // Restore memory controller
    for (int i = 0; i < 8; i++) {
        MEM_CONFIG[i] = saved_context.memory_config[i];
    }
    
    // Restore cache configuration
    CACHE_CTRL->icache_config = saved_context.icache_config;
    CACHE_CTRL->dcache_config = saved_context.dcache_config;
    
    // Restore CSRs
    __asm volatile (
        "lw t0, %0\n"
        "csrw mstatus, t0"
        : : "m"(saved_context.mstatus)
        : "t0", "memory"
    );
    
    __asm volatile (
        "lw t0, %0\n"
        "csrw mie, t0"
        : : "m"(saved_context.mie)
        : "t0", "memory"
    );
    
    // Restore general purpose registers (except x0)
    __asm volatile (
        "lw x1, %0" : : "m"(saved_context.general_registers[1]) : "memory"
    );
    __asm volatile (
        "lw x2, %0" : : "m"(saved_context.general_registers[2]) : "memory"
    );
    // ... continue for all registers
}

uint32_t calculate_context_checksum(const system_context_t *context) {
    uint32_t checksum = 0;
    const uint32_t *data = (const uint32_t *)context;
    size_t words = (sizeof(system_context_t) - sizeof(uint32_t)) / sizeof(uint32_t);
    
    for (size_t i = 0; i < words; i++) {
        checksum ^= data[i];
        checksum = (checksum << 1) | (checksum >> 31); // Rotate left
    }
    
    return checksum;
}
```

---

## 13.4 Thermal Management

### 13.4.1 Temperature Monitoring

**Thermal Sensor Interface:**
```verilog
module thermal_sensor_interface (
    input wire clk,
    input wire reset,
    
    // Analog inputs from thermal sensors
    input wire [7:0] core_temp_analog,
    input wire [7:0] ambient_temp_analog,
    input wire [7:0] junction_temp_analog,
    
    // Digital temperature outputs (8-bit, 0-255 representing 0-125°C)
    output wire [7:0] core_temperature,
    output wire [7:0] ambient_temperature,
    output wire [7:0] junction_temperature,
    
    // Thermal alerts
    output wire thermal_warning,
    output wire thermal_critical,
    output wire thermal_shutdown_req,
    
    // Calibration interface
    input wire [7:0] temp_offset,
    input wire [7:0] temp_gain
);

// Temperature thresholds (in digital units)
parameter TEMP_WARNING_THRESHOLD = 8'd200;    // ~78°C
parameter TEMP_CRITICAL_THRESHOLD = 8'd230;   // ~90°C
parameter TEMP_SHUTDOWN_THRESHOLD = 8'd250;   // ~98°C

// ADC for temperature conversion
reg [7:0] core_temp_digital;
reg [7:0] ambient_temp_digital;
reg [7:0] junction_temp_digital;

// Temperature conversion with calibration
always_ff @(posedge clk) begin
    if (reset) begin
        core_temp_digital <= 8'd64;      // Default to ~25°C
        ambient_temp_digital <= 8'd64;
        junction_temp_digital <= 8'd64;
    end else begin
        // Apply gain and offset calibration
        core_temp_digital <= ((core_temp_analog * temp_gain) >> 8) + temp_offset;
        ambient_temp_digital <= ((ambient_temp_analog * temp_gain) >> 8) + temp_offset;
        junction_temp_digital <= ((junction_temp_analog * temp_gain) >> 8) + temp_offset;
    end
end

// Thermal alert generation
assign thermal_warning = (core_temperature > TEMP_WARNING_THRESHOLD) ||
                        (junction_temperature > TEMP_WARNING_THRESHOLD);

assign thermal_critical = (core_temperature > TEMP_CRITICAL_THRESHOLD) ||
                         (junction_temperature > TEMP_CRITICAL_THRESHOLD);

assign thermal_shutdown_req = (core_temperature > TEMP_SHUTDOWN_THRESHOLD) ||
                             (junction_temperature > TEMP_SHUTDOWN_THRESHOLD);

assign core_temperature = core_temp_digital;
assign ambient_temperature = ambient_temp_digital;
assign junction_temperature = junction_temp_digital;

endmodule
```

### 13.4.2 Thermal Protection

**Thermal Management Software:**
```c
#define THERMAL_BASE        0x4000A000

typedef volatile struct {
    uint32_t core_temperature;      /* 0x00: Core temperature (RO) */
    uint32_t ambient_temperature;   /* 0x04: Ambient temperature (RO) */
    uint32_t junction_temperature;  /* 0x08: Junction temperature (RO) */
    uint32_t temp_thresholds;       /* 0x0C: Temperature thresholds */
    uint32_t thermal_status;        /* 0x10: Thermal status (RO) */
    uint32_t thermal_control;       /* 0x14: Thermal control */
    uint32_t fan_control;          /* 0x18: Fan control (if available) */
    uint32_t thermal_history;      /* 0x1C: Temperature history */
} thermal_mgmt_t;

#define THERMAL ((thermal_mgmt_t*)THERMAL_BASE)

// Thermal status bits
#define THERMAL_STATUS_WARNING      (1 << 0)
#define THERMAL_STATUS_CRITICAL     (1 << 1)
#define THERMAL_STATUS_SHUTDOWN     (1 << 2)
#define THERMAL_STATUS_FAN_FAIL     (1 << 3)

// Thermal control bits
#define THERMAL_CTRL_ENABLE         (1 << 0)
#define THERMAL_CTRL_AUTO_FAN       (1 << 1)
#define THERMAL_CTRL_THROTTLE_EN    (1 << 2)
#define THERMAL_CTRL_SHUTDOWN_EN    (1 << 3)

// Thermal management functions
void init_thermal_management(void) {
    // Set temperature thresholds (warning: 78°C, critical: 90°C, shutdown: 98°C)
    THERMAL->temp_thresholds = (200 << 0) | (230 << 8) | (250 << 16);
    
    // Enable thermal monitoring with automatic fan control and throttling
    THERMAL->thermal_control = THERMAL_CTRL_ENABLE |
                              THERMAL_CTRL_AUTO_FAN |
                              THERMAL_CTRL_THROTTLE_EN |
                              THERMAL_CTRL_SHUTDOWN_EN;
    
    // Initialize fan to minimum speed
    THERMAL->fan_control = 0x40; // 25% fan speed
}

void thermal_management_task(void) {
    uint32_t core_temp = THERMAL->core_temperature & 0xFF;
    uint32_t status = THERMAL->thermal_status;
    
    // Check for thermal events
    if (status & THERMAL_STATUS_SHUTDOWN) {
        // Emergency thermal shutdown
        emergency_thermal_shutdown();
    } else if (status & THERMAL_STATUS_CRITICAL) {
        // Critical temperature - aggressive throttling
        handle_critical_thermal_event(core_temp);
    } else if (status & THERMAL_STATUS_WARNING) {
        // Warning temperature - moderate throttling
        handle_thermal_warning(core_temp);
    } else {
        // Normal temperature - no throttling needed
        handle_normal_thermal_state(core_temp);
    }
    
    // Update fan speed based on temperature
    update_fan_speed(core_temp);
    
    // Log temperature history
    log_temperature_history(core_temp);
}

void handle_critical_thermal_event(uint32_t temperature) {
    // Reduce CPU frequency to minimum
    PWR_MGMT->power_mode = POWER_MODE_POWER_SAVER;
    
    // Reduce voltage to minimum safe level
    PWR_MGMT->voltage_control = 0x8888; // Reduced voltage
    
    // Disable high-power peripherals
    disable_nonessential_peripherals();
    
    // Set maximum fan speed
    THERMAL->fan_control = 0xFF;
    
    // Log critical thermal event
    log_system_event(EVENT_THERMAL_CRITICAL, temperature);
}

void handle_thermal_warning(uint32_t temperature) {
    // Moderate performance reduction
    if (PWR_MGMT->power_mode == POWER_MODE_PERFORMANCE) {
        PWR_MGMT->power_mode = POWER_MODE_BALANCED;
    }
    
    // Increase fan speed proportionally
    uint32_t fan_speed = 0x80 + ((temperature - 200) * 2); // Ramp from 50% to 100%
    THERMAL->fan_control = (fan_speed > 0xFF) ? 0xFF : fan_speed;
    
    // Warn applications about thermal condition
    send_thermal_warning_signal();
}

void handle_normal_thermal_state(uint32_t temperature) {
    // Allow normal performance
    if (PWR_MGMT->power_mode == POWER_MODE_POWER_SAVER) {
        PWR_MGMT->power_mode = POWER_MODE_BALANCED;
    }
    
    // Normal fan speed control
    if (temperature > 150) { // > 58°C
        uint32_t fan_speed = 0x40 + (temperature - 150); // Ramp from 25%
        THERMAL->fan_control = (fan_speed > 0x80) ? 0x80 : fan_speed;
    } else {
        THERMAL->fan_control = 0x40; // Minimum fan speed
    }
}

void emergency_thermal_shutdown(void) {
    // Disable all unnecessary systems immediately
    disable_all_peripherals();
    
    // Save critical data quickly
    emergency_context_save();
    
    // Log shutdown event
    log_system_event(EVENT_THERMAL_SHUTDOWN, THERMAL->core_temperature);
    
    // Enter emergency power-down mode
    PWR_MGMT->power_mode = POWER_MODE_HIBERNATE;
    
    // Wait for temperature to drop before restart
    __asm volatile ("wfi");
}

void update_fan_speed(uint32_t temperature) {
    // Automatic fan speed control based on temperature curve
    uint32_t fan_speed;
    
    if (temperature < 120) {        // < 47°C
        fan_speed = 0x20;          // 12.5% minimum
    } else if (temperature < 160) { // < 62°C
        fan_speed = 0x40;          // 25%
    } else if (temperature < 200) { // < 78°C
        fan_speed = 0x60;          // 37.5%
    } else if (temperature < 230) { // < 90°C
        fan_speed = 0x80 + ((temperature - 200) * 2); // Ramp 50-87.5%
    } else {
        fan_speed = 0xFF;          // 100% maximum
    }
    
    THERMAL->fan_control = fan_speed;
}
```

---

## 13.5 Battery and Power Supply Management

### 13.5.1 Battery Monitoring

**Battery Management Interface:**
```c
#define BATTERY_MGMT_BASE   0x4000B000

typedef volatile struct {
    uint32_t battery_voltage;       /* 0x00: Battery voltage (RO) */
    uint32_t battery_current;       /* 0x04: Battery current (RO) */
    uint32_t battery_capacity;      /* 0x08: Remaining capacity (RO) */
    uint32_t battery_status;        /* 0x0C: Battery status (RO) */
    uint32_t power_source;          /* 0x10: Power source status (RO) */
    uint32_t charging_control;      /* 0x14: Charging control */
    uint32_t power_budget;          /* 0x18: Power budget control */
    uint32_t battery_config;        /* 0x1C: Battery configuration */
} battery_mgmt_t;

#define BATTERY ((battery_mgmt_t*)BATTERY_MGMT_BASE)

// Battery status bits
#define BATTERY_STATUS_PRESENT      (1 << 0)
#define BATTERY_STATUS_CHARGING     (1 << 1)
#define BATTERY_STATUS_DISCHARGING  (1 << 2)
#define BATTERY_STATUS_FULL         (1 << 3)
#define BATTERY_STATUS_LOW          (1 << 4)
#define BATTERY_STATUS_CRITICAL     (1 << 5)
#define BATTERY_STATUS_FAULT        (1 << 6)
#define BATTERY_STATUS_OVERHEAT     (1 << 7)

// Power source bits
#define POWER_SOURCE_AC_PRESENT     (1 << 0)
#define POWER_SOURCE_USB_PRESENT    (1 << 1)
#define POWER_SOURCE_BATTERY_MAIN   (1 << 2)
#define POWER_SOURCE_BACKUP_POWER   (1 << 3)

// Battery management functions
void init_battery_management(void) {
    // Configure battery parameters
    BATTERY->battery_config = 
        (3700 << 0) |  // 3.7V nominal voltage (mV)
        (2000 << 16);  // 2000mAh capacity
    
    // Set power budget for different modes
    BATTERY->power_budget = 
        (500 << 0) |   // Performance mode: 500mA max
        (250 << 8) |   // Balanced mode: 250mA max
        (100 << 16) |  // Power saver mode: 100mA max
        (10 << 24);    // Sleep mode: 10mA max
    
    // Enable charging if AC power present
    if (BATTERY->power_source & POWER_SOURCE_AC_PRESENT) {
        BATTERY->charging_control = 1; // Enable charging
    }
}

void battery_management_task(void) {
    uint32_t battery_status = BATTERY->battery_status;
    uint32_t capacity = BATTERY->battery_capacity & 0xFF; // 0-100%
    uint32_t voltage = BATTERY->battery_voltage; // mV
    
    // Handle low battery conditions
    if (battery_status & BATTERY_STATUS_CRITICAL) {
        handle_critical_battery();
    } else if (battery_status & BATTERY_STATUS_LOW) {
        handle_low_battery();
    }
    
    // Adjust power management based on battery level
    if (!(BATTERY->power_source & POWER_SOURCE_AC_PRESENT)) {
        // Running on battery - adjust performance based on capacity
        adjust_performance_for_battery_level(capacity);
    }
    
    // Handle charging
    if (BATTERY->power_source & POWER_SOURCE_AC_PRESENT) {
        manage_battery_charging();
    }
    
    // Update power budget
    update_power_budget();
}

void handle_critical_battery(void) {
    // Battery critically low - prepare for shutdown
    
    // Save all critical data immediately
    save_critical_system_state();
    
    // Switch to minimum power mode
    PWR_MGMT->power_mode = POWER_MODE_DEEP_SLEEP;
    
    // Disable all non-essential peripherals
    disable_nonessential_peripherals();
    
    // Set wake-up only on AC power or external event
    PWR_MGMT->wake_sources = WAKE_SOURCE_EXTERNAL;
    
    // Log critical battery event
    log_system_event(EVENT_BATTERY_CRITICAL, BATTERY->battery_capacity);
    
    // Notify user/applications
    signal_critical_battery();
}

void handle_low_battery(void) {
    // Battery low - reduce power consumption
    
    // Force power saver mode
    if (PWR_MGMT->power_mode < POWER_MODE_POWER_SAVER) {
        PWR_MGMT->power_mode = POWER_MODE_POWER_SAVER;
    }
    
    // Reduce peripheral activity
    reduce_peripheral_activity();
    
    // Lower screen brightness if applicable
    reduce_display_brightness();
    
    // Notify applications of low battery
    signal_low_battery();
}

void adjust_performance_for_battery_level(uint32_t capacity) {
    if (capacity > 50) {
        // Good battery level - allow performance mode
        // No restrictions
    } else if (capacity > 20) {
        // Medium battery - prefer balanced mode
        if (PWR_MGMT->power_mode == POWER_MODE_PERFORMANCE) {
            PWR_MGMT->power_mode = POWER_MODE_BALANCED;
        }
    } else if (capacity > 10) {
        // Low battery - force power saver
        PWR_MGMT->power_mode = POWER_MODE_POWER_SAVER;
    } else {
        // Very low battery - minimum power
        PWR_MGMT->power_mode = POWER_MODE_DEEP_SLEEP;
    }
}

void manage_battery_charging(void) {
    uint32_t status = BATTERY->battery_status;
    uint32_t voltage = BATTERY->battery_voltage;
    
    if (status & BATTERY_STATUS_FULL) {
        // Battery full - stop charging
        BATTERY->charging_control = 0;
    } else if (status & BATTERY_STATUS_OVERHEAT) {
        // Battery overheating - stop charging
        BATTERY->charging_control = 0;
        log_system_event(EVENT_BATTERY_OVERHEAT, voltage);
    } else if (voltage < 3300) {
        // Low voltage - enable trickle charging
        BATTERY->charging_control = 1 | (1 << 8); // Enable + trickle mode
    } else {
        // Normal charging
        BATTERY->charging_control = 1; // Enable normal charging
    }
}

uint32_t estimate_battery_life(void) {
    uint32_t capacity = BATTERY->battery_capacity & 0xFF;
    uint32_t current = BATTERY->battery_current & 0xFFFF; // mA
    uint32_t power_mode = PWR_MGMT->power_mode;
    
    // Estimate remaining time based on current consumption
    uint32_t total_capacity = (BATTERY->battery_config >> 16) & 0xFFFF; // mAh
    uint32_t remaining_capacity = (total_capacity * capacity) / 100;
    
    if (current == 0) {
        return 0xFFFFFFFF; // Essentially unlimited (AC powered)
    }
    
    // Return estimated minutes of battery life
    return (remaining_capacity * 60) / current;
}
```

---

*This chapter provided comprehensive coverage of MCU-32X power management capabilities, enabling efficient operation across the full range from high-performance desktop computing to ultra-low-power embedded applications.*