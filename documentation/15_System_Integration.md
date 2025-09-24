# Chapter 15: System Integration
## MCU-32X Technical Reference Manual

---

## 15.1 System Architecture Overview

The MCU-32X is designed for seamless integration into complex system environments, supporting both embedded applications and desktop computing requirements typical of late-1990s high-performance systems.

### 15.1.1 System Integration Capabilities

**Integration Features:**
- **Multi-Master Bus Support**: Compatible with existing system buses
- **Cache Coherency Protocol**: Maintains data consistency in multi-processor systems  
- **DMA Integration**: Direct memory access with processor coordination
- **External Memory Interface**: Support for various memory types and configurations
- **Peripheral Integration**: Standardized interfaces for system components

**Target System Environments:**
- **Embedded Systems**: Real-time control, automotive, industrial automation
- **Desktop Computers**: Co-processor, accelerator, or main processor applications
- **Communications Equipment**: Network processors, telecommunications infrastructure
- **Multimedia Systems**: Digital signal processing, graphics acceleration

### 15.1.2 System Bus Architecture

**External Bus Specifications:**
```
Bus Width: 32-bit address, 32-bit data
Clock Frequency: Up to 100 MHz (synchronous to processor)
Transfer Modes: Single, burst, split-transaction
Address Range: 4GB total addressable space
Bus Masters: Up to 8 masters supported
Arbitration: Fixed priority or round-robin
```

**Bus Signal Groups:**
- **Address/Data**: Multiplexed or separate address/data buses
- **Control**: Read/write, burst control, transfer acknowledgment
- **Arbitration**: Bus request, grant, busy signals  
- **Interrupt**: Interrupt request inputs, acknowledgment outputs
- **Clock/Reset**: System clock, reset, and power management

---

## 15.2 External Memory Interface

### 15.2.1 Memory Controller Architecture

**Supported Memory Types:**
- **SDRAM**: Synchronous DRAM with various speeds and configurations
- **DDR SDRAM**: Double data rate SDRAM for higher bandwidth
- **Flash Memory**: NOR and NAND flash for non-volatile storage
- **SRAM**: High-speed static RAM for cache and buffer memory
- **ROM/EPROM**: Non-volatile program and data storage

**Memory Interface Specifications:**
```verilog
module memory_interface (
    // Processor interface
    input wire [31:0] cpu_addr,
    input wire [31:0] cpu_data_out,
    output wire [31:0] cpu_data_in,
    input wire cpu_read,
    input wire cpu_write,
    input wire [3:0] cpu_byte_enable,
    output wire cpu_ready,
    
    // External memory interface
    output wire [25:0] mem_addr,       // Up to 64MB per bank
    inout wire [31:0] mem_data,
    output wire mem_read_n,
    output wire mem_write_n,
    output wire [3:0] mem_byte_enable_n,
    input wire mem_ready,
    output wire [1:0] mem_bank_select,
    
    // SDRAM specific signals
    output wire sdram_cas_n,
    output wire sdram_ras_n,
    output wire sdram_we_n,
    output wire [1:0] sdram_ba,
    output wire sdram_cke,
    output wire sdram_cs_n,
    input wire sdram_clk
);

// Memory timing parameters
parameter SDRAM_CAS_LATENCY = 2;
parameter SDRAM_RAS_PRECHARGE = 3;
parameter SDRAM_REFRESH_PERIOD = 7800;  // 7.8us in clock cycles

// Memory controller state machine
typedef enum {
    IDLE,
    ACTIVE,
    READ_CMD,
    READ_DATA,
    WRITE_CMD,
    WRITE_DATA,
    PRECHARGE,
    REFRESH
} memory_state_t;

memory_state_t current_state, next_state;

// SDRAM controller implementation
always_ff @(posedge sdram_clk) begin
    if (reset) begin
        current_state <= IDLE;
        refresh_counter <= 0;
    end else begin
        current_state <= next_state;
        refresh_counter <= refresh_counter + 1;
    end
end

// SDRAM command generation
always_comb begin
    case (current_state)
        IDLE: begin
            if (refresh_counter >= SDRAM_REFRESH_PERIOD) begin
                next_state = REFRESH;
            end else if (cpu_read || cpu_write) begin
                next_state = ACTIVE;
            end else begin
                next_state = IDLE;
            end
        end
        
        ACTIVE: begin
            // Activate row
            sdram_ras_n = 1'b0;
            sdram_cas_n = 1'b1;
            sdram_we_n = 1'b1;
            sdram_ba = cpu_addr[25:24];
            mem_addr = cpu_addr[23:11];  // Row address
            
            if (cpu_read) begin
                next_state = READ_CMD;
            end else begin
                next_state = WRITE_CMD;
            end
        end
        
        READ_CMD: begin
            // Read command
            sdram_ras_n = 1'b1;
            sdram_cas_n = 1'b0;
            sdram_we_n = 1'b1;
            mem_addr[10:0] = cpu_addr[10:0];  // Column address
            
            next_state = READ_DATA;
        end
        
        READ_DATA: begin
            // Wait for CAS latency
            if (cas_delay_counter >= SDRAM_CAS_LATENCY) begin
                cpu_data_in = mem_data;
                cpu_ready = 1'b1;
                next_state = PRECHARGE;
            end
        end
        
        // Additional states...
    endcase
end

endmodule
```

### 15.2.2 Memory Map and Configuration

**System Memory Map:**
```c
// MCU-32X System Memory Map
#define INTERNAL_SRAM_BASE      0x00000000  // 128KB internal SRAM
#define INTERNAL_SRAM_SIZE      0x00020000

#define EXTERNAL_SDRAM_BASE     0x10000000  // External SDRAM
#define EXTERNAL_SDRAM_SIZE     0x10000000  // Up to 256MB

#define FLASH_MEMORY_BASE       0x20000000  // Flash memory
#define FLASH_MEMORY_SIZE       0x02000000  // Up to 32MB

#define PERIPHERAL_BASE         0x40000000  // Memory-mapped peripherals
#define PERIPHERAL_SIZE         0x10000000

#define EXTERNAL_BUS_BASE       0x80000000  // External bus interface
#define EXTERNAL_BUS_SIZE       0x80000000

// Memory configuration registers
typedef volatile struct {
    uint32_t sdram_config;      /* SDRAM configuration */
    uint32_t sdram_timing;      /* SDRAM timing parameters */
    uint32_t flash_config;      /* Flash memory configuration */
    uint32_t bus_config;        /* External bus configuration */
    uint32_t cache_config;      /* Cache configuration */
    uint32_t memory_test;       /* Memory test control */
} memory_config_t;

#define MEM_CONFIG ((memory_config_t*)0x40006000)

// SDRAM configuration
void configure_sdram(void) {
    // Configure SDRAM parameters
    MEM_CONFIG->sdram_config = 
        (2 << 0) |              // CAS latency = 2
        (3 << 4) |              // RAS precharge = 3 cycles  
        (1 << 8) |              // Burst length = 2
        (1 << 12);              // Auto refresh enable
    
    // Set SDRAM timing
    MEM_CONFIG->sdram_timing = 
        (60 << 0) |             // tRP = 60ns (6 cycles @ 100MHz)
        (60 << 8) |             // tRCD = 60ns  
        (780 << 16);            // tREF = 7.8us refresh period
    
    // Enable SDRAM controller
    MEM_CONFIG->sdram_config |= (1 << 31);
}
```

---

## 15.3 Direct Memory Access (DMA) Integration

### 15.3.1 DMA Controller Architecture

**DMA Capabilities:**
- **8 Independent Channels**: Simultaneous transfers on multiple channels
- **Memory-to-Memory**: High-speed data copying and initialization
- **Memory-to-Peripheral**: Efficient I/O data transfers
- **Peripheral-to-Memory**: Input data buffering and processing
- **Scatter-Gather**: Complex transfer patterns with descriptor lists

**DMA Channel Configuration:**
```verilog
module dma_controller (
    input wire clk,
    input wire reset,
    
    // Processor interface
    input wire [31:0] proc_addr,
    input wire [31:0] proc_data_in,
    output wire [31:0] proc_data_out,
    input wire proc_read,
    input wire proc_write,
    output wire proc_ready,
    
    // Memory interface
    output wire [31:0] mem_addr,
    output wire [31:0] mem_data_out,
    input wire [31:0] mem_data_in,
    output wire mem_read,
    output wire mem_write,
    input wire mem_ready,
    
    // Peripheral interfaces
    output wire [7:0] peripheral_dma_req,
    input wire [7:0] peripheral_dma_ack,
    
    // Interrupt output
    output wire [7:0] dma_interrupt
);

// DMA channel registers
typedef struct {
    uint32_t source_addr;       // Source address
    uint32_t dest_addr;         // Destination address  
    uint32_t transfer_count;    // Number of transfers
    uint32_t control;           // Control and status
    uint32_t next_descriptor;   // Next descriptor (scatter-gather)
} dma_channel_t;

// DMA control bits
parameter DMA_ENABLE        = 0;
parameter DMA_DIRECTION     = 1;  // 0=mem->peripheral, 1=peripheral->mem
parameter DMA_TRANSFER_SIZE = 2;  // 00=byte, 01=halfword, 10=word
parameter DMA_BURST_MODE    = 4;  // Enable burst transfers
parameter DMA_INTERRUPT_EN  = 8;  // Enable completion interrupt
parameter DMA_SCATTER_GATHER = 16; // Enable scatter-gather mode

// DMA channel state machine
typedef enum {
    DMA_IDLE,
    DMA_SETUP,
    DMA_TRANSFER,
    DMA_COMPLETE
} dma_state_t;

dma_state_t dma_state[8];
dma_channel_t dma_channels[8];

// DMA transfer logic for each channel
always_ff @(posedge clk) begin
    for (int channel = 0; channel < 8; channel++) begin
        case (dma_state[channel])
            DMA_IDLE: begin
                if (dma_channels[channel].control & (1 << DMA_ENABLE)) begin
                    dma_state[channel] <= DMA_SETUP;
                end
            end
            
            DMA_SETUP: begin
                // Setup transfer addresses and count
                current_src_addr[channel] <= dma_channels[channel].source_addr;
                current_dest_addr[channel] <= dma_channels[channel].dest_addr;
                remaining_count[channel] <= dma_channels[channel].transfer_count;
                dma_state[channel] <= DMA_TRANSFER;
            end
            
            DMA_TRANSFER: begin
                if (mem_ready && remaining_count[channel] > 0) begin
                    // Perform transfer based on direction
                    if (dma_channels[channel].control & (1 << DMA_DIRECTION)) begin
                        // Peripheral to memory
                        mem_addr <= current_dest_addr[channel];
                        mem_data_out <= peripheral_data[channel];
                        mem_write <= 1'b1;
                    end else begin
                        // Memory to peripheral  
                        mem_addr <= current_src_addr[channel];
                        mem_read <= 1'b1;
                        peripheral_data[channel] <= mem_data_in;
                    end
                    
                    // Update addresses and count
                    current_src_addr[channel] <= current_src_addr[channel] + 4;
                    current_dest_addr[channel] <= current_dest_addr[channel] + 4;
                    remaining_count[channel] <= remaining_count[channel] - 1;
                    
                    if (remaining_count[channel] == 1) begin
                        dma_state[channel] <= DMA_COMPLETE;
                    end
                end
            end
            
            DMA_COMPLETE: begin
                // Clear enable bit and generate interrupt if enabled
                dma_channels[channel].control &= ~(1 << DMA_ENABLE);
                if (dma_channels[channel].control & (1 << DMA_INTERRUPT_EN)) begin
                    dma_interrupt[channel] <= 1'b1;
                end
                dma_state[channel] <= DMA_IDLE;
            end
        endcase
    end
end

endmodule
```

### 15.3.2 DMA Programming Interface

**DMA Channel Configuration:**
```c
#define DMA_BASE            0x40007000

typedef volatile struct {
    uint32_t source_addr;       /* 0x00: Source address */
    uint32_t dest_addr;         /* 0x04: Destination address */
    uint32_t transfer_count;    /* 0x08: Transfer count */
    uint32_t control;           /* 0x0C: Control register */
    uint32_t status;            /* 0x10: Status register */
    uint32_t next_descriptor;   /* 0x14: Next descriptor pointer */
    uint32_t reserved[2];       /* 0x18-0x1C: Reserved */
} dma_channel_regs_t;

#define DMA_CHANNEL(n) ((dma_channel_regs_t*)(DMA_BASE + (n) * 0x20))

// DMA control register bits
#define DMA_CTRL_ENABLE         (1 << 0)
#define DMA_CTRL_DIR_P2M        (1 << 1)    // Peripheral to Memory
#define DMA_CTRL_SIZE_BYTE      (0 << 2)
#define DMA_CTRL_SIZE_HALFWORD  (1 << 2)
#define DMA_CTRL_SIZE_WORD      (2 << 2)
#define DMA_CTRL_BURST_ENABLE   (1 << 4)
#define DMA_CTRL_INT_ENABLE     (1 << 8)
#define DMA_CTRL_SCATTER_GATHER (1 << 16)

// DMA status register bits
#define DMA_STATUS_ACTIVE       (1 << 0)
#define DMA_STATUS_COMPLETE     (1 << 1)
#define DMA_STATUS_ERROR        (1 << 2)

// DMA channel initialization
void dma_init_channel(int channel, uint32_t src, uint32_t dst, 
                     uint32_t count, uint32_t control) {
    dma_channel_regs_t *dma = DMA_CHANNEL(channel);
    
    // Disable channel during configuration
    dma->control = 0;
    
    // Configure transfer parameters
    dma->source_addr = src;
    dma->dest_addr = dst;
    dma->transfer_count = count;
    
    // Clear status
    dma->status = DMA_STATUS_COMPLETE | DMA_STATUS_ERROR;
    
    // Set control and enable
    dma->control = control | DMA_CTRL_ENABLE;
}

// Memory-to-memory copy using DMA
void dma_memcpy(void *dest, const void *src, size_t size) {
    int channel = allocate_dma_channel();
    if (channel < 0) {
        // Fall back to CPU copy
        memcpy(dest, src, size);
        return;
    }
    
    uint32_t word_count = (size + 3) / 4;  // Round up to word boundary
    
    dma_init_channel(channel, 
                    (uint32_t)src, 
                    (uint32_t)dest,
                    word_count,
                    DMA_CTRL_SIZE_WORD | DMA_CTRL_BURST_ENABLE);
    
    // Wait for completion
    dma_channel_regs_t *dma = DMA_CHANNEL(channel);
    while (!(dma->status & DMA_STATUS_COMPLETE)) {
        // Could yield to other tasks here
    }
    
    free_dma_channel(channel);
}

// UART receive using DMA
void uart_dma_receive(void *buffer, size_t size) {
    int channel = allocate_dma_channel();
    
    dma_init_channel(channel,
                    UART_BASE,              // UART data register
                    (uint32_t)buffer,
                    size,
                    DMA_CTRL_DIR_P2M |      // Peripheral to memory
                    DMA_CTRL_SIZE_BYTE |    // Byte transfers
                    DMA_CTRL_INT_ENABLE);   // Interrupt on completion
    
    // DMA will transfer data as UART receives it
    // Interrupt handler will process completion
}
```

---

## 15.4 Bus Arbitration and Multi-Master Support

### 15.4.1 Bus Arbitration Protocol

**Arbitration Schemes:**
- **Fixed Priority**: Processors assigned static priorities
- **Round-Robin**: Fair sharing among masters
- **Weighted Round-Robin**: Priority-based fair sharing
- **Longest Time Since Grant**: Prevents starvation

**Bus Arbitration Implementation:**
```verilog
module bus_arbiter (
    input wire clk,
    input wire reset,
    
    // Bus master requests
    input wire [7:0] bus_request,
    output wire [7:0] bus_grant,
    input wire [7:0] bus_busy,
    
    // Arbitration mode control
    input wire [1:0] arbitration_mode,
    input wire [7:0] master_priority,
    
    // Bus control signals
    output wire bus_available,
    output wire [2:0] current_master
);

// Arbitration modes
parameter ARB_FIXED_PRIORITY = 0;
parameter ARB_ROUND_ROBIN = 1;
parameter ARB_WEIGHTED_RR = 2;
parameter ARB_LONGEST_WAIT = 3;

reg [2:0] last_granted_master;
reg [7:0] master_wait_time[8];
wire [7:0] arbitration_result;

// Priority encoder for fixed priority arbitration
priority_encoder pri_enc (
    .request(bus_request & master_priority),
    .grant(arbitration_result)
);

// Round-robin arbitration
always_ff @(posedge clk) begin
    if (reset) begin
        last_granted_master <= 0;
        for (int i = 0; i < 8; i++) begin
            master_wait_time[i] <= 0;
        end
    end else begin
        case (arbitration_mode)
            ARB_FIXED_PRIORITY: begin
                bus_grant <= arbitration_result;
                current_master <= find_first_set(arbitration_result);
            end
            
            ARB_ROUND_ROBIN: begin
                // Start from next master after last granted
                wire [7:0] rotated_request;
                rotate_left #(.WIDTH(8)) rot (
                    .data_in(bus_request),
                    .shift_amount(last_granted_master + 1),
                    .data_out(rotated_request)
                );
                
                wire [7:0] rotated_grant;
                priority_encoder pri_enc_rr (
                    .request(rotated_request),
                    .grant(rotated_grant)
                );
                
                // Rotate back to original position
                rotate_right #(.WIDTH(8)) rot_back (
                    .data_in(rotated_grant),
                    .shift_amount(last_granted_master + 1),
                    .data_out(bus_grant)
                );
                
                if (|bus_grant) begin
                    last_granted_master <= find_first_set(bus_grant);
                end
            end
            
            ARB_LONGEST_WAIT: begin
                // Find master with longest wait time
                reg [2:0] longest_wait_master;
                reg [7:0] max_wait_time;
                
                max_wait_time = 0;
                longest_wait_master = 0;
                
                for (int i = 0; i < 8; i++) begin
                    if (bus_request[i] && master_wait_time[i] > max_wait_time) begin
                        max_wait_time = master_wait_time[i];
                        longest_wait_master = i;
                    end
                end
                
                bus_grant <= (1 << longest_wait_master);
                current_master <= longest_wait_master;
            end
        endcase
        
        // Update wait times
        for (int i = 0; i < 8; i++) begin
            if (bus_request[i] && !bus_grant[i]) begin
                master_wait_time[i] <= master_wait_time[i] + 1;
            end else if (bus_grant[i]) begin
                master_wait_time[i] <= 0;
            end
        end
    end
end

assign bus_available = !(|bus_busy);

endmodule
```

### 15.4.2 Cache Coherency Protocol

**Coherency Implementation:**
```verilog
module cache_coherency_controller (
    input wire clk,
    input wire reset,
    
    // Processor cache interfaces
    input wire [7:0] cache_read_request,
    input wire [7:0] cache_write_request,
    input wire [31:0] cache_address[8],
    input wire [31:0] cache_write_data[8],
    output wire [31:0] cache_read_data[8],
    
    // Snoop bus interface
    output wire [31:0] snoop_address,
    output wire snoop_read,
    output wire snoop_write,
    input wire [7:0] snoop_hit,
    input wire [7:0] snoop_dirty,
    
    // Memory interface
    output wire [31:0] memory_address,
    output wire [31:0] memory_write_data,
    input wire [31:0] memory_read_data,
    output wire memory_read,
    output wire memory_write,
    input wire memory_ready
);

// Cache line states (MESI protocol)
typedef enum {
    INVALID = 0,
    SHARED = 1,
    EXCLUSIVE = 2,
    MODIFIED = 3
} cache_state_t;

// Coherency transaction types
typedef enum {
    READ_REQUEST,
    WRITE_REQUEST,
    INVALIDATE,
    FLUSH
} coherency_transaction_t;

// Snoop response generation
always_comb begin
    for (int cache = 0; cache < 8; cache++) begin
        if (cache_read_request[cache]) begin
            // Check if other caches have this line
            wire other_cache_hit = |(snoop_hit & ~(1 << cache));
            wire other_cache_dirty = |(snoop_dirty & ~(1 << cache));
            
            if (other_cache_dirty) begin
                // Another cache has modified data - flush and share
                snoop_address = cache_address[cache];
                snoop_write = 1'b1;
                // Set state to SHARED after flush
            end else if (other_cache_hit) begin
                // Other caches have clean copies - set to SHARED
                cache_read_data[cache] = memory_read_data;
            end else begin
                // No other copies - set to EXCLUSIVE
                cache_read_data[cache] = memory_read_data;
            end
        end
        
        if (cache_write_request[cache]) begin
            // Invalidate all other copies
            snoop_address = cache_address[cache];
            // Send invalidate to all other caches
            for (int other = 0; other < 8; other++) begin
                if (other != cache && snoop_hit[other]) begin
                    // Send invalidate message
                    send_invalidate(other, cache_address[cache]);
                end
            end
            // Set local state to MODIFIED
        end
    end
end

endmodule
```

---

## 15.5 System Configuration and Boot Sequence

### 15.5.1 Boot ROM and Initialization

**Boot Sequence Implementation:**
```c
// Boot ROM located at 0x1FC00000 (reset vector)
__attribute__((section(".boot")))
void boot_sequence(void) {
    // 1. Hardware initialization
    initialize_clocks();
    initialize_memory_controller();
    configure_cache_system();
    
    // 2. Memory testing (optional)
    if (BOOT_CONFIG & BOOT_MEMORY_TEST) {
        memory_test_result = perform_memory_test();
        if (memory_test_result != 0) {
            signal_boot_error(MEMORY_TEST_FAILED);
            halt_system();
        }
    }
    
    // 3. Load and verify application
    application_header_t *app_header = (application_header_t*)APPLICATION_BASE;
    
    if (app_header->magic != APPLICATION_MAGIC) {
        signal_boot_error(INVALID_APPLICATION);
        enter_recovery_mode();
        return;
    }
    
    if (BOOT_CONFIG & BOOT_VERIFY_CHECKSUM) {
        uint32_t calculated_checksum = calculate_checksum(
            (uint8_t*)APPLICATION_BASE, 
            app_header->size
        );
        
        if (calculated_checksum != app_header->checksum) {
            signal_boot_error(CHECKSUM_MISMATCH);
            enter_recovery_mode();
            return;
        }
    }
    
    // 4. Setup application environment
    setup_interrupt_vectors(app_header->vector_table);
    configure_memory_protection(app_header->memory_regions);
    initialize_peripherals(app_header->peripheral_config);
    
    // 5. Transfer control to application
    void (*application_entry)(void) = (void(*)(void))app_header->entry_point;
    
    // Clear boot-specific registers and jump to application
    clear_boot_state();
    application_entry();
    
    // Should never reach here
    signal_boot_error(APPLICATION_RETURNED);
    halt_system();
}

// Clock initialization
void initialize_clocks(void) {
    // Configure PLL for 100MHz operation
    CLOCK_CTRL->pll_config = 
        PLL_ENABLE |
        (50 << PLL_MULTIPLIER_SHIFT) |  // 2MHz * 50 = 100MHz
        (1 << PLL_DIVIDER_SHIFT);       // Input divider = 1
    
    // Wait for PLL lock
    while (!(CLOCK_CTRL->status & PLL_LOCKED)) {
        // Wait for stable clock
    }
    
    // Switch to PLL clock
    CLOCK_CTRL->clock_select = CLOCK_SOURCE_PLL;
    
    // Configure peripheral clocks
    CLOCK_CTRL->peripheral_clocks = 
        UART_CLOCK_ENABLE |
        TIMER_CLOCK_ENABLE |
        DMA_CLOCK_ENABLE |
        GPIO_CLOCK_ENABLE;
}

// Memory controller initialization  
void initialize_memory_controller(void) {
    // Configure SDRAM interface
    MEM_CONFIG->sdram_config = 
        (2 << SDRAM_CAS_LATENCY) |      // CAS latency 2
        (3 << SDRAM_RAS_PRECHARGE) |    // RAS precharge 3 cycles
        (1 << SDRAM_BURST_LENGTH) |     // Burst length 2
        SDRAM_AUTO_REFRESH_ENABLE;
    
    // Set SDRAM timing parameters
    MEM_CONFIG->sdram_timing = 
        (6 << SDRAM_TRP_CYCLES) |       // tRP = 60ns @ 100MHz
        (6 << SDRAM_TRCD_CYCLES) |      // tRCD = 60ns
        (780 << SDRAM_REFRESH_CYCLES);  // 7.8us refresh
    
    // Initialize SDRAM
    sdram_init_sequence();
    
    // Configure cache memory
    configure_cache_system();
}
```

### 15.5.2 System Configuration Registers

**Configuration Register Map:**
```c
#define SYSTEM_CONFIG_BASE  0x40008000

typedef volatile struct {
    uint32_t chip_id;           /* 0x00: Chip identification */
    uint32_t revision;          /* 0x04: Silicon revision */
    uint32_t boot_config;       /* 0x08: Boot configuration */
    uint32_t system_config;     /* 0x0C: System configuration */
    uint32_t clock_config;      /* 0x10: Clock configuration */
    uint32_t reset_config;      /* 0x14: Reset configuration */
    uint32_t power_config;      /* 0x18: Power management */
    uint32_t test_config;       /* 0x1C: Test and debug */
    uint32_t peripheral_enable; /* 0x20: Peripheral enable */
    uint32_t interrupt_config;  /* 0x24: Global interrupt config */
    uint32_t dma_config;        /* 0x28: DMA configuration */
    uint32_t memory_config;     /* 0x2C: Memory configuration */
    uint32_t debug_config;      /* 0x30: Debug interface config */
    uint32_t reserved[3];       /* 0x34-0x3C: Reserved */
    uint32_t scratch[4];        /* 0x40-0x4C: Scratch registers */
} system_config_t;

#define SYS_CONFIG ((system_config_t*)SYSTEM_CONFIG_BASE)

// Chip identification
#define CHIP_ID_MCU32X      0x32580100
#define REVISION_MASK       0x0000FFFF

// Boot configuration bits
#define BOOT_MEMORY_TEST    (1 << 0)
#define BOOT_VERIFY_CHECKSUM (1 << 1)
#define BOOT_SAFE_MODE      (1 << 2)
#define BOOT_DEBUG_ENABLE   (1 << 3)

// System configuration
void configure_system(void) {
    // Verify chip identification
    if (SYS_CONFIG->chip_id != CHIP_ID_MCU32X) {
        signal_error(UNKNOWN_CHIP_ID);
        return;
    }
    
    uint32_t revision = SYS_CONFIG->revision & REVISION_MASK;
    if (revision < MINIMUM_SUPPORTED_REVISION) {
        signal_error(UNSUPPORTED_REVISION);
        return;
    }
    
    // Configure system features based on capabilities
    SYS_CONFIG->system_config = 
        CACHE_ENABLE |
        BRANCH_PREDICTION_ENABLE |
        FPU_ENABLE |
        DMA_ENABLE;
    
    // Enable required peripherals
    SYS_CONFIG->peripheral_enable = 
        UART_PERIPHERAL_ENABLE |
        TIMER_PERIPHERAL_ENABLE |
        GPIO_PERIPHERAL_ENABLE |
        INTERRUPT_CONTROLLER_ENABLE;
    
    // Configure global interrupt settings
    SYS_CONFIG->interrupt_config = 
        (8 << INTERRUPT_PRIORITY_LEVELS) |  // 8 priority levels
        VECTORED_INTERRUPTS_ENABLE |
        NESTED_INTERRUPTS_ENABLE;
}
```

---

## 15.6 System Debugging and Test Interface

### 15.6.1 JTAG Debug Interface

**Debug Access Port Implementation:**
```verilog
module jtag_debug_interface (
    // JTAG signals
    input wire tck,         // Test clock
    input wire tms,         // Test mode select
    input wire tdi,         // Test data input
    output wire tdo,        // Test data output
    input wire trst_n,      // Test reset (active low)
    
    // Processor debug interface
    output wire debug_request,
    input wire debug_acknowledge,
    output wire [31:0] debug_address,
    output wire [31:0] debug_write_data,
    input wire [31:0] debug_read_data,
    output wire debug_read,
    output wire debug_write,
    
    // System control
    output wire system_reset,
    output wire processor_halt,
    input wire processor_halted
);

// JTAG TAP controller states
typedef enum {
    TEST_LOGIC_RESET,
    RUN_TEST_IDLE,
    SELECT_DR_SCAN,
    CAPTURE_DR,
    SHIFT_DR,
    EXIT1_DR,
    PAUSE_DR,
    EXIT2_DR,
    UPDATE_DR,
    SELECT_IR_SCAN,
    CAPTURE_IR,
    SHIFT_IR,
    EXIT1_IR,
    PAUSE_IR,
    EXIT2_IR,
    UPDATE_IR
} jtag_state_t;

jtag_state_t jtag_state, jtag_next_state;

// JTAG instruction register
reg [3:0] instruction_register;
reg [3:0] instruction_shift_register;

// JTAG instructions
parameter BYPASS_INST       = 4'hF;
parameter IDCODE_INST       = 4'h1;
parameter DEBUG_ACCESS_INST = 4'h2;
parameter MEMORY_ACCESS_INST = 4'h3;
parameter REGISTER_ACCESS_INST = 4'h4;

// Device identification code
parameter IDCODE_VALUE = 32'h32580001;  // MCU-32X identification

// TAP controller state machine
always_ff @(posedge tck or negedge trst_n) begin
    if (!trst_n) begin
        jtag_state <= TEST_LOGIC_RESET;
    end else begin
        jtag_state <= jtag_next_state;
    end
end

// JTAG state transitions
always_comb begin
    case (jtag_state)
        TEST_LOGIC_RESET: 
            jtag_next_state = tms ? TEST_LOGIC_RESET : RUN_TEST_IDLE;
        RUN_TEST_IDLE:
            jtag_next_state = tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
        SELECT_DR_SCAN:
            jtag_next_state = tms ? SELECT_IR_SCAN : CAPTURE_DR;
        CAPTURE_DR:
            jtag_next_state = tms ? EXIT1_DR : SHIFT_DR;
        SHIFT_DR:
            jtag_next_state = tms ? EXIT1_DR : SHIFT_DR;
        EXIT1_DR:
            jtag_next_state = tms ? UPDATE_DR : PAUSE_DR;
        PAUSE_DR:
            jtag_next_state = tms ? EXIT2_DR : PAUSE_DR;
        EXIT2_DR:
            jtag_next_state = tms ? UPDATE_DR : SHIFT_DR;
        UPDATE_DR:
            jtag_next_state = tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
        SELECT_IR_SCAN:
            jtag_next_state = tms ? TEST_LOGIC_RESET : CAPTURE_IR;
        CAPTURE_IR:
            jtag_next_state = tms ? EXIT1_IR : SHIFT_IR;
        SHIFT_IR:
            jtag_next_state = tms ? EXIT1_IR : SHIFT_IR;
        EXIT1_IR:
            jtag_next_state = tms ? UPDATE_IR : PAUSE_IR;
        PAUSE_IR:
            jtag_next_state = tms ? EXIT2_IR : PAUSE_IR;
        EXIT2_IR:
            jtag_next_state = tms ? UPDATE_IR : SHIFT_IR;
        UPDATE_IR:
            jtag_next_state = tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
        default:
            jtag_next_state = TEST_LOGIC_RESET;
    endcase
end

// Debug register access
reg [31:0] debug_data_register;
reg [5:0] debug_address_register;
reg debug_read_enable, debug_write_enable;

// Debug access implementation
always_ff @(posedge tck) begin
    case (jtag_state)
        CAPTURE_IR: begin
            instruction_shift_register <= 4'h1;  // Capture fixed pattern
        end
        
        SHIFT_IR: begin
            instruction_shift_register <= {tdi, instruction_shift_register[3:1]};
        end
        
        UPDATE_IR: begin
            instruction_register <= instruction_shift_register;
        end
        
        CAPTURE_DR: begin
            case (instruction_register)
                IDCODE_INST: 
                    debug_data_register <= IDCODE_VALUE;
                DEBUG_ACCESS_INST:
                    debug_data_register <= debug_read_data;
                MEMORY_ACCESS_INST:
                    debug_data_register <= debug_read_data;
                default:
                    debug_data_register <= 32'h0;
            endcase
        end
        
        SHIFT_DR: begin
            debug_data_register <= {tdi, debug_data_register[31:1]};
        end
        
        UPDATE_DR: begin
            case (instruction_register)
                DEBUG_ACCESS_INST: begin
                    debug_address <= debug_data_register[31:6];
                    debug_write_data <= debug_data_register;
                    debug_write <= debug_data_register[0];
                    debug_read <= debug_data_register[1];
                end
                
                MEMORY_ACCESS_INST: begin
                    debug_address <= debug_address_register;
                    debug_write_data <= debug_data_register;
                    debug_write <= 1'b1;
                end
            endcase
        end
    endcase
end

assign tdo = debug_data_register[0];

endmodule
```

### 15.6.2 Debug Software Interface

**GDB Integration:**
```c
// GDB stub for remote debugging
#include "jtag_debug.h"

typedef struct {
    uint32_t registers[32];     // General purpose registers
    uint32_t pc;               // Program counter
    uint32_t status;           // Processor status
    uint32_t cause;            // Exception cause
    uint32_t epc;              // Exception PC
} debug_context_t;

static debug_context_t debug_context;
static int debug_mode_active = 0;

// GDB protocol implementation
void gdb_stub_main(void) {
    char command_buffer[256];
    char response_buffer[256];
    
    while (debug_mode_active) {
        // Receive GDB command
        int cmd_len = receive_gdb_packet(command_buffer, sizeof(command_buffer));
        
        if (cmd_len <= 0) continue;
        
        // Process GDB command
        switch (command_buffer[0]) {
            case 'g': // Read registers
                format_registers(response_buffer, &debug_context);
                send_gdb_packet(response_buffer);
                break;
                
            case 'G': // Write registers
                parse_registers(command_buffer + 1, &debug_context);
                send_gdb_packet("OK");
                break;
                
            case 'm': // Read memory
                handle_memory_read(command_buffer + 1, response_buffer);
                send_gdb_packet(response_buffer);
                break;
                
            case 'M': // Write memory
                handle_memory_write(command_buffer + 1);
                send_gdb_packet("OK");
                break;
                
            case 'c': // Continue execution
                debug_mode_active = 0;
                restore_processor_context(&debug_context);
                break;
                
            case 's': // Single step
                set_single_step_mode();
                debug_mode_active = 0;
                restore_processor_context(&debug_context);
                break;
                
            case 'z': // Remove breakpoint
                handle_remove_breakpoint(command_buffer + 1);
                send_gdb_packet("OK");
                break;
                
            case 'Z': // Insert breakpoint
                handle_insert_breakpoint(command_buffer + 1);
                send_gdb_packet("OK");
                break;
                
            default:
                send_gdb_packet("");  // Empty response for unsupported commands
                break;
        }
    }
}

// Breakpoint management
#define MAX_BREAKPOINTS 8

typedef struct {
    uint32_t address;
    uint32_t original_instruction;
    int active;
} breakpoint_t;

static breakpoint_t breakpoints[MAX_BREAKPOINTS];

int insert_breakpoint(uint32_t address) {
    // Find free breakpoint slot
    for (int i = 0; i < MAX_BREAKPOINTS; i++) {
        if (!breakpoints[i].active) {
            // Save original instruction
            breakpoints[i].original_instruction = 
                jtag_read_memory(address);
            
            // Insert breakpoint instruction (ebreak)
            jtag_write_memory(address, 0x00100073);  // ebreak instruction
            
            breakpoints[i].address = address;
            breakpoints[i].active = 1;
            
            return i;
        }
    }
    return -1;  // No free slots
}

void remove_breakpoint(uint32_t address) {
    for (int i = 0; i < MAX_BREAKPOINTS; i++) {
        if (breakpoints[i].active && breakpoints[i].address == address) {
            // Restore original instruction
            jtag_write_memory(address, breakpoints[i].original_instruction);
            breakpoints[i].active = 0;
            break;
        }
    }
}
```

---

*This chapter provided comprehensive coverage of MCU-32X system integration capabilities, enabling deployment in complex multi-processor environments while maintaining the performance and compatibility standards expected of late-1990s high-end embedded processors.*