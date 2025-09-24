# Chapter 12: Interrupt Handling
## MCU-32X Technical Reference Manual

---

## 12.1 Interrupt System Overview

The MCU-32X implements a comprehensive interrupt system based on the RISC-V machine mode specification, optimized for real-time embedded applications and desktop system requirements.

### 12.1.1 Interrupt Architecture

**Interrupt Capabilities:**
- **Machine Mode Only**: Simplified privilege model
- **Vectored Interrupts**: Fast dispatch to specific handlers
- **Nested Interrupts**: Support for interrupt prioritization
- **Low Latency**: 6-cycle worst-case response time
- **32 External Sources**: Expandable interrupt controller

**Interrupt Types:**
1. **Machine Software Interrupt (MSI)**: Inter-processor communication
2. **Machine Timer Interrupt (MTI)**: System timer and scheduling
3. **Machine External Interrupt (MEI)**: Peripheral and I/O devices
4. **Local Interrupts**: Processor-specific events

### 12.1.2 Interrupt Processing Flow

**Interrupt Response Sequence:**
```
1. Interrupt Request (IRQ) asserted by source
2. Interrupt Controller prioritizes and signals CPU
3. CPU completes current instruction (if possible)
4. CPU saves current PC and status to CSRs
5. CPU jumps to interrupt vector
6. Handler executes and services interrupt
7. Handler executes MRET instruction
8. CPU restores PC and status from CSRs
9. Normal execution resumes
```

**Timing Characteristics:**
- **Response Latency**: 3-6 cycles from assertion to handler entry
- **Context Save**: 2-4 cycles (automatic CSR save)
- **Handler Entry**: 1 cycle (vector jump)
- **Return Overhead**: 1 cycle (MRET instruction)

---

## 12.2 Interrupt Control and Status Registers

### 12.2.1 Machine Status Register (mstatus)

**mstatus CSR (0x300) - Key Fields for Interrupts:**

| Bits | Name | Description | Reset Value |
|------|------|-------------|-------------|
| 3 | MIE | Machine Interrupt Enable | 0 |
| 7 | MPIE | Previous Machine Interrupt Enable | 0 |
| 12:11 | MPP | Previous Privilege Mode | 11 (Machine) |

**Interrupt Enable Control:**
```assembly
# Enable all machine interrupts
csrsi mstatus, 0x8          # Set MIE bit

# Disable all machine interrupts  
csrci mstatus, 0x8          # Clear MIE bit

# Atomic interrupt disable and save state
csrrci t0, mstatus, 0x8     # t0 = old mstatus, MIE cleared

# Restore interrupt state
csrw mstatus, t0            # Restore previous state
```

### 12.2.2 Machine Interrupt Enable Register (mie)

**mie CSR (0x304) - Interrupt Enable Bits:**

| Bits | Name | Description |
|------|------|-------------|
| 3 | MSIE | Machine Software Interrupt Enable |
| 7 | MTIE | Machine Timer Interrupt Enable |
| 11 | MEIE | Machine External Interrupt Enable |
| 31:16 | Local | Local Interrupt Enables (implementation-specific) |

**Individual Interrupt Control:**
```assembly
# Enable timer interrupts
csrsi mie, 0x80             # Set MTIE bit (bit 7)

# Enable external interrupts
csrsi mie, 0x800            # Set MEIE bit (bit 11)

# Enable all interrupt types
li    t0, 0x888             # MSI(3) + MTI(7) + MEI(11)  
csrs  mie, t0               # Set all interrupt enable bits

# Read current interrupt enables
csrr  t0, mie               # Get interrupt enable mask
```

### 12.2.3 Machine Interrupt Pending Register (mip)

**mip CSR (0x344) - Interrupt Pending Status:**

| Bits | Name | Description | Access |
|------|------|-------------|---------|
| 3 | MSIP | Machine Software Interrupt Pending | RW |
| 7 | MTIP | Machine Timer Interrupt Pending | RO |
| 11 | MEIP | Machine External Interrupt Pending | RO |
| 31:16 | Local | Local Interrupt Pending | RO/RW |

**Interrupt Status Checking:**
```assembly
# Check which interrupts are pending
csrr  t0, mip               # Read pending interrupts
andi  t1, t0, 0x80          # Check timer interrupt (MTIP)
bnez  t1, timer_pending     # Branch if timer interrupt pending

# Software interrupt trigger (for multicore systems)
csrsi mip, 0x8              # Set MSIP bit (software interrupt)

# Clear software interrupt
csrci mip, 0x8              # Clear MSIP bit
```

---

## 12.3 Interrupt Vector Table

### 12.3.1 Machine Trap Vector Register (mtvec)

**mtvec CSR (0x305) - Vector Configuration:**

| Bits | Name | Description |
|------|------|-------------|
| 31:2 | BASE | Interrupt vector base address |
| 1:0 | MODE | Vector mode (0=Direct, 1=Vectored) |

**Vector Modes:**
- **Direct Mode (0)**: All exceptions and interrupts jump to BASE
- **Vectored Mode (1)**: Interrupts jump to BASE + 4×cause

### 12.3.2 Interrupt Vector Table Setup

**Vector Table Initialization:**
```assembly
# Set up vectored interrupt mode
.align 6                    # Align to 64-byte boundary
interrupt_vector_table:
    j    exception_handler      # 0: Exception entry point
    .word 0                     # 1: Reserved
    .word 0                     # 2: Reserved  
    j    software_interrupt     # 3: Machine software interrupt
    .word 0                     # 4: Reserved
    .word 0                     # 5: Reserved
    .word 0                     # 6: Reserved
    j    timer_interrupt        # 7: Machine timer interrupt
    .word 0                     # 8: Reserved
    .word 0                     # 9: Reserved
    .word 0                     # 10: Reserved
    j    external_interrupt     # 11: Machine external interrupt
    # Local interrupt vectors (12-31)
    j    local_int_12          # 12: Local interrupt 12
    j    local_int_13          # 13: Local interrupt 13
    # ... additional vectors ...

# Initialize mtvec register
init_interrupts:
    la   t0, interrupt_vector_table
    ori  t0, t0, 1             # Set vectored mode
    csrw mtvec, t0             # Set vector base and mode
    
    # Enable global interrupts
    csrsi mstatus, 0x8         # Set MIE bit
    ret
```

### 12.3.3 Direct Mode Handler

**Single Entry Point for All Interrupts:**
```assembly
# Direct mode interrupt handler
.align 4
direct_interrupt_handler:
    # Save minimal context
    addi sp, sp, -16
    sw   t0, 0(sp)
    sw   t1, 4(sp)
    
    # Read interrupt cause
    csrr t0, mcause
    blt  t0, x0, handle_interrupt   # MSB set = interrupt
    
    # Handle exceptions
    j    exception_dispatch
    
handle_interrupt:
    # Clear interrupt bit and dispatch
    slli t0, t0, 1             # Clear MSB (interrupt bit)
    srli t0, t0, 1             # Restore cause value
    
    # Dispatch based on cause
    li   t1, 3
    beq  t0, t1, software_isr
    li   t1, 7
    beq  t0, t1, timer_isr
    li   t1, 11
    beq  t0, t1, external_isr
    
    # Unknown interrupt
    j    unknown_interrupt
    
software_isr:
    jal  ra, handle_software_interrupt
    j    interrupt_return
    
timer_isr:
    jal  ra, handle_timer_interrupt
    j    interrupt_return
    
external_isr:
    jal  ra, handle_external_interrupt
    j    interrupt_return
    
interrupt_return:
    # Restore context
    lw   t1, 4(sp)
    lw   t0, 0(sp)
    addi sp, sp, 16
    mret                       # Return from interrupt
```

---

## 12.4 External Interrupt Controller

### 12.4.1 Interrupt Controller Architecture

**External Interrupt Sources:**
- 32 external interrupt inputs (IRQ0-IRQ31)
- Programmable priority levels (0-15)
- Level and edge triggered modes
- Interrupt masking and acknowledgment

**Memory-Mapped Interrupt Controller Registers:**
```c
#define INT_CTRL_BASE    0x40005000

typedef volatile struct {
    uint32_t irq_status;        /* 0x00: Interrupt status (RO) */
    uint32_t irq_enable;        /* 0x04: Interrupt enable mask */
    uint32_t irq_pending;       /* 0x08: Pending interrupts (RO) */
    uint32_t irq_clear;         /* 0x0C: Interrupt acknowledge (WO) */
    uint32_t irq_trigger;       /* 0x10: Trigger mode (0=level, 1=edge) */
    uint32_t irq_priority[8];   /* 0x14-0x30: Priority config (4 IRQs per reg) */
    uint32_t priority_mask;     /* 0x34: Priority threshold */
    uint32_t current_priority;  /* 0x38: Current handling priority (RO) */
} interrupt_controller_t;

#define IRQ_CTRL ((interrupt_controller_t*)INT_CTRL_BASE)
```

### 12.4.2 Interrupt Controller Programming

**Interrupt Configuration Example:**
```c
// Initialize interrupt controller
void init_interrupt_controller(void) {
    // Disable all interrupts initially
    IRQ_CTRL->irq_enable = 0;
    
    // Clear any pending interrupts
    IRQ_CTRL->irq_clear = 0xFFFFFFFF;
    
    // Configure interrupt priorities (0=highest, 15=lowest)
    // UART interrupt (IRQ0) - high priority
    IRQ_CTRL->irq_priority[0] = (IRQ_CTRL->irq_priority[0] & ~0xF) | 2;
    
    // Timer interrupt (IRQ1) - highest priority  
    IRQ_CTRL->irq_priority[0] = (IRQ_CTRL->irq_priority[0] & ~0xF0) | (1 << 4);
    
    // GPIO interrupt (IRQ2) - medium priority
    IRQ_CTRL->irq_priority[0] = (IRQ_CTRL->irq_priority[0] & ~0xF00) | (5 << 8);
    
    // Configure trigger modes (0=level, 1=edge)
    IRQ_CTRL->irq_trigger = 0x00000007;  // IRQ0-2 edge triggered
    
    // Set priority threshold (only handle priority <= threshold)
    IRQ_CTRL->priority_mask = 10;        // Handle priorities 0-10
    
    // Enable specific interrupts
    IRQ_CTRL->irq_enable = 0x00000007;   // Enable IRQ0-2
}

// Interrupt service routine framework
void external_interrupt_handler(void) {
    uint32_t pending = IRQ_CTRL->irq_pending & IRQ_CTRL->irq_enable;
    
    // Find highest priority pending interrupt
    for (int irq = 0; irq < 32; irq++) {
        if (pending & (1 << irq)) {
            // Call specific handler
            call_irq_handler(irq);
            
            // Acknowledge interrupt
            IRQ_CTRL->irq_clear = (1 << irq);
            
            // Handle only one interrupt per invocation for fairness
            break;
        }
    }
}
```

---

## 12.5 Specific Interrupt Handlers

### 12.5.1 Timer Interrupt Handler

**System Timer Programming:**
```c
#define TIMER_BASE      0x40004000

typedef volatile struct {
    uint32_t timer_low;         /* 0x00: Timer count low 32 bits */
    uint32_t timer_high;        /* 0x04: Timer count high 32 bits */
    uint32_t compare_low;       /* 0x08: Compare value low 32 bits */
    uint32_t compare_high;      /* 0x0C: Compare value high 32 bits */
    uint32_t control;           /* 0x10: Timer control register */
} timer_regs_t;

#define TIMER ((timer_regs_t*)TIMER_BASE)

// Timer interrupt handler
void handle_timer_interrupt(void) {
    // Read current timer value
    uint64_t current_time = ((uint64_t)TIMER->timer_high << 32) | TIMER->timer_low;
    
    // Set next interrupt (1ms @ 100MHz = 100,000 cycles)
    uint64_t next_interrupt = current_time + 100000;
    TIMER->compare_low = (uint32_t)next_interrupt;
    TIMER->compare_high = (uint32_t)(next_interrupt >> 32);
    
    // Update system tick counter
    system_tick_count++;
    
    // Call scheduler for task switching
    if ((system_tick_count % SCHEDULER_TICKS) == 0) {
        schedule_next_task();
    }
    
    // Handle timeouts and delays
    update_system_timers();
}
```

### 12.5.2 UART Interrupt Handler

**Serial Communication Interrupt:**
```c
#define UART_BASE       0x40001000

typedef volatile struct {
    uint32_t data;              /* 0x00: Transmit/receive data */
    uint32_t status;            /* 0x04: Status register */
    uint32_t control;           /* 0x08: Control register */
    uint32_t baud_div;          /* 0x0C: Baud rate divisor */
    uint32_t int_enable;        /* 0x10: Interrupt enable */
    uint32_t int_status;        /* 0x14: Interrupt status */
} uart_regs_t;

#define UART ((uart_regs_t*)UART_BASE)

// UART status bits
#define UART_STATUS_RX_READY    (1 << 0)
#define UART_STATUS_TX_EMPTY    (1 << 1)
#define UART_STATUS_RX_FULL     (1 << 2)
#define UART_STATUS_TX_FULL     (1 << 3)

// UART interrupt handler
void handle_uart_interrupt(void) {
    uint32_t int_status = UART->int_status;
    
    // Handle receive interrupt
    if (int_status & UART_STATUS_RX_READY) {
        while (UART->status & UART_STATUS_RX_READY) {
            char received_char = UART->data & 0xFF;
            
            // Add to receive buffer
            if (uart_rx_buffer_count < UART_RX_BUFFER_SIZE) {
                uart_rx_buffer[uart_rx_buffer_tail] = received_char;
                uart_rx_buffer_tail = (uart_rx_buffer_tail + 1) % UART_RX_BUFFER_SIZE;
                uart_rx_buffer_count++;
            }
            
            // Signal waiting tasks
            sem_post(&uart_rx_semaphore);
        }
    }
    
    // Handle transmit interrupt  
    if (int_status & UART_STATUS_TX_EMPTY) {
        if (uart_tx_buffer_count > 0) {
            UART->data = uart_tx_buffer[uart_tx_buffer_head];
            uart_tx_buffer_head = (uart_tx_buffer_head + 1) % UART_TX_BUFFER_SIZE;
            uart_tx_buffer_count--;
        } else {
            // Disable TX interrupt when buffer empty
            UART->int_enable &= ~UART_STATUS_TX_EMPTY;
        }
    }
}
```

### 12.5.3 GPIO Interrupt Handler

**General Purpose I/O Interrupts:**
```c
#define GPIO_BASE       0x40000000

typedef volatile struct {
    uint32_t data;              /* 0x00: GPIO data register */
    uint32_t direction;         /* 0x04: Direction (1=output) */
    uint32_t int_enable;        /* 0x08: Interrupt enable */
    uint32_t int_status;        /* 0x0C: Interrupt status */
    uint32_t int_edge;          /* 0x10: Edge detection (1=edge, 0=level) */
    uint32_t int_polarity;      /* 0x14: Polarity (1=high/rising, 0=low/falling) */
} gpio_regs_t;

#define GPIO ((gpio_regs_t*)GPIO_BASE)

// GPIO interrupt handler
void handle_gpio_interrupt(void) {
    uint32_t int_status = GPIO->int_status;
    
    // Process each pending GPIO interrupt
    for (int pin = 0; pin < 32; pin++) {
        if (int_status & (1 << pin)) {
            // Call registered callback for this pin
            if (gpio_callbacks[pin] != NULL) {
                gpio_callbacks[pin](pin);
            }
            
            // Clear interrupt for edge-triggered pins
            if (GPIO->int_edge & (1 << pin)) {
                GPIO->int_status = (1 << pin);  // Write 1 to clear
            }
        }
    }
}

// GPIO interrupt registration
typedef void (*gpio_callback_t)(int pin);
static gpio_callback_t gpio_callbacks[32];

void gpio_register_interrupt(int pin, gpio_callback_t callback, 
                           int edge_triggered, int active_high) {
    // Disable interrupt during configuration
    GPIO->int_enable &= ~(1 << pin);
    
    // Configure trigger mode and polarity
    if (edge_triggered) {
        GPIO->int_edge |= (1 << pin);       // Edge triggered
    } else {
        GPIO->int_edge &= ~(1 << pin);      // Level triggered
    }
    
    if (active_high) {
        GPIO->int_polarity |= (1 << pin);   // High/rising active
    } else {
        GPIO->int_polarity &= ~(1 << pin);  // Low/falling active
    }
    
    // Register callback
    gpio_callbacks[pin] = callback;
    
    // Clear any pending interrupt
    GPIO->int_status = (1 << pin);
    
    // Enable interrupt
    GPIO->int_enable |= (1 << pin);
}
```

---

## 12.6 Nested Interrupt Support

### 12.6.1 Interrupt Nesting Implementation

**Priority-Based Nesting:**
```assembly
# Enhanced interrupt handler with nesting support
nested_interrupt_handler:
    # Save critical registers
    addi sp, sp, -32
    sw   t0, 0(sp)
    sw   t1, 4(sp)
    sw   t2, 8(sp)
    sw   ra, 12(sp)
    
    # Save current interrupt level
    csrr t0, mcause
    sw   t0, 16(sp)
    
    # Get interrupt priority
    jal  ra, get_interrupt_priority  # Returns priority in a0
    
    # Check if higher priority than current
    lw   t1, current_priority
    bge  a0, t1, no_nesting         # Skip if same or lower priority
    
    # Save old priority and set new priority
    sw   t1, 20(sp)                 # Save old priority
    sw   a0, current_priority       # Set new priority
    
    # Re-enable interrupts for higher priority
    csrsi mstatus, 0x8              # Enable interrupts
    
    # Call interrupt service routine
    jal  ra, dispatch_interrupt
    
    # Disable interrupts for cleanup
    csrci mstatus, 0x8              # Disable interrupts
    
    # Restore old priority
    lw   t1, 20(sp)
    sw   t1, current_priority
    
    j    interrupt_cleanup

no_nesting:
    # Call interrupt service routine without nesting
    jal  ra, dispatch_interrupt

interrupt_cleanup:
    # Restore registers
    lw   ra, 12(sp)
    lw   t2, 8(sp)
    lw   t1, 4(sp)
    lw   t0, 0(sp)
    addi sp, sp, 32
    mret
```

### 12.6.2 Critical Section Management

**Interrupt-Safe Critical Sections:**
```c
// Critical section implementation
typedef struct {
    uint32_t saved_mstatus;
    int nesting_level;
} critical_section_t;

static critical_section_t critical_state = {0, 0};

void enter_critical_section(void) {
    uint32_t mstatus;
    
    // Read and disable interrupts atomically
    asm volatile ("csrrci %0, mstatus, 8" : "=r"(mstatus));
    
    if (critical_state.nesting_level == 0) {
        critical_state.saved_mstatus = mstatus;
    }
    critical_state.nesting_level++;
}

void exit_critical_section(void) {
    critical_state.nesting_level--;
    
    if (critical_state.nesting_level == 0) {
        // Restore original interrupt state
        asm volatile ("csrw mstatus, %0" : : "r"(critical_state.saved_mstatus));
    }
}

// RAII-style critical section for C++
#ifdef __cplusplus
class CriticalSection {
public:
    CriticalSection() { enter_critical_section(); }
    ~CriticalSection() { exit_critical_section(); }
};

#define CRITICAL_SECTION() CriticalSection _cs
#endif
```

---

## 12.7 Interrupt Performance Optimization

### 12.7.1 Fast Interrupt Handlers

**Optimized Handler for Time-Critical Interrupts:**
```assembly
# Fast interrupt handler (minimal overhead)
.align 4
fast_interrupt_handler:
    # Use mscratch to save one register without stack access
    csrrw t0, mscratch, t0      # Swap t0 with mscratch
    
    # Determine interrupt source quickly
    csrr  t1, mcause            # Read cause
    andi  t1, t1, 0x1F          # Mask to get cause code
    
    # Jump table for fast dispatch (single cycle)
    la    t2, fast_jump_table
    slli  t1, t1, 2             # Multiply by 4 for word offset
    add   t2, t2, t1            # Calculate jump address
    lw    t1, 0(t2)             # Load handler address
    jr    t1                    # Jump to handler
    
fast_jump_table:
    .word unknown_fast_handler  # 0: Reserved
    .word unknown_fast_handler  # 1: Reserved  
    .word unknown_fast_handler  # 2: Reserved
    .word software_fast_handler # 3: Software interrupt
    .word unknown_fast_handler  # 4: Reserved
    .word unknown_fast_handler  # 5: Reserved
    .word unknown_fast_handler  # 6: Reserved  
    .word timer_fast_handler    # 7: Timer interrupt
    # ... additional entries ...

timer_fast_handler:
    # Handle timer interrupt with minimal overhead
    # Acknowledge timer interrupt
    la   t1, TIMER_BASE
    sw   x0, 16(t1)             # Clear timer interrupt
    
    # Increment tick counter (direct memory access)
    la   t1, system_tick_count
    lw   t2, 0(t1)
    addi t2, t2, 1
    sw   t2, 0(t1)
    
    # Restore and return  
    csrrw t0, mscratch, t0      # Restore t0
    mret

software_fast_handler:
    # Handle software interrupt
    csrci mip, 8                # Clear software interrupt
    # ... minimal processing ...
    csrrw t0, mscratch, t0      # Restore t0  
    mret
```

### 12.7.2 Interrupt Latency Measurement

**Performance Monitoring:**
```c
// Interrupt latency measurement
typedef struct {
    uint64_t total_interrupts;
    uint64_t total_cycles;
    uint32_t min_latency;
    uint32_t max_latency;
    uint32_t last_latency;
} interrupt_stats_t;

static interrupt_stats_t irq_stats[32];

void measure_interrupt_latency(int irq_num) {
    static uint64_t interrupt_start_time;
    uint64_t current_time;
    uint32_t latency;
    
    // Read current cycle count
    asm volatile ("csrr %0, mcycle" : "=r"(current_time));
    
    if (irq_stats[irq_num].total_interrupts == 0) {
        // First interrupt - just record start time
        interrupt_start_time = current_time;
    } else {
        // Calculate latency from previous interrupt
        latency = (uint32_t)(current_time - interrupt_start_time);
        
        // Update statistics
        irq_stats[irq_num].total_cycles += latency;
        irq_stats[irq_num].last_latency = latency;
        
        if (latency < irq_stats[irq_num].min_latency || 
            irq_stats[irq_num].min_latency == 0) {
            irq_stats[irq_num].min_latency = latency;
        }
        
        if (latency > irq_stats[irq_num].max_latency) {
            irq_stats[irq_num].max_latency = latency;
        }
    }
    
    irq_stats[irq_num].total_interrupts++;
    interrupt_start_time = current_time;
}

// Get interrupt statistics
uint32_t get_average_interrupt_latency(int irq_num) {
    if (irq_stats[irq_num].total_interrupts == 0) {
        return 0;
    }
    return irq_stats[irq_num].total_cycles / irq_stats[irq_num].total_interrupts;
}
```

---

*This chapter provided comprehensive coverage of the MCU-32X interrupt handling system, enabling efficient real-time programming and system integration for both embedded and desktop applications.*