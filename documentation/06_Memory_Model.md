# Chapter 6: Memory Model
## MCU-32X Technical Reference Manual

---

## 6.1 Memory Architecture Overview

The MCU-32X implements a Harvard architecture with separate instruction and data memory spaces, optimized for embedded and desktop applications requiring predictable performance.

### 6.1.1 Memory Space Organization

**Address Space Layout:**
```
0xFFFFFFFF ┌─────────────────┐
           │   Reserved      │
0x80000000 ├─────────────────┤
           │ Flash Memory    │ 16MB - Program Storage
           │ (Non-volatile)  │
0x40000000 ├─────────────────┤
           │ I/O Space       │ 1GB - Memory-mapped I/O
           │ (Uncached)      │
0x20000000 ├─────────────────┤
           │ External RAM    │ 512MB - Main Memory  
           │ (Cached)        │
0x10000000 ├─────────────────┤
           │ Data Memory     │ 64KB - Fast Local RAM
           │ (Cached)        │
0x00001000 ├─────────────────┤
           │ Instruction     │ 64KB - Fast Local ROM
           │ Memory (ROM)    │
0x00000000 └─────────────────┘
```

### 6.1.2 Memory Hierarchy

**Level 1: On-Chip Caches**
- Instruction Cache: 32KB, 4-way set associative
- Data Cache: 32KB, 4-way set associative  
- Cache Line Size: 32 bytes (8 instructions)
- Write Policy: Write-through with write buffer

**Level 2: Local Memory**
- Instruction Memory: 64KB embedded SRAM
- Data Memory: 64KB embedded SRAM
- Access Time: 1 cycle (cache hit equivalent)
- Use: Critical code/data, interrupt handlers

**Level 3: External Memory**
- Main Memory: Up to 512MB DDR SDRAM
- Access Time: 12 cycles typical
- Bandwidth: 400MB/s @ 100MHz
- Use: Application code, large data structures

---

## 6.2 Address Translation and Protection

### 6.2.1 Physical Memory Map

**Memory Region Characteristics:**

| Region | Base Address | Size | Cacheable | Executable | Description |
|--------|--------------|------|-----------|------------|-------------|
| IMEM | 0x00000000 | 64KB | Yes | Yes | Local instruction memory |
| DMEM | 0x10000000 | 64KB | Yes | No | Local data memory |
| RAM | 0x20000000 | 512MB | Yes | Yes | External main memory |
| I/O | 0x40000000 | 1GB | No | No | Memory-mapped I/O |
| Flash | 0x80000000 | 16MB | Yes | Yes | Non-volatile storage |

### 6.2.2 Memory Protection Unit (MPU)

**Protection Regions:**
The MCU-32X implements 8 configurable protection regions for access control.

**Region Configuration:**
```c
typedef struct mpu_region {
    uint32_t base_address;      /* Region start address */
    uint32_t size;              /* Region size (power of 2) */
    uint32_t attributes;        /* Access permissions */
    uint32_t enable;            /* Region enable bit */
} mpu_region_t;

/* Attribute definitions */
#define MPU_ATTR_READ      (1 << 0)  /* Read permission */
#define MPU_ATTR_WRITE     (1 << 1)  /* Write permission */ 
#define MPU_ATTR_EXEC      (1 << 2)  /* Execute permission */
#define MPU_ATTR_CACHE     (1 << 3)  /* Cacheable */
#define MPU_ATTR_BUFFER    (1 << 4)  /* Write bufferable */
#define MPU_ATTR_SHARE     (1 << 5)  /* Shareable */
```

**Default Protection Regions:**
```c
/* Typical MPU configuration */
mpu_region_t default_regions[8] = {
    /* Region 0: Instruction memory (RX) */
    {0x00000000, 0x10000, MPU_ATTR_READ | MPU_ATTR_EXEC | MPU_ATTR_CACHE, 1},
    
    /* Region 1: Data memory (RW) */  
    {0x10000000, 0x10000, MPU_ATTR_READ | MPU_ATTR_WRITE | MPU_ATTR_CACHE, 1},
    
    /* Region 2: Main RAM (RWX) */
    {0x20000000, 0x20000000, MPU_ATTR_READ | MPU_ATTR_WRITE | MPU_ATTR_EXEC | 
                              MPU_ATTR_CACHE | MPU_ATTR_BUFFER, 1},
    
    /* Region 3: I/O space (RW, uncached) */
    {0x40000000, 0x40000000, MPU_ATTR_READ | MPU_ATTR_WRITE, 1},
    
    /* Region 4: Flash memory (RX) */
    {0x80000000, 0x01000000, MPU_ATTR_READ | MPU_ATTR_EXEC | MPU_ATTR_CACHE, 1},
    
    /* Regions 5-7: Available for application use */
    {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}
};
```

---

## 6.3 Cache Architecture

### 6.3.1 Instruction Cache Design

**Configuration:**
- **Size**: 32KB total capacity
- **Associativity**: 4-way set associative
- **Line Size**: 32 bytes (8 RISC-V instructions)
- **Sets**: 256 sets
- **Index Bits**: 8 bits (bits 12:5 of address)
- **Offset Bits**: 5 bits (bits 4:0 of address)
- **Tag Bits**: 19 bits (bits 31:13 of address)

**Cache Line Structure:**
```
Tag (19 bits) | Valid | LRU | Instruction Data (32 bytes)
```

**Replacement Policy**: Pseudo-LRU (least recently used)

**Performance Characteristics:**
- Hit Time: 1 cycle
- Miss Penalty: 12 cycles (external memory)
- Typical Hit Rate: 96-98%

### 6.3.2 Data Cache Design

**Configuration:**
- **Size**: 32KB total capacity
- **Associativity**: 4-way set associative
- **Line Size**: 32 bytes
- **Sets**: 256 sets
- **Write Policy**: Write-through with write buffer

**Write Buffer:**
- **Depth**: 8 entries
- **Width**: 32 bits + byte enables
- **Coalescing**: Combines writes to same line
- **Bypass**: Read-after-write bypass for performance

**Cache Operations:**
```assembly
# Cache maintenance instructions (if implemented)
cache.iflush        # Flush instruction cache
cache.dflush        # Flush data cache  
cache.iinval        # Invalidate instruction cache
cache.dinval        # Invalidate data cache
```

### 6.3.3 Cache Coherency

**Single-Core Coherency:**
- Hardware maintains coherency between I-cache and D-cache
- Self-modifying code requires explicit cache management
- Write buffer drains maintain memory ordering

**Cache Behavior:**
```c
/* Example: Handling self-modifying code */
void update_instruction(uint32_t *addr, uint32_t new_inst) {
    *addr = new_inst;                    /* Write new instruction */
    __asm__ volatile ("fence.i");       /* Instruction fence */
    /* Hardware invalidates I-cache line containing addr */
}
```

---

## 6.4 Memory Ordering and Synchronization

### 6.4.1 Memory Ordering Model

The MCU-32X follows the RISC-V weak memory ordering model with explicit synchronization instructions.

**Ordering Rules:**
1. **Program Order**: Instructions execute in program order within a single hart
2. **Memory Barriers**: Explicit fences enforce ordering between memory operations
3. **Device Ordering**: I/O accesses are strictly ordered
4. **Cache Coherency**: Maintained automatically in hardware

**Memory Fence Instructions:**
```assembly
fence           # Full memory fence (all operations)
fence.i         # Instruction fence (synchronize I-cache)
fence r, w      # Fence with predecessor/successor sets
fence w, r      # Store-to-load fence
```

### 6.4.2 Atomic Operations

While the base MCU-32X doesn't implement atomic extensions, software synchronization primitives are available:

**Software Spinlock:**
```assembly
# Simple spinlock using test-and-set simulation
acquire_lock:
    li      t1, 1               # Lock value
retry:
    lw      t0, lock_var        # Load current lock state
    bnez    t0, retry           # Spin if locked
    sw      t1, lock_var        # Attempt to acquire
    lw      t2, lock_var        # Read back
    bne     t1, t2, retry       # Retry if failed
    fence   w, rw               # Memory barrier
    ret

release_lock:
    fence   rw, w               # Memory barrier  
    sw      x0, lock_var        # Release lock
    ret
```

**Critical Section Management:**
```assembly
# Interrupt-based critical sections
enter_critical:
    csrci   mstatus, 0x8        # Disable interrupts
    ret

exit_critical:  
    csrsi   mstatus, 0x8        # Enable interrupts
    ret
```

---

## 6.5 Memory Access Performance

### 6.5.1 Access Latencies

**Cache Hit Performance:**
- L1 I-Cache Hit: 1 cycle
- L1 D-Cache Hit: 1 cycle  
- Write Buffer Hit: 0 cycles (pipelined)

**Cache Miss Performance:**
- L1 Miss to External RAM: 12 cycles
- L1 Miss to Flash: 15 cycles
- Write Buffer Full Stall: 1-4 cycles

**Memory Access Patterns:**
```c
/* Performance analysis of access patterns */

/* Sequential access (good cache behavior) */
void sequential_copy(uint32_t *src, uint32_t *dst, int count) {
    for (int i = 0; i < count; i++) {
        dst[i] = src[i];        /* ~1.2 cycles per word */
    }
}

/* Strided access (moderate cache behavior) */  
void strided_access(uint32_t *array, int stride, int count) {
    for (int i = 0; i < count; i++) {
        array[i * stride] = i;  /* ~3.1 cycles per access */
    }
}

/* Random access (poor cache behavior) */
void random_access(uint32_t *array, int *indices, int count) {
    for (int i = 0; i < count; i++) {
        array[indices[i]] = i;  /* ~8.5 cycles per access */
    }
}
```

### 6.5.2 Memory Bandwidth Optimization

**Optimized Memory Copy:**
```assembly
# Fast memory copy routine (32-byte aligned)
fast_memcopy:
    # Assume a0=dst, a1=src, a2=count (in bytes)
    
    # Check alignment
    or      t0, a0, a1
    andi    t0, t0, 31
    bnez    t0, byte_copy       # Fall back if not aligned
    
    # 32-byte aligned copy (one cache line)
    srli    t1, a2, 5           # Count / 32
    beqz    t1, remainder
    
line_loop:
    lw      t2, 0(a1)           # Load 8 words
    lw      t3, 4(a1)
    lw      t4, 8(a1)
    lw      t5, 12(a1)
    lw      t6, 16(a1)
    lw      s2, 20(a1)
    lw      s3, 24(a1)
    lw      s4, 28(a1)
    
    sw      t2, 0(a0)           # Store 8 words
    sw      t3, 4(a0)
    sw      t4, 8(a0)
    sw      t5, 12(a0)
    sw      t6, 16(a0)
    sw      s2, 20(a0)
    sw      s3, 24(a0)
    sw      s4, 28(a0)
    
    addi    a1, a1, 32          # Advance pointers
    addi    a0, a0, 32
    addi    t1, t1, -1          # Decrement count
    bnez    t1, line_loop
    
remainder:
    andi    a2, a2, 31          # Remaining bytes
    # Handle remainder with byte copy...

# Performance: ~1.1 cycles per byte for aligned data
```

---

## 6.6 Memory-Mapped I/O

### 6.6.1 I/O Space Organization

**I/O Address Map (0x40000000 - 0x7FFFFFFF):**

| Base Address | Size | Device | Description |
|--------------|------|--------|-------------|
| 0x40000000 | 4KB | GPIO | General-purpose I/O ports |
| 0x40001000 | 4KB | UART | Serial communication |
| 0x40002000 | 4KB | SPI | Serial peripheral interface |
| 0x40003000 | 4KB | I2C | Inter-integrated circuit |
| 0x40004000 | 4KB | Timer | System timers |
| 0x40005000 | 4KB | Interrupt | Interrupt controller |
| 0x40006000 | 4KB | DMA | Direct memory access |
| 0x40007000 | 4KB | Debug | Debug interface |
| 0x40008000 | - | Reserved | Future expansion |

### 6.6.2 I/O Access Characteristics

**Access Properties:**
- All I/O accesses are uncached
- Strict ordering enforced
- No speculative access
- Side effects preserved

**I/O Access Example:**
```c
/* Memory-mapped I/O register definitions */
#define GPIO_BASE       0x40000000
#define UART_BASE       0x40001000

typedef volatile struct {
    uint32_t data;          /* 0x00: Data register */
    uint32_t direction;     /* 0x04: Direction (1=output) */
    uint32_t interrupt;     /* 0x08: Interrupt enable */
    uint32_t status;        /* 0x0C: Status register */
} gpio_regs_t;

typedef volatile struct {
    uint32_t data;          /* 0x00: Transmit/receive data */
    uint32_t status;        /* 0x04: Status register */
    uint32_t control;       /* 0x08: Control register */
    uint32_t baud_div;      /* 0x0C: Baud rate divisor */
} uart_regs_t;

/* Access I/O registers */
gpio_regs_t *gpio = (gpio_regs_t *)GPIO_BASE;
uart_regs_t *uart = (uart_regs_t *)UART_BASE;

void set_gpio_pin(int pin) {
    gpio->data |= (1 << pin);       /* Atomic read-modify-write */
}

void uart_send_char(char c) {
    while (!(uart->status & 0x1));  /* Wait for TX ready */
    uart->data = c;                  /* Send character */
}
```

---

## 6.7 Memory Layout for Applications

### 6.7.1 Typical Application Memory Layout

**C Program Memory Organization:**
```
High Memory
┌─────────────────┐ 0x2FFFFFFF
│     Stack       │ ↓ (grows down)
│                 │
├─────────────────┤
│     Heap        │ ↑ (grows up)  
│                 │
├─────────────────┤
│ .bss (uninit)   │ Zero-initialized data
├─────────────────┤
│ .data (init)    │ Initialized data
├─────────────────┤
│ .rodata         │ Read-only data
├─────────────────┤
│ .text           │ Program code
└─────────────────┘ 0x20000000
Low Memory
```

### 6.7.2 Linker Memory Sections

**Section Placement Strategy:**
```ld
SECTIONS {
    /* Critical code in fast local memory */
    .boot 0x00000000 : {
        *(.boot)                /* Boot code */
        *(.isr)                 /* Interrupt handlers */
    } > IMEM
    
    /* Main program in external memory */
    .text 0x20000000 : {
        *(.text)                /* Program code */
        *(.text.*)
    } > RAM
    
    /* Read-only data */
    .rodata : {
        *(.rodata)              /* String literals, const data */
        *(.rodata.*)
    } > RAM
    
    /* Initialized data (copied from ROM) */
    .data : {
        *(.data)                /* Global initialized variables */
        *(.data.*)
    } > RAM AT > FLASH
    
    /* Uninitialized data */
    .bss : {
        *(.bss)                 /* Global uninitialized variables */
        *(.bss.*)
        *(COMMON)
    } > RAM
    
    /* Fast data in local memory */
    .fast_data 0x10000000 : {
        *(.fast_data)           /* Critical variables */
        *(.fast_bss)
    } > DMEM
}
```

### 6.7.3 Memory Allocation Strategies

**Static Allocation:**
```c
/* Place critical data in fast memory */
__attribute__((section(".fast_data")))
volatile uint32_t interrupt_count = 0;

__attribute__((section(".fast_bss")))  
uint32_t dma_buffer[256];

/* Regular global data */
int global_counter = 42;        /* Goes in .data */
char text_buffer[1024];         /* Goes in .bss */
```

**Dynamic Allocation:**
```c
/* Custom allocator for different memory regions */
void *malloc_fast(size_t size) {
    /* Allocate from fast local memory (limited) */
    static char fast_heap[8192];
    static size_t fast_offset = 0;
    
    if (fast_offset + size > sizeof(fast_heap)) {
        return NULL;            /* Fast memory exhausted */
    }
    
    void *ptr = &fast_heap[fast_offset];
    fast_offset += (size + 3) & ~3;    /* Word align */
    return ptr;
}

void *malloc_external(size_t size) {
    /* Allocate from external RAM (abundant) */
    return malloc(size);        /* Use standard allocator */
}
```

---

## 6.8 Memory Performance Optimization

### 6.8.1 Cache-Friendly Programming

**Data Structure Layout:**
```c
/* Cache-unfriendly structure (poor locality) */
struct bad_structure {
    int frequently_used;        /* 4 bytes */
    char padding[60];           /* Wastes cache space */
    int also_frequent;          /* 4 bytes */
};

/* Cache-friendly structure (good locality) */
struct good_structure {
    int frequently_used;        /* 4 bytes */
    int also_frequent;          /* 4 bytes */
    char other_data[56];        /* Less frequently accessed */
} __attribute__((packed));
```

**Loop Optimization:**
```c
/* Cache-friendly matrix traversal (row-major) */
void matrix_process_good(int matrix[100][100]) {
    for (int row = 0; row < 100; row++) {
        for (int col = 0; col < 100; col++) {
            matrix[row][col] *= 2;      /* Sequential access */
        }
    }
}

/* Cache-unfriendly traversal (column-major) */ 
void matrix_process_bad(int matrix[100][100]) {
    for (int col = 0; col < 100; col++) {
        for (int row = 0; row < 100; row++) {
            matrix[row][col] *= 2;      /* Strided access */
        }
    }
}
```

### 6.8.2 Memory Bandwidth Utilization

**Prefetch Strategies:**
```c
/* Software prefetch simulation */
void prefetch_array_processing(uint32_t *data, int count) {
    const int PREFETCH_DISTANCE = 8;   /* Cache lines ahead */
    
    for (int i = 0; i < count; i++) {
        /* Process current data */
        data[i] = data[i] * 2 + 1;
        
        /* Prefetch future data (if supported) */
        if (i + PREFETCH_DISTANCE < count) {
            /* __builtin_prefetch(&data[i + PREFETCH_DISTANCE], 0, 1); */
            /* Simulated by touching next cache line */
            volatile uint32_t dummy = data[i + PREFETCH_DISTANCE];
            (void)dummy;
        }
    }
}
```

---

*This chapter provided comprehensive coverage of the MCU-32X memory architecture, enabling efficient memory usage and system optimization for both embedded and desktop applications.*