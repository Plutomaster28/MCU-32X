# MCU-32X Bootstrap Linker Script
# Defines memory layout and section placement for the bootstrap loader
# Compatible with MCU-32X memory architecture

OUTPUT_ARCH(riscv)
ENTRY(_start)

/* Memory layout for MCU-32X */
MEMORY
{
    /* Boot ROM - First 64KB of flash for bootloader */
    ROM (rx)    : ORIGIN = 0x00000000, LENGTH = 64K
    
    /* Application area - Remaining flash space */
    FLASH (rx)  : ORIGIN = 0x00010000, LENGTH = 448K
    
    /* Internal SRAM - 64KB total */
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 64K
    
    /* External DRAM - 16MB starting at 0x40000000 */
    DRAM (rwx)  : ORIGIN = 0x40000000, LENGTH = 16M
}

/* Define stack size */
STACK_SIZE = 0x2000;   /* 8KB stack */

/* Define global pointer for efficient data access */
PROVIDE(__global_pointer$ = __sdata_start + 0x800);

SECTIONS
{
    /* Interrupt vectors must be at the beginning */
    .vectors : ALIGN(256)
    {
        PROVIDE(_vector_start = .);
        KEEP(*(.vectors))
        PROVIDE(_vector_end = .);
    } > ROM

    /* Startup code immediately after vectors */
    .startup : ALIGN(4)
    {
        PROVIDE(_startup_start = .);
        *(.startup)
        PROVIDE(_startup_end = .);
    } > ROM

    /* Main text section */
    .text : ALIGN(4)
    {
        PROVIDE(_text_start = .);
        
        /* Keep critical functions first for better cache utilization */
        *(.text.system_init)
        *(.text.clock_init)
        *(.text.memory_init)
        *(.text.cache_init)
        *(.text.interrupt_init)
        
        /* All other text */
        *(.text)
        *(.text.*)
        
        /* GNU linker scripts */
        *(.gnu.linkonce.t.*)
        
        PROVIDE(_text_end = .);
    } > ROM

    /* Read-only data */
    .rodata : ALIGN(4)
    {
        PROVIDE(_rodata_start = .);
        *(.rodata)
        *(.rodata.*)
        *(.gnu.linkonce.r.*)
        
        /* String literals */
        *(.srodata)
        *(.srodata.*)
        
        PROVIDE(_rodata_end = .);
    } > ROM

    /* Exception handling (if needed) */
    .eh_frame : ALIGN(4)
    {
        KEEP(*(.eh_frame))
    } > ROM

    /* Global constructor/destructor tables */
    .init_array : ALIGN(4)
    {
        PROVIDE(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end = .);
    } > ROM

    .fini_array : ALIGN(4)
    {
        PROVIDE(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE(__fini_array_end = .);
    } > ROM

    /* Store load address of data section */
    PROVIDE(_data_load_start = LOADADDR(.data));

    /* Initialized data section (copied from ROM to RAM at startup) */
    .data : ALIGN(4)
    {
        PROVIDE(_data_start = .);
        
        /* Small data section for efficient GP-relative access */
        PROVIDE(__sdata_start = .);
        *(.sdata)
        *(.sdata.*)
        *(.gnu.linkonce.s.*)
        PROVIDE(__sdata_end = .);
        
        /* Regular data */
        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d.*)
        
        /* Ensure 4-byte alignment */
        . = ALIGN(4);
        PROVIDE(_data_end = .);
    } > RAM AT > ROM

    /* Uninitialized data section (cleared at startup) */
    .bss : ALIGN(4)
    {
        PROVIDE(_bss_start = .);
        
        /* Small BSS section */
        PROVIDE(__sbss_start = .);
        *(.sbss)
        *(.sbss.*)
        *(.gnu.linkonce.sb.*)
        *(.scommon)
        PROVIDE(__sbss_end = .);
        
        /* Regular BSS */
        *(.bss)
        *(.bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        
        /* Ensure 4-byte alignment */
        . = ALIGN(4);
        PROVIDE(_bss_end = .);
    } > RAM

    /* Stack section */
    .stack : ALIGN(16)
    {
        PROVIDE(_stack_start = .);
        . += STACK_SIZE;
        PROVIDE(_stack_end = .);
        PROVIDE(__stack_top = .);
    } > RAM

    /* Heap section (optional, grows upward from end of stack) */
    .heap : ALIGN(4)
    {
        PROVIDE(_heap_start = .);
        PROVIDE(_heap_end = ORIGIN(RAM) + LENGTH(RAM));
    } > RAM

    /* Application signature area in flash */
    .app_signature 0x00010000 : ALIGN(4)
    {
        PROVIDE(_app_signature = .);
        /* Reserve space for application signature and entry point */
        . += 16;
    } > FLASH

    /* Debug sections (not loaded into target memory) */
    .debug_info     0 : { *(.debug_info) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    .debug_ranges   0 : { *(.debug_ranges) }
    .debug_pubtypes 0 : { *(.debug_pubtypes) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    
    /* DWARF 3 */
    .debug_ranges   0 : { *(.debug_ranges) }
    
    /* DWARF 4 */
    .debug_types    0 : { *(.debug_types) }
    
    /* Discard unwanted sections */
    /DISCARD/ :
    {
        *(.note.GNU-stack)
        *(.gnu_debuglink)
        *(.gnu.lto_*)
        *(.comment)
    }

    /* Memory usage summary symbols */
    PROVIDE(_rom_used = _rodata_end - _vector_start);
    PROVIDE(_ram_used = _stack_end - _data_start);
    PROVIDE(_rom_available = LENGTH(ROM) - _rom_used);
    PROVIDE(_ram_available = LENGTH(RAM) - _ram_used);
}

/* Verify memory usage doesn't exceed available space */
ASSERT(_rom_used <= LENGTH(ROM), "ROM overflow: bootloader too large")
ASSERT(_ram_used <= LENGTH(RAM), "RAM overflow: not enough memory for data and stack")

/* Ensure minimum stack space */
ASSERT(STACK_SIZE >= 0x1000, "Stack too small: minimum 4KB required")

/* Verify alignment requirements */
ASSERT((_vector_start & 0xFF) == 0, "Vector table must be 256-byte aligned")
ASSERT((_data_start & 0x3) == 0, "Data section must be 4-byte aligned")
ASSERT((_bss_start & 0x3) == 0, "BSS section must be 4-byte aligned")

/* Export memory layout symbols for debugging */
PROVIDE(_memory_layout_rom_start = ORIGIN(ROM));
PROVIDE(_memory_layout_rom_size = LENGTH(ROM));
PROVIDE(_memory_layout_ram_start = ORIGIN(RAM));
PROVIDE(_memory_layout_ram_size = LENGTH(RAM));
PROVIDE(_memory_layout_flash_start = ORIGIN(FLASH));
PROVIDE(_memory_layout_flash_size = LENGTH(FLASH));
PROVIDE(_memory_layout_dram_start = ORIGIN(DRAM));
PROVIDE(_memory_layout_dram_size = LENGTH(DRAM));