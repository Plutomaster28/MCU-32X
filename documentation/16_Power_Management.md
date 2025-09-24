# Chapter 16: Power Management
## MCU-32X Technical Reference Manual

---

## 16.1 Power Management Overview

The MCU-32X incorporates sophisticated power management capabilities designed to meet the diverse requirements of both high-performance desktop computing and battery-powered embedded applications prevalent in 1999. The power management subsystem provides multiple operating modes, dynamic voltage and frequency scaling, and comprehensive thermal protection.

### 16.1.1 Power Management Architecture

**Power Management Features:**
- **Dynamic Voltage and Frequency Scaling (DVFS)**: Real-time performance adjustment
- **Multiple Sleep Modes**: Standby, Sleep, Deep Sleep, and Hibernation
- **Clock Gating**: Fine-grained clock control for power reduction  
- **Power Islands**: Independent power control for major functional blocks
- **Thermal Management**: Automatic throttling and shutdown protection
- **Battery Management**: Support for battery-powered applications
- **Wake-up Sources**: Multiple configurable wake-up events

**Power Domains:**
```
Core Domain:     CPU pipeline, register file, L1 caches
System Domain:   System bus, interrupt controller, timers
Peripheral Domain: UART, SPI, I2C, GPIO controllers
Memory Domain:   External memory interfaces, memory controllers
PLL Domain:      Phase-locked loops and clock generation
Always-On Domain: RTC, wake-up logic, power control registers
```

### 16.1.2 Power Management Register Interface

**Power Management Unit (PMU) Registers:**
```c
#define PMU_BASE                0x40005000

typedef volatile struct {
    uint32_t control;           /* 0x00: Power control register */
    uint32_t status;            /* 0x04: Power status register */
    uint32_t sleep_control;     /* 0x08: Sleep mode control */
    uint32_t wakeup_enable;     /* 0x0C: Wake-up source enable */
    uint32_t wakeup_status;     /* 0x10: Wake-up status */
    uint32_t clock_gate;        /* 0x14: Clock gating control */
    uint32_t voltage_control;   /* 0x18: Voltage regulator control */
    uint32_t frequency_control; /* 0x1C: Frequency scaling control */
    uint32_t thermal_control;   /* 0x20: Thermal management */
    uint32_t thermal_status;    /* 0x24: Thermal status */
    uint32_t battery_control;   /* 0x28: Battery management */
    uint32_t power_good;        /* 0x2C: Power rail status */
    uint32_t reset_control;     /* 0x30: Reset source control */
    uint32_t scratch[4];        /* 0x34-0x40: Scratch registers */
} pmu_regs_t;

#define PMU ((pmu_regs_t*)PMU_BASE)

// Power control register bits
#define PMU_CTRL_CORE_ENABLE    (1 << 0)   // Core domain enable
#define PMU_CTRL_SYS_ENABLE     (1 << 1)   // System domain enable  
#define PMU_CTRL_PERIPH_ENABLE  (1 << 2)   // Peripheral domain enable
#define PMU_CTRL_MEM_ENABLE     (1 << 3)   // Memory domain enable
#define PMU_CTRL_PLL_ENABLE     (1 << 4)   // PLL domain enable
#define PMU_CTRL_DVFS_ENABLE    (1 << 8)   // DVFS enable
#define PMU_CTRL_THERMAL_ENABLE (1 << 9)   // Thermal management enable

// Sleep mode definitions
#define PMU_SLEEP_STANDBY       0x1         // Standby mode
#define PMU_SLEEP_SLEEP         0x2         // Sleep mode
#define PMU_SLEEP_DEEP_SLEEP    0x3         // Deep sleep mode
#define PMU_SLEEP_HIBERNATE     0x4         // Hibernation mode

// Clock gating control bits
#define PMU_CLK_GATE_CPU        (1 << 0)   // CPU clock gate
#define PMU_CLK_GATE_CACHE      (1 << 1)   // Cache clock gate
#define PMU_CLK_GATE_BUS        (1 << 2)   // System bus clock gate
#define PMU_CLK_GATE_UART       (1 << 8)   // UART clock gate
#define PMU_CLK_GATE_SPI        (1 << 9)   // SPI clock gate
#define PMU_CLK_GATE_I2C        (1 << 10)  // I2C clock gate
#define PMU_CLK_GATE_GPIO       (1 << 11)  // GPIO clock gate
#define PMU_CLK_GATE_TIMER      (1 << 12)  // Timer clock gate

// Wake-up source enable bits
#define PMU_WAKEUP_GPIO         (1 << 0)   // GPIO interrupt wake-up
#define PMU_WAKEUP_UART         (1 << 1)   // UART activity wake-up
#define PMU_WAKEUP_TIMER        (1 << 2)   // Timer wake-up
#define PMU_WAKEUP_RTC          (1 << 3)   // RTC alarm wake-up
#define PMU_WAKEUP_EXTERNAL     (1 << 4)   // External wake-up pin
#define PMU_WAKEUP_USB          (1 << 5)   // USB activity wake-up
#define PMU_WAKEUP_ETHERNET     (1 << 6)   // Ethernet wake-on-LAN
```

---

## 16.2 Dynamic Voltage and Frequency Scaling (DVFS)

### 16.2.1 DVFS Operation

**Performance States:**
```c
// DVFS operating points
typedef struct {
    uint32_t cpu_freq_mhz;      // CPU frequency in MHz
    uint32_t bus_freq_mhz;      // Bus frequency in MHz
    uint32_t core_voltage_mv;   // Core voltage in millivolts
    uint32_t io_voltage_mv;     // I/O voltage in millivolts
    uint32_t power_mw;          // Estimated power consumption
} dvfs_operating_point_t;

// Predefined operating points for MCU-32X
static const dvfs_operating_point_t dvfs_points[] = {
    // High Performance Mode (Desktop applications)
    { 100, 50, 1800, 3300, 500 },    // Maximum performance
    {  80, 40, 1650, 3300, 350 },    // High performance
    {  60, 30, 1500, 3300, 220 },    // Medium performance
    
    // Balanced Mode (Mixed workloads)  
    {  40, 20, 1350, 3300, 130 },    // Balanced operation
    {  25, 12, 1250, 3300,  80 },    // Low performance
    
    // Power Save Mode (Battery applications)
    {  12,  6, 1150, 3300,  35 },    // Power save
    {   6,  3, 1100, 3300,  15 },    // Ultra low power
    {   1,  1, 1050, 3300,   8 }     // Minimum operation
};

#define DVFS_NUM_POINTS (sizeof(dvfs_points) / sizeof(dvfs_operating_point_t))

// DVFS control functions
typedef enum {
    DVFS_MODE_PERFORMANCE = 0,  // Maximum performance
    DVFS_MODE_BALANCED = 1,     // Balanced power/performance
    DVFS_MODE_POWER_SAVE = 2,   // Maximum battery life
    DVFS_MODE_MANUAL = 3        // Manual control
} dvfs_mode_t;

// Initialize DVFS subsystem
void dvfs_init(void) {
    // Enable DVFS in PMU
    PMU->control |= PMU_CTRL_DVFS_ENABLE;
    
    // Set initial operating point (balanced mode)
    dvfs_set_operating_point(3);  // 40MHz balanced point
    
    // Configure voltage regulator for dynamic adjustment
    PMU->voltage_control = 0x1800;  // 1.8V core, auto adjustment enabled
    
    // Enable automatic thermal throttling
    PMU->thermal_control |= (1 << 0);  // Auto throttle enable
}

// Set DVFS operating point
void dvfs_set_operating_point(int point_index) {
    if (point_index >= DVFS_NUM_POINTS) return;
    
    const dvfs_operating_point_t *point = &dvfs_points[point_index];
    
    // Disable interrupts during frequency change
    uint32_t int_state = disable_interrupts();
    
    // Step 1: Lower frequency first (if reducing)
    uint32_t current_freq = get_current_cpu_frequency();
    if (point->cpu_freq_mhz < current_freq) {
        set_cpu_frequency(point->cpu_freq_mhz);
        set_bus_frequency(point->bus_freq_mhz);
    }
    
    // Step 2: Adjust voltage
    set_core_voltage(point->core_voltage_mv);
    
    // Wait for voltage to stabilize
    delay_microseconds(100);
    
    // Step 3: Raise frequency (if increasing)
    if (point->cpu_freq_mhz > current_freq) {
        set_cpu_frequency(point->cpu_freq_mhz);
        set_bus_frequency(point->bus_freq_mhz);
    }
    
    // Update frequency control register
    PMU->frequency_control = (point->cpu_freq_mhz << 16) | point->bus_freq_mhz;
    
    // Re-enable interrupts
    restore_interrupts(int_state);
    
    // Update system timing parameters
    update_system_timing(point->cpu_freq_mhz);
}

// Automatic DVFS based on CPU utilization
void dvfs_auto_adjust(void) {
    static uint32_t last_idle_count = 0;
    static uint32_t last_total_count = 0;
    
    uint32_t idle_count = get_idle_loop_count();
    uint32_t total_count = get_total_cpu_cycles();
    
    uint32_t idle_delta = idle_count - last_idle_count;
    uint32_t total_delta = total_count - last_total_count;
    
    if (total_delta > 0) {
        uint32_t utilization_percent = 100 - ((idle_delta * 100) / total_delta);
        
        // Determine appropriate operating point based on utilization
        int target_point;
        if (utilization_percent > 80) {
            target_point = 0;  // High performance
        } else if (utilization_percent > 60) {
            target_point = 1;  // Medium-high performance
        } else if (utilization_percent > 40) {
            target_point = 3;  // Balanced
        } else if (utilization_percent > 20) {
            target_point = 5;  // Power save
        } else {
            target_point = 7;  // Minimum power
        }
        
        dvfs_set_operating_point(target_point);
    }
    
    last_idle_count = idle_count;
    last_total_count = total_count;
}
```

### 16.2.2 Voltage Regulation

**Voltage Regulator Control:**
```c
// Voltage regulator configuration
#define VREG_BASE               0x40005100

typedef volatile struct {
    uint32_t core_voltage;      /* 0x00: Core voltage control */
    uint32_t io_voltage;        /* 0x04: I/O voltage control */
    uint32_t pll_voltage;       /* 0x08: PLL voltage control */
    uint32_t regulator_status;  /* 0x0C: Regulator status */
    uint32_t load_switch;       /* 0x10: Load switch control */
    uint32_t power_good_mask;   /* 0x14: Power good mask */
} voltage_regulator_t;

#define VREG ((voltage_regulator_t*)VREG_BASE)

// Voltage control functions
void set_core_voltage(uint32_t millivolts) {
    // Voltage range: 1.05V to 1.8V in 50mV steps
    if (millivolts < 1050) millivolts = 1050;
    if (millivolts > 1800) millivolts = 1800;
    
    uint32_t voltage_code = (millivolts - 1050) / 50;
    
    // Set voltage with slew rate control
    VREG->core_voltage = (voltage_code << 0) | (1 << 8);  // Enable regulator
    
    // Wait for voltage to reach target (typical 50µs)
    while (!(VREG->regulator_status & (1 << 0))) {
        // Wait for core voltage ready
    }
}

void set_io_voltage(uint32_t millivolts) {
    // I/O voltage: typically 3.3V or 2.5V for compatibility
    if (millivolts < 2500) millivolts = 2500;
    if (millivolts > 3300) millivolts = 3300;
    
    uint32_t voltage_code = (millivolts == 3300) ? 1 : 0;
    VREG->io_voltage = (voltage_code << 0) | (1 << 8);  // Enable I/O regulator
    
    // Wait for I/O voltage ready
    while (!(VREG->regulator_status & (1 << 1))) {
        // Wait for I/O voltage ready
    }
}

// Power supply monitoring
typedef struct {
    float core_voltage;
    float io_voltage;
    float pll_voltage;
    int power_good;
} power_status_t;

power_status_t get_power_status(void) {
    power_status_t status;
    
    // Read voltage levels from ADC or voltage monitors
    uint32_t core_code = VREG->core_voltage & 0xFF;
    status.core_voltage = 1.05f + (core_code * 0.05f);
    
    uint32_t io_code = VREG->io_voltage & 0xFF;
    status.io_voltage = (io_code == 1) ? 3.3f : 2.5f;
    
    status.pll_voltage = 1.8f;  // Fixed PLL voltage
    
    status.power_good = (VREG->regulator_status & 0x7) == 0x7;
    
    return status;
}
```

---

## 16.3 Sleep Modes

### 16.3.1 Sleep Mode Configuration

**Sleep Mode Definitions:**
```c
// Sleep mode characteristics
typedef struct {
    const char *name;
    uint32_t power_consumption_mw;  // Typical power consumption
    uint32_t wakeup_latency_us;     // Wake-up latency
    uint32_t context_preserved;     // What state is preserved
    uint32_t clocks_active;         // Which clocks remain active
} sleep_mode_info_t;

static const sleep_mode_info_t sleep_modes[] = {
    {
        "Standby",
        50,     // 50mW power consumption
        10,     // 10µs wake-up latency
        0xFFFFFFFF,  // All context preserved
        0x0001       // Only RTC clock active
    },
    {
        "Sleep", 
        15,     // 15mW power consumption
        50,     // 50µs wake-up latency
        0xFFFFFFF0,  // CPU state lost, peripherals preserved
        0x0001       // Only RTC clock active
    },
    {
        "Deep Sleep",
        3,      // 3mW power consumption  
        200,    // 200µs wake-up latency
        0x000000FF,  // Only always-on domain preserved
        0x0001       // Only RTC clock active
    },
    {
        "Hibernation",
        0.5,    // 0.5mW power consumption
        5000,   // 5ms wake-up latency (includes boot)
        0x00000001,  // Only RTC preserved
        0x0001       // Only RTC clock active
    }
};

// Sleep mode entry functions
void enter_standby_mode(void) {
    // Save current processor state
    save_cpu_context();
    
    // Disable non-essential clocks
    PMU->clock_gate |= PMU_CLK_GATE_CPU | PMU_CLK_GATE_CACHE | PMU_CLK_GATE_BUS;
    
    // Enable wake-up sources
    PMU->wakeup_enable = PMU_WAKEUP_GPIO | PMU_WAKEUP_UART | PMU_WAKEUP_TIMER;
    
    // Enter standby mode
    PMU->sleep_control = PMU_SLEEP_STANDBY;
    
    // Execute WFI (Wait For Interrupt) instruction
    __asm volatile ("wfi");
    
    // Wake-up: restore clocks
    PMU->clock_gate &= ~(PMU_CLK_GATE_CPU | PMU_CLK_GATE_CACHE | PMU_CLK_GATE_BUS);
    
    // Restore processor state
    restore_cpu_context();
}

void enter_sleep_mode(void) {
    // Save peripheral state
    save_peripheral_context();
    
    // Disable more clocks (CPU context will be lost)
    PMU->clock_gate |= 0xFFFE;  // Disable all except RTC
    
    // Reduce core voltage for power savings
    uint32_t original_voltage = VREG->core_voltage;
    set_core_voltage(1100);  // Minimum voltage for memory retention
    
    // Configure wake-up sources
    PMU->wakeup_enable = PMU_WAKEUP_GPIO | PMU_WAKEUP_RTC | PMU_WAKEUP_EXTERNAL;
    
    // Enter sleep mode
    PMU->sleep_control = PMU_SLEEP_SLEEP;
    __asm volatile ("wfi");
    
    // Wake-up: restore voltage and clocks
    set_core_voltage((original_voltage & 0xFF) * 50 + 1050);
    PMU->clock_gate = 0x0001;  // Re-enable all clocks except what's gated
    
    // Restore peripheral state
    restore_peripheral_context();
    
    // CPU will restart from reset vector, OS must handle context restoration
}

void enter_deep_sleep_mode(void) {
    // Save critical system state to always-on memory
    save_system_state_to_backup_memory();
    
    // Power down most domains
    PMU->control &= ~(PMU_CTRL_CORE_ENABLE | PMU_CTRL_SYS_ENABLE | 
                      PMU_CTRL_PERIPH_ENABLE | PMU_CTRL_MEM_ENABLE);
    
    // Configure limited wake-up sources (only external pins and RTC)
    PMU->wakeup_enable = PMU_WAKEUP_RTC | PMU_WAKEUP_EXTERNAL;
    
    // Enter deep sleep
    PMU->sleep_control = PMU_SLEEP_DEEP_SLEEP;
    __asm volatile ("wfi");
    
    // Wake-up: system will reset, bootloader handles restore
}

void enter_hibernation_mode(void) {
    // Save minimal state to RTC backup registers
    save_hibernation_context();
    
    // Set hibernation flag for bootloader
    PMU->scratch[0] = 0xDEADBEEF;  // Hibernation magic number
    
    // Power down everything except RTC and wake-up logic
    PMU->control = 0;  // Disable all power domains
    
    // Only external wake-up pin can wake from hibernation
    PMU->wakeup_enable = PMU_WAKEUP_EXTERNAL;
    
    // Enter hibernation (system powers down)
    PMU->sleep_control = PMU_SLEEP_HIBERNATE;
    __asm volatile ("wfi");
    
    // System is now powered down
}
```

### 16.3.2 Wake-up Management

**Wake-up Source Configuration:**
```c
// Wake-up source management
typedef enum {
    WAKEUP_SOURCE_GPIO,
    WAKEUP_SOURCE_UART,
    WAKEUP_SOURCE_TIMER,
    WAKEUP_SOURCE_RTC,
    WAKEUP_SOURCE_EXTERNAL,
    WAKEUP_SOURCE_USB,
    WAKEUP_SOURCE_ETHERNET,
    WAKEUP_SOURCE_MAX
} wakeup_source_t;

typedef void (*wakeup_callback_t)(wakeup_source_t source);
static wakeup_callback_t wakeup_callbacks[WAKEUP_SOURCE_MAX];

// Configure wake-up sources
void configure_wakeup_source(wakeup_source_t source, int enable, 
                           wakeup_callback_t callback) {
    uint32_t wakeup_bit = 1 << source;
    
    if (enable) {
        PMU->wakeup_enable |= wakeup_bit;
        wakeup_callbacks[source] = callback;
    } else {
        PMU->wakeup_enable &= ~wakeup_bit;
        wakeup_callbacks[source] = NULL;
    }
}

// Wake-up interrupt handler
void pmu_wakeup_handler(void) {
    uint32_t wakeup_status = PMU->wakeup_status;
    
    // Process each wake-up source
    for (int i = 0; i < WAKEUP_SOURCE_MAX; i++) {
        if (wakeup_status & (1 << i)) {
            if (wakeup_callbacks[i] != NULL) {
                wakeup_callbacks[i]((wakeup_source_t)i);
            }
        }
    }
    
    // Clear wake-up status
    PMU->wakeup_status = wakeup_status;
}

// GPIO wake-up configuration
void configure_gpio_wakeup(int port, int pin, int edge_type) {
    // Configure GPIO pin for wake-up
    gpio_configure_interrupt(port, pin, GPIO_INT_EDGE, edge_type, NULL);
    
    // Enable GPIO as wake-up source
    configure_wakeup_source(WAKEUP_SOURCE_GPIO, 1, gpio_wakeup_callback);
}

void gpio_wakeup_callback(wakeup_source_t source) {
    // Handle GPIO wake-up
    uint32_t gpio_wakeup_pins = get_gpio_wakeup_status();
    
    // Process specific GPIO wake-up events
    handle_gpio_wakeup_event(gpio_wakeup_pins);
}

// RTC alarm wake-up
void configure_rtc_alarm_wakeup(uint32_t alarm_time_seconds) {
    // Set RTC alarm
    set_rtc_alarm(alarm_time_seconds);
    
    // Enable RTC wake-up
    configure_wakeup_source(WAKEUP_SOURCE_RTC, 1, rtc_wakeup_callback);
}

void rtc_wakeup_callback(wakeup_source_t source) {
    // Handle RTC alarm wake-up
    clear_rtc_alarm();
    handle_scheduled_wakeup();
}
```

---

## 16.4 Thermal Management

### 16.4.1 Temperature Monitoring

**Thermal Management System:**
```c
#define THERMAL_BASE            0x40005200

typedef volatile struct {
    uint32_t control;           /* 0x00: Thermal control */
    uint32_t status;            /* 0x04: Thermal status */
    uint32_t temperature;       /* 0x08: Current temperature */
    uint32_t threshold_warn;    /* 0x0C: Warning threshold */
    uint32_t threshold_critical;/* 0x10: Critical threshold */
    uint32_t throttle_control;  /* 0x14: Throttling control */
    uint32_t fan_control;       /* 0x18: Fan control (if available) */
    uint32_t history[8];        /* 0x1C-0x3B: Temperature history */
} thermal_mgmt_t;

#define THERMAL ((thermal_mgmt_t*)THERMAL_BASE)

// Thermal control bits
#define THERMAL_ENABLE          (1 << 0)   // Enable thermal monitoring
#define THERMAL_AUTO_THROTTLE   (1 << 1)   // Enable automatic throttling
#define THERMAL_INTERRUPT_EN    (1 << 2)   // Enable thermal interrupts
#define THERMAL_SHUTDOWN_EN     (1 << 3)   // Enable emergency shutdown

// Temperature thresholds (in degrees Celsius)
#define TEMP_NORMAL_MAX         70      // Normal operation limit
#define TEMP_WARNING            80      // Warning threshold  
#define TEMP_CRITICAL           90      // Critical threshold
#define TEMP_SHUTDOWN           100     // Emergency shutdown

// Initialize thermal management
void thermal_init(void) {
    // Enable thermal monitoring
    THERMAL->control = THERMAL_ENABLE | THERMAL_AUTO_THROTTLE | 
                       THERMAL_INTERRUPT_EN | THERMAL_SHUTDOWN_EN;
    
    // Set temperature thresholds
    THERMAL->threshold_warn = TEMP_WARNING;
    THERMAL->threshold_critical = TEMP_CRITICAL;
    
    // Configure automatic throttling
    THERMAL->throttle_control = 0x0804;  // 50% throttle at warning, 75% at critical
    
    // Initialize fan control (if available)
    THERMAL->fan_control = 0x0020;  // 25% fan speed initially
}

// Read current temperature
int get_cpu_temperature(void) {
    // Temperature in degrees Celsius (signed 8-bit value)
    return (int8_t)(THERMAL->temperature & 0xFF);
}

// Thermal monitoring task
void thermal_monitor_task(void) {
    static int last_temp = 0;
    static uint32_t overheat_count = 0;
    
    int current_temp = get_cpu_temperature();
    
    // Update temperature history
    static int history_index = 0;
    THERMAL->history[history_index] = current_temp;
    history_index = (history_index + 1) % 8;
    
    // Check thermal thresholds
    if (current_temp >= TEMP_CRITICAL) {
        // Critical temperature reached
        overheat_count++;
        
        if (overheat_count > 3) {
            // Sustained overheating - emergency shutdown
            initiate_thermal_shutdown();
        } else {
            // Temporary spike - aggressive throttling
            set_thermal_throttling(75);  // 75% performance reduction
        }
        
    } else if (current_temp >= TEMP_WARNING) {
        // Warning temperature - moderate throttling
        set_thermal_throttling(50);  // 50% performance reduction
        overheat_count = 0;
        
    } else if (current_temp <= TEMP_NORMAL_MAX) {
        // Normal temperature - remove throttling
        set_thermal_throttling(0);   // No throttling
        overheat_count = 0;
    }
    
    // Adjust fan speed based on temperature
    adjust_fan_speed(current_temp);
    
    last_temp = current_temp;
}

// Thermal throttling implementation
void set_thermal_throttling(int throttle_percent) {
    if (throttle_percent > 75) throttle_percent = 75;
    if (throttle_percent < 0) throttle_percent = 0;
    
    // Implement throttling by reducing CPU frequency
    int current_point = get_current_dvfs_point();
    
    if (throttle_percent > 50) {
        // Severe throttling - minimum frequency
        dvfs_set_operating_point(7);  // 1MHz minimum
    } else if (throttle_percent > 25) {
        // Moderate throttling - reduce by 2 levels
        int target_point = current_point + 2;
        if (target_point >= DVFS_NUM_POINTS) target_point = DVFS_NUM_POINTS - 1;
        dvfs_set_operating_point(target_point);
    } else if (throttle_percent > 0) {
        // Light throttling - reduce by 1 level
        int target_point = current_point + 1;
        if (target_point >= DVFS_NUM_POINTS) target_point = DVFS_NUM_POINTS - 1;
        dvfs_set_operating_point(target_point);
    } else {
        // No throttling - return to normal operation
        dvfs_set_operating_point(current_point);  // Maintain current point
    }
    
    // Update throttle control register
    THERMAL->throttle_control = (throttle_percent << 0);
}

// Fan speed control (for desktop applications)
void adjust_fan_speed(int temperature) {
    int fan_speed_percent;
    
    if (temperature <= 50) {
        fan_speed_percent = 0;      // Fan off
    } else if (temperature <= 60) {
        fan_speed_percent = 25;     // Low speed
    } else if (temperature <= 70) {
        fan_speed_percent = 50;     // Medium speed
    } else if (temperature <= 80) {
        fan_speed_percent = 75;     // High speed
    } else {
        fan_speed_percent = 100;    // Maximum speed
    }
    
    // Update fan control register
    THERMAL->fan_control = fan_speed_percent;
}

// Emergency thermal shutdown
void initiate_thermal_shutdown(void) {
    // Log shutdown reason
    PMU->scratch[1] = 0x7HERMAL;  // Thermal shutdown signature
    
    // Disable all non-essential power domains
    PMU->control &= PMU_CTRL_CORE_ENABLE;  // Keep only core for shutdown
    
    // Set minimum voltage and frequency
    set_core_voltage(1050);  // Minimum voltage
    dvfs_set_operating_point(7);  // Minimum frequency
    
    // Notify system of thermal shutdown
    system_thermal_shutdown();
}
```

---

## 16.5 Battery Management

### 16.5.1 Battery Monitoring

**Battery Management System:**
```c
#define BATTERY_BASE            0x40005300

typedef volatile struct {
    uint32_t control;           /* 0x00: Battery control */
    uint32_t status;            /* 0x04: Battery status */
    uint32_t voltage;           /* 0x08: Battery voltage */
    uint32_t current;           /* 0x0C: Battery current */
    uint32_t capacity;          /* 0x10: Estimated capacity */
    uint32_t charge_control;    /* 0x14: Charging control */
    uint32_t low_battery_warn;  /* 0x18: Low battery warning threshold */
    uint32_t critical_battery;  /* 0x1C: Critical battery threshold */
} battery_mgmt_t;

#define BATTERY ((battery_mgmt_t*)BATTERY_BASE)

// Battery management functions
typedef struct {
    float voltage_v;            // Battery voltage in volts
    float current_ma;           // Battery current in mA (+ = charging)
    float capacity_percent;     // Estimated capacity percentage
    int charging;              // Charging status
    int low_battery;           // Low battery warning
    int critical_battery;      // Critical battery level
} battery_status_t;

battery_status_t get_battery_status(void) {
    battery_status_t status;
    
    // Read battery voltage (12-bit ADC, 0-4.2V range)
    uint32_t voltage_raw = BATTERY->voltage & 0xFFF;
    status.voltage_v = (voltage_raw * 4.2f) / 4095.0f;
    
    // Read battery current (signed 12-bit, ±2A range)
    int32_t current_raw = (int32_t)(BATTERY->current & 0xFFF);
    if (current_raw > 2047) current_raw -= 4096;  // Sign extend
    status.current_ma = (current_raw * 2000.0f) / 2048.0f;
    
    // Read estimated capacity
    status.capacity_percent = (BATTERY->capacity & 0xFF);
    
    // Read status flags
    uint32_t battery_status_reg = BATTERY->status;
    status.charging = (battery_status_reg & (1 << 0)) != 0;
    status.low_battery = (battery_status_reg & (1 << 1)) != 0;
    status.critical_battery = (battery_status_reg & (1 << 2)) != 0;
    
    return status;
}

// Battery-aware power management
void battery_power_management(void) {
    battery_status_t battery = get_battery_status();
    
    if (battery.critical_battery) {
        // Critical battery - enter hibernation to preserve data
        save_critical_system_state();
        enter_hibernation_mode();
        
    } else if (battery.low_battery) {
        // Low battery - aggressive power saving
        dvfs_set_operating_point(6);  // Very low power mode
        
        // Disable non-essential peripherals
        PMU->clock_gate |= PMU_CLK_GATE_SPI | PMU_CLK_GATE_I2C;
        
        // Reduce display brightness (if applicable)
        set_display_brightness(20);
        
        // Enable low battery warning
        enable_low_battery_notification();
        
    } else if (battery.capacity_percent < 30 && !battery.charging) {
        // Medium battery - moderate power saving
        dvfs_set_operating_point(4);  // Low power mode
        
        // Reduce non-critical background tasks
        reduce_background_processing();
        
    } else {
        // Normal battery operation
        // Use performance-based DVFS
        dvfs_auto_adjust();
    }
}

// Charging management
void battery_charging_management(void) {
    battery_status_t battery = get_battery_status();
    
    if (battery.charging) {
        // Charging active - can use higher performance modes
        if (battery.capacity_percent > 80) {
            // Battery nearly full - allow maximum performance
            dvfs_set_operating_point(0);  // Maximum performance
        } else {
            // Charging but not full - balanced performance
            dvfs_set_operating_point(3);  // Balanced mode
        }
        
        // Monitor charging current and temperature
        if (battery.current_ma > 1000 || get_cpu_temperature() > 45) {
            // Reduce charging current to prevent overheating
            BATTERY->charge_control &= ~0xFF;
            BATTERY->charge_control |= 0x80;  // 50% charge rate
        }
        
    } else {
        // Not charging - use battery-aware power management
        battery_power_management();
    }
}
```

### 16.5.2 Power Estimation

**Power Consumption Estimation:**
```c
// Power consumption models for different components
typedef struct {
    uint32_t cpu_power_mw;      // CPU core power
    uint32_t cache_power_mw;    // Cache power  
    uint32_t bus_power_mw;      // System bus power
    uint32_t memory_power_mw;   // Memory interface power
    uint32_t peripheral_power_mw;// Peripheral power
    uint32_t io_power_mw;       // I/O power
    uint32_t total_power_mw;    // Total system power
} power_consumption_t;

power_consumption_t estimate_power_consumption(void) {
    power_consumption_t power = {0};
    
    // Get current DVFS operating point
    int dvfs_point = get_current_dvfs_point();
    const dvfs_operating_point_t *op_point = &dvfs_points[dvfs_point];
    
    // CPU power (base + frequency dependent)
    power.cpu_power_mw = 50 + (op_point->cpu_freq_mhz * 3);  // ~3mW per MHz
    
    // Cache power (depends on access rate and frequency)
    uint32_t cache_activity = get_cache_activity_percent();
    power.cache_power_mw = (cache_activity * op_point->cpu_freq_mhz) / 10;
    
    // Bus power (depends on bus utilization)
    uint32_t bus_activity = get_bus_activity_percent();
    power.bus_power_mw = (bus_activity * op_point->bus_freq_mhz) / 5;
    
    // Memory power (base + access dependent)
    power.memory_power_mw = 20;  // Base memory power
    if (PMU->control & PMU_CTRL_MEM_ENABLE) {
        power.memory_power_mw += get_memory_access_rate() / 1000;  // Dynamic power
    }
    
    // Peripheral power (sum of active peripherals)
    uint32_t active_peripherals = ~PMU->clock_gate;
    if (active_peripherals & PMU_CLK_GATE_UART) power.peripheral_power_mw += 5;
    if (active_peripherals & PMU_CLK_GATE_SPI) power.peripheral_power_mw += 8;
    if (active_peripherals & PMU_CLK_GATE_I2C) power.peripheral_power_mw += 3;
    if (active_peripherals & PMU_CLK_GATE_GPIO) power.peripheral_power_mw += 2;
    if (active_peripherals & PMU_CLK_GATE_TIMER) power.peripheral_power_mw += 4;
    
    // I/O power (depends on I/O activity and drive strength)
    power.io_power_mw = estimate_io_power();
    
    // Total power
    power.total_power_mw = power.cpu_power_mw + power.cache_power_mw + 
                          power.bus_power_mw + power.memory_power_mw +
                          power.peripheral_power_mw + power.io_power_mw;
    
    return power;
}

// Battery life estimation
uint32_t estimate_battery_life_minutes(void) {
    battery_status_t battery = get_battery_status();
    power_consumption_t power = estimate_power_consumption();
    
    if (battery.charging || power.total_power_mw == 0) {
        return 0xFFFFFFFF;  // Infinite (charging) or invalid
    }
    
    // Calculate remaining capacity in mWh
    // Assuming 3.7V nominal, 2000mAh typical battery
    float battery_capacity_mwh = 3.7f * 2000.0f * (battery.capacity_percent / 100.0f);
    
    // Calculate runtime in hours
    float runtime_hours = battery_capacity_mwh / power.total_power_mw;
    
    // Convert to minutes
    return (uint32_t)(runtime_hours * 60.0f);
}

// Power optimization recommendations
void optimize_power_consumption(void) {
    power_consumption_t power = estimate_power_consumption();
    battery_status_t battery = get_battery_status();
    
    // Identify highest power consumers
    if (power.cpu_power_mw > power.total_power_mw / 2) {
        // CPU is dominant - reduce frequency
        int current_point = get_current_dvfs_point();
        if (current_point < DVFS_NUM_POINTS - 1) {
            dvfs_set_operating_point(current_point + 1);
        }
    }
    
    if (power.peripheral_power_mw > 50) {
        // Peripherals using significant power - disable unused ones
        disable_unused_peripherals();
    }
    
    if (power.io_power_mw > 20) {
        // I/O power high - reduce drive strengths
        optimize_io_power_settings();
    }
    
    // Adaptive optimization based on battery level
    if (battery.capacity_percent < 20) {
        // Aggressive optimization for low battery
        set_power_optimization_level(POWER_OPT_AGGRESSIVE);
    } else if (battery.capacity_percent < 50) {
        // Moderate optimization
        set_power_optimization_level(POWER_OPT_MODERATE);
    } else {
        // Light optimization
        set_power_optimization_level(POWER_OPT_LIGHT);
    }
}
```

---

*This chapter provided comprehensive coverage of MCU-32X power management capabilities, enabling efficient operation across the full spectrum from high-performance desktop computing to ultra-low-power battery applications typical of late-1990s embedded systems.*