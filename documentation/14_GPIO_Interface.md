# Chapter 14: GPIO and I/O
## MCU-32X Technical Reference Manual

---

## 14.1 General Purpose I/O Overview

The MCU-32X provides comprehensive General Purpose I/O (GPIO) capabilities designed to interface with a wide variety of external devices and systems. The GPIO subsystem supports both traditional embedded applications and desktop computing peripherals common in late-1990s systems.

### 14.1.1 GPIO Architecture

**GPIO Features:**
- **32 GPIO Pins**: Individually configurable direction and function
- **Alternate Functions**: UART, SPI, I2C, Timer, PWM multiplexing
- **Interrupt Support**: Edge and level triggered interrupts on all pins
- **Drive Strength Control**: Programmable output drive strength
- **Pull-up/Pull-down**: Internal resistor configuration
- **5V Tolerance**: Compatible with 5V logic levels (with 3.3V I/O)

**GPIO Pin Organization:**
```
Port A: GPIO[7:0]   - General purpose I/O, UART functions
Port B: GPIO[15:8]  - General purpose I/O, SPI functions  
Port C: GPIO[23:16] - General purpose I/O, I2C functions
Port D: GPIO[31:24] - General purpose I/O, Timer/PWM functions
```

### 14.1.2 GPIO Register Interface

**GPIO Memory Map:**
```c
#define GPIO_BASE           0x40000000

typedef volatile struct {
    uint32_t data;              /* 0x00: GPIO data register */
    uint32_t direction;         /* 0x04: Direction control (1=output) */
    uint32_t alternate_func;    /* 0x08: Alternate function select */
    uint32_t pull_config;       /* 0x0C: Pull-up/down configuration */
    uint32_t drive_strength;    /* 0x10: Output drive strength */
    uint32_t slew_rate;         /* 0x14: Output slew rate control */
    uint32_t input_filter;      /* 0x18: Input noise filter enable */
    uint32_t interrupt_enable;  /* 0x1C: Interrupt enable */
    uint32_t interrupt_type;    /* 0x20: Interrupt type (edge/level) */
    uint32_t interrupt_polarity;/* 0x24: Interrupt polarity */
    uint32_t interrupt_status;  /* 0x28: Interrupt status (RW1C) */
    uint32_t interrupt_mask;    /* 0x2C: Interrupt mask */
} gpio_regs_t;

#define GPIO ((gpio_regs_t*)GPIO_BASE)

// Individual port access
#define GPIO_PORTA ((gpio_regs_t*)(GPIO_BASE + 0x000))
#define GPIO_PORTB ((gpio_regs_t*)(GPIO_BASE + 0x100))
#define GPIO_PORTC ((gpio_regs_t*)(GPIO_BASE + 0x200))
#define GPIO_PORTD ((gpio_regs_t*)(GPIO_BASE + 0x300))
```

**GPIO Configuration Bits:**
```c
// Pull configuration (2 bits per pin)
#define GPIO_PULL_NONE      0  // No pull resistor
#define GPIO_PULL_UP        1  // Pull-up resistor
#define GPIO_PULL_DOWN      2  // Pull-down resistor
#define GPIO_PULL_REPEATER  3  // Bus keeper/repeater

// Drive strength (2 bits per pin)  
#define GPIO_DRIVE_2MA      0  // 2mA drive strength
#define GPIO_DRIVE_4MA      1  // 4mA drive strength
#define GPIO_DRIVE_8MA      2  // 8mA drive strength
#define GPIO_DRIVE_12MA     3  // 12mA drive strength

// Slew rate control
#define GPIO_SLEW_SLOW      0  // Slow slew rate (reduce EMI)
#define GPIO_SLEW_FAST      1  // Fast slew rate (high speed)

// Interrupt types
#define GPIO_INT_LEVEL      0  // Level triggered
#define GPIO_INT_EDGE       1  // Edge triggered

// Interrupt polarity  
#define GPIO_INT_LOW_FALL   0  // Active low/falling edge
#define GPIO_INT_HIGH_RISE  1  // Active high/rising edge
```

---

## 14.2 GPIO Basic Operations

### 14.2.1 Pin Configuration

**Basic GPIO Setup:**
```c
// GPIO initialization function
void gpio_init_pin(int port, int pin, int direction, int pull_type) {
    gpio_regs_t *gpio_port;
    
    // Select GPIO port
    switch (port) {
        case 0: gpio_port = GPIO_PORTA; break;
        case 1: gpio_port = GPIO_PORTB; break;
        case 2: gpio_port = GPIO_PORTC; break;
        case 3: gpio_port = GPIO_PORTD; break;
        default: return; // Invalid port
    }
    
    // Configure pin direction
    if (direction) {
        gpio_port->direction |= (1 << pin);   // Output
    } else {
        gpio_port->direction &= ~(1 << pin);  // Input
    }
    
    // Configure pull resistor
    uint32_t pull_mask = ~(3 << (pin * 2));
    uint32_t pull_value = pull_type << (pin * 2);
    gpio_port->pull_config = (gpio_port->pull_config & pull_mask) | pull_value;
    
    // Set default drive strength and slew rate for outputs
    if (direction) {
        uint32_t drive_mask = ~(3 << (pin * 2));
        uint32_t drive_value = GPIO_DRIVE_4MA << (pin * 2);
        gpio_port->drive_strength = (gpio_port->drive_strength & drive_mask) | drive_value;
        
        gpio_port->slew_rate &= ~(1 << pin); // Default to slow slew rate
    }
    
    // Disable alternate function (use as GPIO)
    gpio_port->alternate_func &= ~(1 << pin);
}

// Read GPIO pin state
int gpio_read_pin(int port, int pin) {
    gpio_regs_t *gpio_port;
    
    switch (port) {
        case 0: gpio_port = GPIO_PORTA; break;
        case 1: gpio_port = GPIO_PORTB; break;
        case 2: gpio_port = GPIO_PORTC; break;
        case 3: gpio_port = GPIO_PORTD; break;
        default: return -1;
    }
    
    return (gpio_port->data >> pin) & 1;
}

// Write GPIO pin state
void gpio_write_pin(int port, int pin, int value) {
    gpio_regs_t *gpio_port;
    
    switch (port) {
        case 0: gpio_port = GPIO_PORTA; break;
        case 1: gpio_port = GPIO_PORTB; break;
        case 2: gpio_port = GPIO_PORTC; break;
        case 3: gpio_port = GPIO_PORTD; break;
        default: return;
    }
    
    if (value) {
        gpio_port->data |= (1 << pin);   // Set pin high
    } else {
        gpio_port->data &= ~(1 << pin);  // Set pin low
    }
}

// Toggle GPIO pin
void gpio_toggle_pin(int port, int pin) {
    gpio_regs_t *gpio_port;
    
    switch (port) {
        case 0: gpio_port = GPIO_PORTA; break;
        case 1: gpio_port = GPIO_PORTB; break;
        case 2: gpio_port = GPIO_PORTC; break;
        case 3: gpio_port = GPIO_PORTD; break;
        default: return;
    }
    
    gpio_port->data ^= (1 << pin);  // XOR toggles the bit
}
```

### 14.2.2 Port-Level Operations

**Efficient Port Access:**
```c
// Read entire port (8 bits)
uint8_t gpio_read_port(int port) {
    gpio_regs_t *gpio_port;
    
    switch (port) {
        case 0: gpio_port = GPIO_PORTA; break;
        case 1: gpio_port = GPIO_PORTB; break;
        case 2: gpio_port = GPIO_PORTC; break;
        case 3: gpio_port = GPIO_PORTD; break;
        default: return 0;
    }
    
    return gpio_port->data & 0xFF;
}

// Write entire port (8 bits)
void gpio_write_port(int port, uint8_t value) {
    gpio_regs_t *gpio_port;
    
    switch (port) {
        case 0: gpio_port = GPIO_PORTA; break;
        case 1: gpio_port = GPIO_PORTB; break;
        case 2: gpio_port = GPIO_PORTC; break;
        case 3: gpio_port = GPIO_PORTD; break;
        default: return;
    }
    
    gpio_port->data = (gpio_port->data & 0xFFFFFF00) | value;
}

// Modify multiple pins atomically
void gpio_modify_pins(int port, uint32_t pin_mask, uint32_t pin_values) {
    gpio_regs_t *gpio_port;
    
    switch (port) {
        case 0: gpio_port = GPIO_PORTA; break;
        case 1: gpio_port = GPIO_PORTB; break;
        case 2: gpio_port = GPIO_PORTC; break;
        case 3: gpio_port = GPIO_PORTD; break;
        default: return;
    }
    
    // Atomic read-modify-write
    uint32_t current_data = gpio_port->data;
    current_data = (current_data & ~pin_mask) | (pin_values & pin_mask);
    gpio_port->data = current_data;
}
```

---

## 14.3 Alternate Function Multiplexing

### 14.3.1 Alternate Function Configuration

**Function Multiplexing Table:**
```
Pin    | GPIO | Alt Func 1 | Alt Func 2 | Alt Func 3
-------|------|------------|------------|------------
PA0    | GPIO | UART0_TX   | -          | -
PA1    | GPIO | UART0_RX   | -          | -
PA2    | GPIO | UART0_RTS  | -          | -
PA3    | GPIO | UART0_CTS  | -          | -
PA4    | GPIO | UART1_TX   | -          | -
PA5    | GPIO | UART1_RX   | -          | -
PA6    | GPIO | -          | -          | -
PA7    | GPIO | -          | -          | -

PB0    | GPIO | SPI0_SCLK  | -          | -
PB1    | GPIO | SPI0_MOSI  | -          | -
PB2    | GPIO | SPI0_MISO  | -          | -
PB3    | GPIO | SPI0_CS0   | -          | -
PB4    | GPIO | SPI0_CS1   | -          | -
PB5    | GPIO | SPI1_SCLK  | -          | -
PB6    | GPIO | SPI1_MOSI  | -          | -
PB7    | GPIO | SPI1_MISO  | -          | -

PC0    | GPIO | I2C0_SDA   | -          | -
PC1    | GPIO | I2C0_SCL   | -          | -
PC2    | GPIO | I2C1_SDA   | -          | -
PC3    | GPIO | I2C1_SCL   | -          | -
PC4    | GPIO | -          | -          | -
PC5    | GPIO | -          | -          | -
PC6    | GPIO | -          | -          | -
PC7    | GPIO | -          | -          | -

PD0    | GPIO | TMR0_OUT   | PWM0       | -
PD1    | GPIO | TMR1_OUT   | PWM1       | -
PD2    | GPIO | TMR2_OUT   | PWM2       | -
PD3    | GPIO | TMR3_OUT   | PWM3       | -
PD4    | GPIO | TMR0_CAP   | -          | -
PD5    | GPIO | TMR1_CAP   | -          | -
PD6    | GPIO | -          | -          | -
PD7    | GPIO | -          | -          | -
```

**Alternate Function Setup:**
```c
// Alternate function configuration
typedef enum {
    GPIO_FUNC_GPIO = 0,    // Regular GPIO
    GPIO_FUNC_ALT1 = 1,    // Alternate function 1
    GPIO_FUNC_ALT2 = 2,    // Alternate function 2  
    GPIO_FUNC_ALT3 = 3     // Alternate function 3
} gpio_function_t;

void gpio_set_alternate_function(int port, int pin, gpio_function_t function) {
    gpio_regs_t *gpio_port;
    
    switch (port) {
        case 0: gpio_port = GPIO_PORTA; break;
        case 1: gpio_port = GPIO_PORTB; break;
        case 2: gpio_port = GPIO_PORTC; break;
        case 3: gpio_port = GPIO_PORTD; break;
        default: return;
    }
    
    // Configure alternate function (2 bits per pin)
    uint32_t func_mask = ~(3 << (pin * 2));
    uint32_t func_value = function << (pin * 2);
    gpio_port->alternate_func = (gpio_port->alternate_func & func_mask) | func_value;
}

// UART0 initialization example
void init_uart0_pins(void) {
    // Configure PA0 as UART0_TX (output)
    gpio_init_pin(0, 0, 1, GPIO_PULL_UP);  // Output with pull-up
    gpio_set_alternate_function(0, 0, GPIO_FUNC_ALT1);
    
    // Configure PA1 as UART0_RX (input)
    gpio_init_pin(0, 1, 0, GPIO_PULL_UP);  // Input with pull-up
    gpio_set_alternate_function(0, 1, GPIO_FUNC_ALT1);
    
    // Configure flow control if needed
    gpio_init_pin(0, 2, 1, GPIO_PULL_UP);  // RTS output
    gpio_set_alternate_function(0, 2, GPIO_FUNC_ALT1);
    
    gpio_init_pin(0, 3, 0, GPIO_PULL_UP);  // CTS input
    gpio_set_alternate_function(0, 3, GPIO_FUNC_ALT1);
}

// SPI0 initialization example
void init_spi0_pins(void) {
    // Configure SPI0 pins
    gpio_init_pin(1, 0, 1, GPIO_PULL_NONE); // SCLK output
    gpio_set_alternate_function(1, 0, GPIO_FUNC_ALT1);
    
    gpio_init_pin(1, 1, 1, GPIO_PULL_NONE); // MOSI output
    gpio_set_alternate_function(1, 1, GPIO_FUNC_ALT1);
    
    gpio_init_pin(1, 2, 0, GPIO_PULL_UP);   // MISO input
    gpio_set_alternate_function(1, 2, GPIO_FUNC_ALT1);
    
    gpio_init_pin(1, 3, 1, GPIO_PULL_UP);   // CS0 output
    gpio_set_alternate_function(1, 3, GPIO_FUNC_ALT1);
    
    // Set higher drive strength for clock and data lines
    GPIO_PORTB->drive_strength |= (GPIO_DRIVE_8MA << (0 * 2)); // SCLK
    GPIO_PORTB->drive_strength |= (GPIO_DRIVE_8MA << (1 * 2)); // MOSI
    GPIO_PORTB->slew_rate |= (1 << 0) | (1 << 1); // Fast slew rate
}

// I2C0 initialization example
void init_i2c0_pins(void) {
    // Configure I2C0 pins (open-drain with pull-ups)
    gpio_init_pin(2, 0, 1, GPIO_PULL_UP);   // SDA
    gpio_set_alternate_function(2, 0, GPIO_FUNC_ALT1);
    
    gpio_init_pin(2, 1, 1, GPIO_PULL_UP);   // SCL
    gpio_set_alternate_function(2, 1, GPIO_FUNC_ALT1);
    
    // Configure for open-drain operation (implementation specific)
    // Set moderate drive strength for I2C
    GPIO_PORTC->drive_strength |= (GPIO_DRIVE_4MA << (0 * 2)); // SDA
    GPIO_PORTC->drive_strength |= (GPIO_DRIVE_4MA << (1 * 2)); // SCL
}
```

---

## 14.4 GPIO Interrupts

### 14.4.1 Interrupt Configuration

**GPIO Interrupt Setup:**
```c
// GPIO interrupt configuration
void gpio_configure_interrupt(int port, int pin, int int_type, int polarity, 
                             gpio_callback_t callback) {
    gpio_regs_t *gpio_port;
    
    switch (port) {
        case 0: gpio_port = GPIO_PORTA; break;
        case 1: gpio_port = GPIO_PORTB; break;
        case 2: gpio_port = GPIO_PORTC; break;
        case 3: gpio_port = GPIO_PORTD; break;
        default: return;
    }
    
    // Disable interrupt during configuration
    gpio_port->interrupt_enable &= ~(1 << pin);
    
    // Configure interrupt type (edge vs level)
    if (int_type == GPIO_INT_EDGE) {
        gpio_port->interrupt_type |= (1 << pin);
    } else {
        gpio_port->interrupt_type &= ~(1 << pin);
    }
    
    // Configure polarity
    if (polarity == GPIO_INT_HIGH_RISE) {
        gpio_port->interrupt_polarity |= (1 << pin);
    } else {
        gpio_port->interrupt_polarity &= ~(1 << pin);
    }
    
    // Register callback function
    register_gpio_callback(port, pin, callback);
    
    // Clear any pending interrupt
    gpio_port->interrupt_status = (1 << pin);
    
    // Enable interrupt
    gpio_port->interrupt_enable |= (1 << pin);
}

// GPIO interrupt handler
void gpio_interrupt_handler(int port) {
    gpio_regs_t *gpio_port;
    
    switch (port) {
        case 0: gpio_port = GPIO_PORTA; break;
        case 1: gpio_port = GPIO_PORTB; break;
        case 2: gpio_port = GPIO_PORTC; break;
        case 3: gpio_port = GPIO_PORTD; break;
        default: return;
    }
    
    uint32_t int_status = gpio_port->interrupt_status & gpio_port->interrupt_enable;
    
    // Process each pending interrupt
    for (int pin = 0; pin < 8; pin++) {
        if (int_status & (1 << pin)) {
            // Call registered callback
            gpio_callback_t callback = get_gpio_callback(port, pin);
            if (callback != NULL) {
                callback(port, pin);
            }
            
            // Clear interrupt (for edge-triggered interrupts)
            if (gpio_port->interrupt_type & (1 << pin)) {
                gpio_port->interrupt_status = (1 << pin);
            }
        }
    }
}
```

### 14.4.2 Interrupt Examples

**Button Input with Debouncing:**
```c
// Button debounce structure
typedef struct {
    uint32_t last_time;
    int last_state;
    int stable_state;
    int debounce_time_ms;
} button_debounce_t;

static button_debounce_t button_states[32];

void button_interrupt_callback(int port, int pin) {
    int button_id = port * 8 + pin;
    button_debounce_t *button = &button_states[button_id];
    
    uint32_t current_time = get_system_time_ms();
    int current_state = gpio_read_pin(port, pin);
    
    // Debounce logic
    if (current_state != button->last_state) {
        button->last_time = current_time;
        button->last_state = current_state;
    } else if ((current_time - button->last_time) > button->debounce_time_ms) {
        if (current_state != button->stable_state) {
            button->stable_state = current_state;
            
            // Process stable button state change
            if (current_state == 0) {  // Button pressed (active low)
                handle_button_press(button_id);
            } else {  // Button released
                handle_button_release(button_id);
            }
        }
    }
}

// Initialize button with interrupt
void init_button(int port, int pin, int debounce_ms) {
    int button_id = port * 8 + pin;
    
    // Configure pin as input with pull-up
    gpio_init_pin(port, pin, 0, GPIO_PULL_UP);
    
    // Setup debounce parameters
    button_states[button_id].debounce_time_ms = debounce_ms;
    button_states[button_id].last_state = 1;      // Assume button not pressed
    button_states[button_id].stable_state = 1;
    
    // Configure interrupt on both edges
    gpio_configure_interrupt(port, pin, GPIO_INT_EDGE, GPIO_INT_LOW_FALL, 
                           button_interrupt_callback);
}

// Rotary encoder handling
typedef struct {
    int last_a, last_b;
    int position;
    int direction;
} rotary_encoder_t;

static rotary_encoder_t encoder;

void rotary_encoder_callback(int port, int pin) {
    int a_state = gpio_read_pin(port, 0);  // Encoder A
    int b_state = gpio_read_pin(port, 1);  // Encoder B
    
    // Gray code decoding
    if (a_state != encoder.last_a) {
        if (a_state == b_state) {
            encoder.position++;
            encoder.direction = 1;  // Clockwise
        } else {
            encoder.position--;
            encoder.direction = -1; // Counter-clockwise
        }
        
        // Notify application of encoder change
        handle_encoder_change(encoder.position, encoder.direction);
    }
    
    encoder.last_a = a_state;
    encoder.last_b = b_state;
}

void init_rotary_encoder(int port) {
    // Configure encoder pins A and B
    gpio_init_pin(port, 0, 0, GPIO_PULL_UP);  // Encoder A
    gpio_init_pin(port, 1, 0, GPIO_PULL_UP);  // Encoder B
    
    // Setup interrupts on both pins for both edges
    gpio_configure_interrupt(port, 0, GPIO_INT_EDGE, GPIO_INT_HIGH_RISE, 
                           rotary_encoder_callback);
    gpio_configure_interrupt(port, 1, GPIO_INT_EDGE, GPIO_INT_HIGH_RISE, 
                           rotary_encoder_callback);
    
    // Initialize encoder state
    encoder.last_a = gpio_read_pin(port, 0);
    encoder.last_b = gpio_read_pin(port, 1);
    encoder.position = 0;
}
```

---

## 14.5 Specialized I/O Interfaces

### 14.5.1 Pulse Width Modulation (PWM)

**PWM Configuration:**
```c
#define PWM_BASE            0x40002000

typedef volatile struct {
    uint32_t control;           /* 0x00: PWM control register */
    uint32_t period;            /* 0x04: PWM period (total cycle time) */
    uint32_t duty_cycle;        /* 0x08: PWM duty cycle (on time) */
    uint32_t prescaler;         /* 0x0C: Clock prescaler */
    uint32_t status;            /* 0x10: PWM status */
    uint32_t interrupt;         /* 0x14: Interrupt control */
} pwm_channel_t;

#define PWM_CHANNEL(n) ((pwm_channel_t*)(PWM_BASE + (n) * 0x20))

// PWM control bits
#define PWM_ENABLE          (1 << 0)
#define PWM_OUTPUT_ENABLE   (1 << 1)
#define PWM_INVERT_OUTPUT   (1 << 2)
#define PWM_CENTER_ALIGNED  (1 << 3)
#define PWM_ONE_SHOT        (1 << 4)

// PWM initialization
void pwm_init_channel(int channel, uint32_t frequency, uint32_t duty_percent) {
    pwm_channel_t *pwm = PWM_CHANNEL(channel);
    
    // Calculate period for desired frequency
    // Assuming 100MHz system clock
    uint32_t system_clock = 100000000;
    uint32_t prescaler_value = 1;
    uint32_t period_value = system_clock / (frequency * prescaler_value);
    
    // Adjust prescaler if period is too large
    while (period_value > 65535 && prescaler_value < 256) {
        prescaler_value *= 2;
        period_value = system_clock / (frequency * prescaler_value);
    }
    
    // Calculate duty cycle value
    uint32_t duty_value = (period_value * duty_percent) / 100;
    
    // Configure PWM channel
    pwm->control = 0;                    // Disable during setup
    pwm->prescaler = prescaler_value - 1;
    pwm->period = period_value - 1;
    pwm->duty_cycle = duty_value;
    
    // Enable PWM output
    pwm->control = PWM_ENABLE | PWM_OUTPUT_ENABLE;
    
    // Configure GPIO pin for PWM output
    int port = 3;  // Port D
    int pin = channel;
    gpio_init_pin(port, pin, 1, GPIO_PULL_NONE);
    gpio_set_alternate_function(port, pin, GPIO_FUNC_ALT2);  // PWM function
    
    // Set appropriate drive strength for PWM
    GPIO_PORTD->drive_strength |= (GPIO_DRIVE_8MA << (pin * 2));
    GPIO_PORTD->slew_rate |= (1 << pin);  // Fast slew rate
}

// PWM duty cycle update
void pwm_set_duty_cycle(int channel, uint32_t duty_percent) {
    pwm_channel_t *pwm = PWM_CHANNEL(channel);
    uint32_t period = pwm->period + 1;
    uint32_t duty_value = (period * duty_percent) / 100;
    pwm->duty_cycle = duty_value;
}

// Servo motor control using PWM
void servo_set_angle(int servo_channel, int angle_degrees) {
    // Standard servo: 1-2ms pulse width, 20ms period
    // 1ms = 0°, 1.5ms = 90°, 2ms = 180°
    
    if (angle_degrees < 0) angle_degrees = 0;
    if (angle_degrees > 180) angle_degrees = 180;
    
    // Calculate pulse width (1000-2000 microseconds)
    uint32_t pulse_width_us = 1000 + ((angle_degrees * 1000) / 180);
    
    // Convert to duty cycle percentage for 20ms period
    uint32_t duty_percent = (pulse_width_us * 100) / 20000;
    
    pwm_set_duty_cycle(servo_channel, duty_percent);
}
```

### 14.5.2 Analog-to-Digital Converter Interface

**ADC GPIO Configuration:**
```c
#define ADC_BASE            0x40003000

typedef volatile struct {
    uint32_t control;           /* 0x00: ADC control */
    uint32_t status;            /* 0x04: ADC status */
    uint32_t data;              /* 0x08: ADC conversion result */
    uint32_t channel_select;    /* 0x0C: Channel selection */
    uint32_t sample_time;       /* 0x10: Sample and hold time */
    uint32_t threshold_low;     /* 0x14: Low threshold for comparator */
    uint32_t threshold_high;    /* 0x18: High threshold for comparator */
    uint32_t interrupt;         /* 0x1C: Interrupt control */
} adc_regs_t;

#define ADC ((adc_regs_t*)ADC_BASE)

// ADC control bits
#define ADC_ENABLE          (1 << 0)
#define ADC_START_CONV      (1 << 1)
#define ADC_CONTINUOUS      (1 << 2)
#define ADC_SINGLE_SHOT     (0 << 2)

// Configure GPIO pins for analog input
void adc_init_channel(int channel) {
    // ADC channels are mapped to specific GPIO pins
    int port = channel / 8;
    int pin = channel % 8;
    
    // Configure GPIO pin for analog input
    gpio_init_pin(port, pin, 0, GPIO_PULL_NONE);  // Input, no pulls
    
    // Set pin to analog mode (disable digital input buffer)
    GPIO_PORTA->input_filter &= ~(1 << pin);  // Disable input filter
    
    // Configure for minimum drive strength to reduce power
    GPIO_PORTA->drive_strength &= ~(3 << (pin * 2));
}

// Read ADC value
uint16_t adc_read_channel(int channel) {
    // Select ADC channel
    ADC->channel_select = channel;
    
    // Start conversion
    ADC->control = ADC_ENABLE | ADC_START_CONV | ADC_SINGLE_SHOT;
    
    // Wait for conversion complete
    while (!(ADC->status & (1 << 0))) {
        // Conversion in progress
    }
    
    // Read result (12-bit ADC)
    return ADC->data & 0xFFF;
}

// Temperature sensor reading
float read_temperature_sensor(int temp_channel) {
    uint16_t adc_value = adc_read_channel(temp_channel);
    
    // Convert ADC reading to temperature
    // Assuming linear temperature sensor: 10mV/°C, 500mV at 0°C
    // ADC reference voltage: 3.3V, 12-bit resolution
    float voltage = (adc_value * 3.3f) / 4095.0f;
    float temperature = (voltage - 0.5f) / 0.01f;  // °C
    
    return temperature;
}

// Battery voltage monitoring
float read_battery_voltage(int battery_channel) {
    uint16_t adc_value = adc_read_channel(battery_channel);
    
    // Convert to voltage (assuming voltage divider for battery monitoring)
    // Voltage divider: R1=10k, R2=10k (divide by 2)
    float measured_voltage = (adc_value * 3.3f) / 4095.0f;
    float battery_voltage = measured_voltage * 2.0f;  // Compensate for divider
    
    return battery_voltage;
}
```

### 14.5.3 Quadrature Encoder Interface

**Dedicated Quadrature Decoder:**
```c
#define QUAD_ENC_BASE       0x40004000

typedef volatile struct {
    uint32_t control;           /* 0x00: Encoder control */
    uint32_t position;          /* 0x04: Current position count */
    uint32_t velocity;          /* 0x08: Velocity measurement */
    uint32_t index_count;       /* 0x0C: Index pulse count */
    uint32_t max_position;      /* 0x10: Maximum position (rollover) */
    uint32_t filter;            /* 0x14: Input filter configuration */
    uint32_t interrupt;         /* 0x18: Interrupt control */
    uint32_t status;            /* 0x1C: Status register */
} quad_encoder_t;

#define QUAD_ENC(n) ((quad_encoder_t*)(QUAD_ENC_BASE + (n) * 0x20))

// Encoder control bits
#define QUAD_ENABLE         (1 << 0)
#define QUAD_X2_MODE        (1 << 1)    // 2x decoding
#define QUAD_X4_MODE        (1 << 2)    // 4x decoding
#define QUAD_INDEX_ENABLE   (1 << 3)    // Enable index pulse
#define QUAD_DIRECTION_INV  (1 << 4)    // Invert direction

// Initialize quadrature encoder
void quad_encoder_init(int encoder_num, int port_a, int pin_a, 
                      int port_b, int pin_b) {
    quad_encoder_t *quad = QUAD_ENC(encoder_num);
    
    // Configure GPIO pins for encoder inputs
    gpio_init_pin(port_a, pin_a, 0, GPIO_PULL_UP);  // Phase A
    gpio_init_pin(port_b, pin_b, 0, GPIO_PULL_UP);  // Phase B
    
    // Set alternate function for quadrature decoder
    gpio_set_alternate_function(port_a, pin_a, GPIO_FUNC_ALT3);
    gpio_set_alternate_function(port_b, pin_b, GPIO_FUNC_ALT3);
    
    // Configure encoder for 4x decoding with filtering
    quad->control = QUAD_ENABLE | QUAD_X4_MODE;
    quad->filter = 0x03;     // Digital filter to reject noise
    quad->position = 0;      // Reset position
    quad->max_position = 0xFFFFFFFF;  // No rollover limit
}

// Read encoder position
int32_t quad_encoder_read_position(int encoder_num) {
    quad_encoder_t *quad = QUAD_ENC(encoder_num);
    return (int32_t)quad->position;
}

// Read encoder velocity (counts per measurement period)
int16_t quad_encoder_read_velocity(int encoder_num) {
    quad_encoder_t *quad = QUAD_ENC(encoder_num);
    return (int16_t)quad->velocity;
}

// Reset encoder position
void quad_encoder_reset(int encoder_num) {
    quad_encoder_t *quad = QUAD_ENC(encoder_num);
    quad->position = 0;
}
```

---

## 14.6 I/O Performance and Timing

### 14.6.1 GPIO Speed Characteristics

**GPIO Timing Specifications:**
```
Parameter               | Min    | Typ    | Max    | Unit  | Notes
------------------------|--------|--------|--------|-------|-------
Input Setup Time        | 2      | -      | -      | ns    | Before clock edge
Input Hold Time         | 1      | -      | -      | ns    | After clock edge
Output Propagation Delay| -      | 5      | 10     | ns    | Clock to output
Rise Time (2mA drive)   | -      | 8      | 15     | ns    | 10%-90%, 50pF load
Rise Time (12mA drive)  | -      | 2      | 4      | ns    | 10%-90%, 50pF load
Fall Time (2mA drive)   | -      | 6      | 12     | ns    | 90%-10%, 50pF load
Fall Time (12mA drive)  | -      | 1.5    | 3      | ns    | 90%-10%, 50pF load
Maximum Toggle Rate     | -      | 50     | -      | MHz   | 12mA drive, min load
Input Leakage Current   | -1     | -      | +1     | µA    | -
Pull-up Resistance      | 20     | 50     | 100    | kΩ    | -
Pull-down Resistance    | 20     | 50     | 100    | kΩ    | -
```

**High-Speed GPIO Operations:**
```c
// High-speed bit manipulation macros
#define GPIO_FAST_SET(port_reg, pin)    (port_reg)->data |= (1 << (pin))
#define GPIO_FAST_CLR(port_reg, pin)    (port_reg)->data &= ~(1 << (pin))
#define GPIO_FAST_TOG(port_reg, pin)    (port_reg)->data ^= (1 << (pin))
#define GPIO_FAST_READ(port_reg, pin)   (((port_reg)->data >> (pin)) & 1)

// High-speed SPI bit-bang implementation
void spi_bitbang_transfer(uint8_t data) {
    gpio_regs_t *port = GPIO_PORTB;
    
    // Configure for maximum speed
    port->drive_strength |= (GPIO_DRIVE_12MA << (0 * 2)); // SCLK
    port->drive_strength |= (GPIO_DRIVE_12MA << (1 * 2)); // MOSI
    port->slew_rate |= (1 << 0) | (1 << 1);              // Fast slew
    
    for (int bit = 7; bit >= 0; bit--) {
        // Set data bit (MOSI = PB1)
        if (data & (1 << bit)) {
            GPIO_FAST_SET(port, 1);
        } else {
            GPIO_FAST_CLR(port, 1);
        }
        
        // Clock high (SCLK = PB0)
        GPIO_FAST_SET(port, 0);
        __asm volatile ("nop"); // Short delay for setup time
        
        // Clock low
        GPIO_FAST_CLR(port, 0);
    }
}

// Fast parallel port implementation (8-bit data bus)
void parallel_write_byte(uint8_t data) {
    // Write entire byte to port A atomically
    GPIO_PORTA->data = (GPIO_PORTA->data & 0xFFFFFF00) | data;
    
    // Strobe write signal (assuming strobe on another pin)
    GPIO_FAST_SET(GPIO_PORTB, 7);  // Strobe high
    __asm volatile ("nop; nop");   // Minimum strobe width
    GPIO_FAST_CLR(GPIO_PORTB, 7);  // Strobe low
}

uint8_t parallel_read_byte(void) {
    // Read entire byte from port A
    return GPIO_PORTA->data & 0xFF;
}
```

### 14.6.2 I/O Power Optimization

**Power-Efficient I/O Configuration:**
```c
// Low-power GPIO configuration
void gpio_configure_low_power(void) {
    // Set unused pins as inputs with pull-downs to avoid floating
    for (int port = 0; port < 4; port++) {
        gpio_regs_t *gpio_port;
        switch (port) {
            case 0: gpio_port = GPIO_PORTA; break;
            case 1: gpio_port = GPIO_PORTB; break;
            case 2: gpio_port = GPIO_PORTC; break;
            case 3: gpio_port = GPIO_PORTD; break;
        }
        
        // Get mask of pins actually used
        uint32_t used_pins = get_used_pin_mask(port);
        uint32_t unused_pins = ~used_pins & 0xFF;
        
        // Configure unused pins
        gpio_port->direction &= ~unused_pins;        // Set as inputs
        
        // Set pull-downs on unused pins to avoid leakage
        for (int pin = 0; pin < 8; pin++) {
            if (unused_pins & (1 << pin)) {
                uint32_t pull_mask = ~(3 << (pin * 2));
                uint32_t pull_value = GPIO_PULL_DOWN << (pin * 2);
                gpio_port->pull_config = (gpio_port->pull_config & pull_mask) | pull_value;
            }
        }
        
        // Disable input filters on unused pins to save power
        gpio_port->input_filter &= ~unused_pins;
    }
}

// Sleep mode I/O configuration
void gpio_prepare_for_sleep(void) {
    // Save current GPIO configuration
    save_gpio_configuration();
    
    // Configure wake-up pins
    configure_wakeup_pins();
    
    // Set all other outputs to safe states
    set_safe_output_states();
    
    // Reduce drive strength on all outputs
    reduce_all_drive_strengths();
}

void gpio_restore_from_sleep(void) {
    // Restore GPIO configuration
    restore_gpio_configuration();
    
    // Re-enable normal drive strengths
    restore_drive_strengths();
}
```

---

*This chapter provided comprehensive coverage of MCU-32X GPIO and I/O capabilities, enabling efficient interfacing with both embedded peripherals and desktop computing devices typical of late-1990s systems.*