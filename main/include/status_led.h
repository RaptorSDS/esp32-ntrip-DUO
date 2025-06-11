#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include <sys/queue.h>

// Status LED flashing modes
typedef enum {
    STATUS_LED_STATIC = 0,
    STATUS_LED_FLASH,
    STATUS_LED_FADE,
    STATUS_LED_BLINK = STATUS_LED_FLASH  // Alias für Rückwärtskompatibilität
} status_led_flashing_mode_t;

// Status LED color structure
struct status_led_color_t {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    
    status_led_flashing_mode_t flashing_mode;
    uint32_t interval;
    uint32_t duration;
    uint8_t expire;
    
    bool active;
    bool remove;
    
    SLIST_ENTRY(status_led_color_t) next;
};

typedef struct status_led_color_t* status_led_handle_t;

// GPIO info structure
typedef struct {
    gpio_num_t red_gpio;
    gpio_num_t green_gpio;
    gpio_num_t blue_gpio;
    gpio_num_t sleep_gpio;
    gpio_num_t rssi_gpio;
    gpio_num_t assoc_gpio;
    bool has_high_speed_mode;
} status_led_gpio_info_t;

// Function prototypes
void status_led_init(void);
void status_led_deinit(void);
void status_led_clear(void);

status_led_handle_t status_led_add(uint32_t rgba, status_led_flashing_mode_t flashing_mode, 
                                  uint32_t interval, uint32_t duration, uint8_t expire);
void status_led_remove(status_led_handle_t color);

void rssi_led_set(uint8_t value);
void rssi_led_fade(uint8_t value, int max_fade_time_ms);

void assoc_led_set(uint8_t value);
void assoc_led_fade(uint8_t value, int max_fade_time_ms);

void sleep_led_set(uint8_t value);
void sleep_led_fade(uint8_t value, int max_fade_time_ms);

// Utility functions
const char* status_led_get_chip_info(void);
void status_led_get_gpio_info(status_led_gpio_info_t *info);
void status_led_test_sequence(void);

// Backward compatibility macros
#define STATUS_LED_BLINK STATUS_LED_FLASH

#endif // STATUS_LED_H
