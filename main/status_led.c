#include "status_led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include <sys/queue.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "status_led";

// Task priority
#define TASK_PRIORITY_STATUS_LED 5

// ESP32-S3 spezifische Konfiguration
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    #define CHIP_ESP32_S3
    #define HAS_HIGH_SPEED_MODE 0
    #define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
    #define LEDC_TIMER_MODE LEDC_TIMER_0
    
    // ESP32-S3 GPIO Zuordnung (angepasst für verfügbare GPIOs)
    #define STATUS_LED_RED_GPIO   GPIO_NUM_38
    #define STATUS_LED_GREEN_GPIO GPIO_NUM_39
    #define STATUS_LED_BLUE_GPIO  GPIO_NUM_40
    #define STATUS_LED_SLEEP_GPIO GPIO_NUM_41
    #define STATUS_LED_RSSI_GPIO  GPIO_NUM_42
    #define STATUS_LED_ASSOC_GPIO GPIO_NUM_2
    
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    #define CHIP_ESP32_S2
    #define HAS_HIGH_SPEED_MODE 0
    #define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
    #define LEDC_TIMER_MODE LEDC_TIMER_0
    
    #define STATUS_LED_RED_GPIO   GPIO_NUM_26
    #define STATUS_LED_GREEN_GPIO GPIO_NUM_25
    #define STATUS_LED_BLUE_GPIO  GPIO_NUM_27
    #define STATUS_LED_SLEEP_GPIO GPIO_NUM_33
    #define STATUS_LED_RSSI_GPIO  GPIO_NUM_32
    #define STATUS_LED_ASSOC_GPIO GPIO_NUM_21
    
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
    #define CHIP_ESP32_C3
    #define HAS_HIGH_SPEED_MODE 0
    #define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
    #define LEDC_TIMER_MODE LEDC_TIMER_0
    
    #define STATUS_LED_RED_GPIO   GPIO_NUM_3
    #define STATUS_LED_GREEN_GPIO GPIO_NUM_4
    #define STATUS_LED_BLUE_GPIO  GPIO_NUM_5
    #define STATUS_LED_SLEEP_GPIO GPIO_NUM_6
    #define STATUS_LED_RSSI_GPIO  GPIO_NUM_7
    #define STATUS_LED_ASSOC_GPIO GPIO_NUM_8
    
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
    #define CHIP_ESP32_C6
    #define HAS_HIGH_SPEED_MODE 0
    #define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
    #define LEDC_TIMER_MODE LEDC_TIMER_0
    
    #define STATUS_LED_RED_GPIO   GPIO_NUM_3
    #define STATUS_LED_GREEN_GPIO GPIO_NUM_4
    #define STATUS_LED_BLUE_GPIO  GPIO_NUM_5
    #define STATUS_LED_SLEEP_GPIO GPIO_NUM_6
    #define STATUS_LED_RSSI_GPIO  GPIO_NUM_7
    #define STATUS_LED_ASSOC_GPIO GPIO_NUM_8
    
#else // ESP32 Classic
    #define CHIP_ESP32_CLASSIC
    #define HAS_HIGH_SPEED_MODE 1
    #define LEDC_SPEED_MODE LEDC_HIGH_SPEED_MODE
    #define LEDC_TIMER_MODE LEDC_TIMER_0
    
    #define STATUS_LED_RED_GPIO   GPIO_NUM_26
    #define STATUS_LED_GREEN_GPIO GPIO_NUM_22
    #define STATUS_LED_BLUE_GPIO  GPIO_NUM_23
    #define STATUS_LED_SLEEP_GPIO GPIO_NUM_27
    #define STATUS_LED_RSSI_GPIO  GPIO_NUM_32
    #define STATUS_LED_ASSOC_GPIO GPIO_NUM_25
#endif

// LEDC channels
#define STATUS_LED_RED_CHANNEL   LEDC_CHANNEL_0
#define STATUS_LED_GREEN_CHANNEL LEDC_CHANNEL_1
#define STATUS_LED_BLUE_CHANNEL  LEDC_CHANNEL_2
#define STATUS_LED_SLEEP_CHANNEL LEDC_CHANNEL_3
#define STATUS_LED_RSSI_CHANNEL  LEDC_CHANNEL_4
#define STATUS_LED_ASSOC_CHANNEL LEDC_CHANNEL_5

// LEDC configuration
#define LEDC_DUTY_RES    LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY   5000

// Global variables
static TaskHandle_t led_task = NULL;
static SemaphoreHandle_t led_mutex = NULL;
static SLIST_HEAD(status_led_list_head, status_led_color_t) status_led_list = SLIST_HEAD_INITIALIZER(status_led_list);

// Function prototypes
static void status_led_task(void *pvParameters);
static void status_led_show(status_led_handle_t color);
static void status_led_set(uint8_t red, uint8_t green, uint8_t blue);
static void status_led_fade(uint8_t red, uint8_t green, uint8_t blue, int max_fade_time_ms);
static void status_led_channel_set(ledc_channel_t channel, uint8_t value);
static void status_led_channel_fade(ledc_channel_t channel, uint8_t value, int max_fade_time_ms);
static esp_err_t configure_ledc_channel(ledc_channel_t channel, gpio_num_t gpio_num, uint32_t duty);

// Task implementation
static void status_led_task(void *pvParameters) {
    status_led_handle_t color;
    
    while (1) {
        if (xSemaphoreTake(led_mutex, portMAX_DELAY) == pdTRUE) {
            SLIST_FOREACH(color, &status_led_list, next) {
                if (color->remove) {
                    SLIST_REMOVE(&status_led_list, color, status_led_color_t, next);
                    free(color);
                    continue;
                }
                
                if (!color->active) {
                    color->active = true;
                    status_led_show(color);
                    
                    if (color->expire > 0) {
                        color->expire--;
                        if (color->expire == 0) {
                            color->remove = true;
                        }
                    }
                }
            }
            xSemaphoreGive(led_mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void status_led_show(status_led_handle_t color) {
    if (color == NULL) return;

    if (color->flashing_mode == STATUS_LED_STATIC) {
        status_led_set(color->red, color->green, color->blue);
        vTaskDelay(pdMS_TO_TICKS(color->duration));
    } else {
        bool fade = (color->flashing_mode == STATUS_LED_FADE);
        bool active = true;
        uint32_t cycles = color->duration / color->interval;
        
        for (uint32_t i = 0; i < cycles && !color->remove; i++, active = !active) {
            uint8_t red = active ? color->red : 0;
            uint8_t green = active ? color->green : 0;
            uint8_t blue = active ? color->blue : 0;
            
            if (fade) {
                status_led_fade(red, green, blue, color->interval / 2);
            } else {
                status_led_set(red, green, blue);
            }

            vTaskDelay(pdMS_TO_TICKS(color->interval));
        }
    }

    // Turn off all LEDs
    status_led_set(0, 0, 0);
    color->active = false;
}

static void status_led_set(uint8_t red, uint8_t green, uint8_t blue) {
    status_led_channel_set(STATUS_LED_RED_CHANNEL, red);
    status_led_channel_set(STATUS_LED_GREEN_CHANNEL, green);
    status_led_channel_set(STATUS_LED_BLUE_CHANNEL, blue);
}

static void status_led_fade(uint8_t red, uint8_t green, uint8_t blue, int max_fade_time_ms) {
    status_led_channel_fade(STATUS_LED_RED_CHANNEL, red, max_fade_time_ms);
    status_led_channel_fade(STATUS_LED_GREEN_CHANNEL, green, max_fade_time_ms);
    status_led_channel_fade(STATUS_LED_BLUE_CHANNEL, blue, max_fade_time_ms);
}

static void status_led_channel_set(ledc_channel_t channel, uint8_t value) {
    esp_err_t err = ledc_set_duty(LEDC_SPEED_MODE, channel, value);
    if (err == ESP_OK) {
        ledc_update_duty(LEDC_SPEED_MODE, channel);
    } else {
        ESP_LOGE(TAG, "Failed to set duty for channel %d: %s", channel, esp_err_to_name(err));
    }
}

static void status_led_channel_fade(ledc_channel_t channel, uint8_t value, int max_fade_time_ms) {
    esp_err_t err = ledc_set_fade_with_time(LEDC_SPEED_MODE, channel, value, max_fade_time_ms);
    if (err == ESP_OK) {
        ledc_fade_start(LEDC_SPEED_MODE, channel, LEDC_FADE_NO_WAIT);
    } else {
        ESP_LOGE(TAG, "Failed to set fade for channel %d: %s", channel, esp_err_to_name(err));
        // Fallback to direct set
        status_led_channel_set(channel, value);
    }
}

// Public API implementation
void status_led_clear(void) {
    if (led_mutex && xSemaphoreTake(led_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        status_led_handle_t color, temp;
        SLIST_FOREACH_SAFE(color, &status_led_list, next, temp) {
            SLIST_REMOVE(&status_led_list, color, status_led_color_t, next);
            free(color);
        }
        xSemaphoreGive(led_mutex);
    }
    
    // Turn off all LEDs
    status_led_set(0, 0, 0);
}

status_led_handle_t status_led_add(uint32_t rgba, status_led_flashing_mode_t flashing_mode, 
                                  uint32_t interval, uint32_t duration, uint8_t expire) {
    status_led_handle_t color = malloc(sizeof(struct status_led_color_t));
    if (color == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for LED color");
        return NULL;
    }

    // Extract RGB values from RGBA
    color->red = (rgba >> 24) & 0xFF;
    color->green = (rgba >> 16) & 0xFF;
    color->blue = (rgba >> 8) & 0xFF;
    
    color->flashing_mode = flashing_mode;
    color->interval = interval;
    color->duration = duration;
    color->expire = expire;
    color->active = false;
    color->remove = false;

    if (led_mutex && xSemaphoreTake(led_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        SLIST_INSERT_HEAD(&status_led_list, color, next);
        xSemaphoreGive(led_mutex);
    } else {
        free(color);
        return NULL;
    }

    return color;
}

void status_led_remove(status_led_handle_t color) {
    if (color == NULL) return;
    
    if (led_mutex && xSemaphoreTake(led_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        color->remove = true;
        xSemaphoreGive(led_mutex);
    }
}

static esp_err_t configure_ledc_channel(ledc_channel_t channel, gpio_num_t gpio_num, uint32_t duty) {
    ledc_channel_config_t ledc_channel = {
        .channel    = channel,
        .duty       = duty,
        .gpio_num   = gpio_num,
        .speed_mode = LEDC_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_MODE,
        .flags.output_invert = 0
    };
    
    return ledc_channel_config(&ledc_channel);
}

void status_led_init(void) {
    ESP_LOGI(TAG, "Initializing Status LED for %s", status_led_get_chip_info());
    
    // Create mutex
    led_mutex = xSemaphoreCreateMutex();
    if (led_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create LED mutex");
        return;
    }

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_SPEED_MODE,
        .timer_num = LEDC_TIMER_MODE,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(err));
        return;
    }

    // Configure LED channels
    configure_ledc_channel(STATUS_LED_RED_CHANNEL, STATUS_LED_RED_GPIO, 255);
    configure_ledc_channel(STATUS_LED_GREEN_CHANNEL, STATUS_LED_GREEN_GPIO, 255);
    configure_ledc_channel(STATUS_LED_BLUE_CHANNEL, STATUS_LED_BLUE_GPIO, 255);
    configure_ledc_channel(STATUS_LED_SLEEP_CHANNEL, STATUS_LED_SLEEP_GPIO, 255);
    configure_ledc_channel(STATUS_LED_RSSI_CHANNEL, STATUS_LED_RSSI_GPIO, 0);
    configure_ledc_channel(STATUS_LED_ASSOC_CHANNEL, STATUS_LED_ASSOC_GPIO, 0);

    // Install fade function
    err = ledc_fade_func_install(0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to install LEDC fade function: %s", esp_err_to_name(err));
        // Continue without fade functionality
    }

    // Turn off all LEDs initially
    status_led_set(0, 0, 0);

    // Create LED task
    BaseType_t task_created = xTaskCreate(
        status_led_task,
        "status_led_task",
        4096,
        NULL,
        TASK_PRIORITY_STATUS_LED,
        &led_task
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create status LED task");
        vSemaphoreDelete(led_mutex);
        led_mutex = NULL;
        return;
    }

    ESP_LOGI(TAG, "Status LED initialized successfully");
}

void status_led_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing Status LED");

    // Delete task
    if (led_task != NULL) {
        vTaskDelete(led_task);
        led_task = NULL;
    }

    // Clear all LEDs
    status_led_clear();

    // Turn off all LEDs
    status_led_set(0, 0, 0);

    // Uninstall fade function
    ledc_fade_func_uninstall();

    // Delete mutex
    if (led_mutex != NULL) {
        vSemaphoreDelete(led_mutex);
        led_mutex = NULL;
    }

    ESP_LOGI(TAG, "Status LED deinitialized");
}

// RSSI LED functions
void rssi_led_set(uint8_t value) {
    status_led_channel_set(STATUS_LED_RSSI_CHANNEL, value);
}

void rssi_led_fade(uint8_t value, int max_fade_time_ms) {
    status_led_channel_fade(STATUS_LED_RSSI_CHANNEL, value, max_fade_time_ms);
}

// Association LED functions
void assoc_led_set(uint8_t value) {
    status_led_channel_set(STATUS_LED_ASSOC_CHANNEL, value);
}

void assoc_led_fade(uint8_t value, int max_fade_time_ms) {
    status_led_channel_fade(STATUS_LED_ASSOC_CHANNEL, value, max_fade_time_ms);
}

// Sleep LED functions
void sleep_led_set(uint8_t value) {
    status_led_channel_set(STATUS_LED_SLEEP_CHANNEL, value);
}

void sleep_led_fade(uint8_t value, int max_fade_time_ms) {
    status_led_channel_fade(STATUS_LED_SLEEP_CHANNEL, value, max_fade_time_ms);
}

// Utility functions
const char* status_led_get_chip_info(void) {
#if defined(CHIP_ESP32_S3)
    return "ESP32-S3";
#elif defined(CHIP_ESP32_S2)
    return "ESP32-S2";
#elif defined(CHIP_ESP32_C3)
    return "ESP32-C3";
#elif defined(CHIP_ESP32_C6)
    return "ESP32-C6";
#else
    return "ESP32";
#endif
}

void status_led_get_gpio_info(status_led_gpio_info_t *info) {
    if (info == NULL) return;

    info->red_gpio = STATUS_LED_RED_GPIO;
    info->green_gpio = STATUS_LED_GREEN_GPIO;
    info->blue_gpio = STATUS_LED_BLUE_GPIO;
    info->sleep_gpio = STATUS_LED_SLEEP_GPIO;
    info->rssi_gpio = STATUS_LED_RSSI_GPIO;
    info->assoc_gpio = STATUS_LED_ASSOC_GPIO;
    info->has_high_speed_mode = HAS_HIGH_SPEED_MODE;
}

void status_led_test_sequence(void) {
    ESP_LOGI(TAG, "Starting LED test sequence for %s", status_led_get_chip_info());
    
    // Test each color for 1 second
    ESP_LOGI(TAG, "Testing RED LED");
    status_led_set(255, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Testing GREEN LED");
    status_led_set(0, 255, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Testing BLUE LED");
    status_led_set(0, 0, 255);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Testing WHITE LED");
    status_led_set(255, 255, 255);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test fade functionality
    ESP_LOGI(TAG, "Testing FADE functionality");
    for (int i = 0; i < 3; i++) {
        status_led_fade(255, 255, 255, 500);
        vTaskDelay(pdMS_TO_TICKS(600));
        status_led_fade(0, 0, 0, 500);
        vTaskDelay(pdMS_TO_TICKS(600));
    }
    
    // Test individual LEDs
    ESP_LOGI(TAG, "Testing RSSI LED");
    rssi_led_set(255);
    vTaskDelay(pdMS_TO_TICKS(500));
    rssi_led_set(0);
    
    ESP_LOGI(TAG, "Testing ASSOC LED");
    assoc_led_set(255);
    vTaskDelay(pdMS_TO_TICKS(500));
    assoc_led_set(0);
    
    ESP_LOGI(TAG, "Testing SLEEP LED");
    sleep_led_set(255);
    vTaskDelay(pdMS_TO_TICKS(500));
    sleep_led_set(0);
    
    // Turn off all LEDs
    status_led_set(0, 0, 0);
    
    ESP_LOGI(TAG, "LED test sequence completed");
}

// Debug function to print current configuration
void status_led_print_config(void) {
    status_led_gpio_info_t info;
    status_led_get_gpio_info(&info);
    
    ESP_LOGI(TAG, "=== Status LED Configuration ===");
    ESP_LOGI(TAG, "Chip: %s", status_led_get_chip_info());
    ESP_LOGI(TAG, "High Speed Mode: %s", info.has_high_speed_mode ? "Yes" : "No");
    ESP_LOGI(TAG, "GPIO Mapping:");
    ESP_LOGI(TAG, "  RED:   GPIO_%d", info.red_gpio);
    ESP_LOGI(TAG, "  GREEN: GPIO_%d", info.green_gpio);
    ESP_LOGI(TAG, "  BLUE:  GPIO_%d", info.blue_gpio);
    ESP_LOGI(TAG, "  SLEEP: GPIO_%d", info.sleep_gpio);
    ESP_LOGI(TAG, "  RSSI:  GPIO_%d", info.rssi_gpio);
    ESP_LOGI(TAG, "  ASSOC: GPIO_%d", info.assoc_gpio);
    ESP_LOGI(TAG, "LEDC Configuration:");
    ESP_LOGI(TAG, "  Speed Mode: %s", (LEDC_SPEED_MODE == LEDC_LOW_SPEED_MODE) ? "LOW_SPEED" : "HIGH_SPEED");
    ESP_LOGI(TAG, "  Timer: LEDC_TIMER_%d", LEDC_TIMER_MODE);
    ESP_LOGI(TAG, "  Frequency: %d Hz", LEDC_FREQUENCY);
    ESP_LOGI(TAG, "  Resolution: %d bit", (1 << LEDC_DUTY_RES));
    ESP_LOGI(TAG, "================================");
}

