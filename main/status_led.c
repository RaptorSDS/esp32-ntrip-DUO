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
        // Handle fade function installation error
        // Continue anyway as basic functionality will still work
    }

    // Turn off all LEDs initially
    status_led_set(0, 0, 0);
    rssi_led_set(0);
    assoc_led_set(0);
    sleep_led_set(0);

    // Create LED task
    BaseType_t task_result = xTaskCreate(
        status_led_task, 
        "status_led", 
        2048, 
        NULL, 
        TASK_PRIORITY_STATUS_LED, 
        &led_task
    );
    
    if (task_result != pdPASS) {
        // Handle task creation error
        led_task = NULL;
    }
}

void rssi_led_set(uint8_t value) {
    status_led_channel_set(STATUS_LED_RSSI_CHANNEL, value);
}

void rssi_led_fade(uint8_t value, int max_fade_time_ms) {
    status_led_channel_fade(STATUS_LED_RSSI_CHANNEL, value, max_fade_time_ms);
}

void assoc_led_set(uint8_t value) {
    status_led_channel_set(STATUS_LED_ASSOC_CHANNEL, value);
}

void assoc_led_fade(uint8_t value, int max_fade_time_ms) {
    status_led_channel_fade(STATUS_LED_ASSOC_CHANNEL, value, max_fade_time_ms);
}

void sleep_led_set(uint8_t value) {
    status_led_channel_set(STATUS_LED_SLEEP_CHANNEL, 0xFF - value);
}

void sleep_led_fade(uint8_t value, int max_fade_time_ms) {
    status_led_channel_fade(STATUS_LED_SLEEP_CHANNEL, 0xFF - value, max_fade_time_ms);
}

// Additional utility functions for better ESP32 variant support

const char* status_led_get_chip_info(void) {
#if defined(CHIP_ESP32_CLASSIC)
    return "ESP32 Classic";
#elif defined(CHIP_ESP32_S2)
    return "ESP32-S2";
#elif defined(CHIP_ESP32_S3)
    return "ESP32-S3";
#elif defined(CHIP_ESP32_C3)
    return "ESP32-C3";
#elif defined(CHIP_ESP32_C2)
    return "ESP32-C2";
#elif defined(CHIP_ESP32_C6)
    return "ESP32-C6";
#elif defined(CHIP_ESP32_H2)
    return "ESP32-H2";
#else
    return "Unknown ESP32";
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

// Test function to verify LED functionality
void status_led_test_sequence(void) {
    // Test Red LED
    status_led_add(0xFF0000FF, STATUS_LED_STATIC, 0, 500, 1);
    vTaskDelay(pdMS_TO_TICKS(600));
    
    // Test Green LED
    status_led_add(0x00FF00FF, STATUS_LED_STATIC, 0, 500, 1);
    vTaskDelay(pdMS_TO_TICKS(600));
    
    // Test Blue LED
    status_led_add(0x0000FFFF, STATUS_LED_STATIC, 0, 500, 1);
    vTaskDelay(pdMS_TO_TICKS(600));
    
    // Test White LED (all colors)
    status_led_add(0xFFFFFFFF, STATUS_LED_STATIC, 0, 500, 1);
    vTaskDelay(pdMS_TO_TICKS(600));
    
    // Test Fade
    status_led_add(0xFF00FFFF, STATUS_LED_FADE, 200, 2000, 1);
    vTaskDelay(pdMS_TO_TICKS(2100));
    
    // Test Flash
    status_led_add(0x00FFFFFF, STATUS_LED_FLASH, 100, 1000, 1);
    vTaskDelay(pdMS_TO_TICKS(1100));
}

// Cleanup function
void status_led_deinit(void) {
    // Stop and delete task
    if (led_task != NULL) {
        vTaskDelete(led_task);
        led_task = NULL;
    }
    
    // Clear all colors
    status_led_clear();
    
    // Turn off all LEDs
    status_led_set(0, 0, 0);
    rssi_led_set(0);
    assoc_led_set(0);
    sleep_led_set(0);
    
    // Uninstall fade function
    ledc_fade_func_uninstall();
}
