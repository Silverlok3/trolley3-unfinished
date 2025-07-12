// main/main.cpp - SLIM APPLICATION INITIALIZATION (TARGET: ~100 LINES)
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Application initialization only
// - Component initialization
// - Task creation  
// - WiFi setup
// - Error handling setup
// - Background monitoring
// 
// All business logic moved to dedicated components:
// - hardware_control: Low-level hardware
// - mode_coordinator: 3-mode system management
// - wire_learning_mode, automatic_mode, manual_mode: Mode implementations
// - web_interface: Complete web UI
// - sensor_health: Sensor validation
// ═══════════════════════════════════════════════════════════════════════════════

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"

// Component includes
#include "hardware_control.h"
#include "mode_coordinator.h"
#include "wire_learning_mode.h"
#include "automatic_mode.h"
#include "manual_mode.h"
#include "web_interface.h"
#include "sensor_health.h"
#include "MPU.hpp"
#include "pin_config.h"

static const char* TAG = "MAIN";

// Global objects
static MPU_t mpu;
static bool system_ready = false;

// ═══════════════════════════════════════════════════════════════════════════════
// SYSTEM INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Initialize MPU6050 sensor
 */
static bool init_mpu6050(void) {
    ESP_LOGI(TAG, "Initializing MPU6050 sensor...");
    
    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = 400000 },
        .clk_flags = 0
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT_NUM, conf.mode, 0, 0, 0));
    
    // Initialize MPU
    if (mpu.initialize() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        return false;
    }
    
    // Configure MPU settings
    mpu.setAccelFullScale(mpud::ACCEL_FS_8G);
    mpu.setGyroFullScale(mpud::GYRO_FS_500DPS);
    mpu.setDigitalLowPassFilter(mpud::DLPF_42HZ);
    mpu.setSampleRate(100);
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return true;
}

/**
 * @brief Initialize all system components
 */
static esp_err_t init_system_components(void) {
    ESP_LOGI(TAG, "Initializing system components...");
    
    // Initialize MPU6050 first
    if (!init_mpu6050()) {
        ESP_LOGE(TAG, "MPU6050 initialization failed");
        return ESP_FAIL;
    }
    
    // Initialize hardware control (ESC, Hall, GPIO)
    ESP_LOGI(TAG, "Initializing hardware control...");
    ESP_ERROR_CHECK(hardware_init());
    
    // Initialize sensor health monitoring
    ESP_LOGI(TAG, "Initializing sensor health...");
    ESP_ERROR_CHECK(sensor_health_init(&mpu));
    
    // Initialize mode coordinator (3-mode system)
    ESP_LOGI(TAG, "Initializing mode coordinator...");
    ESP_ERROR_CHECK(mode_coordinator_init());
    
    // Initialize individual mode components
    ESP_LOGI(TAG, "Initializing mode components...");
    ESP_ERROR_CHECK(wire_learning_mode_init());
    ESP_ERROR_CHECK(automatic_mode_init());
    ESP_ERROR_CHECK(manual_mode_init());
    
    // Initialize web interface
    ESP_LOGI(TAG, "Initializing web interface...");
    ESP_ERROR_CHECK(web_interface_init(NULL)); // Use default config
    
    ESP_LOGI(TAG, "All components initialized successfully");
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// BACKGROUND TASKS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Main system update task
 */
static void system_update_task(void* pvParameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "System update task started");
    
    while (1) {
        // Update all components
        hardware_update();
        sensor_health_update();
        mode_coordinator_update();
        web_interface_update();
        
        // Update active mode components
        wire_learning_mode_update();
        automatic_mode_update();
        manual_mode_update();
        
        // Run every 50ms for responsive system
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

/**
 * @brief System monitoring task
 */
static void system_monitor_task(void* pvParameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t counter = 0;
    
    ESP_LOGI(TAG, "System monitor task started");
    
    while (1) {
        // Log system status every 30 seconds
        if (++counter % 60 == 0) {
            system_mode_status_t mode_status = mode_coordinator_get_status();
            hardware_status_t hw_status = hardware_get_status();
            
            ESP_LOGI(TAG, "System Status - Mode: %s, ESC: %s, Sensors: %s, WiFi Clients: %d",
                    mode_coordinator_mode_to_string(mode_status.current_mode),
                    hw_status.esc_armed ? "Armed" : "Disarmed",
                    mode_status.sensors_validated ? "OK" : "Not Validated",
                    web_wifi_get_client_count());
        }
        
        // Check system health
        if (!mode_coordinator_is_system_healthy()) {
            ESP_LOGW(TAG, "System health issue detected: %s", 
                    mode_coordinator_get_error_message());
        }
        
        // Monitor memory usage
        size_t free_heap, min_free_heap;
        web_get_memory_usage(&free_heap, &min_free_heap);
        if (free_heap < 50000) { // Less than 50KB free
            ESP_LOGW(TAG, "Low memory warning: %zu bytes free", free_heap);
        }
        
        // Run every 500ms
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
}

/**
 * @brief Serial command task (for debugging)
 */
static void serial_command_task(void* pvParameter) {
    char input_char;
    
    ESP_LOGI(TAG, "Serial command task started");
    printf("\n=== ESP32-S3 TROLLEY - 3-MODE SYSTEM ===\n");
    printf("Hardware: ESP32-S3 + Eco II 2807 + Littlebee 30A ESC\n");
    printf("WiFi: ESP32S3_TROLLEY_3MODE (http://192.168.4.1)\n");
    printf("Commands: T=Status, R=Reset, P=Emergency Stop, K=Help\n");
    printf("Note: Full control available via web interface\n\n");
    
    while (1) {
        // Read single character from UART
        int chars_read = uart_read_bytes(UART_NUM_0, &input_char, 1, pdMS_TO_TICKS(1000));
        
        if (chars_read > 0) {
            printf("Command: '%c'\n", input_char);
            
            // Process basic commands (full command set available via web)
            char response[256];
            esp_err_t result = web_process_command(input_char, "serial", response, sizeof(response));
            
            printf("Response: %s\n", response);
            if (result != ESP_OK) {
                printf("Error: Command failed\n");
            }
            printf("\n");
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// MAIN APPLICATION
// ═══════════════════════════════════════════════════════════════════════════════

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== ESP32-S3 TROLLEY SYSTEM STARTING ===");
    ESP_LOGI(TAG, "Chip: %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "Architecture: Modular 3-Mode System");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize UART for serial commands
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0));
    
    // Initialize all system components
    if (init_system_components() != ESP_OK) {
        ESP_LOGE(TAG, "FATAL: System component initialization failed");
        esp_restart();
    }
    
    // Initialize WiFi and start web server
    ESP_LOGI(TAG, "Starting WiFi and web server...");
    ESP_ERROR_CHECK(web_wifi_init_ap("ESP32S3_TROLLEY_3MODE", ""));
    ESP_ERROR_CHECK(web_interface_start());
    
    // Create system tasks
    ESP_LOGI(TAG, "Creating system tasks...");
    xTaskCreate(system_update_task, "sys_update", 4096, NULL, 8, NULL);
    xTaskCreate(system_monitor_task, "sys_monitor", 3072, NULL, 5, NULL);
    xTaskCreate(serial_command_task, "serial_cmd", 3072, NULL, 3, NULL);
    
    // System ready
    system_ready = true;
    ESP_LOGI(TAG, "=== SYSTEM INITIALIZATION COMPLETE ===");
    ESP_LOGI(TAG, "Mode System: 3-mode operation (Wire Learning → Automatic → Manual)");
    ESP_LOGI(TAG, "Web Interface: http://192.168.4.1");
    ESP_LOGI(TAG, "Status: Sensor validation required before operation");
    
    // Main task complete - system runs via created tasks
    ESP_LOGI(TAG, "Main initialization complete - system operational");
    
    // Keep main task alive but minimal
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000)); // Sleep 30 seconds
        
        // Periodic health check
        if (!system_ready) {
            ESP_LOGE(TAG, "System not ready - restarting");
            esp_restart();
        }
    }
}