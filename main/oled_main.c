/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "wim_ios.h"
#include "Oled1306.h"
#include "fonts.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "..\components\MCP3564\MCP3564.h"

//=================================================================================================================

//#include <stdint.h>
//#include "freertos/queue.h"
#include "driver/gptimer.h" 
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_timer.h"

#define TIMER_INTERVAL_MS     (20)  // 20 ms timer interval

#define PCNT_HIGH_LIMIT       4000
#define PCNT_INPUT_GPIO       LOOP1   // Change this to the GPIO you're using for PCNT input

#define QUEUE_LENGTH          1024
#define QUEUE_ITEM_SIZE       sizeof(uint32_t)

static const char *TAG = "MAIN";

// PCNT unit handle
static pcnt_unit_handle_t pcnt_unit = NULL;

// ESP Timer handle
static esp_timer_handle_t periodic_timer;

// Time variables
static uint64_t start_time = 0;

// Queue implementation
static uint32_t queue[QUEUE_LENGTH];
static int pe = 0, ps = 0;


static uint32_t dequeue() {
    if (ps == pe) return 0; // Queue is empty
    ps = (ps + 1) & (QUEUE_LENGTH - 1);
    return queue[ps];
}

static void IRAM_ATTR timer_callback(void* arg) {
    start_time = esp_timer_get_time();
    //gpio_set_level(LED_RED,1);
    //pcnt_unit_stop(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);
    //gpio_set_level(LED_RED,0);

}

static bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {

    uint64_t end_time = esp_timer_get_time();

    //gpio_set_level(LED_YELLOW,1);

    // Poe na Fila
    pe = (pe + 1) & (QUEUE_LENGTH - 1);
    queue[pe] = (uint32_t)(end_time - start_time);

    pcnt_unit_stop(pcnt_unit);
    //gpio_set_level(LED_YELLOW,0);    
    return true;
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing...");

    // Configure LED pins as outputs

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_RED) | (1ULL << LED_YELLOW) | (1ULL << LED_GREEN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Initialize UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);

    // Initialize PCNT
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = -1,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 100,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = PCNT_INPUT_GPIO,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    //ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));


    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));

    // add high limit watch point
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 3600));

    // Enable PCNT unit
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));

    ESP_LOGI(TAG, "Starting timer and PCNT...");

    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit)); 

    // Initialize Timer
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &timer_callback,
        .name = "periodic"
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));   

    // Start the timer
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMER_INTERVAL_MS * 1000));

        while (1) {
        uint32_t delta_time = dequeue();
        if (delta_time > 0) {
            //printf("d= %lu us\n", delta_time);
            printf("%lu\n", delta_time);            
        }
        vTaskDelay(1); // Small delay to prevent watchdog trigger
    }

}    