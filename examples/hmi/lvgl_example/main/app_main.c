// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* component includes */
#include <stdio.h>

/* freertos includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_freertos_hooks.h"
#include "board.h"
#include "lvgl_gui.h"
#include "esp_log.h"

#include "driver/rmt.h"

#include "lv_examples/src/lv_demo_printer/lv_demo_printer.h"
#include "lv_examples/src/lv_demo_widgets/lv_demo_widgets.h"
#include "lv_examples/src/lv_ex_get_started/lv_ex_get_started.h"
#include "lv_examples/src/lv_demo_benchmark/lv_demo_benchmark.h"
#include "lv_examples/src/lv_demo_keypad_encoder/lv_demo_keypad_encoder.h"
#include "lv_examples/src/lv_demo_stress/lv_demo_stress.h"
#include "lv_examples/src/lv_ex_style/lv_ex_style.h"

//#include "ir_demo.h"

static const char *TAG = "example_lvgl";

//============================================

#include "ir_tools.h"



//static rmt_channel_t example_tx_channel = RMT_CHANNEL_0;
static rmt_channel_t example_rx_channel = RMT_CHANNEL_2;

/**
 * @brief RMT Receive Task
 *
 */
static void example_ir_rx_task(void *arg)
{
    uint32_t addr = 0;
    uint32_t cmd = 0;
    size_t length = 0;
    bool repeat = false;
    RingbufHandle_t rb = NULL;
    rmt_item32_t *items = NULL;

    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(CONFIG_EXAMPLE_RMT_RX_GPIO, example_rx_channel);
    rmt_config(&rmt_rx_config);
    rmt_driver_install(example_rx_channel, 1000, 0);
    ir_parser_config_t ir_parser_config = IR_PARSER_DEFAULT_CONFIG((ir_dev_t)example_rx_channel);
    ir_parser_config.flags |= IR_TOOLS_FLAGS_PROTO_EXT; // Using extended IR protocols (both NEC and RC5 have extended version)
    ir_parser_t *ir_parser = NULL;
#if CONFIG_EXAMPLE_IR_PROTOCOL_NEC
    ir_parser = ir_parser_rmt_new_nec(&ir_parser_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_RC5
    ir_parser = ir_parser_rmt_new_rc5(&ir_parser_config);
#endif

    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(example_rx_channel, &rb);
    assert(rb != NULL);
    // Start receive
    rmt_rx_start(example_rx_channel, true);
    while (1) {
        items = (rmt_item32_t *) xRingbufferReceive(rb, &length, portMAX_DELAY);
        if (items) {
            length /= 4; // one RMT = 4 Bytes
            if (ir_parser->input(ir_parser, items, length) == ESP_OK) {
                if (ir_parser->get_scan_code(ir_parser, &addr, &cmd, &repeat) == ESP_OK) {
                    ESP_LOGI(TAG, "Scan Code %s --- addr: 0x%04x cmd: 0x%04x", repeat ? "(repeat)" : "", addr, cmd);
                }
            }
            //after parsing the data, return spaces to ringbuffer.
            vRingbufferReturnItem(rb, (void *) items);
        }
    }
    ir_parser->del(ir_parser);
    rmt_driver_uninstall(example_rx_channel);
    vTaskDelete(NULL);
}

//============================================

void app_main()
{
    iot_board_init();
    spi_bus_handle_t spi2_bus = iot_board_get_handle(BOARD_SPI2_ID);

    scr_driver_t lcd_drv;
    touch_panel_driver_t touch_drv;
    scr_interface_spi_config_t spi_lcd_cfg = {
        .spi_bus = spi2_bus,
        .pin_num_cs = BOARD_LCD_SPI_CS_PIN,
        .pin_num_dc = BOARD_LCD_SPI_DC_PIN,
        .clk_freq = BOARD_LCD_SPI_CLOCK_FREQ,
        .swap_data = true,
    };

    scr_interface_driver_t *iface_drv;
    scr_interface_create(SCREEN_IFACE_SPI, &spi_lcd_cfg, &iface_drv);

    scr_controller_config_t lcd_cfg = {
        .interface_drv = iface_drv,
        .pin_num_rst = 18,
        .pin_num_bckl = 23,
        .rst_active_level = 0,
        .bckl_active_level = 1,
        .offset_hor = 0,
        .offset_ver = 0,
        .width = 240,
        .height = 320,
        .rotate = SCR_DIR_TBLR,
    };
    scr_find_driver(SCREEN_CONTROLLER_ILI9341, &lcd_drv);
    lcd_drv.init(&lcd_cfg);

    touch_panel_config_t touch_cfg = {
        .interface_spi = {
            .spi_bus = spi2_bus,
            .pin_num_cs = BOARD_TOUCH_SPI_CS_PIN,
            .clk_freq = 2000000,
        },
        .interface_type = TOUCH_PANEL_IFACE_SPI,
        .pin_num_int = -1,
        .direction = TOUCH_DIR_BTLR,
        .width = 240,
        .height = 320,
    };
    touch_panel_find_driver(TOUCH_PANEL_CONTROLLER_XPT2046, &touch_drv);
    touch_drv.init(&touch_cfg);
    touch_drv.calibration_run(&lcd_drv, false);

    /* Initialize LittlevGL GUI */
    lvgl_init(&lcd_drv, &touch_drv);

    //=====================================
    xTaskCreate(example_ir_rx_task, "ir_rx_task", 2048, NULL, 10, NULL); 
    //=====================================

#ifdef CONFIG_LV_DEMO_BENCHMARK
    lv_demo_benchmark();
#elif defined CONFIG_LV_DEMO_PRINTER
    lv_demo_printer();
#elif defined CONFIG_LV_DEMO_WIDGETS
    lv_demo_widgets();
#elif defined CONFIG_LV_EX_GET_STARTED
    lv_ex_get_started_2();
#elif defined CONFIG_LV_DEMO_STRESS
    lv_demo_stress();
#elif defined CONFIG_LV_EX_STYLE
    lv_ex_style_1();
#endif

    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
}

