#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "WIFI_Manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"

#define LCD_NUM_NC -1
#define LCD_PIN_NUM_MOSI 19
#define LCD_PIN_NUM_MISO -1
#define LCD_PIN_NUM_SCLK 18
#define LCD_PIN_NUM_CS 5
#define LCD_PIN_NUM_DC 16
#define LCD_PIN_NUM_BL 4
#define LCD_PIN_NUM_RST 23

#define LCD_PIXEL_CLOCK_HZ (20000000) // 20 MHz
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

#define LCD_H_RES 135
#define LCD_V_RES 240
#define LCD_SPI_HOST SPI2_HOST

#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_ERROR
#endif

static const char *TAG_MAIN = "WIFI_STATION";
/**** LVGL START *****/
#define LVGL_TICK_PERIOD_MS 2

// LVGL library is not thread-safe, this will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;

esp_lcd_panel_handle_t panel_handle = NULL; // LCD panel handle
esp_lcd_panel_io_handle_t io_handle = NULL; // LCD IO handle

void App_Display_Init(void);

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void lvgl_port_update_callback(lv_display_t *disp)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);
    switch (rotation) {
    case LV_DISPLAY_ROTATION_0:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        esp_lcd_panel_set_gap(panel_handle, 52, 40); // The gap is 52 pixels on the left and 40 pixels on the top
        break;
    case LV_DISPLAY_ROTATION_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        esp_lcd_panel_set_gap(panel_handle, 40, 53); // The gap is 40 pixels on the left and 52 pixels on the top
        break;
    case LV_DISPLAY_ROTATION_180:
        // Rotate LCD display
        //esp_lcd_panel_swap_xy(panel_handle, false);
        //esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISPLAY_ROTATION_270:
        // Rotate LCD display
        //esp_lcd_panel_swap_xy(panel_handle, true);
        //esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    lvgl_port_update_callback(disp);
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // because SPI LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
    
}

static void lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_port_task(void *arg)
{
    //uint32_t time_till_next_ms = 0;
    //uint32_t time_threshold_ms = 2000 / CONFIG_FREERTOS_HZ;
    while (1) {
        //time_till_next_ms = lv_timer_handler();
  
        // in case of triggering a task watch dog time out
        //time_till_next_ms = MAX(time_till_next_ms, time_threshold_ms);
        //vTaskDelay(time_till_next_ms);
        
        lv_timer_handler();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void app_main(void)
{
    time_t now;
    char strftime_buf[64];
    struct tm timeinfo;



    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    App_Display_Init();
    /*Initialize LVGL library*/
    lv_init();

    // create a lvgl display
    lv_display_t *display = lv_display_create(LCD_H_RES, LCD_V_RES);

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    size_t draw_buffer_sz = LCD_H_RES * 20 * sizeof(lv_color16_t);
    void *buf1 = spi_bus_dma_memory_alloc(LCD_SPI_HOST, draw_buffer_sz, 0);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_SPI_HOST, draw_buffer_sz, 0);

    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    // associate the mipi panel handle to the display
    lv_display_set_user_data(display, panel_handle);
    
    // set color depth
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, lvgl_flush_cb);
    
    lv_display_set_rotation(display, LV_DISPLAY_ROTATION_90);

    /*nstall LVGL tick timer*/
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    /*Register io panel event callback for LVGL flush ready notification*/
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));
    

	/* Run the UI */
    // Lock the mutex due to the LVGL APIs are not thread-safe
    _lock_acquire(&lvgl_api_lock);
    //ui_init();
    _lock_release(&lvgl_api_lock);
    
    /*Create LVGL task*/
    xTaskCreate(lvgl_port_task, "LVGL", 4*4096, NULL, 2, NULL);
    int cnt = 0;



    esp_log_level_set("wifi", ESP_LOG_INFO);
    ESP_LOGI(TAG_MAIN, "ESP_WIFI_MODE_STA");
    wifi_initialize_station();

    for(;;) {
      time(&now);
      // Set timezone to UTC+3
      setenv("TZ", "UTC+3", 1);
      tzset();

      localtime_r(&now, &timeinfo);
      strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
      printf("The current date/time is: %s\r\n", strftime_buf);
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void App_Display_Init(void){
    gpio_set_direction(LCD_PIN_NUM_BL, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_PIN_NUM_BL, 1); // Turn on the backlight

    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PIN_NUM_SCLK,
        .mosi_io_num = LCD_PIN_NUM_MOSI,
        .miso_io_num = LCD_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t), // transfer 80 lines of pixels (assume pixel is RGB565) at most in one SPI transaction
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO)); // Enable the DMA feature
    

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PIN_NUM_DC, // Data/Command control pin
        .cs_gpio_num = LCD_PIN_NUM_CS, // Chip select pin
        .pclk_hz = LCD_PIXEL_CLOCK_HZ, // Pixel clock
        .lcd_cmd_bits = LCD_CMD_BITS, // Command bits
        .lcd_param_bits = LCD_PARAM_BITS, // Parameter bits
        .spi_mode = 0, // SPI mode 0
        .trans_queue_depth = 10, // Transaction queue depth
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_config, &io_handle));
    
    esp_lcd_panel_dev_config_t panel_config = {
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG, // Data endian
        .reset_gpio_num = LCD_PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .color_space = ESP_LCD_COLOR_SPACE_RGB, // Color space
        .bits_per_pixel = 16,
    };
    // Create LCD panel handle for ST7789, with the SPI IO device handle
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, true); 
    
    esp_lcd_panel_disp_on_off(panel_handle, true);
}