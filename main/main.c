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

#define LOG_LOCAL_LEVEL ESP_LOG_ERROR

static const char *TAG_MAIN = "WIFI_STATION";

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