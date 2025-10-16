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
//#include "sdkconfig.h"
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