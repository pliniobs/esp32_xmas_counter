idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "." REQUIRES WIFI_Manager esp_wifi esp_event nvs_flash esp_system esp_netif lwip lvgl esp_lcd esp_timer)