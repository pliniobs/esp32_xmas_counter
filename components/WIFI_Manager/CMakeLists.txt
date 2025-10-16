idf_component_register(SRCS "WIFI_Manager.c"
                    INCLUDE_DIRS "include" REQUIRES esp_wifi esp_event nvs_flash esp_system esp_netif lwip)
