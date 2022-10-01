
#include <esp_err.h>
#include <nvs_flash.h>
#include "esp_log.h"

static const char *TAG = "smart inverter";

void dcdc_main();
void wifi_init_sta();
void web_main();

void app_main() {
    dcdc_main();

    /*
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
*/
    //web_main();

}


