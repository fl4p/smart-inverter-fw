#include <stdio.h>


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <esp_check.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "dcdc.h"

#include "pwm.h"


#define GPIO_PWM0A_OUT GPIO_NUM_12
#define GPIO_PWM0B_OUT GPIO_NUM_13
#define GPIO_SD_OUT GPIO_NUM_14

static const char *TAG = "dcdc";


struct dcdc_instance_t {
    PWMSignal pwmHIN, pwmEN;
};


dcdc_instance_t *dcdc_instance = nullptr;


extern "C"
esp_err_t dcdc_set_params(dcdc_params params) {

    auto &dcdc(*dcdc_instance);

    ESP_RETURN_ON_FALSE(params.frequency >= 5000 && params.frequency <= 200000, ESP_ERR_INVALID_ARG, TAG,
                        "unexpected frequency");
    ESP_RETURN_ON_FALSE(params.deadTime100Ns >= 5, ESP_ERR_INVALID_ARG, TAG, "unexpected deadTime100Ns");

    auto dtMax = (uint32_t)(1e7f * .5f / params.frequency + .5f);
    ESP_RETURN_ON_FALSE(params.deadTime100Ns <= dtMax, ESP_ERR_INVALID_ARG, TAG, "deadTime100Ns too large");

    uint32_t period_ticks = BLDC_MCPWM_TIMER_RESOLUTION_HZ / params.frequency;

    if (dcdc.pwmEN.softStarting) {
        ESP_LOGE(TAG, "still soft-starting, please wait");
        return ESP_FAIL;
    }

    if (period_ticks != 0 && dcdc.pwmHIN.period_ticks != period_ticks) {

        ESP_LOGI(TAG, "New freq %lu, MCPWM timer period change %lu -> %lu", params.frequency,
                 dcdc.pwmHIN.period_ticks, period_ticks);

        dcdc.pwmEN.stop(); // TODO does this keep EN always low?
        dcdc.pwmHIN.change_frequency(period_ticks);
        dcdc.pwmEN.change_frequency(period_ticks);

    } else {
        auto duty_cycle = (uint32_t)(BLDC_MCPWM_TIMER_RESOLUTION_HZ / params.deadTime100Ns * 1e-7f + .5f);
        ESP_LOGI(TAG, "Set dead-time %.2f us", params.deadTime100Ns * .1f);
        dcdc.pwmEN.set_duty_cycle(duty_cycle);
    }

    return ESP_OK;
}



extern "C"
void dcdc_main() {
    const dcdc_params params = {.deadTime100Ns = 20, .frequency=42000};
    const float softStartSeconds = 3;
    const int softStartSteps = 100;

    // create HIN timer
    auto period_ticks = BLDC_MCPWM_TIMER_RESOLUTION_HZ / params.frequency;

    uint32_t duty_cycle = period_ticks / 4;
    //period_ticks - (uint32_t)(BLDC_MCPWM_TIMER_RESOLUTION_HZ * (params.deadTime100Ns * 1e-7f) + .5f);

    ESP_LOGI(TAG, "freq=%lu, periodTicks=%lu, deadTime100Ns=%lu, duty_cycle=%lu", params.frequency, period_ticks,
             params.deadTime100Ns, duty_cycle);

    PWMSignal pwmHIN(period_ticks, period_ticks / 2, GPIO_NUM_12);

    PWMSignal pwmEN{period_ticks / 2, duty_cycle, GPIO_NUM_13};

    dcdc_instance = new dcdc_instance_t{pwmHIN, pwmEN};


    // setup transitions for generatorA

    /*
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
            generatorA,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW),
            MCPWM_GEN_COMPARE_EVENT_ACTION_END()));
    */

    pwmHIN.start();
    pwmEN.start();
    //pwmEN.soft_start(softStartSeconds * 1000, softStartSteps);

}



