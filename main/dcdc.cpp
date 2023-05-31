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
    PWMTimerSync sync;
};


dcdc_instance_t *dcdc_instance = nullptr;

float sync_delay_s = 0.25e-6f;

extern "C"
esp_err_t dcdc_set_params(dcdc_params params) {

    auto &dcdc(*dcdc_instance);

    ESP_RETURN_ON_FALSE(params.frequency >= 5000 && params.frequency <= 200000, ESP_ERR_INVALID_ARG, TAG,
                        "unexpected frequency");
    ESP_RETURN_ON_FALSE(params.deadTime100Ns >= (2e7f * sync_delay_s), ESP_ERR_INVALID_ARG, TAG,
                        "unexpected deadTime100Ns");

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

        dcdc.pwmEN.stop();
        dcdc.pwmHIN.stop();
        dcdc.sync.stop();
        dcdc.pwmHIN.change_frequency(period_ticks);
        dcdc.pwmEN.change_frequency(period_ticks / 2);
        auto sync_delay_ticks = (uint32_t)(
                dcdc.pwmEN.period_ticks - (sync_delay_s * BLDC_MCPWM_TIMER_RESOLUTION_HZ)); // 500ns?
        dcdc.sync = PWMTimerSync(dcdc.pwmHIN, dcdc.pwmEN, sync_delay_ticks);
        dcdc.pwmHIN.set_duty_cycle(dcdc.pwmHIN.period_ticks / 2);
        float period = (1.f / params.frequency);
        float duty = (period - 2.f * params.deadTime100Ns * 1e-7f) / period;
        auto duty_cycle = (uint32_t)(dcdc.pwmEN.period_ticks * duty);
        ESP_LOGI(TAG, "Set dead-time %.2f us (duty %lu, %.1f%%)", params.deadTime100Ns * .1f, duty_cycle, duty * 100);
        dcdc.pwmEN.set_duty_cycle(duty_cycle);
        dcdc.pwmHIN.start();
        dcdc.pwmEN.start();
        vTaskDelay(pdMS_TO_TICKS(dcdc.pwmHIN.period_ticks * 1000 / BLDC_MCPWM_TIMER_RESOLUTION_HZ * 2));
    } else {
        float period = (1.f / params.frequency);
        float duty = (period - 2.f * params.deadTime100Ns * 1e-7f) / period;
        auto duty_cycle = (uint32_t)(dcdc.pwmEN.period_ticks * duty);
        ESP_LOGI(TAG, "Set dead-time %.2f us (duty %lu, %.1f%%)", params.deadTime100Ns * .1f, duty_cycle, duty * 100);
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

    uint32_t duty_cycle = period_ticks / 8;
    //period_ticks - (uint32_t)(BLDC_MCPWM_TIMER_RESOLUTION_HZ * (params.deadTime100Ns * 1e-7f) + .5f);

    ESP_LOGI(TAG, "freq=%lu, periodTicks=%lu, deadTime100Ns=%lu, duty_cycle=%lu", params.frequency, period_ticks,
             params.deadTime100Ns, duty_cycle);

    PWMSignal pwmHIN(period_ticks, period_ticks / 2, GPIO_NUM_12);

    PWMSignal pwmEN{period_ticks / 2, duty_cycle, GPIO_NUM_27}; // 13 is not suitable- 5ms high at boot

    // keep the 2 timers in sync
    // phase-shift (delay) EN timer, so HIN can settle before EN rises
    auto sync_delay_ticks = (uint32_t)(pwmEN.period_ticks - (sync_delay_s * BLDC_MCPWM_TIMER_RESOLUTION_HZ)); // 500ns?
    PWMTimerSync sync{pwmHIN, pwmEN, sync_delay_ticks};

    dcdc_instance = new dcdc_instance_t{pwmHIN, pwmEN, sync};


    pwmHIN.start();
    pwmEN.start();
    //pwmEN.soft_start(softStartSeconds * 1000, softStartSteps);

}



