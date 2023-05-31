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
    PWMTimer timer;
    PWMSignal pwmHIN;
    PWMSignalHalfPeriod pwmEN;
};


dcdc_instance_t *dcdc_instance = nullptr;


extern "C"
esp_err_t dcdc_set_params(dcdc_params params) {

    auto &dcdc(*dcdc_instance);

    ESP_RETURN_ON_FALSE(params.frequency >= 5000 && params.frequency <= 200000, ESP_ERR_INVALID_ARG, TAG,
                        "unexpected frequency");
    ESP_RETURN_ON_FALSE(params.dutyCycle >= 0 and params.dutyCycle <= 1, ESP_ERR_INVALID_ARG, TAG,
                        "unexpected dutyCycle");

    uint32_t period_ticks = BLDC_MCPWM_TIMER_RESOLUTION_HZ / params.frequency;

    if (dcdc.pwmEN.softStarting) {
        ESP_LOGE(TAG, "still soft-starting, please wait");
        return ESP_FAIL;
    }

    if (period_ticks != 0 && dcdc.timer.period_ticks != period_ticks) {

        ESP_LOGI(TAG, "New freq %lu, MCPWM timer period change %lu -> %lu", params.frequency,
                 dcdc.timer.period_ticks, period_ticks);


        dcdc.pwmEN.enable(false);
        dcdc.pwmHIN.enable(false, true);

        dcdc.timer.change_frequency(period_ticks);

        dcdc.pwmHIN.connect_timer(dcdc.timer);
        dcdc.pwmEN.connect_timer(dcdc.timer);

        dcdc.pwmHIN.set_duty_cycle(period_ticks / 2);
        dcdc.pwmEN.set_duty_cycle(period_ticks * params.dutyCycle);

        dcdc.pwmHIN.enable(true);
        dcdc.pwmEN.enable(true);

    } else {
        dcdc.pwmEN.set_duty_cycle(period_ticks * params.dutyCycle);
    }

    return ESP_OK;
}



extern "C"
void dcdc_main() {
    const dcdc_params params = {.dutyCycle = 0.5f, .frequency=42000};
    const float softStartSeconds = 3;
    const int softStartSteps = 100;

    // create HIN timer
    auto period_ticks = BLDC_MCPWM_TIMER_RESOLUTION_HZ / params.frequency;

    uint32_t duty_cycle = period_ticks / 4;
    //period_ticks - (uint32_t)(BLDC_MCPWM_TIMER_RESOLUTION_HZ * (params.deadTime100Ns * 1e-7f) + .5f);

    ESP_LOGI(TAG, "freq=%lu, periodTicks=%lu, dutyCycle=%.2f, duty_cycle=%lu", params.frequency, period_ticks,
             params.dutyCycle, duty_cycle);

    PWMTimer pwmTimer(period_ticks);

    PWMSignal pwmHIN(pwmTimer, period_ticks / 2, GPIO_NUM_12);
    PWMSignalHalfPeriod pwmEN{pwmTimer, duty_cycle, GPIO_NUM_13};

    dcdc_instance = new dcdc_instance_t{pwmTimer, pwmHIN, pwmEN};


    pwmHIN.enable(true);
    pwmEN.enable(true);
    pwmTimer.start();

    //pwmEN.soft_start(softStartSeconds * 1000, softStartSteps);

}



