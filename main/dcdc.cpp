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
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"

#include "dcdc.h"

#define BLDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us


#define GPIO_PWM0A_OUT GPIO_NUM_12
#define GPIO_PWM0B_OUT GPIO_NUM_13
#define GPIO_SD_OUT GPIO_NUM_14

static const char *TAG = "dcdc";


struct dcdc_instance_t {
    mcpwm_timer_handle_t timer = nullptr;
    uint32_t period_ticks = 0;
    mcpwm_oper_handle_t operator_ = nullptr;
    mcpwm_cmpr_handle_t comparator = nullptr;
    mcpwm_gen_handle_t genA = nullptr, genB = nullptr;
};

struct soft_start_args {
    mcpwm_gen_handle_t genA, genB;
    uint32_t steps, i;
    dcdc_params params;
    esp_timer_handle_t timer;

    soft_start_args(mcpwm_gen_handle_t genA, mcpwm_gen_handle_t genB, uint32_t steps, dcdc_params params)
            : genA(genA), genB(genB), steps(steps), i(0), params(params), timer(NULL) {}
};

void soft_start(dcdc_params params, uint32_t milliseconds, uint32_t steps);

dcdc_instance_t dcdc_instance = {};

bool softStarting = false;


float map(float x, float x0, float x1, float y0, float y1) {
    return (x - x0) / (x1 - x0) * (y1 - y0) + y0;
}

void set_driver_dead_time(float deadTimeUS, mcpwm_gen_handle_t genA, mcpwm_gen_handle_t genB) {
    uint32_t t = (uint32_t) (BLDC_MCPWM_TIMER_RESOLUTION_HZ * deadTimeUS * 1e-6f + 0.5f);
    mcpwm_dead_time_config_t dt_configA = {.posedge_delay_ticks = t},
            dt_configB = {.negedge_delay_ticks = t, .flags = {.invert_output = true},};
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(genA, genA, &dt_configA));
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(genA, genB, &dt_configB));
}

void update_soft_start_callback(void *args_) {
    soft_start_args *args = (soft_start_args *) args_;

    float p = (float) args->i / (float) args->steps;
    float dt = map(p * p * p, 0, 1, 1e6f * .5f / args->params.frequency, args->params.deadTime100Ns * 0.1f);

    ESP_LOGD(TAG, "soft-start p=%.2f dt=%.2f", p, dt);

    set_driver_dead_time(dt, args->genA, args->genB);

    if (args->i == 0) {
        gpio_set_level(GPIO_SD_OUT, 0);
        ESP_LOGI(TAG, "Disabled SD");
    }

    if (args->i >= args->steps) {
        ESP_LOGI(TAG, "Soft-start finished");
        esp_timer_stop(args->timer);
        esp_timer_delete(args->timer);
        delete args;
        softStarting = false;
    }

    ++(args->i);
}


extern "C"
esp_err_t dcdc_set_params(dcdc_params params) {

    ESP_RETURN_ON_FALSE(params.frequency >= 5000 && params.frequency <= 200000, ESP_ERR_INVALID_ARG, TAG,
                        "unexpected frequency");
    ESP_RETURN_ON_FALSE(params.deadTime100Ns >= 5, ESP_ERR_INVALID_ARG, TAG, "unexpected deadTime100Ns");

    auto dtMax = (uint32_t) (1e7f * .5f / params.frequency + .5f);
    ESP_RETURN_ON_FALSE(params.deadTime100Ns <= dtMax, ESP_ERR_INVALID_ARG, TAG, "deadTime100Ns too large");

    uint32_t period_ticks = BLDC_MCPWM_TIMER_RESOLUTION_HZ / params.frequency;

    if(softStarting) {
        ESP_LOGE(TAG, "still soft-starting, please wait");
        return ESP_FAIL;
    }

    if (period_ticks != 0 && dcdc_instance.period_ticks != period_ticks) {

        ESP_LOGI(TAG, "New freq %lu, MCPWM timer period change %lu -> %lu", params.frequency,
                 dcdc_instance.period_ticks, period_ticks);

        gpio_set_level(GPIO_SD_OUT, 1);
        vTaskDelay(pdMS_TO_TICKS(2));

        mcpwm_timer_disable(dcdc_instance.timer);
        mcpwm_del_timer(dcdc_instance.timer);
        dcdc_instance.timer = nullptr;

        mcpwm_timer_handle_t timer = nullptr;
        mcpwm_timer_config_t timer_config = {
                .group_id = 0,
                .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
                .resolution_hz = BLDC_MCPWM_TIMER_RESOLUTION_HZ,
                .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
                .period_ticks = period_ticks,
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(dcdc_instance.operator_, timer));

        // set 50% duty cycle
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(dcdc_instance.comparator, period_ticks / 2));

        ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

        dcdc_instance.timer = timer;
        dcdc_instance.period_ticks = period_ticks;

        //gpio_set_level(GPIO_SD_OUT, 0);
        soft_start(params, 10, 20);
    } else {
        ESP_LOGI(TAG, "Set dead-time %.2f us", params.deadTime100Ns * .1f);
        set_driver_dead_time(params.deadTime100Ns * .1f, dcdc_instance.genA, dcdc_instance.genB);
    }

    return ESP_OK;
}

void soft_start(dcdc_params params, uint32_t milliseconds, uint32_t steps) {
    if(softStarting) {
        ESP_LOGE(TAG, "already soft-starting, please wait");
        return;
    }

    ESP_LOGI(TAG, "Soft-start..");
    softStarting = true;
    soft_start_args *timer_args = new soft_start_args(dcdc_instance.genA, dcdc_instance.genB, steps, params);
    const esp_timer_create_args_t periodic_timer_args = {.callback = update_soft_start_callback, .arg = timer_args};
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &timer_args->timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_args->timer, milliseconds * 1000 / steps));
}


extern "C"
void dcdc_main() {
    const dcdc_params params = {.deadTime100Ns = 20, .frequency=42000};
    const float softStartSeconds = 3;
    const int softStartSteps = 100;

    ESP_LOGI(TAG, "Enable SD during init (pin %d)", GPIO_SD_OUT);

    gpio_config_t drv_en_config = {.pin_bit_mask = 1ULL
            << GPIO_SD_OUT, .mode = GPIO_MODE_OUTPUT, .pull_up_en=GPIO_PULLUP_ENABLE};
    ESP_ERROR_CHECK(gpio_config(&drv_en_config));
    gpio_set_level(GPIO_SD_OUT, 1);

    //vTaskDelay(pdMS_TO_TICKS(1000));

    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = BLDC_MCPWM_TIMER_RESOLUTION_HZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = BLDC_MCPWM_TIMER_RESOLUTION_HZ / params.frequency,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
    dcdc_instance.timer = timer;
    dcdc_instance.period_ticks = timer_config.period_ticks;

    //mcpwm_oper_handle_t operator_;
    mcpwm_operator_config_t operator_config = {
            .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &dcdc_instance.operator_));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(dcdc_instance.operator_, timer));


    mcpwm_cmpr_handle_t comparator;
    mcpwm_comparator_config_t compare_config = {.flags = {.update_cmp_on_tez = true}};
    //compare_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(dcdc_instance.operator_, &compare_config, &comparator));
    dcdc_instance.comparator = comparator;

    // set 50% duty cycle
    ESP_ERROR_CHECK(
            mcpwm_comparator_set_compare_value(comparator, BLDC_MCPWM_TIMER_RESOLUTION_HZ / params.frequency / 2));

    mcpwm_gen_handle_t generatorA, generatorB;
    mcpwm_generator_config_t gen_config = {};
    gen_config.gen_gpio_num = GPIO_PWM0A_OUT;
    ESP_ERROR_CHECK(mcpwm_new_generator(dcdc_instance.operator_, &gen_config, &generatorA));
    gen_config.gen_gpio_num = GPIO_PWM0B_OUT;
    ESP_ERROR_CHECK(mcpwm_new_generator(dcdc_instance.operator_, &gen_config, &generatorB));

    dcdc_instance.genA = generatorA;
    dcdc_instance.genB = generatorB;


    // setup transitions for generatorA
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generatorA,
                                                               MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                                            MCPWM_TIMER_EVENT_EMPTY,
                                                                                            MCPWM_GEN_ACTION_HIGH),
                                                               MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generatorA,
                                                                 MCPWM_GEN_COMPARE_EVENT_ACTION(
                                                                         MCPWM_TIMER_DIRECTION_UP, comparator,
                                                                         MCPWM_GEN_ACTION_LOW),
                                                                 MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    set_driver_dead_time(1e6f * .5f / params.frequency, generatorA, generatorB);


    ESP_LOGI(TAG, "Enable MCPWM timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    soft_start(params, softStartSeconds * 1000, softStartSteps);

}
