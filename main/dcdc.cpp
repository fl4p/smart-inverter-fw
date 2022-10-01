#include <stdio.h>


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"

#define BLDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us


#define GPIO_PWM0A_OUT GPIO_NUM_12
#define GPIO_PWM0B_OUT GPIO_NUM_13
#define GPIO_SD_OUT GPIO_NUM_14

static const char *TAG = "dcdc";


float map(float x, float x0, float x1, float y0, float y1) {
    return (x - x0) / (x1 - x0) * (y1 - y0) + y0;
}


extern "C"
void dcdc_main() {
    const int deadTimeUS = 2;

    const int frequency = 42000;


    ESP_LOGI(TAG, "Enable SD during init (pin %d)", GPIO_SD_OUT);
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_SD_OUT, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_SD_OUT, 1));

    vTaskDelay(pdMS_TO_TICKS(1000));

    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = BLDC_MCPWM_TIMER_RESOLUTION_HZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = BLDC_MCPWM_TIMER_RESOLUTION_HZ / frequency,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));


    mcpwm_oper_handle_t operator_;
    mcpwm_operator_config_t operator_config = {
            .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator_));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_, timer));


    mcpwm_cmpr_handle_t comparator;
    mcpwm_comparator_config_t compare_config = {.flags = {.update_cmp_on_tez = true}};
    //compare_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(operator_, &compare_config, &comparator));

    // set 50% duty cycle
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, BLDC_MCPWM_TIMER_RESOLUTION_HZ / frequency / 2));

    mcpwm_gen_handle_t generatorA, generatorB;
    mcpwm_generator_config_t gen_config = {};
    gen_config.gen_gpio_num = GPIO_PWM0A_OUT;
    ESP_ERROR_CHECK(mcpwm_new_generator(operator_, &gen_config, &generatorA));
    gen_config.gen_gpio_num = GPIO_PWM0B_OUT;
    ESP_ERROR_CHECK(mcpwm_new_generator(operator_, &gen_config, &generatorB));


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

    // set dead time

    mcpwm_dead_time_config_t dt_configA = {
            .posedge_delay_ticks = BLDC_MCPWM_TIMER_RESOLUTION_HZ * deadTimeUS / 1000000},
            dt_configB = {
            .negedge_delay_ticks = BLDC_MCPWM_TIMER_RESOLUTION_HZ * deadTimeUS / 1000000,
            .flags = {.invert_output = true},
    };
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generatorA, generatorA, &dt_configA));
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generatorA, generatorB, &dt_configB));



    //ESP_LOGI(TAG, "Turn off all the gates");
    //ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generatorA, 1, true));
    //ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generatorB, 1, true));





    ESP_LOGI(TAG, "Enable MCPWM timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));


    ESP_LOGI(TAG, "Start a timer to adjust motor speed periodically");

    //auto soft_start_func = []() {

    //};
    /*
    void update_soft_start_callback(void *_) {
        const float softStartSeconds = 10;
        const int softStartSteps = 100;

        static int i = 0;


        float p = (float) i / (float) softStartSteps;
        int dt = map(p * p * p, 0, 1, 1e6f * .25f / frequency, deadTimeUS) * 10;



        if (i == 0) {
            gpio_set_level(GPIO_SD_OUT, 0);
            printf("disabled SD\n");
        }
        vTaskDelay(softStartSeconds * 1000 / softStartSteps / portTICK_PERIOD_MS);



        ++i;
    }
     */

    /*
    esp_timer_handle_t periodic_timer = NULL;
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = update_softstart_callback,
            .arg = softStartSteps,
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, BLDC_SPEED_UPDATE_PERIOD_US));
     */


    //ESP_LOGI(TAG, "Turn on all the gates");
    //ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generatorA, -1, true));
    //ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generatorB, -1, true));

    return;


}
