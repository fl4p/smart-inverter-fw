#pragma once

#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"


#define BLDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us


struct soft_start_args {
    void *arg;
    uint32_t steps, i, target_duty_cycle;
    esp_timer_handle_t timer;

    soft_start_args(void *arg, uint32_t target_duty_cycle, uint32_t steps) : arg(arg), steps(steps), i(0),
                                                                             target_duty_cycle(target_duty_cycle),
                                                                             timer(NULL) {}
};

void update_soft_start_callback(void *args_);

struct PWMSignal;

struct PWMTimer {
    mcpwm_timer_handle_t timer = nullptr;
    uint32_t period_ticks;

    //std::vector<PWMSignal &> signals;

    explicit PWMTimer(uint32_t period_ticks) : period_ticks(period_ticks) {
        _newTimer(period_ticks);
    }

    void _newTimer(uint32_t period_ticks) {
        if (timer) {
            mcpwm_del_timer(timer);
        }

        mcpwm_timer_config_t timer_config = {
                .group_id = 0,
                .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
                .resolution_hz = BLDC_MCPWM_TIMER_RESOLUTION_HZ,
                .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
                .period_ticks = period_ticks,
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
    }

    void start() {
        ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    }

    void change_frequency(uint32_t new_period_ticks) {
        if (period_ticks == new_period_ticks)
            return;

        _newTimer(new_period_ticks);

        period_ticks = new_period_ticks;
    }
};

struct PWMComparator {
    mcpwm_oper_handle_t operator_ = nullptr;
    mcpwm_cmpr_handle_t comparator  = nullptr;

    PWMComparator() {
        mcpwm_operator_config_t operator_config = {.group_id = 0,};
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator_));

        mcpwm_comparator_config_t compare_config = {.flags = {.update_cmp_on_tez = true}};
        ESP_ERROR_CHECK(mcpwm_new_comparator(operator_, &compare_config, &comparator));
    }

    void connect_timer(const PWMTimer &timer) {
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_, timer.timer));
    }

    void set_compare_value(uint32_t ticks) {
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comp0, ticks));
    }
};

struct PWMSignal {
    PWMTimer & timer;

    uint32_t duty_cycle;
    mcpwm_oper_handle_t operator_ = nullptr;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator = nullptr;

    bool softStarting = false;


    PWMSignal(PWMTimer &timer, uint32_t duty_cycle, uint8_t gpio_num)
            : timer(timer), duty_cycle(duty_cycle) {
        mcpwm_operator_config_t operator_config = {.group_id = 0,};
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator_));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_, timer.timer));


        mcpwm_comparator_config_t compare_config = {.flags = {.update_cmp_on_tez = true}};
        //compare_config.flags.update_cmp_on_tez = true;
        ESP_ERROR_CHECK(mcpwm_new_comparator(operator_, &compare_config, &comparator));

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty_cycle)); // set duty cycle


        mcpwm_generator_config_t gen_config = {.gen_gpio_num = gpio_num};
        ESP_ERROR_CHECK(mcpwm_new_generator(operator_, &gen_config, &generator));
    }

    void enable(bool enable, bool wait = false) {
        auto high = enable ? MCPWM_GEN_ACTION_HIGH : MCPWM_GEN_ACTION_LOW;
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                generator,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, high)
        ));
        if (wait)
            vTaskDelay(pdMS_TO_TICKS(timer.period_ticks * 1000 / BLDC_MCPWM_TIMER_RESOLUTION_HZ * 2));
    }

    void set_duty_cycle(uint32_t new_duty_cycle) {
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, new_duty_cycle));
        duty_cycle = new_duty_cycle;
    }

    void connect_timer( PWMTimer &timer_) {
        timer = timer_;
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_, timer.timer));
        //ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty_cycle));
    }
};


struct PWMSignalHalfPeriod {
    PWMTimer & timer;

    uint32_t duty_cycle;
    PWMComparator comp0{}, comp1{}, comp2{};
    mcpwm_gen_handle_t generator = nullptr;

    bool softStarting = false;


    PWMSignalHalfPeriod(PWMTimer &timer, uint32_t duty_cycle, uint8_t gpio_num)
            : timer(timer), duty_cycle(duty_cycle) {

        comp0.connect_timer(timer);
        comp1.connect_timer(timer);
        comp2.connect_timer(timer);

        _updateComps();

        mcpwm_generator_config_t gen_config = {.gen_gpio_num = gpio_num};
        ESP_ERROR_CHECK(mcpwm_new_generator(operator_, &gen_config, &generator));
    }

    void _updateComps() {
        comp0.set_compare_value(duty_cycle);
        comp1.set_compare_value( timer.period_ticks / 2);
        comp2.set_compare_value(timer.period_ticks / 2 + duty_cycle);
    }

    void enable(bool enable, bool wait = false) {
        auto high = enable ? MCPWM_GEN_ACTION_HIGH : MCPWM_GEN_ACTION_LOW;
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                generator,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, high)
        ));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
                generator,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comp0.comparator, MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comp1.comparator, high),
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comp2.comparator, MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END()
        ));
        if (wait)
            vTaskDelay(pdMS_TO_TICKS(timer.period_ticks * 1000 / BLDC_MCPWM_TIMER_RESOLUTION_HZ * 2));
    }

    void set_duty_cycle(uint32_t new_duty_cycle) {
        duty_cycle = new_duty_cycle;
        _updateComps();
    }

    void connect_timer(PWMTimer &timer_) {
        timer = timer_;
        comp0.connect_timer(timer);
        comp1.connect_timer(timer);
        comp2.connect_timer(timer);
        //ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_, timer.timer));
        //ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty_cycle));
    }
};

void pwm_soft_start(PWMSignal &signal, uint32_t milliseconds, uint32_t steps) {
    if (signal.softStarting) {
        ESP_LOGE("pwm", "already soft-starting, please wait");
        return;
    }

    ESP_LOGI("pwm", "Soft-start to %lu..", signal.duty_cycle);
    signal.softStarting = true;
    soft_start_args *timer_args = new soft_start_args(&signal, signal.duty_cycle, steps);

    signal.set_duty_cycle(0);
    signal.enable(true);

    const esp_timer_create_args_t periodic_timer_args = {.callback = update_soft_start_callback, .arg = timer_args};
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &timer_args->timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_args->timer, milliseconds * 1000 / steps));
}

float map(float x, float x0, float x1, float y0, float y1) {
    return (x - x0) / (x1 - x0) * (y1 - y0) + y0;
}

void update_soft_start_callback(void *args_) {
    soft_start_args *args = (soft_start_args *) args_;

    PWMSignal *pwmSignal = (PWMSignal *) args->arg;

    float p = (float) args->i / (float) args->steps;
    float duty_cycle = map(p * p * p, 0, 1, 0, pwmSignal->duty_cycle);

    ESP_LOGD("pwm", "soft-start p=%.2f duty_cycle=%.2f", p, duty_cycle);

    pwmSignal->set_duty_cycle(duty_cycle);

    if (args->i >= args->steps) {
        ESP_LOGI("pwm", "Soft-start finished");
        esp_timer_stop(args->timer);
        esp_timer_delete(args->timer);
        delete args;
        pwmSignal->softStarting = false;
    } else {
        ++(args->i);
    }
}