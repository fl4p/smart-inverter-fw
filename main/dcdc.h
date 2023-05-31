
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct  {
    float dutyCycle;//0-1
    uint32_t frequency;
} dcdc_params;

void dcdc_main();

esp_err_t dcdc_set_params(dcdc_params params);


#ifdef __cplusplus
}
#endif