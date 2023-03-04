
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct  {
    uint32_t deadTime100Ns;
    uint32_t frequency;
} dcdc_params;

void dcdc_main();

esp_err_t dcdc_set_params(dcdc_params params);


#ifdef __cplusplus
}
#endif