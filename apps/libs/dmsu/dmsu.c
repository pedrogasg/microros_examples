

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "dmsu.h"


#define DEFAULT_VREF    1100        //Use default vref for start

typedef struct dmsu_impl_t
{
    dmsu_options_t options;
    esp_adc_cal_characteristics_t * adc_chars;


} dmsu_impl_t;

rcl_ret_t
init_dmsu_sensor(
    dmsu_t * sensor,
    const dmsu_options_t * options,
    const rcl_allocator_t * allocator)
{
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ALLOCATOR_WITH_MSG(allocator, "invalid allocator", return RCL_RET_INVALID_ARGUMENT);

    sensor->impl = (dmsu_impl_t *)allocator->allocate(sizeof(dmsu_impl_t), allocator->state);

    sensor->impl->options = *options;


    adc1_config_width(options->width);
    adc1_config_channel_atten(options->channel, ADC_ATTEN_DB_11);

    sensor->impl->adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, options->width, DEFAULT_VREF, sensor->impl->adc_chars);

    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }

    return RCL_RET_OK;
}

uint32_t
read_voltage(
    dmsu_t * sensor)
{
    uint32_t adc_reading = 0;
    //Multisampling
    uint32_t no_samples = sensor->impl->options.no_samples;
    for (int i = 0; i < no_samples; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)sensor->impl->options.channel);
    }
    adc_reading /= no_samples;
    return esp_adc_cal_raw_to_voltage(adc_reading, sensor->impl->adc_chars);
}

float
get_distance(
    dmsu_t * sensor)
{

    float voltage = read_voltage(sensor);

    const float a = sensor->impl->options.model.a;
    const float b = sensor->impl->options.model.b;
    float dist = 0;
    if ( voltage > b ) {
        dist = a / (voltage - b);
    }
    return dist;
}
