# pragma once

#include <rcl/rcl.h>
#include "freertos/FreeRTOS.h"
#include "driver/adc.h"

struct dmsu_impl_t ;

typedef struct dmsu_t
{
    struct dmsu_impl_t * impl;

} dmsu_t;

typedef struct dmsu_model_t
{
    float a;
    float b;
    
} dmsu_model_t;

typedef struct dmsu_options_t
{
    adc_channel_t channel;
    adc_bits_width_t width;
    uint32_t no_samples;
    dmsu_model_t model;

} dmsu_options_t;


rcl_ret_t
init_dmsu_sensor(
    dmsu_t * sensor,
    const dmsu_options_t * options,
    const rcl_allocator_t * allocator);


uint32_t
read_voltage(
    dmsu_t * sensor);

float
get_distance(
    dmsu_t * sensor);




static const dmsu_model_t GP2Y0A41SK0F = {59.4375,0.019375};

static const dmsu_model_t GP2Y0A51SK0F = {27.1875,0.01375};