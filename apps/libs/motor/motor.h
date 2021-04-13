# pragma once

#include <rcl/rcl.h>

struct motor_impl_t;

typedef struct motor_t
{
    struct motor_impl_t * impl;
    
}motor_t;

typedef struct motor_options_t
{   int32_t in1; // PINOUT in the ESP32 board    
    int32_t in2; // PINOUT in the ESP32 board    
    int32_t duty; // PINOUT in the ESP32 board    
    int32_t frequency; //500Hz, i.e. for every motor motor time period should be 2ms
}motor_options_t;


rcl_ret_t
motor_init(
    motor_t * motor,
    const motor_options_t * options,
    const rcl_allocator_t * allocator);

void
motor_reverse(
    motor_t * motor,
    bool reverse);

void
motor_control(
    motor_t * motor,
    int32_t duty);
    