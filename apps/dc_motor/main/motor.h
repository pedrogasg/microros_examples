# pragma once

#include <rcl/rcl.h>

#define motor_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define motor_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define motor_MAX_DEGREE 90 //Maximum angle in degree upto which motor can rotate


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
    