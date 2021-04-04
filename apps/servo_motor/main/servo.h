# pragma once

#include <rcl/rcl.h>

#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate


struct servo_impl_t;

typedef struct servo_t
{
    struct servo_impl_t * impl;
    
}servo_t;

typedef struct servo_options_t
{   int32_t gpio; // PINOUT in the ESP32 board    
    int32_t max_pulse; //Minimum pulse width in microsecond
    int32_t min_pulse; //Maximum pulse width in microsecond
    int32_t max_degree; //Maximum angle in degree upto which servo can rotate
    int32_t frequency; //50Hz, i.e. for every servo motor time period should be 20ms
}servo_options_t;


rcl_ret_t
servo_init(
    servo_t * servo,
    const servo_options_t * options,
    const rcl_allocator_t * allocator);

int32_t
servo_per_degree(
    servo_t * servo,
    int32_t degree_of_rotation);

void
servo_control(
    servo_t * servo,
    int32_t degree);
    