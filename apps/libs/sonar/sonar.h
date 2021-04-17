# pragma once

#include <rcl/rcl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct sonar_impl_t;

typedef struct sonar_t
{
    struct sonar_impl_t * impl;
    
} sonar_t;

typedef struct sonar_options_t
{
    int32_t trigger;
    int32_t echo;
    bool send;

} sonar_options_t;

rcl_ret_t
sonar_init(
    sonar_t * motor,
    const sonar_options_t * options,
    const rcl_allocator_t * allocator);


void
scan(
    sonar_t * sonar);

bool
receive(
    sonar_t * sonar,
    void * const pvBuffer,
    TickType_t xTicksToWait);