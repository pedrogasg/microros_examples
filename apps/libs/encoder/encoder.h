# pragma once

#include <rcl/rcl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct encoder_impl_t;

typedef struct encoder_t
{
    struct encoder_impl_t * impl;
    
}encoder_t;

typedef struct encoder_options_t
{   int32_t a;
    int32_t b;
    bool send;
}encoder_options_t;

rcl_ret_t
encoder_init(
    encoder_t * encoder,
    const encoder_options_t * options,
    const rcl_allocator_t * allocator);


int32_t
get_counter(
    encoder_t * encoder);

bool
receive(
    encoder_t * encoder, void * const pvBuffer, TickType_t xTicksToWait);

void
reset(
    encoder_t * encoder);

