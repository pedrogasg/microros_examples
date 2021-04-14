#include <rcl/rcl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "encoder.h"

// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20
#define R_START 0x0
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6



typedef struct encoder_impl_t
{
    encoder_options_t options;
    QueueHandle_t queue;
    int32_t counter;
    uint8_t state;
    
}encoder_impl_t;

static const uint8_t ttable[7][4] = {
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};

static void init_gpio(gpio_num_t pin)
{
    gpio_pad_select_gpio(pin);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE);
}

static void IRAM_ATTR _isr_process(void * args)
{
    encoder_impl_t * encoder = (encoder_impl_t *)args;

    uint8_t pin_state = (gpio_get_level(encoder->options.b) << 1) | gpio_get_level(encoder->options.a);
    bool update = false;
    encoder->state = ttable[encoder->state & 0xf][pin_state];

    switch (encoder->state & 0x30)
    {
    case DIR_CW:
        update = true;
        encoder->counter++;
        break;
    case DIR_CCW:
        encoder->counter--;
        update = true;
        break;
    default:
        break;
    }

    if(encoder->options.send && update)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueOverwriteFromISR(encoder->queue, &encoder->counter, &xHigherPriorityTaskWoken );
        if( xHigherPriorityTaskWoken == pdTRUE )
        {
            portYIELD_FROM_ISR();
        }
    }
}

rcl_ret_t
encoder_init(
    encoder_t * encoder,
    const encoder_options_t * options,
    const rcl_allocator_t * allocator)
{
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ALLOCATOR_WITH_MSG(allocator, "invalid allocator", return RCL_RET_INVALID_ARGUMENT);

    encoder->impl = (encoder_impl_t *)allocator->allocate(sizeof(encoder_impl_t), allocator->state);

    encoder->impl->options = *options;

    encoder->impl->counter = 0;

    encoder->impl->state = R_START;

    init_gpio(options->a);
    init_gpio(options->b);

    if(options->send)
    {
        encoder->impl->queue = xQueueCreate(1, sizeof(int32_t));
    }

    gpio_isr_handler_add(options->a, _isr_process, encoder->impl);
    gpio_isr_handler_add(options->b, _isr_process, encoder->impl);
    return RCL_RET_OK;
}

int32_t
get_counter(
    encoder_t * encoder)
{
    return encoder->impl->counter;
}

bool
receive(
    encoder_t * encoder,
    void * const pvBuffer,
    TickType_t xTicksToWait)
{
    return xQueueReceive(encoder->impl->queue, pvBuffer, xTicksToWait) == pdPASS;
}

void
reset(
    encoder_t * encoder)
{
    encoder->impl->counter = 0;
    encoder->impl->state = R_START;
    return;
}

