#include <rcl/rcl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "soc/rtc.h"

#include "sonar.h"


typedef struct sonar_impl_t
{
    sonar_options_t options;
    uint32_t current_cap_value;
    uint32_t previous_cap_value;
    uint32_t capture_signal;
    QueueHandle_t queue;

} sonar_impl_t;

static void IRAM_ATTR isr_sonar_handler(void * args)
{
    sonar_impl_t * sonar = (sonar_impl_t *)args;
    uint8_t pin_state = gpio_get_level(sonar->options.echo);
    if(pin_state == 1)
    {
        sonar->previous_cap_value = esp_timer_get_time();

    } else if (pin_state == 0)
    {

        sonar->current_cap_value = esp_timer_get_time();
        sonar->capture_signal = sonar->current_cap_value - sonar->previous_cap_value;
        if(sonar->options.send && sonar->capture_signal < 10000)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            float distance = sonar->capture_signal / 58.2;
            xQueueOverwriteFromISR(sonar->queue, &distance, &xHigherPriorityTaskWoken );
            if( xHigherPriorityTaskWoken == pdTRUE )
            {
                portYIELD_FROM_ISR();
            }
        }


    }

}

rcl_ret_t
sonar_init(
    sonar_t * sonar,
    const sonar_options_t * options,
    const rcl_allocator_t * allocator)
{
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ALLOCATOR_WITH_MSG(allocator, "invalid allocator", return RCL_RET_INVALID_ARGUMENT);

    sonar->impl = (sonar_impl_t *)allocator->allocate(sizeof(sonar_impl_t), allocator->state);

    sonar->impl->options = *options;
    gpio_config_t gp;
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode = GPIO_MODE_OUTPUT;
    gp.pin_bit_mask = BIT(options->trigger) ;
    gpio_config(&gp);



    gpio_pad_select_gpio(options->echo);
    gpio_set_pull_mode(options->echo, GPIO_PULLUP_ONLY);
    gpio_set_direction(options->echo, GPIO_MODE_INPUT);
    gpio_set_intr_type(options->echo, GPIO_INTR_ANYEDGE);

    gpio_set_level(options->trigger, 0); //Set trigger low

    if(options->send)
    {
        sonar->impl->queue = xQueueCreate(1, sizeof(float));
    }
    gpio_isr_handler_add(options->echo, isr_sonar_handler, sonar->impl);
    return RCL_RET_OK;
}

void
scan(
    sonar_t * sonar)
{

    gpio_set_level(sonar->impl->options.trigger, 1); // Set trigger hight for 50 ns
    vTaskDelay((TickType_t) 50);
    gpio_set_level(sonar->impl->options.trigger, 0); // Set trigger low

}

bool
receive(
    sonar_t * sonar,
    void * const pvBuffer,
    TickType_t xTicksToWait)
{
    return xQueueReceive(sonar->impl->queue, pvBuffer, xTicksToWait) == pdPASS;
}
