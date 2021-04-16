#include <rcl/rcl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "soc/rtc.h"

#include "sonar.h"

#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};

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
    uint32_t mcpwm_intr_status;
    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status
    //calculate the interval in the ISR,
    //so that the interval will be always correct even when cap_queue is not handled in time and overflow.

    if (mcpwm_intr_status & CAP0_INT_EN) 
    { //Check for interrupt on rising edge on CAP0 signal
        sonar->current_cap_value = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
        sonar->capture_signal = (sonar->current_cap_value - sonar->previous_cap_value) / (rtc_clk_apb_freq_get() / 1000000);
        sonar->previous_cap_value = sonar->current_cap_value;
        if(sonar->options.send)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueOverwriteFromISR(sonar->queue, &sonar->capture_signal, &xHigherPriorityTaskWoken );
            if( xHigherPriorityTaskWoken == pdTRUE )
            {
                portYIELD_FROM_ISR();
            }
        }
    }

    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status; // Clear status 
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

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, options->echo);
	// Configuration

    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_CAP_0, MCPWM_POS_EDGE, 0);



    gpio_set_level(options->trigger, 0); //Set trigger low
    gpio_pulldown_en(options->echo);

    MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN;  //Enable interrupt on  CAP0, https://github.com/espressif/esp-idf/blob/master/components/soc/esp32s3/include/soc/mcpwm_struct.h

    mcpwm_isr_register(MCPWM_UNIT_0, isr_sonar_handler, sonar->impl, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler

    if(options->send)
    {
        sonar->impl->queue = xQueueCreate(1, sizeof(uint32_t));
    }

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

uint32_t
get_signal(
    sonar_t * sonar)
{
    return sonar->impl->capture_signal;
}

bool
receive(
    sonar_t * sonar,
    void * const pvBuffer,
    TickType_t xTicksToWait)
{
    return xQueueReceive(sonar->impl->queue, pvBuffer, xTicksToWait) == pdPASS;
}
