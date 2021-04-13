#include <rcl/rcl.h>

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "motor.h"

typedef struct motor_impl_t
{
    motor_options_t options;
    bool forward;

}motor_impl_t;

rcl_ret_t
motor_init(
    motor_t * motor,
    const motor_options_t * options,
    const rcl_allocator_t * allocator)
{
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ALLOCATOR_WITH_MSG(allocator, "invalid allocator", return RCL_RET_INVALID_ARGUMENT);

    motor->impl = (motor_impl_t *)allocator->allocate(sizeof(motor_impl_t), allocator->state);

    motor->impl->options = *options;

    motor->impl->forward = true;

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, options->duty);
	// Configuration

    mcpwm_config_t pwm_config;
    pwm_config.frequency = options->frequency;    //frequency = 50Hz, i.e. for every motor motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    gpio_config_t gp;
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode = GPIO_MODE_OUTPUT;
    gp.pin_bit_mask = BIT(options->in1) | BIT(options->in2) ;
    gpio_config(&gp);

    gpio_set_level(options->in1, 1); //Set IN1 high
    gpio_set_level(options->in2, 0); //Set IN2 low

    return RCL_RET_OK;
}

void
motor_reverse(
    motor_t * motor,
    bool reverse)
{
    motor->impl->forward = reverse;
    gpio_set_level(motor->impl->options.in1, !motor->impl->forward); // Change direction
    gpio_set_level(motor->impl->options.in2, motor->impl->forward); // Change direction
//    motor->impl->forward = !motor->impl->forward;
}

void
motor_control(
    motor_t * motor,
    int32_t duty)
{
	printf("pulse width: %dus\n", duty);
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
    return;
}
