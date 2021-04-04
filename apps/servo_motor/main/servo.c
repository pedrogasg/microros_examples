#include <rcl/rcl.h>

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "servo.h"

typedef struct servo_impl_t
{
    servo_options_t options;

}servo_impl_t;

rcl_ret_t
servo_init(
    servo_t * servo,
    const servo_options_t * options,
    const rcl_allocator_t * allocator)
{
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ALLOCATOR_WITH_MSG(allocator, "invalid allocator", return RCL_RET_INVALID_ARGUMENT);

    servo->impl = (servo_impl_t *)allocator->allocate(sizeof(servo_impl_t), allocator->state);

    servo->impl->options = *options;

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, options->gpio);    //Set GPIO 14 as PWM0A, to which servo is connected

	// Configuration

    mcpwm_config_t pwm_config;
    pwm_config.frequency = options->frequency;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    return RCL_RET_OK;
}

int32_t
servo_per_degree(
    servo_t * servo,
    int32_t degree_of_rotation)
{
    int32_t cal_pulsewidth = 0;
    if(degree_of_rotation > -1 && degree_of_rotation <= servo->impl->options.max_degree){
        cal_pulsewidth = (servo->impl->options.min_pulse + (((servo->impl->options.max_pulse - servo->impl->options.min_pulse) * (degree_of_rotation)) / (servo->impl->options.max_degree)));
    }
    return cal_pulsewidth;
}

void
servo_control(
    servo_t * servo,
    int32_t degree)
{
	int32_t angle;
	angle = servo_per_degree(servo, degree);
	printf("pulse width: %dus\n", angle);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
    return;
}
