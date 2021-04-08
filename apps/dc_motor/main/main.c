#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <std_msgs/msg/bool.h>

#include <stdio.h>
#include <unistd.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include <uros_network_interfaces.h>
#include <rmw_uros/options.h>
#include "uxr/client/config.h"

#include "motor.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t duty_subscriber;
rcl_subscription_t reverse_subscriber;
std_msgs__msg__Int32 duty_msg;
std_msgs__msg__Bool reverse_msg;
motor_t motor;

void subscription_duty_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * duty_msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Duty received: %d\n", duty_msg->data);
	motor_control(&motor, duty_msg->data);
}

void subscription_reverse_callback(const void * msgin)
{
	const std_msgs__msg__Bool * reverse_msg = (const std_msgs__msg__Bool *)msgin;
	printf("Reverse received: %d\n", reverse_msg->data);
	motor_reverse(&motor, reverse_msg->data);
}

void micro_ros_task(void * arg)
{
	printf("Startuing task");

  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	printf("rclc init");
	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
	printf("uros ip init");
	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "motor_duty_subscriber_rclc", "", &support));
	printf("rosnode init");
	motor_options_t options;
	options.frequency = 1000; // ~ 20ms
	options.in1 = 12; // you can choose any
	options.in2 = 14; // you can choose any
	options.duty = 27; // you can choose any

	motor_init(&motor, &options, &allocator);
	printf("motor init");

	// create duty_subscriber
	RCCHECK(rclc_subscription_init_default(
		&duty_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"/motor/duty"));

	// create reverse_subscriber
	RCCHECK(rclc_subscription_init_default(
		&reverse_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
		"/motor/reverse"));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &duty_subscriber, &duty_msg, &subscription_duty_callback, ON_NEW_DATA));
	printf("duty_subscriber init");
	RCCHECK(rclc_executor_add_subscription(&executor, &reverse_subscriber, &reverse_msg, &subscription_reverse_callback, ON_NEW_DATA));
	printf("reverse_subscriber init");
	while(1){
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			usleep(100000);
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&duty_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&reverse_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
	
	vTaskDelete(NULL);

	
}
void app_main(void)
{   
#ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif  // UCLIENT_PROFILE_UDP

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task, 
            "uros_task", 
            CONFIG_MICRO_ROS_APP_STACK, 
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO, 
            NULL); 
}
