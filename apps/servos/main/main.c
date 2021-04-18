#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <stdio.h>
#include <unistd.h>


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_attr.h>
#include <esp_log.h>

#include <uros_network_interfaces.h>
#include <rmw_uros/options.h>
#include "uxr/client/config.h"

#include <pca9685.h>

#define ADDR PCA9685_ADDR_BASE
#define SDA_GPIO 12
#define SCL_GPIO 14
#define PWM_FREQ_HZ 50

#define SERVO_MAX 500
#define SERVO_MIN 120
#define MAX_DEGREE 180


static const char *TAG = "pca9685_test";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
i2c_dev_t dev;

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	uint32_t pulse;
	if(msg->data > -1 && msg->data <= MAX_DEGREE)
	{
        pulse = SERVO_MIN + (((SERVO_MAX - SERVO_MIN) * msg->data) / MAX_DEGREE);

		if (pca9685_set_pwm_value(&dev, 0, pulse) != ESP_OK)
		ESP_LOGE(TAG, "Could not set PWM value to ch0");

		if (pca9685_set_pwm_value(&dev, 1, pulse) != ESP_OK)
				ESP_LOGE(TAG, "Could not set PWM value to ch1");

		if (pca9685_set_pwm_value(&dev, 2, pulse) != ESP_OK)
				ESP_LOGE(TAG, "Could not set PWM value to ch2");
	}
}



void micro_ros_task(void * arg)
{

  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "servo_subscriber_rclc", "", &support));

	i2cdev_init();

    memset(&dev, 0, sizeof(i2c_dev_t));

	ESP_ERROR_CHECK(pca9685_init_desc(&dev, ADDR, 0, SDA_GPIO, SCL_GPIO));
	ESP_ERROR_CHECK(pca9685_init(&dev));

	ESP_ERROR_CHECK(pca9685_restart(&dev));

    uint16_t freq;
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&dev, PWM_FREQ_HZ));
    ESP_ERROR_CHECK(pca9685_get_pwm_frequency(&dev, &freq));

	ESP_LOGI(TAG, "Freq %dHz, real %d", PWM_FREQ_HZ, freq);


	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"/servo/degrees"));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));



	while(1){
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			usleep(100000);
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
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
