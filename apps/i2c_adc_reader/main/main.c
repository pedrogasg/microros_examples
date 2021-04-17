#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_uros/options.h>
#include "uxr/client/config.h"

#include <ads111x.h>

#define SDA_GPIO 12
#define SCL_GPIO 14

#define I2C_PORT 0

#define GAIN ADS111X_GAIN_4V096 // +-4.096V

static i2c_dev_t device;

static float gain_val;

static float measure()
{
	ESP_ERROR_CHECK(ads111x_start_conversion(&device));

    bool busy;
    do
    {
        ads111x_is_busy(&device, &busy);
    }
    while (busy);

    // Read result
    int16_t raw = 0;
    if (ads111x_get_value(&device, &raw) == ESP_OK)
    {
        float voltage = gain_val / ADS111X_MAX_VALUE * raw;
        return voltage;
    }
    else
        return 0.0;
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) 
	{
		ESP_ERROR_CHECK(ads111x_set_input_mux(&device, ADS111X_MUX_0_GND));    // positive = AIN0, negative = GND
		msg.data = measure();
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		ESP_ERROR_CHECK(ads111x_set_input_mux(&device, ADS111X_MUX_1_GND));    // positive = AIN0, negative = GND
		msg.data = measure();
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

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
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "distance_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"distance_publisher"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	ESP_ERROR_CHECK(i2cdev_init());

    // Clear device descriptors
    memset(&device, 0, sizeof(i2c_dev_t));

	gain_val = ads111x_gain_values[GAIN];

    // Setup ICs

	ESP_ERROR_CHECK(ads111x_init_desc(&device, ADS111X_ADDR_GND, I2C_PORT, SDA_GPIO, SCL_GPIO));

	ESP_ERROR_CHECK(ads111x_set_mode(&device, ADS111X_MODE_SINGLE_SHOT));    // Continuous conversion mode
	ESP_ERROR_CHECK(ads111x_set_data_rate(&device, ADS111X_DATA_RATE_32)); // 32 samples per second
	
	ESP_ERROR_CHECK(ads111x_set_gain(&device, GAIN));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));


	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
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