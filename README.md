./launch

echo "CONFIG_ESP_WIFI_SSID=\"$SSID\"" >> /microros_ws/component/apps/servo_motor/sdkconfig.defaults
echo "CONFIG_ESP_WIFI_PASSWORD=\"$WIFI_PASSWORD\"" >> /microros_ws/component/apps/servo_motor/sdkconfig.defaults

echo "CONFIG_MICRO_ROS_AGENT_IP=\"$MYIP\"" >> /microros_ws/component/apps/servo_motor/sdkconfig.defaults

echo "CONFIG_MICRO_ROS_AGENT_PORT=\"8888\"" >> /microros_ws/component/apps/servo_motor/sdkconfig.defaults

ros2 topic pub --once /motor/duty std_msgs/Int32 "data: 50"


sed -i 's/"-DRMW_UXRCE_MAX_SUBSCRIPTIONS=1"/"-DRMW_UXRCE_MAX_SUBSCRIPTIONS=2"/g' colcon.meta
