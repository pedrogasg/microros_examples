./launch

source /opt/ros/foxy/setup.bash

source /microros_ws/install/local_setup.bash 

echo "CONFIG_ESP_WIFI_SSID=$SSID" >> /microros_ws/firmware/freertos_apps/microros_esp32_extensions/sdkconfig.defaults
echo "CONFIG_ESP_WIFI_PASSWORD=$WIFI_PASSWORD" >> /microros_ws/firmware/freertos_apps/microros_esp32_extensions/sdkconfig.defaults

ros2 run micro_ros_setup configure_firmware.sh servo_motor -t udp -i ${MYIP} -p 8888

ros2 run micro_ros_setup build_firmware.sh

ros2 run micro_ros_setup flash_firmware.sh

ros2 topic pub --once /servo/degrees std_msgs/int32 "data: 50"