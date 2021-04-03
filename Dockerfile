FROM ros:foxy

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash" 

RUN mkdir microros_ws

WORKDIR /microros_ws

RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

RUN sudo apt update && rosdep update
RUN rosdep install --from-path src --ignore-src -y

RUN sudo apt-get install python3-pip -y

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && source /microros_ws/install/local_setup.bash && /opt/ros/foxy/bin/ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32"

COPY apps /microros_ws/firmware/freertos_apps/apps/