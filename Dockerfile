FROM microros/esp-idf-microros:latest

WORKDIR /microros_ws

RUN git clone -b foxy https://github.com/micro-ROS/micro_ros_espidf_component.git component



RUN git clone -b master https://github.com/UncleRus/esp-idf-lib.git esp-idf-lib