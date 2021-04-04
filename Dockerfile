FROM microros/esp-idf-microros:latest

WORKDIR /microros_ws

RUN git clone -b foxy https://github.com/pedrogasg/micro_ros_espidf_component.git component



