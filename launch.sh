#!/bin/sh

docker build -t esp-microros .

docker run -it --rm --env WIFI_PASSWORD --env SSID --env MYIP --volume="$PWD"/apps:/microros_ws/component/apps:rw --net=host -v /dev:/dev --privileged esp-microros /bin/bash
