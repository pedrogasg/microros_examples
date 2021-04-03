#!/bin/sh

docker build -t espmicroros .

docker run -it --rm --env WIFI_PASSWORD --env SSID --env MYIP --volume="$PWD"/apps:/tmp/apps:rw --net=host -v /dev:/dev --privileged espmicroros