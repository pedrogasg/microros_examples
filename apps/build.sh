#!/bin/bash

APP_DIRECTORY=$1

if [ -d "$APP_DIRECTORY" ] 
then
    cd $APP_DIRECTORY
    
    if [ ! -f "sdkconfig" ]; then
        echo "CONFIG_ESP_WIFI_SSID=\"$SSID\"" >> sdkconfig.defaults

        echo "CONFIG_ESP_WIFI_PASSWORD=\"$WIFI_PASSWORD\"" >> sdkconfig.defaults

        echo "CONFIG_MICRO_ROS_AGENT_IP=\"$MYIP\"" >> sdkconfig.defaults

        echo "CONFIG_MICRO_ROS_AGENT_PORT=\"8888\"" >> sdkconfig.defaults
    fi

    idf.py build

    read -n 1 -s -r -p 'Flash the device y/n: ' flash; echo

    if [ "$flash" = "y" ]; then

        read -n 1 -s -r -p 'Monitor after flash y/n: ' monitor; echo

        if [ "$monitor" = "y" ]; then
            idf.py flash monitor
        else
            idf.py flash
        fi
    fi


else
    echo "Error: Directory $APP_DIRECTORY does not exists in applications folder."
fi

