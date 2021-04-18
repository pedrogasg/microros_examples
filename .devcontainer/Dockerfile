FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN echo "Set disable_coredump false" >> /etc/sudo.conf

# We need libpython2.7 due to GDB tools
RUN apt-get update && apt-get install -y \
    apt-utils \
    bison \
    ca-certificates \
    ccache \
    check \
    curl \
    flex \
    git \
    gperf \
    lcov \
    libncurses-dev \
    libusb-1.0-0-dev \
    make \
    ninja-build \
    libpython2.7 \
    python3 \
    python3-pip \
    unzip \
    wget \
    xz-utils \
    zip \
   && apt-get autoremove -y \
   && rm -rf /var/lib/apt/lists/* \
   && update-alternatives --install /usr/bin/python python /usr/bin/python3 10

RUN python -m pip install --upgrade pip virtualenv

# To build the image for a branch or a tag of IDF, pass --build-arg IDF_CLONE_BRANCH_OR_TAG=name.
# To build the image with a specific commit ID of IDF, pass --build-arg IDF_CHECKOUT_REF=commit-id.
# It is possibe to combine both, e.g.:
#   IDF_CLONE_BRANCH_OR_TAG=release/vX.Y
#   IDF_CHECKOUT_REF=<some commit on release/vX.Y branch>.

ARG IDF_CLONE_URL=https://github.com/espressif/esp-idf.git
ARG IDF_CLONE_BRANCH_OR_TAG=master
ARG IDF_CHECKOUT_REF=

ENV IDF_PATH=/opt/esp/idf
ENV IDF_TOOLS_PATH=/opt/esp

RUN echo IDF_CHECKOUT_REF=$IDF_CHECKOUT_REF IDF_CLONE_BRANCH_OR_TAG=$IDF_CLONE_BRANCH_OR_TAG && \
    git clone --recursive \
      ${IDF_CLONE_BRANCH_OR_TAG:+-b $IDF_CLONE_BRANCH_OR_TAG} \
      $IDF_CLONE_URL $IDF_PATH && \
    if [ -n "$IDF_CHECKOUT_REF" ]; then \
      cd $IDF_PATH && \
      git checkout $IDF_CHECKOUT_REF && \
      git submodule update --init --recursive; \
    fi

# Install all the required tools, plus CMake
RUN $IDF_PATH/tools/idf_tools.py --non-interactive install required \
  && $IDF_PATH/tools/idf_tools.py --non-interactive install cmake \
  && $IDF_PATH/tools/idf_tools.py --non-interactive install-python-env \
  && rm -rf $IDF_TOOLS_PATH/dist

# Ccache is installed, enable it by default
ENV IDF_CCACHE_ENABLE=1

RUN apt update -q && \
    apt install -yq sudo lsb-release gosu nano && \
    rm -rf /var/lib/apt/lists/*

ARG TZ_ARG=UTC
ENV TZ=$TZ_ARG
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone


# Install ROS2
RUN apt-get update && apt-get install -y \
    zsh \
    curl \
    gnupg2 \
    lsb-release \
  && curl http://repo.ros2.org/repos.key | apt-key add - \
  && sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
  && apt-get update && apt-get install -y \
    ros-foxy-ros-base \
    python3-argcomplete \
    python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*


RUN /bin/zsh -c ". $IDF_PATH/export.sh && pip3 install catkin_pkg lark-parser empy"

ARG USER_ID=espidf

RUN useradd --create-home --home-dir /home/$USER_ID --shell /bin/bash --user-group --groups adm,sudo $USER_ID && \
    echo $USER_ID:$USER_ID | chpasswd && \
    echo "$USER_ID ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update && apt-get install -y \
    ros-foxy-osrf-testing-tools-cpp \
    ros-foxy-test-msgs

RUN mkdir -p /rclc_ws/src

RUN git clone -b foxy https://github.com/ros2/rclc.git /rclc_ws/src

RUN git clone -b foxy https://github.com/ros2/example_interfaces.git /rclc_ws/src/example_interfaces

WORKDIR /rclc_ws

RUN /bin/zsh -c "source /opt/ros/foxy/setup.zsh && colcon build"

WORKDIR /microros_ws

RUN git clone -b foxy https://github.com/micro-ROS/micro_ros_espidf_component.git component



RUN git clone -b master https://github.com/UncleRus/esp-idf-lib.git esp-idf-lib

