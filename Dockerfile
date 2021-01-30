FROM ros:foxy-ros-base-focal
RUN apt update
RUN apt install -y x11-apps python3-pip
ENV WS=/dev_ws
RUN mkdir -p $WS/src
COPY ros/requirements.txt $WS/src/requirements.txt

RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-common-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*
RUN usermod -a -G video root

WORKDIR $WS/src
RUN pip3 install -r requirements.txt

RUN git clone --single-branch foxy-devel https://github.com/ros-teleop/teleop_tools.git

WORKDIR $WS
RUN colcon build --symlink-install && source install/setup.bash