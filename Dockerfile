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
    mesa-utils \
    ros-foxy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*
RUN usermod -a -G video root

WORKDIR $WS/src
RUN pip3 install -r requirements.txt

RUN git clone https://github.com/ros-teleop/teleop_tools.git --single-branch foxy-devel
RUN mv foxy-devel teleop_tools
COPY . firebot

WORKDIR $WS
RUN bash -c "source /opt/ros/foxy/setup.bash && colcon build --symlink-install"

RUN echo "source /dev_ws/install/setup.bash" >> ~/.bashrc
