FROM ros:foxy-ros-base-focal
RUN apt update
RUN apt install -y x11-apps python3-pip
ENV WS=/dev_ws/src
RUN mkdir -p $WS
COPY ros/requirements.txt $WS/requirements.txt
WORKDIR $WS

#RUN apt-get install -y xserver-xorg-video-all
#RUN apt-get install mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev

RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-common-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*
RUN usermod -a -G video root

RUN pip3 install -r requirements.txt
