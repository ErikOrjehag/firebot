#!/bin/sh

git_root="$(git rev-parse --show-toplevel)"

xhost + > /dev/null

docker run\
  --name firebot\
  --network host\
  -e TOKEN\
  -v /tmp/.X11-unix:/tmp/.X11-unix\
  -e DISPLAY=$DISPLAY\
  --env="QT_X11_NO_MITSHM=1"\
  --device=/dev/dri:/dev/dri\
  -v "${git_root}":/dev_ws/src/firebot\
  -it --rm firebot bash
