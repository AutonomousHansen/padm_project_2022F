xhost +
sudo docker run -it --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix padm-project-2022f bash
xhost -