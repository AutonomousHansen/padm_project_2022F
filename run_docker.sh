xhost +
sudo docker run -it --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/roomba1/16.413/padm_project_2022F/pybullet/src/lib.py:/app/pybullet/src/lib.py padm-project-2022f /bin/bash
xhost -