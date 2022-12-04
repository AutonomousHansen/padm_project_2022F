xhost +
sudo docker run -it --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/roomba1/16.413/padm_project_2022F/pybullet/src:/app/pybullet/src  -v /home/roomba1/16.413/padm_project_2022F/padm-project-2022f/ss-pybullet/pybullet_tools/utils.py:/app/padm-project-2022f/ss-pybullet/pybullet_tools/utils.py padm-project-2022f /bin/bash
xhost -