FROM python:3.9

# install dependencies
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
  apt-get update -y && \
  apt-get install -y clang cmake git-lfs nano && \
  git lfs install && \
  pip install numpy scipy pybullet scikit-learn

# Dependencies for glvnd and X11.
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/*
  
# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# build project
WORKDIR /app
COPY . /app

RUN cd padm-project-2022f/ss-pybullet/pybullet_tools/ikfast/franka_panda && \
  python setup.py

WORKDIR /app

RUN cd padm-project-2022f/pddl-parser && \
  python setup.py install
