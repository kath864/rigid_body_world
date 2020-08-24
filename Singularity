bootstrap: docker
from: ubuntu:18.04

%environment
  export PATH=$PATH:/miniconda/bin

%runscript
  exec bash "$@"

%post
 apt-get update
 apt-get update
 apt-get install -y software-properties-common
 add-apt-repository ppa:jonathonf/ffmpeg-4
 apt-get update --fix-missing
 apt-get install -y  build-essential \
                     graphviz \
                     git \
                     wget \
                     ffmpeg \
                     libglu1 \
                     libxi6 \
                     libc6 \
                     libgl1-mesa-dev \
                     mesa-utils \
                     xvfb \
                     gettext \
                     gettext-base \
                     libgtk-3-dev \
                     libglib2.0-dev
 apt-get clean

 # Setup conda
 wget "https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh" -O conda.sh
 bash conda.sh -b -p /miniconda
 rm conda.sh
