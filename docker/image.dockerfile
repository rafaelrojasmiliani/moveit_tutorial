# This file tells docker what image must be created
# in order to be ahble to test this library
FROM ubuntu:20.04


ENV TZ=Europe/Rome
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
                    gnupg lsb-release apt-utils

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update
# Install packages
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
                    ros-noetic-desktop-full \
                    ros-noetic-moveit-setup-assistant \
                    iputils-ping \
                    net-tools \
                    netcat \
                    screen \
                    build-essential \
                    ros-noetic-moveit-simple-controller-manager \
                    ros-noetic-grid-map-rviz-plugin \
                    ros-noetic-jsk-rviz-plugins \
                    ros-noetic-octomap-rviz-plugins \
                    ros-noetic-rviz-visual-tools \
                    ros-noetic-ompl \
                    ros-noetic-moveit-planners \
                    ros-noetic-moveit-commander \
                    python3-catkin-tools \
                    python3-rosinstall-generator \
                    python3-rosinstall \
                    python3-rosdep \
                    ros-noetic-moveit-visual-tools \
                    python3-tk \
                    ros-noetic-plotjuggler \
                    ros-noetic-joint-trajectory-controller \
                    python3-sympy \
                    coinor-libipopt-dev \
                    sudo \
                    build-essential \
                    pkg-config \
                    git \
                    less \
                    liblapack-dev \
                    liblapack3 \
                    libopenblas-base \
                    libopenblas-dev \
                    libgfortran-7-dev \
                    python3-pip \
                    lsb-release \
                    gnupg2 \
                    curl \
                    python3-tk \
                    python3-dev \
                    python-dev \
                    ros-noetic-rosparam-shortcuts \
                    ros-noetic-marker-msgs

RUN pip3 install setuptools matplotlib scipy quadpy six cython osrf-pycommon

# user handling
ARG myuser
ARG myuid
ARG mygroup
ARG mygid
RUN addgroup --gid ${mygid} ${mygroup} --force-badname
RUN adduser --gecos "" --disabled-password  --uid ${myuid} --gid ${mygid} ${myuser} --force-badname
#add user to sudoers
RUN echo "${myuser} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc
WORKDIR /


COPY ./vim_installation.bash /
RUN cd / && bash vim_installation.bash
COPY ./configfiles/screenrc /usr/local/etc/screenrc
COPY ./configfiles/vimrc /etc/vim/
COPY ./configfiles/ycm_extra_conf.py /etc/vim/
