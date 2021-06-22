# This file tells docker what image must be created
# in order to be ahble to test this library
FROM moveit/moveit:master-source


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
                    iputils-ping \
                    net-tools \
                    netcat \
                    screen \
                    build-essential \
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
                    ros-melodic-gazebo-ros \
                    ros-melodic-graph-msgs \
                    ros-melodic-tf-conversions \
                    ros-melodic-ros-controllers \
                    ros-melodic-gazebo-ros-control \
                    ros-melodic-joint-trajectory-action \
                    ros-melodic-rqt \
                    ros-melodic-rqt-reconfigure \
                    ros-melodic-rqt-robot-plugins


RUN pip3 install setuptools matplotlib scipy quadpy six cython osrf-pycommon

# user handling
#add user to sudoers
WORKDIR /


COPY ./vim_installation.bash /
RUN cd / && bash vim_installation.bash
COPY ./configfiles/screenrc /usr/local/etc/screenrc
COPY ./configfiles/vimrc /etc/vim/
COPY ./configfiles/ycm_extra_conf.py /etc/vim/
