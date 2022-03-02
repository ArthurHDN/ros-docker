FROM ros:melodic AS ros_base
ENV DEBIAN_FRONTEND noninteractive
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc


########################
FROM ros_base AS ros_dev
RUN apt-get update && \
    apt-get install -y --fix-missing \
    ros-$ROS_DISTRO-desktop-full 
RUN apt-get update 
RUN apt-get install -y \
    git \
    x11vnc \
    wget \
    python3 \
    python3-numpy \
    unzip \
    xvfb \
    icewm \
    tree \
    dos2unix \
    vim \
    net-tools \
    iputils-ping \
    iproute2 \
    iptables \
    python3-catkin-tools \
    tcpdump
RUN cd /root && git clone https://github.com/kanaka/noVNC.git && \
    cd noVNC/utils && git clone https://github.com/kanaka/websockify websockify
ENV DISPLAY :1
ENV SCREEN 0
ENV SCREEN_SIZE 1280x768x24
COPY start_novnc.sh start_novnc.sh
# RUN vim start_novnc.sh +"set ff=unix" +wq
RUN dos2unix start_novnc.sh && chmod 0755 start_novnc.sh
WORKDIR /workspaces


##########################
FROM ros_base AS ros_mocap
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-mocap-optitrack


#############################
FROM ros_base AS ros_gmapping
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-gmapping
