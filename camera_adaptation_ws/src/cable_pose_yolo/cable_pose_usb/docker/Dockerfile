FROM ros:humble-ros-base

# ROS2 dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    x11-xserver-utils \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rqt-image-view \
    ros-humble-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies (for vision modules)
COPY ./config/requirements.txt /root/requirements.txt
RUN pip install -r /root/requirements.txt
RUN pip install "setuptools<66"
RUN chmod -R 777 /root