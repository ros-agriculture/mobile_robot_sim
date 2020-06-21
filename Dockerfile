FROM osrf/ros:melodic-desktop-full
ENV CATKIN_WS=/root/rover_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

RUN apt-get -qq update && \
    apt-get -qq install -y \
	apt-utils \
	libeigen3-dev \
        python-catkin-tools  \
        less \
        ssh \
	vim \
	terminator \
        git-core \
        bash-completion \
        wget && \
    rm -rf /var/lib/apt/lists/*

# HACK, replacing shell with bash for later docker build commands
RUN mv /bin/sh /bin/sh-old && \
    ln -s /bin/bash /bin/sh

# ADD SOURCE
COPY . .

#ROS specific stuff (sources, rosdep)
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc

#Not sure why we have to do this manually/again. I thought the base image would have this
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt-get install ros-`rosversion -d`-ompl #not sure why we have to do this manually
RUN sudo apt-get install ros-`rosversion -d`-mbf-msgs
RUN rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    apt-get -qq upgrade


# build repo
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8 
RUN source /ros_entrypoint.sh && \
    catkin build --no-status
RUN sudo apt-get install ros-`rosversion -d`-behaviortree-cpp-v3

