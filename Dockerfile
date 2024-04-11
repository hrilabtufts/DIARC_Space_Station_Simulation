FROM ubuntu:14.04 AS pr2_ros_indigo

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y \
    build-essential \
    openssh-client \
    openssh-server \
    software-properties-common \
    cmake3 \
    apt-utils \
    wget \
    curl

RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update

RUN apt-get install -y \
    ros-indigo-desktop-full \
    ros-indigo-rosbridge-suite \
    ros-indigo-map-server \
    ros-indigo-pr2-common \
    ros-indigo-pr2-2dnav \
    ros-indigo-pr2-simulator \
    ros-indigo-dwa-local-planner \
    ros-indigo-app-manager \
    ros-indigo-controller-manager \
    ros-indigo-controller-manager-msgs \
    ros-indigo-diff-drive-controller \
    ros-indigo-fcl \
    ros-indigo-gazebo-ros-control \
    ros-indigo-geographic-msgs \
    ros-indigo-gmapping \
    ros-indigo-hector-compressed-map-transport \
    ros-indigo-hector-gazebo-plugins \
    ros-indigo-hector-geotiff \
    ros-indigo-hector-geotiff-plugins \
    ros-indigo-hector-imu-attitude-to-tf \
    ros-indigo-hector-map-server \
    ros-indigo-hector-map-tools \
    ros-indigo-hector-mapping \
    ros-indigo-hector-marker-drawing \
    ros-indigo-hector-nav-msgs \
    ros-indigo-hector-slam \
    ros-indigo-hector-slam-launch \
    ros-indigo-hector-trajectory-server \
    ros-indigo-household-objects-database-msgs \
    ros-indigo-husky-control \
    ros-indigo-husky-description \
    ros-indigo-husky-desktop \
    ros-indigo-husky-gazebo \
    ros-indigo-husky-msgs \
    ros-indigo-husky-simulator \
    ros-indigo-husky-viz \
    ros-indigo-interactive-marker-twist-server \
    ros-indigo-joint-limits-interface \
    ros-indigo-joint-state-controller \
    ros-indigo-joint-trajectory-controller \
    ros-indigo-joy \
    ros-indigo-libccd \
    ros-indigo-lms1xx \
    ros-indigo-manipulation-msgs \
    ros-indigo-map-store \
    ros-indigo-moveit \
    ros-indigo-moveit-commander \
    ros-indigo-moveit-core \
    ros-indigo-moveit-fake-controller-manager \
    ros-indigo-moveit-full \
    ros-indigo-moveit-full-pr2 \
    ros-indigo-moveit-kinematics \
    ros-indigo-moveit-msgs \
    ros-indigo-moveit-planners \
    ros-indigo-moveit-planners-ompl \
    ros-indigo-moveit-plugins \
    ros-indigo-moveit-pr2 \
    ros-indigo-moveit-ros \
    ros-indigo-moveit-ros-benchmarks \
    ros-indigo-moveit-ros-benchmarks-gui \
    ros-indigo-moveit-ros-control-interface \
    ros-indigo-moveit-ros-manipulation \
    ros-indigo-moveit-ros-move-group \
    ros-indigo-moveit-ros-perception \
    ros-indigo-moveit-ros-planning \
    ros-indigo-moveit-ros-planning-interface \
    ros-indigo-moveit-ros-robot-interaction \
    ros-indigo-moveit-ros-visualization \
    ros-indigo-moveit-ros-warehouse \
    ros-indigo-moveit-setup-assistant \
    ros-indigo-moveit-simple-controller-manager \
    ros-indigo-object-recognition-msgs \
    ros-indigo-octomap-msgs \
    ros-indigo-ompl \
    ros-indigo-openslam-gmapping \
    ros-indigo-pr2-app-manager \
    ros-indigo-pr2-apps \
    ros-indigo-pr2-arm-kinematics \
    ros-indigo-pr2-kinematics \
    ros-indigo-pr2-mannequin-mode \
    ros-indigo-pr2-moveit-config \
    ros-indigo-pr2-moveit-plugins \
    ros-indigo-pr2-position-scripts \
    ros-indigo-pr2-teleop-general \
    ros-indigo-pr2-tuckarm \
    ros-indigo-ps3joy \
    ros-indigo-robot-localization \
    ros-indigo-rosdoc-lite \
    ros-indigo-rosjava-test-msgs \
    ros-indigo-rqt \
    ros-indigo-rviz-imu-plugin \
    ros-indigo-srdfdom \
    ros-indigo-teleop-twist-joy \
    ros-indigo-transmission-interface \
    ros-indigo-twist-mux \
    ros-indigo-twist-mux-msgs \
    ros-indigo-ur-description \
    ros-indigo-urdfdom-py \
    ros-indigo-uuid-msgs \
    ros-indigo-warehouse-ros \
    ros-indigo-willow-maps \
    python-vcstools \
    python-catkin-tools \
    python-rosdep \
    python-wstool \
    python-rosinstall \
    python-rosinstall-generator

COPY ./share/. /opt/ros/indigo/share/

FROM pr2_ros_indigo

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV ROS_WORKSPACE=/catkin_ws
ENV CODE=/code
ENV SCRIPTS=$CODE/scripts

RUN mkdir -p $ROS_WORKSPACE
RUN mkdir -p $CODE 
RUN mkdir -p $SCRIPTS

# We're going to need this for java 8
RUN add-apt-repository ppa:openjdk-r/ppa -y
RUN apt-get update

# Get java 17
RUN apt-get install -y openjdk-17-jdk ant gcc g++

# set java 17 to default
RUN update-alternatives --set javac /usr/lib/jvm/java-17-openjdk-amd64/bin/javac 
RUN update-alternatives --set java /usr/lib/jvm/java-17-openjdk-amd64/bin/java

# Clone unity_ros package
RUN mkdir -p $ROS_WORKSPACE/src
RUN git clone https://github.com/hrilabtufts/unity_ros.git $ROS_WORKSPACE/src/unity_ros
RUN git clone https://github.com/hrilabtufts/spoof_sensing.git $ROS_WORKSPACE/src/spoof_sensing
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git $ROS_WORKSPACE/src/ros_tcp_endpoint

# Build unity_ros package
WORKDIR $ROS_WORKSPACE
RUN source /opt/ros/indigo/setup.bash && catkin_make

# Remove catkin_tools
RUN rm -rf ~/.catkin_tools

# Make ssh dir
RUN mkdir -p /root/.ssh/
# Remember, this is copying over my private key: delete it later.
# Copy over private key, and set permissions
COPY id_rsa /root/.ssh/id_rsa
# Create known_hosts
RUN touch /root/.ssh/known_hosts

# Add DIARC git repo key
RUN ssh-keyscan -p 22222 hrilab.tufts.edu >> /root/.ssh/known_hosts
# clone DIARC
RUN mkdir -p ${CODE}
WORKDIR ${CODE}
RUN git clone -b tr_master --single-branch ssh://git@hrilab.tufts.edu:22222/ade/ade.git ${CODE}/diarc
# Delete private key
RUN rm /root/.ssh/id_rsa

# Clone ROSJAVA
RUN git clone https://github.com/evankrause/rosjava_core.git ${CODE}/rosjava_core
WORKDIR ${CODE}/rosjava_core
RUN git checkout reporelease

# Set build ENV
ENV JAVA_TOOL_OPTIONS=-Dfile.encoding=UTF8
ENV ROS_PACKAGE_PATH=$DIARC:$ROSJAVA:$ROS_PACKAGE_PATH
ENV DIARC=${CODE}/diarc
ENV ROSJAVA=${CODE}/rosjava_core

COPY ./docker_scripts/build_code.sh $SCRIPTS/build_code.sh

COPY ./config/gradle/gradle.properties /root/.gradle/gradle.properties

WORKDIR ${SCRIPTS}

RUN bash build_code.sh 