#!/bin/bash

GREEN='\033[0;32m'
NC='\033[0m'

DIARC="/code/diarc"
ROSJAVA="/code/rosjava_core"

# source ROS
source /opt/ros/indigo/setup.bash
source /catkin_ws/devel/setup.bash

# set ENV
source /code/scripts/.env
export ROS_PACKAGE_PATH=$DIARC:$ROSJAVA:$ROS_PACKAGE_PATH
echo $ROS_PACKAGE_PATH
export ROBOT=sim
export ROS_IP=$(hostname -I | xargs)

# start DIARC
cd $DIARC

roslaunch unity_ros startup_pr2_docker.launch &

if [[ "${WAIT}" == "" ]]; then
  WAIT=40
fi

printf "\n${GREEN}Starting DIARC${NC} in ${WAIT} seconds...\n"

sleep ${WAIT}

printf "\n${GREEN}Starting DIARC${NC}\n"

echo "---------LAUNCHING CONFIG-----------"
echo ${DIARC_CONFIG}
./gradlew launch -Pmain=${DIARC_CONFIG} --args="-unity ${UNITY_IP}"