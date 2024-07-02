#!/bin/bash

set -e

GREEN='\033[0;32m'
NC='\033[0m'

DIARC="/code/diarc"
ROSJAVA="/code/rosjava_core"

source /code/scripts/.env

# source ROS
source /opt/ros/indigo/setup.bash

if [[ "${DIARC_ENVIRONMENT}" == "development" ]] && [[ "${REBUILD}" != "false" ]]; then
  printf "\n${GREEN}Cleaning and rebuilding${NC} Java projects..."
  cd $ROSJAVA
  ./gradlew clean buildAndPublishDiarcRos

  # rebuild DIARC
  cd $DIARC
  ./gradlew clean assemble
else
  echo "Skipping clean and rebuild..."
fi


# source ROS
source /catkin_ws/devel/setup.bash

# set ENV
export ROS_PACKAGE_PATH=$DIARC:$ROSJAVA:$ROS_PACKAGE_PATH
echo $ROS_PACKAGE_PATH
export ROBOT=sim
export ROS_IP=$(hostname -I | xargs)

roslaunch unity_ros startup_pr2_docker.launch &

if [[ "${WAIT}" == "" ]]; then
  WAIT=40
fi

if [[ "${DIARC_ROBOT}" != "robot1" ]] || [[ "${SMM}" == "true" ]]; then
  sleep 20
fi

#printf "\n${GREEN}Starting Unity simulation server${NC} in ${WAIT} seconds...\n"
#d /server
#./Space_Station_SMM_Server.x86_64


printf "\n${GREEN}Starting DIARC${NC} in ${WAIT} seconds...\n"

sleep ${WAIT}

# start DIARC
cd $DIARC
printf "\n${GREEN}Starting DIARC${NC}\n"
rm -rf /root/.gradle
mkdir -p /root/.gradle
cp /root/gradle.properties /root/.gradle/gradle.properties
echo "---------LAUNCHING CONFIG-----------"

if [[ "${DIARC_ROBOT}" == "robot1" ]] || [[ "${SMM}" == "false" ]]; then
  echo ${DIARC_CONFIG}
  ./gradlew launch -Pmain=${DIARC_CONFIG} --args="-unity ${UNITY_IP} -port ${UNITY_PORT} -llm ${LLM} -agent ${DIARC_ROBOT}"
else 
  echo ${DIARC_CONFIG_SPOKE}
  ./gradlew launch -Pmain=${DIARC_CONFIG_SPOKE} --args="-unity ${UNITY_IP} -port ${UNITY_PORT} -llm ${LLM} -agent ${DIARC_ROBOT}"
fi