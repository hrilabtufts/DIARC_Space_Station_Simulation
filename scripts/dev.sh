#!/usr/bin/env bash

set -e

if [ ! -f .env ]; then
  cp default.env .env
fi

source .env

if [ "${ROBOTS}" -gt 2 ]; then
  echo "This script supports maximum 2 robots at this time"
  exit 3
fi

if [[ "${1}" == "" ]] || [ ! -d "$(realpath ${1})" ]; then 
  echo "Please provide relative path to local DIARC repository."
  exit 2
fi

if [ ! -d "$(realpath ${2})" ]; then 
  echo "Please provide relative path to local DIARC repository as third argument."
fi


askContinue () {
  echo "${1}"
  echo "Are you ready to continue? (yes/no)"
  read input
  if [ "$input" != "n" ] && [ "$input" != "no" ]
  then
    echo "${2}"
  else
    exit 1
  fi
}

DOCKER_COMPOSE="./docker-compose.yaml"
DIARC_SRC="$(realpath ${1})"
TMPDISPLAY=$(mktemp)
TIMESTAMP=$(date '+%Y_%m_%d-%H_%M_%S')
TRADE_PROPERTIES_CONF="diarc.tradePropertiesFile=/root/trade.properties"
TRADE_PROPERTIES_DEFAULT="diarc.tradePropertiesFile=src/main/resources/default/trade.properties.default"
TRADE_PROPERTIES_TMPL=$(< "./config/diarc/trade.hub.properties")
ROS_TMPL_FILE="startup_pr2_docker.launch.tmpl"
PORT=9090
UNITY_PORT=1755
ROBOT_START=4

if [[ "${3}" != "" ]] && [[ "${3}" != "false" ]]; then
  echo "Starting with GUIs enabled..."
  HEADLESS="false"
  DISPLAY_RVIZ="True"
  DISPLAY_GAZEBO="False" #always false for now
else
  echo "Starting headless..."
  HEADLESS="true"
  DISPLAY_RVIZ="False"
  DISPLAY_GAZEBO="False"
fi

if [ -z $ROBOTS ]; then
  ROBOTS=1
fi

additional=$(< config/docker/docker-compose-additional-services.yaml.tmpl)
additional=${additional//NETWORK_PREFIX/$NETWORK_PREFIX}

mkdir -p logs
mkdir -p cache

if [[ "${HEADLESS}" == "false" ]]; then
  xhost +local:docker
  export XAUTH=/tmp/.docker.xauth
  sudo rm -f $XAUTH
  echo "Preparing Xauthority data..."
  xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
  if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
      echo $xauth_list | xauth -f $XAUTH nmerge -
    else
      touch $XAUTH
    fi
    chmod 777 $XAUTH
  fi

  echo "Done."
  echo ""
  echo "Verifying file contents:"
  file $XAUTH
  echo "--> It should say \"X11 Xauthority data\"."
  echo ""
  echo "Permissions:"
  ls -FAlh $XAUTH
  echo ""
fi


rm -f ${DOCKER_COMPOSE}
cat config/docker/docker-compose.yaml.tmpl >> ${DOCKER_COMPOSE}
echo "$additional" >> ${DOCKER_COMPOSE}

echo "Starting DIARC with ROS in the following configurations: "

for (( r=1; r<=$ROBOTS; r++)); do
  mkdir -p logs/${TIMESTAMP}-${PORT}-ros/
  mkdir -p logs/${TIMESTAMP}-${PORT}-diarc/

  if [[ "${HEADLESS}" == "true" ]]; then
    robot=$(< config/docker/docker-compose-robot-dev-headless.yaml.tmpl)
  else
    robot=$(< config/docker/docker-compose-robot-dev.yaml.tmpl)
  fi

  ROS_TMP=$(mktemp)
  TRADE_PROPERTIES=$(mktemp)
  ROBOT_IP="${NETWORK_PREFIX}.0.${ROBOT_START}"

  TRADE_PROPERTIES_TMPL=${TRADE_PROPERTIES_TMPL//NETWORK_PREFIX/$NETWORK_PREFIX}
  echo "${TRADE_PROPERTIES_TMPL}" > ${TRADE_PROPERTIES}

  GRADLE_PROPERTIES=$(mktemp)
  GRADLE_PROPERTIES_TMPL=$(< "./config/gradle/gradle.properties")
  if [[ "${SMM}" == "true" ]]; then
    GRADLE_PROPERTIES_TMPL=${GRADLE_PROPERTIES_TMPL//TRADE_PROPERTIES/$TRADE_PROPERTIES_CONF}
  else 
    GRADLE_PROPERTIES_TMPL=${GRADLE_PROPERTIES_TMPL//TRADE_PROPERTIES/$TRADE_PROPERTIES_DEFAULT}
  fi
  echo "${GRADLE_PROPERTIES_TMPL}" > ${GRADLE_PROPERTIES}

  ROS_TMPL=$(< config/ros/${ROS_TMPL_FILE})
  ROS_TMPL=${ROS_TMPL//DISPLAY_RVIZ/$DISPLAY_RVIZ}
  ROS_TMPL=${ROS_TMPL//DISPLAY_GAZEBO/$DISPLAY_GAZEBO}
  ROS_TMPL=${ROS_TMPL//ROBOTNAME/robot$r}
  ROS_TMPL=${ROS_TMPL//ROBOTNUMBER/$r}
  echo "${ROS_TMPL//PORT/$PORT}" > ${ROS_TMP}

  robot=${robot//DEBUG_PORT/$DEBUG_PORT}
  robot=${robot//ROSPORT/$PORT}
  robot=${robot//TIMESTAMP/$TIMESTAMP}
  robot=${robot//ROS_TMP/$ROS_TMP}
  robot=${robot//ROBOTNAME/robot$r}
  robot=${robot//DIARC_SRC/$DIARC_SRC}
  robot=${robot//ROBOT_IP/$ROBOT_IP}
  robot=${robot//TRADE_PROPERTIES/$TRADE_PROPERTIES}
  robot=${robot//UNITYPORT/$UNITY_PORT}
  robot=${robot//GRADLE_PROPERTIES/$GRADLE_PROPERTIES}

  robot=${robot//ENVDISPLAY/$DISPLAY}
  echo "$robot" >> ${DOCKER_COMPOSE}

  echo "robot${r}:" >> ${TMPDISPLAY}
  echo "  (remote) Unity Server        : ${UNITY_IP}:${UNITY_PORT}" >> ${TMPDISPLAY}
  echo "  (local)  Rosbridge Port      : ${PORT}" >> ${TMPDISPLAY}
  PORT=$((PORT+1))
  ROBOT_START=$((ROBOT_START+1))
  if [ ! -z "${2}" ]; then
    DIARC_SRC="$(realpath ${2})"
  fi
  if [[ "${SMM}" == "true" ]]; then
    TRADE_PROPERTIES_TMPL=$(< "./config/diarc/trade.spoke.properties")
  else 
    UNITY_PORT=$((UNITY_PORT+1))
  fi
done

network=$(< config/docker/docker-compose-network.yaml.tmpl)
network=${network//NETWORK_PREFIX/$NETWORK_PREFIX}

echo "$network" >> ${DOCKER_COMPOSE}

cat ${ROS_TMP}
cat ${DOCKER_COMPOSE}
echo " "
cat ${TMPDISPLAY}
rm ${TMPDISPLAY}

askContinue

${BIN} -f ${DOCKER_COMPOSE} up 
