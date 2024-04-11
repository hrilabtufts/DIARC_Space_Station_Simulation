#!/usr/bin/env bash

set -e

if [ ! -f .env ]; then
  cp default.env .env
fi

source .env

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

ROS_TMPL_FILE="startup_pr2_docker.launch.tmpl"
PORT=9090
ROBOTS=1

if [[ "${2}" != "" ]]; then
  echo "Starting with GUIs enabled..."
  HEADLESS="false"
  DISPLAY_RVIZ="True"
  DISPLAY_GAZEBO="False" #always false for now
  DISPLAY_DIALOGUE="-g"
  DISPLAY_BELIEF="-beliefg"
  DISPLAY_EDITOR="-editor"
  robot=$(cat config/docker/docker-compose-robot.yaml.tmpl)
else
  echo "Starting headless..."
  HEADLESS="true"
  DISPLAY_RVIZ="False"
  DISPLAY_GAZEBO="False"
  DISPLAY_DIALOGUE=""
  DISPLAY_BELIEF=""
  DISPLAY_EDITOR=""
  robot=$(cat config/docker/docker-compose-robot-headless.yaml.tmpl)
fi

mkdir -p logs

TIMESTAMP=$(date '+%s')

mkdir -p logs/

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

DOCKER_COMPOSE=docker-compose.yaml
rm -f ${DOCKER_COMPOSE}

cat config/docker/docker-compose.yaml.tmpl >> ${DOCKER_COMPOSE}

echo "Starting DIARC with ROS in the following configurations: "

TMPDISPLAY=$(mktemp)

for (( r=1; r<=$ROBOTS; r++)); do
  mkdir -p logs/${TIMESTAMP}-${PORT}-ros/
  mkdir -p logs/${TIMESTAMP}-${PORT}-diarc/

  ROS_TMP=$(mktemp)

  ROS_TMPL=$(cat config/ros/${ROS_TMPL_FILE})
  ROS_TMPL=${ROS_TMPL/DISPLAY_RVIZ/$DISPLAY_RVIZ}
  ROS_TMPL=${ROS_TMPL/DISPLAY_GAZEBO/$DISPLAY_GAZEBO}
  ROS_TMPL=${ROS_TMPL//ROBOTNAME/robot$r}
  ROS_TMPL=${ROS_TMPL//ROBOTNUMBER/$r}
  echo "${ROS_TMPL/PORT/$PORT}" > ${ROS_TMP}

  robot=${robot//PORT/$PORT}
  robot=${robot//TIMESTAMP/$TIMESTAMP}
  robot=${robot/ROS_TMP/$ROS_TMP}
  robot=${robot/ROBOTNAME/robot$r}

  robot=${robot//ENVDISPLAY/$DISPLAY}
  echo "$robot" >> ${DOCKER_COMPOSE}

  echo "robot${r}:" >> ${TMPDISPLAY}
  echo "  (remote) Unity Server   : ${1}:1755" >> ${TMPDISPLAY}
  echo "  (local)  Rosbridge Port : ${PORT}" >> ${TMPDISPLAY}
  
  PORT=$((PORT+1))
done

echo "$additional" >> ${DOCKER_COMPOSE}

cat ${ROS_TMP}
cat ${DOCKER_COMPOSE}
echo " "
cat ${TMPDISPLAY}
rm ${TMPDISPLAY}

askContinue

${BIN} -f ${DOCKER_COMPOSE} up 
