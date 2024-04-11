#!/bin/bash

set -e

# source ROS
source /opt/ros/indigo/setup.bash

echo "DEBUG: INSTALL DEPENDENCIES"

cd $DIARC

echo "DEBUG: BUILD ROSJAVA"

# build rosjava
cd $ROSJAVA
./gradlew clean buildAndPublishDiarcRos

echo "DEBUG: BUILD DIARC"

# build DIARC
cd $DIARC

./gradlew clean assemble
