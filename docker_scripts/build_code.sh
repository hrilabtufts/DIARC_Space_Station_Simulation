#!/bin/bash

set -e

# source ROS
source /opt/ros/indigo/setup.bash

echo "DEBUG: INSTALL DEPENDENCIES"

cd $DIARC

echo "DEBUG: BUILD DIARCROS"

# build DIARCROS
cd $DIARCROS
./gradlew clean buildAndPublishDiarcRos

echo "DEBUG: BUILD DIARC"

# build DIARC
cd $DIARC

./gradlew clean assemble
