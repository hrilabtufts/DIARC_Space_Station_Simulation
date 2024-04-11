#!/bin/bash

# The meaner version of the stop script

docker kill $(docker container ls | grep unity_space_station | awk '{print $1}')