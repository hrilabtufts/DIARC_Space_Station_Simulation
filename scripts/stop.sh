#!/usr/bin/env bash

# Fork command for timeout handling
{
    # Stop the container after sleep
    sleep 5
    docker stop unity_space_station
} &
timer_pid=$!
echo "timer pid: $timer_pid"

docker kill --signal "SIGINT" unity_space_station
echo "sent SIGINT signal to 'unity_space_station' for smooth shutdown"
echo "waiting for 'unity_space_station' to stop ..."
docker wait unity-spacestation
echo "'unity_space_station' stopped"
# The container stopped after sigint, cancel the timeout fork
kill -SIGKILL $timer_pid
echo "exiting unity_space_station"
exit 0

