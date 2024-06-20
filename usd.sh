#!/bin/bash

help () {
	echo "Usage: usd [command] <options>"
	echo "Commands:"
	echo "	build   - Build this container with optional -f/--force arguments for forcing re-build of entire image"
	echo "	run     - Run container: usd run <UNITY SERVER IP> [NUM ROBOTS] [GUI ON]"
	echo "	dev     - Run container in development mode: usd dev <LOCAL REPO PATH> [GUI ON]"
	echo "	stop    - Gracefully bring down container. Not needed for docker compose commands run and dev."
	echo "	kill    - Ungracefully bring down container. Not needed for docker compose commands run and dev." 
}

if [[ "${1}" == "build" ]]; then
	bash ./scripts/build.sh "${2}"
elif [[ "${1}" == "run" ]]; then
	bash ./scripts/run.sh
elif [[ "${1}" == "dev" ]]; then
	bash ./scripts/dev.sh "${2}" "${3}" "${4}"
elif [[ "${1}" == "stop" ]]; then
	bash ./scripts/stop.sh
elif [[ "${1}" == "kill" ]]; then
	bash ./scripts/kill.sh
elif [[ "${1}" == "help" ]] || [[ "${1}" == "-h" ]]; then
	help
else
	echo "Please provide a command to the script"
	help
fi
