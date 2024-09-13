#!/bin/bash

if [ ! -f .env ]; then
    echo "Please create a .env file to configure this repo. See default.env for an example."
    exit 2
fi

source .env

if [[ "${1}" == "--force" ]] || [[ "${1}" == "-f" ]]; then
    ARG="--no-cache"
else
    ARG=""
fi

docker build ${ARG} --progress=plain -t hrilabtufts/unity_space_station -f Dockerfile .

if [ ! -d ./workload_analysis ]; then
    git clone https://github.com/hrilabtufts/workload_analysis.git
fi

cd workload_analysis

docker build ${ARG} --progress=plain -t hrilabtufts/workload_analysis -f Dockerfile .
