#!/bin/bash

pushd "$(dirname "$0")"

pkill -f view_game

# Stop Roscore
ssh pi@192.168.42.102 "bash -s" < stop_ros.sh
ssh pi@192.168.42.101 "bash -s" < stop_ros.sh
ssh pi@192.168.42.100 "bash -s" < stop_ros.sh

popd