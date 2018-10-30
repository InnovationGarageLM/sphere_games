#!/bin/bash

pushd "$(dirname "$0")"

pkill -f view_game

# Stop Roscore
pkill -f /ros

killall -9 gzserver gzclient

popd