#!/bin/bash

export BAG_FILE=$1

export TEMP_DIR=$2

# Arena Stream
rosbag filter $BAG_FILE game_images.bag 'topic == "/arena/game_image"'

# Game Data
rosbag filter $BAG_FILE game_data.bag '(topic != "/arena/game_image") and (topic != "/raspicam_node/image/compressed")'

mv game_images.bag /tmp

export C_DIR=$(pwd)

pushd "$(dirname "$0")"

roslaunch export.launch

mkdir $TEMP_DIR

mv ~/.ros/frame*.jpg $TEMP_DIR

pushd $TEMP_DIR

mencoder "mf://*.jpg" -mf type=jpg:fps=15 -o output.mpg -speed 1 -ofps 30 -ovc lavc -lavcopts vcodec=mpeg2video:vbitrate=2500 -oac copy -of mpeg

mv output.mpg $C_DIR

rm -rf *.jpg

cd $C_DIR