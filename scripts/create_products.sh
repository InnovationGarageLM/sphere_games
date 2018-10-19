#!/bin/bash

export BAG_FILE=$1

# Compressed Image Stream
rosbag filter $BAG_FILE compressed_images.bag 'topic == "/raspicam_node/image/compressed"'

# Game Data
rosbag filter $BAG_FILE game_data.bag '(topic != "/arena/game_image") and (topic != "/raspicam_node/image/compressed")'