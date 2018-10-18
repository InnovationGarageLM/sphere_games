#!/bin/bash

pushd "$(dirname "$0")"

source ~/python2_env/bin/activate

export ROS_MASTER_URI="http://192.168.42.100:11311"
export ROS_IP=`hostname -I | cut -d' ' -f1`

# Setup ROS Core on redteam-node
ssh pi@192.168.42.100 "bash -s" < start_roscore.sh &

# Start Raspberry Pi Camera
ssh pi@192.168.42.102 "bash -s" < start_camera.sh &

./view_game.sh &

popd

echo "Waiting for roscore to come up... you'll see errors while it waits"
until rosnode info rosout | grep Pid; do sleep 2; done

# Start Tracker
python ../host/sphero_tracker.py