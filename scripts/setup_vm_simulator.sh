#!/bin/bash

pushd "$(dirname "$0")"

source ~/python2_env/bin/activate

export ROS_MASTER_URI="http://127.0.0.1:11311"
export ROS_IP=`hostname -I | cut -d' ' -f1`

export AUTONOMY_SIMULATOR="True"

# Setup ROS Core
sh ./start_roscore.sh &

echo "Waiting for roscore to come up... you'll see errors while it waits"
until rosnode info rosout | grep Pid; do sleep 2; done

sh ./view_game.sh &

source /usr/share/gazebo/setup.sh

cd ../arenas

vglrun rosrun gazebo_ros gazebo ctf_1v1_arena.world &

cd ../host

# Start Tracker
python ../host/sim_tracker2.py

popd

