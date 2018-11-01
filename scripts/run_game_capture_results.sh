#!/bin/bash

pushd "$(dirname "$0")"

python ../host/reset_game.py

sh capture_bag.sh &

python ../host/start_game.py

wait