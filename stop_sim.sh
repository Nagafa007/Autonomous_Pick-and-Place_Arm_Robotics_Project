#!/bin/bash
# stop_sim.sh

echo "Stopping all simulation processes..."
pkill -9 gz
pkill -9 ruby
pkill -9 ros2
pkill -9 python3
echo "Done."
