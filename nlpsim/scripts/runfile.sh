#!/usr/bin/env bash

sudo ~/terminate.sh
gnome-terminal -e "bash -c \"roslaunch nlpsim test1.launch; exec bash\"" &
sleep 8
gnome-terminal -e "bash -c \"rosrun nlpsim random_pose_generator.py; exec bash\"" &
sleep 10
gnome-terminal -e "bash -c \"rosrun image_view image_view image:=/detections_image_topic; exec bash\"" &
sleep 3
gnome-terminal -e "bash -c \"rosrun nlpsim interaction.py; exec bash\"" &
sleep 10
# gnome-terminal -e "bash -c \"rosrun nlpsim move_person.sh; exec bash\"" &
# sleep 5
echo "Done!"