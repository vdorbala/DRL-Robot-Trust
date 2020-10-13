#!/usr/bin/env bash

sudo ~/terminate.sh

# WORLD SETUP

gnome-terminal -e "bash -c \"roscore; exec bash\"" &
sleep 3
gnome-terminal --tab -e "bash -c \"roslaunch nlpsim test1.launch; exec bash\"" &
sleep 4
gnome-terminal -e "bash -c \"rosrun nlpsim random_pose_generator.py; exec bash\"" &
sleep 4
# # gnome-terminal -e "bash -c \"rosrun image_view image_view image:=/detections_image_topic; exec bash\"" &
# # sleep 3
gnome-terminal -e "bash -c \"rosrun nlpsim move_person.sh; exec bash\"" &
sleep 4

# ROBOT SETUP

# gnome-terminal -e "bash -c \"rosrun nlpsim send_goal.py; exec bash\"" &
# sleep 10
gnome-terminal -e "bash -c \"rosrun nlpsim intersection_nav.py; exec bash\"" &
sleep 4
gnome-terminal -e "bash -c \"rosrun nlpsim ktm.py; exec bash\"" &
sleep 4
gnome-terminal -e "bash -c \"rosrun nlpsim servoing_interaction.py; exec bash\"" &
sleep 4

echo "Done!"