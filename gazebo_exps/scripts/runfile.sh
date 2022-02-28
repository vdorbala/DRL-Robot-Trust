#!/usr/bin/env bash

sudo ~/terminate.sh

# WORLD SETUP

gnome-terminal -e "bash -c \"roscore; exec bash\"" &
sleep 3
gnome-terminal --tab -e "bash -c \"roslaunch gazebosim test1.launch; exec bash\"" &
sleep 4
gnome-terminal -e "bash -c \"rosrun gazebosim random_pose_generator.py; exec bash\"" &
sleep 4
# # gnome-terminal -e "bash -c \"rosrun image_view image_view image:=/detections_image_topic; exec bash\"" &
# # sleep 3
gnome-terminal -e "bash -c \"rosrun gazebosim move_person.sh; exec bash\"" &
sleep 4

# ROBOT SETUP

# gnome-terminal -e "bash -c \"rosrun gazebosim send_goal.py; exec bash\"" &
# sleep 10
gnome-terminal -e "bash -c \"rosrun gazebosim intersection_nav.py; exec bash\"" &
sleep 4
gnome-terminal -e "bash -c \"rosrun gazebosim ktm.py; exec bash\"" &
sleep 4
gnome-terminal -e "bash -c \"rosrun gazebosim servoing_interaction.py; exec bash\"" &
sleep 4

echo "Done!"