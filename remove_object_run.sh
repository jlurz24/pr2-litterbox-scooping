#!/bin/bash
# Clear any existing data
rosservice call /collider_node/reset
rosservice call /move_base_node/clear_costmaps
rosservice call /octomap_server/reset
sleep 1
rosparam load objects/box.model box_description
rosrun gazebo spawn_model -param box_description -gazebo -model box$1 -x 2.25 -z 0.75
rosparam set /pixel_occupied/model_name box$1
sleep 15
rosservice call gazebo/delete_model '{model_name: box'$1'}'
