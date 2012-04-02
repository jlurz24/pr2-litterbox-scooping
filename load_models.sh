#!/bin/bash
rosservice call gazebo/delete_model '{model_name: trashcan}'
rosservice call gazebo/delete_model '{model_name: litterbox}'
rosparam load objects/litterbox.model litterbox_description
rosparam load objects/trashcan.model trashcan_description
rosrun gazebo spawn_model -param trashcan_description -gazebo -model trashcan -x 0 -z 1
rosrun gazebo spawn_model -param litterbox_description -gazebo -model litterbox -x 0 -z 1 # Note: 1.5 for X places the litterbox directly in front


