#!/bin/bash
rosparam load objects/box.model box_description
rosrun gazebo spawn_model -param box_description -gazebo -model box1 -x 2.25 -z 0.75
