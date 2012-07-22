#!/bin/bash
rosparam load objects/box.model box_description
rosrun gazebo spawn_model -param box_description -gazebo -model box -x 0 -z 1

