ln -s colors_simulation.txt colors.txt
launch pr2_gazebo pr2_empty_world.launch
cd litterbox
./load_models.sh
roslaunch launch/arm_navigation.launch
roslaunch launch/action_nodes.launch
roslaunch launch/sensor_nodes.launch
roslaunch launch/rviz_move_base.launch
# Set 2d pose estimate in map
python scripts/scoop_litterbox_sm.py
rosrun smach_viewer smach_viewer.py

