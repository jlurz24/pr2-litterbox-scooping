## Summary
ROS application to use a PR2 robot to scoop a litterbox

## Launch instructions
ln -s colors_simulation.txt colors.txt (first run only)
roslaunch launch/all.launch
roslaunch launch/rviz_move_base.launch (optional)
rosrun smach_viewer smach_viewer.py (optional)

