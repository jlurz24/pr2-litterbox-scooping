#! /usr/bin/python
import roslib
roslib.load_manifest('litterbox')
import rospy, traceback, actionlib, time

from arm_navigation_msgs.msg import MoveArmAction, MoveArmGoal
from arm_navigation_msgs.msg import PositionConstraint, OrientationConstraint
from optparse import OptionParser
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped, Quaternion

import tf

def test_move_arm():
  rospy.loginfo("Moving the arm");

  client = actionlib.SimpleActionClient("move_right_arm", MoveArmAction)
  client.wait_for_server()

  goal = MoveArmGoal()
  goal.motion_plan_request.group_name = "right_arm"
  goal.motion_plan_request.num_planning_attempts = 3
  goal.motion_plan_request.planner_id = ""
  goal.planner_service_name = "ompl_planning/plan_kinematic_path"
  goal.motion_plan_request.allowed_planning_time = rospy.Duration(60.)
  
  position_constraint = PositionConstraint()
  position_constraint.header.frame_id = "/torso_lift_link"
  position_constraint.link_name = "r_wrist_roll_link"

  position_constraint.position.x = 0.45
  position_constraint.position.y = -0.188
  position_constraint.position.z = 0.0
  position_constraint.constraint_region_shape.type = position_constraint.constraint_region_shape.BOX
  tolerance = 0.04
  position_constraint.constraint_region_shape.dimensions = [tolerance, tolerance, tolerance]
  position_constraint.constraint_region_orientation.x = 0.
  position_constraint.constraint_region_orientation.y = 0.
  position_constraint.constraint_region_orientation.z = 0.
  position_constraint.constraint_region_orientation.w = 1.        
  position_constraint.weight = 1.0
  
  orientation_constraint = OrientationConstraint()
  orientation_constraint.header.frame_id = "/torso_lift_link"
  orientation_constraint.link_name = "r_wrist_roll_link"
  
  orientation_constraint.orientation.x = 0.
  orientation_constraint.orientation.y = 0.
  orientation_constraint.orientation.z = 0.
  orientation_constraint.orientation.w = 1.

  orientation_constraint.absolute_roll_tolerance = 0.04
  orientation_constraint.absolute_pitch_tolerance = 0.04
  orientation_constraint.absolute_yaw_tolerance = 0.04
  orientation_constraint.weight = 1.0

  goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
  goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)
  goal.disable_collision_monitoring = False

  rospy.loginfo("Calling the move arm client")
  state = client.send_goal_and_wait(goal, rospy.Duration(120.))
  if state == actionlib.GoalStatus.SUCCEEDED:
    rospy.loginfo("Succeeded")
  else:
    rospy.loginfo(state)

if __name__ == '__main__':
  try:
    rospy.init_node('test_move_arm')
    parser = OptionParser()
    (options, args) = parser.parse_args()

    test_move_arm()
  except rospy.ROSInterruptException:
    rospy.loginfo("Interupted")

