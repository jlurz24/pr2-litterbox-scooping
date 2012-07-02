#! /usr/bin/python
import roslib
roslib.load_manifest('litterbox')
import rospy, traceback, actionlib, time

from arm_navigation_msgs.msg import MoveArmAction, MoveArmGoal
from arm_navigation_msgs.msg import PositionConstraint, OrientationConstraint
import argparse
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped, Quaternion

import tf

def quaternion_to_msg(q):
  msg = Quaternion()
  msg.x = q[0]
  msg.y = q[1]
  msg.z = q[2]
  msg.w = q[3]
  return msg

def test_move_arm(position, orientation, frame):
  rospy.loginfo("Moving the arm to x: %f y: %f z: %f r: %f p: %f y: %f", position[0], position[1], position[2], orientation[0], orientation[1], orientation[2]);

  client = actionlib.SimpleActionClient("move_right_arm", MoveArmAction)
  client.wait_for_server()

  goal = MoveArmGoal()
  goal.motion_plan_request.group_name = "right_arm"
  goal.motion_plan_request.num_planning_attempts = 3
  goal.motion_plan_request.planner_id = ""
  goal.planner_service_name = "ompl_planning/plan_kinematic_path"
  goal.motion_plan_request.allowed_planning_time = rospy.Duration(15.)
  
  position_constraint = PositionConstraint()
  position_constraint.header.frame_id = frame
  position_constraint.link_name = "r_wrist_roll_link"

  position_constraint.position.x = position[0]
  position_constraint.position.y = position[1]
  position_constraint.position.z = position[2]
  position_constraint.constraint_region_shape.type = position_constraint.constraint_region_shape.BOX
  tolerance = 0.04
  position_constraint.constraint_region_shape.dimensions = [tolerance, tolerance, tolerance]
  position_constraint.constraint_region_orientation.x = 0.
  position_constraint.constraint_region_orientation.y = 0.
  position_constraint.constraint_region_orientation.z = 0.
  position_constraint.constraint_region_orientation.w = 1.
  position_constraint.weight = 1.0
  
  orientation_constraint = OrientationConstraint()
  orientation_constraint.header.frame_id = frame
  orientation_constraint.link_name = "r_wrist_roll_link"
  
  orientation_constraint.orientation = quaternion_to_msg(tf.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2]))

  orientation_constraint.absolute_roll_tolerance = 0.04
  orientation_constraint.absolute_pitch_tolerance = 0.04
  orientation_constraint.absolute_yaw_tolerance = 0.04
  orientation_constraint.weight = 1.0

  goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
  goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)
  goal.disable_collision_monitoring = True

  rospy.loginfo("Calling the move arm client")
  state = client.send_goal_and_wait(goal, rospy.Duration(120.))
  if state == actionlib.GoalStatus.SUCCEEDED:
    rospy.loginfo("Succeeded")
  else:
    rospy.loginfo(state)

if __name__ == '__main__':
  try:
    rospy.init_node('test_move_arm')
    parser = argparse.ArgumentParser(description='Reads parameters for moving arm')
    parser.add_argument('position', metavar='N', type=float, nargs=3, help='x y z')
    parser.add_argument('rpy', metavar='N', type=float, nargs=3, help='r p y')

    parser.add_argument('--frame')

    args = parser.parse_args()
    frame = '/torso_lift_link'
    if len(args.frame) > 0:
      frame = args.frame

    position = (args.position[0], args.position[1], args.position[2])
    orientation = (args.rpy[0], args.rpy[1], args.rpy[2])
    test_move_arm(position, orientation, frame)
  except rospy.ROSInterruptException:
    rospy.loginfo("Interupted")

