#include <litterbox/utils.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <boost/math/constants/constants.hpp>
#include <tf/transform_listener.h>

// Generated messages
#include <litterbox/DumpPoopAction.h>

typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;

static const double PI = boost::math::constants::pi<double>();

using namespace std;

/**
 * Dumps poop into a trash can
 */
class DumpPoopAction {
public:
  DumpPoopAction(const string& name): as(nh, name, boost::bind(&DumpPoopAction::dumpPoop, this, _1), false), actionName(name){
    as.registerPreemptCallback(boost::bind(&DumpPoopAction::preemptCB, this));
    as.start();
  }
  
  void preemptCB(){
    auto_ptr<MoveArmClient> rightArmClient = initClient<MoveArmClient>("move_right_arm");
    moveArmToCarryingPosition(rightArmClient.get());
    as.setPreempted();
  }

  /**
   * Main function to dump poop
   */
  void dumpPoop(const litterbox::DumpPoopGoalConstPtr& goal){
   if(!as.isActive()){
     ROS_INFO("Dump poop action cancelled before started");
     return;
   }
    
   ROS_INFO("Dumping Poop");
    
   // Move arm over trash.
   auto_ptr<MoveArmClient> rightArmClient = initClient<MoveArmClient>("move_right_arm");
   ROS_INFO("Moving arm over trash");
   moveArmToOverTrashPosition(goal, rightArmClient.get());
   if(!as.isActive()){
     ROS_INFO("Dump poop preempted");
     moveArmToCarryingPosition(rightArmClient.get());
     return;
   }
 
   // Dump poop.
   geometry_msgs::Point noMove;
   ROS_INFO("Rotating scoop 1");
   moveRightArm(noMove, verticalOrientation(), "r_wrist_roll_link", rightArmClient.get());
   ROS_INFO("Rotating scoop 2");
   moveRightArm(noMove, verticalOrientation(), "r_wrist_roll_link", rightArmClient.get());
   ROS_INFO("Returning scoop to carry position");
   moveArmToCarryingPosition(rightArmClient.get());
 
   as.setSucceeded(result);
  }

  
  private:
    ros::NodeHandle nh;
    tf::TransformListener tf;

    // Actionlib classes
    actionlib::SimpleActionServer<litterbox::DumpPoopAction> as;
    string actionName;
  
    // create messages that are used to published feedback/result
    litterbox::DumpPoopFeedback feedback;
    litterbox::DumpPoopResult result;

  void moveArmToOverTrashPosition(const litterbox::DumpPoopGoalConstPtr& goal, MoveArmClient* rightArmClient){
    ROS_INFO("Moving arm to over trashposition");
    
    ROS_INFO("Trashcan is at %f %f %f in %s", goal->target.pose.position.x, goal->target.pose.position.y, goal->target.pose.position.z, goal->target.header.frame_id.c_str());
    geometry_msgs::PoseStamped trashPose = goal->target;
    // Move arm slightly back due to length of scoop.
    // TODO: Make this smarter
    // trashPose.pose.position.x -= 0.3;
    trashPose.pose.position.z = 0.4; // TODO: Assumes trashcan height and target frame

    geometry_msgs::PoseStamped trashInWristFrame;
    tf.waitForTransform(trashPose.header.frame_id, "r_wrist_roll_link", trashPose.header.stamp, ros::Duration(10.0));
    tf.transformPose("r_wrist_roll_link", trashPose, trashInWristFrame);
    moveRightArm(trashInWristFrame.pose.position, identityOrientation(), "r_wrist_roll_link", rightArmClient);
  }

  void moveArmToCarryingPosition(MoveArmClient* rightArmClient){
    ROS_INFO("Moving arm to carrying position");

    geometry_msgs::PointStamped carryPosition;
    carryPosition.point.x = -0.35;
    carryPosition.point.y = -0.5;
    carryPosition.point.z = -0.15;
    carryPosition.header.frame_id = "torso_lift_link";
    carryPosition.header.stamp = ros::Time::now();

    geometry_msgs::PointStamped carryPositionInWristFrame;
    tf.waitForTransform(carryPosition.header.frame_id, "r_wrist_roll_link", carryPosition.header.stamp, ros::Duration(10.0));
    tf.transformPoint("r_wrist_roll_link", carryPosition, carryPositionInWristFrame);
    moveRightArm(carryPositionInWristFrame.point, identityOrientation(), "r_wrist_roll_link", rightArmClient);
  }

  /**
   * Return the identity orientation
   * @return The identity orientation
   */
   geometry_msgs::Quaternion identityOrientation() const {
     return tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
   }

   geometry_msgs::Quaternion verticalOrientation() const {
     return tf::createQuaternionMsgFromRollPitchYaw(PI / 2.0, 0, 0);
   }
   
   bool moveRightArm(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation, const string& referenceFrame, MoveArmClient* rightArmClient){
     ROS_INFO("Moving to position %f %f %f in frame %s", position.x, position.y, position.z, referenceFrame.c_str());
     arm_navigation_msgs::MoveArmGoal goal;
     goal.motion_plan_request.group_name = "right_arm";
     goal.motion_plan_request.num_planning_attempts = 3;
     goal.motion_plan_request.planner_id = "";
     goal.planner_service_name = "ompl_planning/plan_kinematic_path";
     goal.motion_plan_request.allowed_planning_time = ros::Duration(15.0);

     arm_navigation_msgs::PositionConstraint desiredPos;
     desiredPos.header.frame_id = referenceFrame;
     desiredPos.link_name = "r_wrist_roll_link";
     desiredPos.position = position;
     desiredPos.constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
     desiredPos.constraint_region_shape.dimensions.push_back(0.02);
     desiredPos.constraint_region_shape.dimensions.push_back(0.02);
     desiredPos.constraint_region_shape.dimensions.push_back(0.02);
     desiredPos.constraint_region_orientation.x = 0;
     desiredPos.constraint_region_orientation.y = 0;
     desiredPos.constraint_region_orientation.z = 0;
     desiredPos.constraint_region_orientation.w = 1;
     desiredPos.weight = 1;

     arm_navigation_msgs::OrientationConstraint desiredOr;
     desiredOr.header.frame_id = referenceFrame;
     desiredOr.link_name = "r_wrist_roll_link";

     desiredOr.orientation = orientation;
     desiredOr.absolute_roll_tolerance = 0.02;
     desiredOr.absolute_pitch_tolerance = 0.02;
     desiredOr.absolute_yaw_tolerance = 0.02;
     desiredOr.weight = 1.0;

     goal.disable_collision_monitoring = true;

     goal.motion_plan_request.goal_constraints.position_constraints.push_back(desiredPos);
     goal.motion_plan_request.goal_constraints.orientation_constraints.push_back(desiredOr);
     return sendGoal(rightArmClient, goal, nh);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "dump_poop");

  DumpPoopAction dumpPoopAction(ros::this_node::getName());
  ros::spin();

  return 0;
}

