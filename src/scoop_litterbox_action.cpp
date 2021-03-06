#include <litterbox/utils.h>
#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <arm_navigation_msgs/utils.h>
#include <boost/math/constants/constants.hpp>
#include <tf/transform_listener.h>

// TODO: Define pre and post conditions.

// Generated messages
#include <litterbox/ScoopLitterboxAction.h>
#include <litterbox/DetermineLBDimensions.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

using namespace std;

static const double PI = boost::math::constants::pi<double>();

static const double SCOOP_LENGTH = 0.3; // Adjusted because it is partially held

static const double TORSO_TO_FRONT = 0.075; 

static const double SCOOP_WIDTH = 0.08;

static const double LB_SIDE_WIDTH = 0.02;

static const double COLLISION_BUFFER = 0.05;

static const double UNREACHABLE_DISTANCE = 0.04; // A small portion at the front of the LB is unreachable
/**
 * Cleans a litterbox
 */
class ScoopLitterboxAction {
public:
  ScoopLitterboxAction(const string& name): as(nh, name, boost::bind(&ScoopLitterboxAction::scoopLitterbox, this, _1), false),
    actionName(name){
    as.registerPreemptCallback(boost::bind(&ScoopLitterboxAction::preemptCB, this));
    as.start();
  }
 
private:
  struct Dimensions {
    double width;
    double depth;
    double height;
    geometry_msgs::PointStamped centroid;
  };

  ros::NodeHandle nh;
  tf::TransformListener tf;

  // Actionlib classes
  actionlib::SimpleActionServer<litterbox::ScoopLitterboxAction> as;
  string actionName;

  // create messages that are used to published feedback/result
  litterbox::ScoopLitterboxFeedback feedback;
  litterbox::ScoopLitterboxResult result;

  void preemptCB(){
    ROS_INFO("Scoop litterbox preempted");
    as.setPreempted();
  }

  arm_navigation_msgs::CollisionObject makeCollisionObject(const geometry_msgs::Pose& pose, const std::string& frameId, const Dimensions& dimensions){
    arm_navigation_msgs::CollisionObject obj;
    obj.header.stamp = ros::Time::now();
    obj.header.frame_id = frameId;
    obj.id = "litterbox";
    obj.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
       
    arm_navigation_msgs::Shape lbShape;
    lbShape.type = arm_navigation_msgs::Shape::BOX;
    lbShape.dimensions.resize(3);
    lbShape.dimensions[0] = dimensions.depth + LB_SIDE_WIDTH + COLLISION_BUFFER;
    lbShape.dimensions[1] = dimensions.width + LB_SIDE_WIDTH + COLLISION_BUFFER;
    lbShape.dimensions[2] = dimensions.height + COLLISION_BUFFER;
    obj.shapes.push_back(lbShape);
    
    // TODO: Use full pose from this result instead of just position.
    geometry_msgs::Pose lbPose = pose;
    tf.waitForTransform(dimensions.centroid.header.frame_id, frameId, dimensions.centroid.header.stamp, ros::Duration(10.0));

    geometry_msgs::PointStamped lbPointInBaseFrame;
    tf.transformPoint(frameId, dimensions.centroid, lbPointInBaseFrame);

    lbPose.position = lbPointInBaseFrame.point;
    obj.poses.push_back(lbPose);
    obj.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD; 
    return obj;
  }

  /**
   * Main function to scoop the litterbox.
   */
  void scoopLitterbox(const litterbox::ScoopLitterboxGoalConstPtr& goal){
    ROS_INFO("Scooping the litterbox");
    
    if(!as.isActive()){
      ROS_INFO("Scoop litterbox cancelled prior to start");
      return;
    }

    torsoDown();
    
    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }
   
    // The robot found the front of the litterbox. Guess
    // at where the middle is.
    geometry_msgs::Point litterboxGuess = goal->target.pose.position;
    litterboxGuess.x += 0.25;
    pointHeadAt(litterboxGuess, goal->target.header.frame_id);
 
    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }

    const Dimensions dim = determineLBDimensions();
 
    // Setup the planning scene object that we'll need later.
    arm_navigation_msgs::CollisionObject collisionObject = makeCollisionObject(goal->target.pose, goal->target.header.frame_id, dim);

    // Initialize the client. Do not store it as a member variable to prevent the listener from being active
    // for longer than necessary.
    auto_ptr<MoveArmClient> rightArmClient = initClient<MoveArmClient>("move_right_arm");
    moveArmOverLitterbox(goal->target.pose.position, dim, collisionObject, rightArmClient.get());
    
    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }

    // Do not pre-empty after this point because the robot
    // may have already scooped and we need to reset the arm
    // to a valid position for moving.
    performScoop(goal->target.pose.position, dim, collisionObject, rightArmClient.get());
   
    moveArmOverLitterbox(goal->target.pose.position, dim, collisionObject, rightArmClient.get());
    
    moveArmToCarryingPosition(collisionObject, rightArmClient.get());
     
    ROS_INFO("Litterbox cleaned successfully");
    as.setSucceeded(result);
  }

  void moveArmToCarryingPosition(const arm_navigation_msgs::CollisionObject& collisionObject, MoveArmClient* rightArmClient){
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
    moveRightArm(carryPositionInWristFrame.point, identityOrientation(), "r_wrist_roll_link", collisionObject, false, rightArmClient);
  }

  Dimensions determineLBDimensions() {
    ROS_INFO("Determining LB dimensions");
    ros::ServiceClient determineLBDimClient = nh.serviceClient<litterbox::DetermineLBDimensions>("determine_lb_dimensions");
    litterbox::DetermineLBDimensions srv;
    Dimensions dim;
    if(!determineLBDimClient.call(srv)){
      ROS_INFO("Failed to call determine_lb_dimensions service");
      return dim;
    }
    
    dim.width = srv.response.width;
    dim.depth = srv.response.depth;
    dim.height = srv.response.height;
    dim.centroid = srv.response.centroid;
    
    ROS_INFO("Received LB dimensions width: %f depth: %f height %f", dim.width, dim.depth, dim.height);
    return dim;
  }

  /**
   * Point the head at a given point
   */
  void pointHeadAt(const geometry_msgs::Point point, const string& frameId){
    ROS_INFO("Pointing head");
    auto_ptr<PointHeadClient> pointHeadClient = initClient<PointHeadClient>("/head_traj_controller/point_head_action");
    pr2_controllers_msgs::PointHeadGoal goal;

    goal.target.point = point;
    goal.target.header.frame_id = frameId;

    sendGoal(pointHeadClient, goal, nh);
    ROS_INFO("Completed pointing head");
  }

  /**
   * Move the torso to the lowest possible height.
   */
  void torsoDown(){
    ROS_INFO("Lowering the torso");
    auto_ptr<TorsoClient> torsoClient = initClient<TorsoClient>("torso_controller/position_joint_action");
    pr2_controllers_msgs::SingleJointPositionGoal down;
    down.position = 0.01;
    down.min_duration = ros::Duration(2.0);
    down.max_velocity = 1.0;

    sendGoal(torsoClient, down, nh);
    ROS_INFO("Torso down movement completed");
  }
  
  /**
   * Perform the scoop of the litterbox
   */
  bool performScoop(const geometry_msgs::Point point, const Dimensions& dim, const arm_navigation_msgs::CollisionObject& collisionObject, MoveArmClient* rightArmClient){

    ROS_INFO("Litterbox base position is %f %f %f", point.x, point.y, point.z);
    
    ROS_INFO("Moving to the back of the litterbox");
    geometry_msgs::Point backPosition;
    backPosition.x = -0.10; // TODO: Calculate this as the base of the triangle
    moveRightArm(backPosition, identityOrientation(), "r_wrist_roll_link", collisionObject, false, rightArmClient);
    
    // Move to a random horizontal position.
    geometry_msgs::Point horizPosition;
    
    // Add randomness to pick a spot in the litter box. The dimensions includes
    // the width of the lb side, so subtract that. The robot should also not try
    // to move the scoop within half of the width of the scoop on each side as
    // that would collide.
    const double scoopableArea = dim.width - (2 * LB_SIDE_WIDTH) - SCOOP_WIDTH;
    ROS_INFO("Scoopable area of the litterbox is %f", scoopableArea);

    double horizontal = scoopableArea / double(2) - ((double(rand()) / RAND_MAX) * scoopableArea);
    horizPosition.x = 0;
    horizPosition.y = 0;
    horizPosition.z = horizontal;

    ROS_INFO("Adjusting horizontal by random %f", horizontal);
    moveRightArm(horizPosition, identityOrientation(), "r_wrist_roll_link", collisionObject, false, rightArmClient);
    
    // Now lower the scoop.
    geometry_msgs::Point noMove;
    ROS_INFO("Lowering scoop");
    moveRightArm(noMove, verticalDownOrientation(), "r_wrist_roll_link", collisionObject, false, rightArmClient);
    
    // Now scoop
    ROS_INFO("Scooping");

    // Transform the wrist point into the body frame since the frame is now pointed downwards and we want to
    // scoop horizontally in the global frame. The body frame should be oriented straight on to the litterbox.
    geometry_msgs::PointStamped wristPosition;
    wristPosition.header.frame_id = "r_wrist_roll_link";
    wristPosition.header.stamp = ros::Time::now();
    geometry_msgs::PointStamped scoopMove;
    tf.waitForTransform(wristPosition.header.frame_id, "torso_lift_link", wristPosition.header.stamp, ros::Duration(10.0));
    tf.transformPoint("torso_lift_link", wristPosition, scoopMove);
    
    scoopMove.point.x += (dim.depth - 2 * LB_SIDE_WIDTH - UNREACHABLE_DISTANCE);
    ROS_INFO("depth %f scoopMove %f", dim.depth, scoopMove.point.x);

    // Transform back for simplicity. This is the same transform as above so we don't have to wait for it.
    geometry_msgs::PointStamped scoopMoveInWristFrame;
    scoopMove.header.frame_id = "torso_lift_link";
    scoopMove.header.stamp = wristPosition.header.stamp;
    tf.transformPoint("r_wrist_roll_link", scoopMove, scoopMoveInWristFrame);
    moveRightArm(scoopMoveInWristFrame.point, identityOrientation(), "r_wrist_roll_link", collisionObject, false, rightArmClient);

    // Now pick the scoop up.
    moveRightArm(noMove, invVerticalDownOrientation(), "r_wrist_roll_link", collisionObject, false, rightArmClient);
    ROS_INFO("Scoop complete");
    return true;
  }
  
 
  /**
   * Move the arm over the litterbox 
   */
  void moveArmOverLitterbox(const geometry_msgs::Point& point, const Dimensions& dim, const arm_navigation_msgs::CollisionObject& collisionObject, MoveArmClient* rightArmClient){
    ROS_INFO("Moving arm over litterbox");

    // Center the arm over the litterbox 0.5 meters above the ground. Point should be the center point of the front
    // edge of the litterbox halfway back in the litterbox.
    ROS_INFO("Litterbox back in base frame x %f y %f z %f", point.x, point.y, point.z);
    geometry_msgs::Point overLitterboxPoint = point;
    
    overLitterboxPoint.x = overLitterboxPoint.x - SCOOP_LENGTH + (dim.depth / 2.0) - TORSO_TO_FRONT;
    // Leave y as its the center of the litterbox.
    overLitterboxPoint.z = -0.435;
    moveRightArm(overLitterboxPoint, verticalOrientation(), "/torso_lift_link", collisionObject, false, rightArmClient);
    ROS_INFO("Move arm complete");
  }

  /**
   * Move the right arm relative to a given frame.
   * @param position Position to move the arm to
   * @param orientation Orientation to move the arm to
   * @param referenceFrame Frame the movement is relative to
   */
  bool moveRightArm(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation, const string& referenceFrame, const arm_navigation_msgs::CollisionObject& collisionObject, bool constrainOrientation, MoveArmClient* rightArmClient){
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

     if(constrainOrientation){
       goal.motion_plan_request.path_constraints.orientation_constraints.resize(1);
       goal.motion_plan_request.path_constraints.orientation_constraints[0].header.frame_id = referenceFrame;
       goal.motion_plan_request.path_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
       goal.motion_plan_request.path_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
   
       goal.motion_plan_request.path_constraints.orientation_constraints[0].orientation = identityOrientation();
       goal.motion_plan_request.path_constraints.orientation_constraints[0].type = arm_navigation_msgs::OrientationConstraint::LINK_FRAME;
       goal.motion_plan_request.path_constraints.orientation_constraints[0].absolute_roll_tolerance = PI / 12.0;
       goal.motion_plan_request.path_constraints.orientation_constraints[0].absolute_pitch_tolerance = PI / 12.0;
       goal.motion_plan_request.path_constraints.orientation_constraints[0].absolute_yaw_tolerance = PI / 12.0;
     }

     // Allow the scoop and the litterbox cube to intersect.
     goal.planning_scene_diff.collision_objects.push_back(collisionObject);
     arm_navigation_msgs::CollisionOperation op;
     op.object1 = "scoop";
     op.object2 = collisionObject.id;
     op.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
     goal.operations.collision_operations.push_back(op);
     return sendGoal(rightArmClient, goal, nh);
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

  geometry_msgs::Quaternion verticalDownOrientation() const {
    return tf::createQuaternionMsgFromRollPitchYaw(0, 0, -0.55);
  }

  geometry_msgs::Quaternion invVerticalDownOrientation() const {
    return tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0.55);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "scoop_litterbox");

  ScoopLitterboxAction scoopAction(ros::this_node::getName());
  ros::spin();

  return 0;
}

