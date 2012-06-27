#include <litterbox/utils.h>
#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <arm_navigation_msgs/utils.h>
#include <boost/math/constants/constants.hpp>

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

/**
 * Cleans a litterbox
 */
class ScoopLitterboxAction {
public:
  ScoopLitterboxAction(const string& name): as(nh, name, boost::bind(&ScoopLitterboxAction::scoopLitterbox, this, _1), false),
    actionName(name){
       
    ROS_INFO("Starting initialization");
    
    as.registerPreemptCallback(boost::bind(&ScoopLitterboxAction::preemptCB, this));
    torsoClient = initClient<TorsoClient>("torso_controller/position_joint_action");
    rightArmClient = initClient<MoveArmClient>("move_right_arm");
    pointHeadClient = initClient<PointHeadClient>("/head_traj_controller/point_head_action"); 
    as.start();
    ROS_INFO("Initialization complete");
  }
 
private:
  struct Dimensions {
    double width;
    double depth;
    double height;
  };

   auto_ptr<TorsoClient> torsoClient;
   auto_ptr<MoveArmClient> rightArmClient;
   auto_ptr<PointHeadClient> pointHeadClient;
   ros::NodeHandle nh;

   // Actionlib classes
   actionlib::SimpleActionServer<litterbox::ScoopLitterboxAction> as;
   string actionName;

   // create messages that are used to published feedback/result
   litterbox::ScoopLitterboxFeedback feedback;
   litterbox::ScoopLitterboxResult result;

  void preemptCB(){
    // TODO: Need much smarter post cancel behavior to put
    //       the robot back in a known state
    ROS_INFO("Scoop litterbox preempted");
    if(torsoClient->getState() == actionlib::SimpleClientGoalState::ACTIVE){
      torsoClient->cancelGoal();
    }

    if(pointHeadClient->getState() == actionlib::SimpleClientGoalState::ACTIVE){
      pointHeadClient->cancelGoal();
    }
    
    if(rightArmClient->getState() == actionlib::SimpleClientGoalState::ACTIVE){
      rightArmClient->cancelGoal();
    }
    
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
    lbShape.dimensions[0] = dimensions.width;
    lbShape.dimensions[1] = dimensions.depth;
    lbShape.dimensions[2] = dimensions.height;
    obj.shapes.push_back(lbShape);
    obj.poses.push_back(pose);

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

    moveArmOverLitterbox(goal->target.pose.position, dim, collisionObject);
    
    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }

    performScoop(goal->target.pose.position, dim, collisionObject);
   
    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }
 
    moveArmOverLitterbox(goal->target.pose.position, dim, collisionObject);
    
    ROS_INFO("Litterbox cleaned successfully");
    as.setSucceeded(result);
  }

  Dimensions determineLBDimensions() {
    ROS_INFO("Determining LB dimensions");
    ros::ServiceClient determineLBDimClient = nh.serviceClient<litterbox::DetermineLBDimensions>("determine_lb_dimensions");
    litterbox::DetermineLBDimensions srv;
    if(!determineLBDimClient.call(srv)){
      ROS_INFO("Failed to call determine_lb_dimensions service");
      struct Dimensions dim = {0, 0, 0};
      return dim;
    }
    
    struct Dimensions dim = {srv.response.width, srv.response.depth, srv.response.height};
    
    ROS_INFO("Received LB dimensions width: %f depth: %f height %f", dim.width, dim.depth, dim.height);
    return dim;
  }

  /**
   * Point the head at a given point
   */
  void pointHeadAt(const geometry_msgs::Point point, const string& frameId){
    ROS_INFO("Pointing head");
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
  bool performScoop(const geometry_msgs::Point point, const Dimensions& dim, const arm_navigation_msgs::CollisionObject& collisionObject){

    ROS_INFO("Litterbox base position is %f %f %f", point.x, point.y, point.z);
    
    ROS_INFO("Moving to the back of the litterbox");
    geometry_msgs::Point backPosition;
    backPosition.x = -0.1;
    moveRightArmRelative(backPosition, identityOrientation(), collisionObject);
    
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
    moveRightArmRelative(horizPosition, identityOrientation(), collisionObject);

    // ros::Duration(180).sleep();
    
    // Now lower the scoop.
    geometry_msgs::Point noMove;
    ROS_INFO("Lowering scoop");
    moveRightArmRelative(noMove, verticalDownOrientation(), collisionObject);
    
    // Now scoop
    ROS_INFO("Scooping");
    geometry_msgs::Point scoopMove;
    scoopMove.x = dim.depth - 2 * LB_SIDE_WIDTH;
    ROS_INFO("depth %f scoopMove %f", dim.depth, scoopMove.x);
    scoopMove.y = 0;
    scoopMove.z = 0;
    moveRightArmRelative(scoopMove, identityOrientation(), collisionObject);


    // Now pick the scoop up.
    moveRightArmRelative(noMove, invVerticalDownOrientation(), collisionObject);
    ROS_INFO("Scoop complete");
    return true;
  }
  
 
  /**
   * Move the arm over the litterbox 
   */
  void moveArmOverLitterbox(const geometry_msgs::Point& point, const Dimensions& dim, const arm_navigation_msgs::CollisionObject& collisionObject){
    ROS_INFO("Moving arm over litterbox");

    // Center the arm over the litterbox 0.5 meters above the ground. Point should be the center point of the front
    // edge of the litterbox halfway back in the litterbox.
    ROS_INFO("Litterbox back in base frame x %f y %f z %f", point.x, point.y, point.z);
    geometry_msgs::Point overLitterboxPoint = point;
    
    overLitterboxPoint.x = overLitterboxPoint.x - SCOOP_LENGTH + (dim.depth / 2.0) - TORSO_TO_FRONT;
    // Leave y as its the center of the litterbox.
    overLitterboxPoint.z = -0.425;
    moveRightArm(overLitterboxPoint, verticalOrientation(), "/torso_lift_link", collisionObject);
    ROS_INFO("Move arm complete");
  }
 
  /**
   * Move the right arm relative to the current position
   * @param position Position to move the arm to
   * @param orientation Orientation to move the arm to
   */
  bool moveRightArmRelative(const geometry_msgs::Point position, const geometry_msgs::Quaternion orientation, const arm_navigation_msgs::CollisionObject& collisionObject){
    return moveRightArm(position, orientation, "r_wrist_roll_link", collisionObject);
  }

  /**
   * Move the right arm relative to a given frame.
   * @param position Position to move the arm to
   * @param orientation Orientation to move the arm to
   * @param referenceFrame Frame the movement is relative to
   */
  bool moveRightArm(const geometry_msgs::Point position, const geometry_msgs::Quaternion orientation, const string referenceFrame, const arm_navigation_msgs::CollisionObject collisionObject){
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
     desiredPos.constraint_region_shape.dimensions.push_back(0.04);
     desiredPos.constraint_region_shape.dimensions.push_back(0.04);
     desiredPos.constraint_region_shape.dimensions.push_back(0.04);
     desiredPos.constraint_region_orientation.x = 0;
     desiredPos.constraint_region_orientation.y = 0;
     desiredPos.constraint_region_orientation.z = 0;
     desiredPos.constraint_region_orientation.w = 1;
     desiredPos.weight = 1;

     arm_navigation_msgs::OrientationConstraint desiredOr;
     desiredOr.header.frame_id = referenceFrame;
     desiredOr.link_name = "r_wrist_roll_link";

     desiredOr.orientation = orientation;
     desiredOr.absolute_roll_tolerance = 0.04;
     desiredOr.absolute_pitch_tolerance = 0.04;
     desiredOr.absolute_yaw_tolerance = 0.04;
     desiredOr.weight = 1.0;

     goal.disable_collision_monitoring = true;

     goal.motion_plan_request.goal_constraints.position_constraints.push_back(desiredPos);
     goal.motion_plan_request.goal_constraints.orientation_constraints.push_back(desiredOr);

     // Allow the scoop and the litterbox cube to intersect.
     /*
     goal.planning_scene_diff.collision_objects.push_back(collisionObject);
     arm_navigation_msgs::CollisionOperation op;
     op.object1 = "scoop";
     op.object2 = collisionObject.id;
     op.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
     goal.operations.collision_operations.push_back(op);
     */
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

