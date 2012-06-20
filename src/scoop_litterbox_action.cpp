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

static const double SCOOP_LENGTH = 0.35;

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
    
    pointHeadAt(goal->target.pose.position, goal->target.header.frame_id);
    
    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }
    
    const Dimensions dim = determineLBDimensions();
    moveArmOverLitterbox(goal->target.pose.position, dim);
    
    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }

    performScoop(goal->target.pose.position, dim);
   
    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }
 
    moveArmOverLitterbox(goal->target.pose.position, dim);
    
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
    
    ROS_INFO("Received LB dimensions");
    struct Dimensions dim = {srv.response.width, srv.response.depth, srv.response.height};
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
  bool performScoop(const geometry_msgs::Point point, const Dimensions& dim){

    // Move the right arm to the back of the litterbox.
    ROS_INFO("Litterbox base position is %f %f %f", point.x, point.y, point.z);

    // Distance from each side to avoid.
    // TODO: This should be based on half the size of the scoop.
    const double BOX_BUFFER = 0.02;

    // Now move to a random horizontal position.
    geometry_msgs::Point position;
    
    // Add randomness to pick a spot in the litter box.
    // TODO: Replace box width with dynamically calculated width.
    const double boxWidth = dim.width - 2 * BOX_BUFFER;

    double horizontal = boxWidth / double(2) - ((double(rand()) / RAND_MAX) * boxWidth);
    position.x = 0;
    position.y = horizontal;
    position.z = 0;
    ROS_INFO("Adjusting horizontal by random %f", horizontal);
    moveRightArmRelative(position, identityOrientation()); // vertical
    
    // Now lower the scoop.
    position.x = 0;
    position.y = 0;
    position.z = -0.25;
    ROS_INFO("Lowering scoop");
    moveRightArmRelative(position, identityOrientation()); // vertical down
    
    // Now scoop
    ROS_INFO("Scooping");
    position.x = dim.depth - boxWidth;
    position.y = 0;
    position.z = 0;
    moveRightArmRelative(position, identityOrientation()); // vertical down

    ROS_INFO("Scoop complete");
    return true;
  }
  
 
  /**
   * Move the arm over the litterbox 
   */
  void moveArmOverLitterbox(const geometry_msgs::Point& point, const Dimensions& dim){
    ROS_INFO("Moving arm over litterbox");

    // Center the arm over the litterbox 0.5 meters above the ground. Point should be the center point of the front
    // edge of the litterbox halfway back in the litterbox.
    ROS_INFO("Litterbox back in base frame x %f y %f z %f", point.x, point.y, point.z);
    geometry_msgs::Point overLitterboxPoint = point;
    overLitterboxPoint.z = 0.25;
    overLitterboxPoint.x; // TODO: This is rough. Scoop size should get us over the lb.
    moveRightArm(overLitterboxPoint, identityOrientation(), "/torso_lift_link");
    ROS_INFO("Move arm complete");
  }
 
  /**
   * Move the right arm relative to the current position
   * @param position Position to move the arm to
   * @param orientation Orientation to move the arm to
   */
  bool moveRightArmRelative(const geometry_msgs::Point position, const geometry_msgs::Quaternion orientation){
    return moveRightArm(position, orientation, "r_wrist_roll_link");
  }

  /**
   * Move the right arm relative to a given frame.
   * @param position Position to move the arm to
   * @param orientation Orientation to move the arm to
   * @param referenceFrame Frame the movement is relative to
   */
  bool moveRightArm(const geometry_msgs::Point position, const geometry_msgs::Quaternion orientation, const string referenceFrame){
     ROS_INFO("Moving to position %f %f %f in frame %s", position.x, position.y, position.z, referenceFrame.c_str());
     arm_navigation_msgs::MoveArmGoal goal;
     goal.motion_plan_request.group_name = "right_arm";
     goal.motion_plan_request.num_planning_attempts = 3;
     goal.motion_plan_request.planner_id = "";
     goal.planner_service_name = "ompl_planning/plan_kinematic_path";
     goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

     arm_navigation_msgs::SimplePoseConstraint desiredPose;
     desiredPose.header.frame_id = referenceFrame;
     desiredPose.link_name = "r_wrist_roll_link";
     desiredPose.pose.position = position;
     desiredPose.pose.orientation = orientation;

     desiredPose.absolute_position_tolerance.x = 0.02;
     desiredPose.absolute_position_tolerance.y = 0.02;
     desiredPose.absolute_position_tolerance.z = 0.02;

     desiredPose.absolute_roll_tolerance = 0.04;
     desiredPose.absolute_pitch_tolerance = 0.04;
     desiredPose.absolute_yaw_tolerance = 0.04;
  
     arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desiredPose, goal);
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
    return tf::createQuaternionMsgFromRollPitchYaw(PI / 2.0, PI / 6.0, 0);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "scoop_litterbox");

  ScoopLitterboxAction scoopAction(ros::this_node::getName());
  ros::spin();

  return 0;
}

