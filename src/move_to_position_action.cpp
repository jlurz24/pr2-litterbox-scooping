#include <litterbox/utils.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <move_base_msgs/MoveBaseAction.h>

// TODO: Define pre and post conditions.

// Generated messages
#include <litterbox/MoveToPositionAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

using namespace std;

/**
 * Moves to a position
 */
class MoveToPositionAction {
public:
  MoveToPositionAction(const string& name): as(nh, name, boost::bind(&MoveToPositionAction::moveToPosition, this, _1), false),
    actionName(name){
    ROS_INFO("Starting init of the move position action");

    as.registerPreemptCallback(boost::bind(&MoveToPositionAction::preemptCB, this));

    pointHeadClient = initClient<PointHeadClient>("/head_traj_controller/point_head_action"); 
    
    baseClient = initClient<MoveBaseClient>("move_base");
    as.start();
  }
  
  void preemptCB(){
    ROS_INFO("Move to position action preempted");
    pointHeadClient->cancelGoal();
    baseClient->cancelGoal();
    as.setPreempted();
  }

  /**
   * Main function to move to a position
   */
  void moveToPosition(const litterbox::MoveToPositionGoalConstPtr& goal){
    ROS_INFO("Moving to position");
    
    if(!as.isActive()){
      ROS_INFO("Move to position action cancelled before started");
      return;
    }

    // Point head at the target.
    pointHeadAt(goal->position);

    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }

    double xLocation = goal->position.point.x;
    double yLocation = goal->position.point.y;

    ROS_INFO("Moving to point %f %f", xLocation, yLocation);
    move_base_msgs::MoveBaseGoal moveGoal;
    moveGoal.target_pose.header.frame_id = "base_link";
    moveGoal.target_pose.header.stamp = ros::Time::now();

    moveGoal.target_pose.pose.position.x = xLocation;
    moveGoal.target_pose.pose.position.y = yLocation;

      // TODO: Do not currently handle orientation.
    moveGoal.target_pose.pose.orientation.w = 1;

    ROS_INFO("Moving to target position");
    printPose(moveGoal.target_pose.pose);
    sendGoal(baseClient, moveGoal, nh);
    ROS_INFO("Target position reached");

    ROS_INFO("Approach to position completed");
    as.setSucceeded(result);
  }

  ~MoveToPositionAction(){
  }
  
  protected:
    ros::NodeHandle nh;
    
    // Actionlib classes
    actionlib::SimpleActionServer<litterbox::MoveToPositionAction> as;
    string actionName;
  
    // create messages that are used to published feedback/result
    litterbox::MoveToPositionFeedback feedback;
    litterbox::MoveToPositionResult result;    
    
    auto_ptr<MoveBaseClient> baseClient;
    auto_ptr<PointHeadClient> pointHeadClient;


  /**
   * Point the head at a given point
   * TODO: Refactor and share this code.
   */
  void pointHeadAt(const geometry_msgs::PointStamped point){
    ROS_INFO("Pointing head");
    pr2_controllers_msgs::PointHeadGoal goal;

    goal.target = point;
    goal.target.header.frame_id = "base_link";
    goal.pointing_frame = "wide_stereo_optical";
    goal.pointing_axis.x = 1;
    goal.pointing_axis.y = 0;
    goal.pointing_axis.z = 0;

    // Take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    // Go no faster than 1 rad/s
    goal.max_velocity = 1.0;

    sendGoal(pointHeadClient, goal, nh);
    ROS_INFO("Completed pointing head");

  }
};

int main(int argc, char** argv){
  ROS_INFO("Main function for move_to_position");
  ros::init(argc, argv, "move_to_position");
  ROS_INFO("ROS_INIT complete");
  MoveToPositionAction moveAction(ros::this_node::getName());
  ROS_INFO("Waiting for actions");
  ros::spin();
  
  return 0;
}
