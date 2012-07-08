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
    if(!as.isActive()){
      ROS_INFO("Action not yet active");
      return;
    }
    if(pointHeadClient->getState() == actionlib::SimpleClientGoalState::ACTIVE){
      pointHeadClient->cancelGoal();
    }
    if(baseClient->getState() == actionlib::SimpleClientGoalState::ACTIVE){
      baseClient->cancelGoal();
    }
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
    if(goal->pointHeadAtTarget){
      pointHeadAt(goal->target.pose.position, goal->target.header.frame_id);

      if(as.isPreemptRequested() || !ros::ok()){
        as.setPreempted();
        return;
      }
    }
    
    ROS_INFO("Moving to target position");
    printPose(goal->target.pose);

    move_base_msgs::MoveBaseGoal moveGoal;
    moveGoal.target_pose = goal->target;
    moveGoal.target_pose.header.stamp = ros::Time::now();
    sendGoal(baseClient, moveGoal, nh, 60);

    ROS_INFO("Target position reached");
    // Don't repoint the head at the target because the robot
    // is now on top of it.
    as.setSucceeded(result);
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
   * Print a position
   */
  static void printPose(const geometry_msgs::Pose& currentPose) {
    std::cout << "position" << " x:" << currentPose.position.x << " y:" << currentPose.position.y << " z:" << currentPose.position.z << std::endl;
    std::cout << "orientation" << " x:" << currentPose.orientation.x << " y:" << currentPose.orientation.y << " z:" << currentPose.orientation.z << " w:" << currentPose.orientation.w << std::endl;
  }

  /**
   * Point the head at a given point
   * TODO: Refactor and share this code.
   */
  void pointHeadAt(const geometry_msgs::Point point, const std::string& frameId){
    ROS_INFO("Pointing head");
    pr2_controllers_msgs::PointHeadGoal goal;

    goal.target.point = point;
    goal.target.header.frame_id = frameId;

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
