#include <litterbox/utils.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>

// Generated messages
#include <litterbox/FaceTargetAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

/**
 * Faces a target
 */
class FaceTargetAction {
public:
  FaceTargetAction(const string& name): as(nh, name, boost::bind(&FaceTargetAction::moveToPosition, this, _1), false),
    actionName(name){
    ROS_INFO("Starting init of the face target action");
    as.registerPreemptCallback(boost::bind(&FaceTargetAction::preemptCB, this));
    baseClient = initClient<MoveBaseClient>("move_base");
    as.start();
  }
  
  void preemptCB(){
    ROS_INFO("Face target action preempted");
    if(!as.isActive()){
      ROS_INFO("Action not yet active");
      return;
    }
    
    if(baseClient->getState() == actionlib::SimpleClientGoalState::ACTIVE){
      baseClient->cancelGoal();
    }
    as.setPreempted();
  }

  /**
   * Main function to move to a position
   */
  void moveToPosition(const litterbox::FaceTargetGoalConstPtr& goal){
    ROS_INFO("Facing target");
    
    if(!as.isActive()){
      ROS_INFO("Move to position action cancelled before started");
      return;
    }
    
    ROS_INFO("Facing target");
    move_base_msgs::MoveBaseGoal moveGoal;

    // Calculate the yaw to the target.
    // TODO: Does this assume a robot relative frameo
    double yaw = atan(goal->target.pose.position.x / -goal->target.pose.position.y);
    moveGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    moveGoal.target_pose.header.frame_id = goal->target.header.frame_id;
    moveGoal.target_pose.header.stamp = ros::Time::now();
    sendGoal(baseClient, moveGoal, nh, 60);

    ROS_INFO("Target orientation");

    as.setSucceeded(result);
  }

  protected:
    ros::NodeHandle nh;
    tf::TransformListener tf;
    
    // Actionlib classes
    actionlib::SimpleActionServer<litterbox::FaceTargetAction> as;
    string actionName;
  
    // create messages that are used to published feedback/result
    litterbox::FaceTargetFeedback feedback;
    litterbox::FaceTargetResult result;    
    
    auto_ptr<MoveBaseClient> baseClient;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "face_target");
  FaceTargetAction moveAction(ros::this_node::getName());
  ros::spin();
  
  return 0;
}
