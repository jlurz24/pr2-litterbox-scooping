#include <litterbox/utils.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <boost/math/constants/constants.hpp>

// TODO: Define pre and post conditions.

// Generated messages
#include <litterbox/ExploreAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

/**
 * Explores the area near the robot
 */
class ExploreAction {
public:
  ExploreAction(const string& name): as(nh, name, boost::bind(&ExploreAction::explore, this, _1), false), actionName(name), moving(false){
    
    as.registerPreemptCallback(boost::bind(&ExploreAction::preemptCB, this));
    ROS_INFO("Starting init of the explore action");
    
    baseClient = initClient<MoveBaseClient>("move_base");
    as.start();
  }
  
  void preemptCB(){
    ROS_INFO("Preempting the explore action");

    if(!as.isActive()){
      ROS_INFO("Explore action cancelled prior to start");
      return;
    }

    if(moving){
      baseClient->cancelGoal();
    }
    as.setPreempted();
  }

  /**
   * Main function to move to a position
   */
  void explore(const litterbox::ExploreGoalConstPtr& goal){
    if(!as.isActive()){
      ROS_INFO("Explore action cancelled prior to start");
      return;
    }

    move_base_msgs::MoveBaseGoal moveGoal;
    moveGoal.target_pose.header.frame_id = "base_link";
    moveGoal.target_pose.header.stamp = ros::Time::now();

    // Rotate one quarter at a time to resolve ambiguity about which way to spin. 
    moveGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, boost::math::constants::pi<double>() / 2.0);

    ROS_INFO("Exploring");
    for(unsigned int i = 0; i < 4; ++i){
      if(as.isPreemptRequested() || !ros::ok()){
        as.setPreempted();
        return;
      }
      
      moving = true;
      sendGoal(baseClient, moveGoal, nh);
      moving = false;
    }

    as.setSucceeded(result);
  }

  ~ExploreAction(){
  }
  
  protected:
    ros::NodeHandle nh;
    
    // Actionlib classes
    actionlib::SimpleActionServer<litterbox::ExploreAction> as;
    string actionName;
    
    // Whether the robot is currently executing a move.  
    bool moving;

    // create messages that are used to published feedback/result
    litterbox::ExploreFeedback feedback;
    litterbox::ExploreResult result;    
    
    std::auto_ptr<MoveBaseClient> baseClient;
};

int main(int argc, char** argv){
  ROS_INFO("Main function for explore_action");
  ros::init(argc, argv, "explore_action");
  ROS_INFO("ROS_INIT complete");
  ExploreAction moveAction(ros::this_node::getName());
  ROS_INFO("Waiting for actions");
  ros::spin();
  
  return 0;
}
