#include <litterbox/utils.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <boost/math/constants/constants.hpp>
#include <pr2_controllers_msgs/PointHeadAction.h>

// TODO: Define pre and post conditions.

// Generated messages
#include <litterbox/ExploreAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

using namespace std;

/**
 * Explores the area near the robot
 */
class ExploreAction {
public:
  ExploreAction(const string& name): as(nh, name, boost::bind(&ExploreAction::explore, this, _1), false), actionName(name){
    
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

    if(baseClient->getState() == actionlib::SimpleClientGoalState::ACTIVE){
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

    // Head pointing is not currently cancellable.
    {
      std::auto_ptr<PointHeadClient> pointerClient = initClient<PointHeadClient>("/head_traj_controller/point_head_action");
      pr2_controllers_msgs::PointHeadGoal phGoal;

      // Look at the ground 10 meters away at 12 oclock.
      phGoal.target.point.x = 10;
      phGoal.target.point.y = 0;
      phGoal.target.point.z = 0;
      phGoal.target.header.frame_id = "base_link";
      sendGoal(pointerClient, phGoal, nh);
    }

    move_base_msgs::MoveBaseGoal moveGoal;
    moveGoal.target_pose.header.frame_id = "base_link";
    moveGoal.target_pose.header.stamp = ros::Time::now();
    const unsigned int NUM_STEPS = 8;

    // Rotate one arc at a time to resolve ambiguity about which way to spin. 
    moveGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, boost::math::constants::pi<double>() / (NUM_STEPS / 2));

    // Pause prior to starting to allow detection to catch up.
    ros::Duration(5.0).sleep();
    for(unsigned int i = 0; i < NUM_STEPS; ++i){
      if(as.isPreemptRequested() || !ros::ok()){
        as.setPreempted();
        return;
      }

      // Pause
      ros::Duration(1.0).sleep(); 

      sendGoal(baseClient, moveGoal, nh);
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
