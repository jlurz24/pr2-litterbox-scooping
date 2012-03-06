#include <litterbox/utils.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>


// TODO: Define pre and post conditions.

// Generated messages
#include <litterbox/InsertScooperAction.h>

using namespace std;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

/**
 * Task to cause the robot to open its hand so the operator can
 * insert the scooper.
 */
class InsertScooper {
public:
  InsertScooper(const string& name): as(nh, name, boost::bind(&InsertScooper::insertScooper, this, _1), false), actionName(name){
    
    ROS_INFO("Starting init of the insert gripper action");
    
    gripperClient = initClient<GripperClient>("r_gripper_controller/gripper_action");

    as.start();
    ROS_INFO("Init of the insert gripper action complete");
  }
  
  /**
   * Main function to initialize the robot
   */
  void insertScooper(const litterbox::InsertScooperGoalConstPtr& goal){
    if(!as.isActive()){
      ROS_INFO("InsertScooper action cancelled prior to start");
      return;
    }
    
    // TODO: Determine if there is anywhere we should allow
    //       canceling in this method.
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;
    sendGoal(gripperClient, open, nh);
  
    ROS_INFO("*** INSERT SCOOPER NOW ****");
    // TODO: Make a parameter
    ros::Duration(10.0).sleep();

    pr2_controllers_msgs::Pr2GripperCommandGoal close;
    close.command.position = 0;
    close.command.max_effort = 50.0;
    sendGoal(gripperClient, close, nh);

    ROS_INFO("Scooper inserted");

    // TODO: Determine whether the gripper was inserted
    result.inserted = true;
    as.setSucceeded(result);
  }

  ~InsertScooper(){
  }
  
  protected:
    ros::NodeHandle nh;
    
    // Actionlib classes
    actionlib::SimpleActionServer<litterbox::InsertScooperAction> as;
    string actionName;
    
    auto_ptr<GripperClient> gripperClient;  
 
    // create messages that are used to published feedback/result
    litterbox::InsertScooperFeedback feedback;
    litterbox::InsertScooperResult result;    
};

int main(int argc, char** argv){
  ROS_INFO("Main function for insert_scooper_action");
  ros::init(argc, argv, "insert_scooper_action");
  ROS_INFO("ROS_INIT complete");
  InsertScooper action(ros::this_node::getName());
  ROS_INFO("Waiting for actions");
  ros::spin();
  
  return 0;
}
