#include <litterbox/utils.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <tf/tf.h>
#include <boost/math/constants/constants.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
// TODO: Define pre and post conditions.

// Generated messages
#include <litterbox/InitAction.h>

using namespace std;

/**
 * Initializes the robot for the scooping tas
 */
class InitAction {
public:
  InitAction(const string& name): as(nh, name, boost::bind(&InitAction::init, this, _1), false), actionName(name){
    
    ROS_INFO("Starting init of the init action");
    collisionPublisher = nh.advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object", 1);    
    as.start();
    initialPosePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
  }
  
  /**
   * Main function to initialize the robot
   */
  void init(const litterbox::InitGoalConstPtr& goal){
    if(!as.isActive()){
      ROS_INFO("Init action cancelled prior to start");
      return;
    }
    
    ROS_INFO("Attaching the virtual scoop");

    // Add the scoop into the collision space
    arm_navigation_msgs::AttachedCollisionObject scoop;
    scoop.link_name = "r_gripper_palm_link";
    scoop.touch_links.push_back("r_gripper_r_finger_link");
    scoop.touch_links.push_back("r_gripper_l_finger_link");
    scoop.touch_links.push_back("r_gripper_l_finger_tip_link");
    scoop.touch_links.push_back("r_gripper_r_finger_tip_link");
    scoop.object.id = "scoop";
    scoop.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    scoop.object.header.frame_id = "/r_gripper_tool_frame";
    scoop.object.header.stamp = ros::Time::now();

    arm_navigation_msgs::Shape object;
    object.type = arm_navigation_msgs::Shape::BOX;
    object.dimensions.resize(3);

    // Buffer dimensions slightly in-case the scoop slips around.
    object.dimensions[0] = 0.05;
    object.dimensions[1] = 0.05;
    object.dimensions[2] = 0.38;

    geometry_msgs::Pose pose;
    pose.position.x = 0.15;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, boost::math::constants::pi<double>() / 2, 0);

    scoop.object.shapes.push_back(object);
    scoop.object.poses.push_back(pose);
    
    while(collisionPublisher.getNumSubscribers() == 0){
      ROS_INFO("Waiting for the collision subscriber to come up");
      ros::Duration(2).sleep();
    }
    collisionPublisher.publish(scoop);

    ROS_INFO("Scoop attached");
    
    // Set the initial pose.
    ROS_INFO("Setting the initial pose");
    geometry_msgs::PoseWithCovarianceStamped initialPose;
    initialPose.header.frame_id = "/map";
    initialPose.pose.pose.position.x = 15;
    initialPose.pose.pose.position.y = 15;
    initialPose.pose.pose.position.z = 0;
    initialPose.pose.pose.orientation.w = 1;

    // Wait for the listener to come up.
    while(initialPosePublisher.getNumSubscribers() == 0){
      ROS_INFO("Waiting for the initial pose subscriber to come up");
      ros::Duration(2).sleep();
    }
    initialPosePublisher.publish(initialPose);
    
    // Set the planning scene so direct arm movement can be performed
    ros::ServiceClient sceneClient = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>("/environment_server/set_planning_scene_diff");
    ROS_INFO("Waiting for planning scene service");
    sceneClient.waitForExistence();
    
    arm_navigation_msgs::SetPlanningSceneDiff::Request planningSceneReq;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planningSceneRes;

    if(!sceneClient.call(planningSceneReq, planningSceneRes)){
      ROS_WARN("Failed to set planning scene");
    }
    else {
      ROS_INFO("Planning scene set successfully");
    }
    as.setSucceeded(result);
  }
  
  protected:
    ros::NodeHandle nh;
    
    // Actionlib classes
    actionlib::SimpleActionServer<litterbox::InitAction> as;
    string actionName;
   
    /**
     * Publisher to publish information about the scoop.
     */
    ros::Publisher collisionPublisher;
 
    /**
     * Publisher for initial position
     */
    ros::Publisher initialPosePublisher;

    // create messages that are used to published feedback/result
    litterbox::InitFeedback feedback;
    litterbox::InitResult result;    
};

int main(int argc, char** argv){
  ROS_INFO("Main function for init_action");
  ros::init(argc, argv, "init_action");
  ROS_INFO("ROS_INIT complete");
  InitAction moveAction(ros::this_node::getName());
  ROS_INFO("Waiting for actions");
  ros::spin();
  
  return 0;
}
