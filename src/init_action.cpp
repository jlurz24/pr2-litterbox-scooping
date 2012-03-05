#include <litterbox/utils.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <tf/tf.h>



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
    collisionPublisher = nh.advertise<mapping_msgs::AttachedCollisionObject>("attached_collision_object", 10);    
    as.start();
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
    mapping_msgs::AttachedCollisionObject scoop;
    scoop.link_name = "r_gripper_palm_link";
    scoop.touch_links.push_back("r_gripper_r_finger_link");
    scoop.touch_links.push_back("r_gripper_l_finger_link");
    scoop.touch_links.push_back("r_gripper_l_finger_tip_link");
    scoop.touch_links.push_back("r_gripper_r_finger_tip_link");
    scoop.object.id = "scoop";
    scoop.object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
    scoop.object.header.frame_id = "r_gripper_r_finger_tip_link";
    scoop.object.header.stamp = ros::Time::now();

    geometric_shapes_msgs::Shape object;
    object.type = geometric_shapes_msgs::Shape::CYLINDER;
    object.dimensions.resize(2);
    object.dimensions[0] = 0.07;
    object.dimensions[1] = 0.35;
    geometry_msgs::Pose pose;
    pose.position.x = 0.12;
    pose.position.y = 0.025;
    pose.position.z = 0.0;
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, boost::math::constants::pi<double>() / 2, 0);

    scoop.object.shapes.push_back(object);
    scoop.object.poses.push_back(pose);

    collisionPublisher.publish(scoop);

    ROS_INFO("Scoop attached");

    as.setSucceeded(result);
  }

  ~InitAction(){
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
