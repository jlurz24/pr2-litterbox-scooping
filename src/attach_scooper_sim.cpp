#include <ros/ros.h>
#include <gazebo/SpawnModel.h>
#include <tinyxml/tinyxml.h>
#include <tf/tf.h>

using namespace std;

int main(int argc, char** argv){
  ROS_INFO("Adding the scooper");
  ros::init(argc, argv, "attach_scooper");
    
  ros::NodeHandle nh("");
  ros::service::waitForService("gazebo/spawn_gazebo_model");

  ros::ServiceClient gazeboClient = nh.serviceClient<gazebo::SpawnModel> ("/gazebo/spawn_gazebo_model");
  ROS_INFO("Connected to gazebo client");

  gazebo::SpawnModel model;
  TiXmlDocument xmlIn("objects/scooper.model");
  xmlIn.LoadFile();
  std::ostringstream stream;
  stream << xmlIn;

  ROS_INFO("XML %s", stream.str().c_str());

  model.request.model_xml = stream.str();
  model.request.robot_namespace = "";
  model.request.model_name = "scoop";  
  model.request.initial_pose.position.x = 0.0;
  model.request.initial_pose.position.y = 0.0;
  model.request.initial_pose.position.z = 0.0;
  model.request.initial_pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  model.request.reference_frame = "r_gripper_palm_link";
  
  gazeboClient.call(model);
  if(!model.success){
    ROS_INFO("Call failed");
  }
  ROS_INFO("Response status: %s", model.response.status_message.c_str());
  ROS_INFO("Added successfully");
  return 0;
}
