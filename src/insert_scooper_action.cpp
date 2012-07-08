#include <litterbox/utils.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <gazebo/SpawnModel.h>
#include <gazebo/DeleteModel.h>
#include <tinyxml.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <boost/math/constants/constants.hpp>
#include <sstream>

// TODO: Define pre and post conditions.

// Generated messages
#include <litterbox/InsertScooperAction.h>

using namespace std;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

static const string SCOOP_MODEL_NAME = "scoop";

/**
 * Task to cause the robot to open its hand so the operator can
 * insert the scooper.
 */
class InsertScooper {
public:
  InsertScooper(const string& name): privateHandle("~"), scoopInserted(false), scoopNumber(0), as(nh, name, boost::bind(&InsertScooper::insertScooper, this, _1), false), actionName(name) {

    ROS_INFO("Starting init of the insert gripper action");

    privateHandle.getParam("simulation", isSimulation);
    privateHandle.getParam("waittime", waitTime);
    privateHandle.getParam("scoopModelFile", scoopModelFile);

    gripperClient = initClient<GripperClient>("r_gripper_controller/gripper_action");

    as.start();
    ROS_INFO("Init of the insert gripper action complete");
  }

  /**
   * Main function to initialize the robot
   */
  void insertScooper(const litterbox::InsertScooperGoalConstPtr& goal){
    ROS_INFO("Received command to insert scoop");

    if(!as.isActive()){
      ROS_INFO("InsertScooper action cancelled prior to start");
      return;
    }

    // TODO: Determine if there is anywhere we should allow
    //       canceling in this method.
    ROS_INFO("Opening the gripper");
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.04;
    open.command.max_effort = -1.0;
    sendGoal(gripperClient, open, nh);

    // On the robot wait for a person to insert the scoop
    if(!isSimulation){
      ROS_INFO("*** INSERT SCOOPER NOW ****");
      ros::Duration(waitTime).sleep();
      // TODO: Determine whether the gripper was inserted
      result.inserted = true;
    }
    else {
      result.inserted = insertScooperSim();
    }

    pr2_controllers_msgs::Pr2GripperCommandGoal close;
    close.command.position = 0;
    close.command.max_effort = 20.0;
    
    // Intentionally do not use the utility method because this method
    // will not complete in sim if the robot cannot close the gripper.
    ROS_INFO("Closing gripper on scoop");
    gripperClient->sendGoal(close);
    gripperClient->waitForResult(ros::Duration(10.0));

    ROS_INFO("Scooper inserted");
    as.setSucceeded(result);
  }

  protected:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;

    bool isSimulation;
    bool scoopInserted;
    int waitTime;
    int scoopNumber;

    actionlib::SimpleActionServer<litterbox::InsertScooperAction> as;
    string actionName;
    string scoopModelFile;

    auto_ptr<GripperClient> gripperClient;

    // create messages that are used to published feedback/result
    litterbox::InsertScooperFeedback feedback;
    litterbox::InsertScooperResult result;

    bool insertScooperSim(){
      if(scoopInserted){
        // Delete the scoop first.
        ros::service::waitForService("gazebo/delete_model");
        ros::ServiceClient deleteClient = nh.serviceClient<gazebo::DeleteModel> ("/gazebo/delete_model", true);
        gazebo::DeleteModel deleteModel;
        stringstream oldScoopName;
        oldScoopName << SCOOP_MODEL_NAME << "_" << scoopNumber - 1;
        deleteModel.request.model_name = oldScoopName.str();
        deleteClient.call(deleteModel);
        if(!deleteModel.response.success){
          ROS_INFO("Delete scoop model failed: %s", deleteModel.response.status_message.c_str());
        } else {
          // Gazebo takes a moment to reset
          ros::Duration(5.0).sleep();
          ROS_INFO("Delete scoop model complete");
        }
      }

        // Now add the model
        ros::service::waitForService("gazebo/spawn_gazebo_model");
        ros::ServiceClient gazeboClient = nh.serviceClient<gazebo::SpawnModel> ("/gazebo/spawn_gazebo_model", true);
        ROS_INFO("Connected to gazebo client");
    
        gazebo::SpawnModel model;
        TiXmlDocument xmlIn(scoopModelFile);
        xmlIn.LoadFile();
        std::ostringstream stream;
        stream << xmlIn;
    
        model.request.model_xml = stream.str();
        model.request.robot_namespace = "";
        stringstream scoopName;
        scoopName << SCOOP_MODEL_NAME << "_" << scoopNumber;
        model.request.model_name = scoopName.str();
        scoopNumber++;
        // Setting the reference frame does not work, so convert using tf
        // to the r_gripper_tool_frame
        tf::TransformListener tf;
    
        ROS_INFO("Waiting for transform from r_gripper_tool_frame to odom_combined");
        tf.waitForTransform("/r_gripper_tool_frame", "/odom_combined", ros::Time(0), ros::Duration(10.0));
    
        geometry_msgs::PoseStamped gripperPose;
        gripperPose.header.frame_id = "/r_gripper_tool_frame";
        gripperPose.pose.position.x = 0.15;
        gripperPose.pose.position.y = 0;
        gripperPose.pose.position.z = 0;
        gripperPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, boost::math::constants::pi<double>() / 2, 0);
    
        geometry_msgs::PoseStamped worldPose;
        tf.transformPose("/odom_combined", gripperPose, worldPose);
    
        model.request.initial_pose = worldPose.pose;
        gazeboClient.call(model);
        if(!model.response.success){
          ROS_INFO("Add model call failed: %s", model.response.status_message.c_str());
        }
        else {
          scoopInserted = true;
          ROS_INFO("Scoop added successfully");
        }
        return model.response.success;
    }
};

int main(int argc, char** argv){
  ROS_INFO("Initializing node insert_scooper_action");
  ros::init(argc, argv, "insert_scooper_action");
  InsertScooper action(ros::this_node::getName());
  ros::spin();
  return 0;
}
