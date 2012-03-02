#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <planning_environment_msgs/GetRobotState.h>
#include <boost/math/constants/constants.hpp>

const static double PI = boost::math::constants::pi<double>();

    /*
     * initialize a client of the specified type and service name.
     * @param serviceName Name of the service.
     * @return Pointer to an initialized client.
     */
    template<class T>
    static std::auto_ptr<T> initClient(const std::string& serviceName){
      ROS_INFO("Initilizing client for %s", serviceName.c_str());
      std::auto_ptr<T> client(new T(serviceName, true));

      // Wait for the action server to come up
      while(!client->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for service %s to come up", serviceName.c_str());
      }
      ROS_INFO("Service %s connected", serviceName.c_str());
      return client;
    }

  /**
   * Get the current right arm joint positions
   */
  static const std::vector<double> getJointState(ros::NodeHandle& nh){
    ROS_INFO("Fetching the robot state");
    ros::service::waitForService("environment_server/get_robot_state");
    ros::ServiceClient getStateClient = nh.serviceClient<planning_environment_msgs::GetRobotState>("environment_server/get_robot_state");

    std::vector<double> result;
    result.resize(7);

    static const std::string links[] = {"r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"};

    const unsigned int TARGET_LINKS = 7;

    planning_environment_msgs::GetRobotState::Request request;
    planning_environment_msgs::GetRobotState::Response response;
    if(getStateClient.call(request,response)){
      for(unsigned int i = 0; i < response.robot_state.joint_state.name.size(); ++i){
        for(unsigned int j = 0; j < TARGET_LINKS; ++j){
          if(response.robot_state.joint_state.name[i] == links[j]){
            result[j] = response.robot_state.joint_state.position[i];
            // Continuous joints may report outside of their positionable range.
            if(links[j] == "r_forearm_roll_joint" || links[j] == "r_wrist_roll_joint"){
              // Shift out of the range spanning 0 to an all positive range, then remove excess rotations
              // and correct back to the original range.
              result[j] = fmod(result[j] + PI, 2 * PI) - PI;
            }
            break;
          }
        }
      }
    }
    else {
      ROS_ERROR("Service call to get robot state failed on %s", getStateClient.getService().c_str());
    }
    return result;
  }

  /*
   * Send a goal to an action client, wait for a result, and 
   * report success or failure.
   * @param client Action client to send the goal to
   * @param goal Goal to send
   * @return Whether the goal was executed successfully.
   */
  template<class T, class U>
  static bool sendGoal(const T& client, const U& goal, ros::NodeHandle& nh){
    bool success = false;
    if (nh.ok()){
      ROS_INFO("Sending goal");
      client->sendGoal(goal);
      if(!client->waitForResult(ros::Duration(200.0))){
        client->cancelGoal();
        ROS_INFO("Timed out achieving goal");
      }
      else {
        actionlib::SimpleClientGoalState state = client->getState();
        success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
        if(success){
          ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else {
          ROS_INFO("Action failed: %s",state.toString().c_str());
        }
    }

  }
  else {
    ROS_INFO("Nodehandle is invalid. Not sending action");
  }

  return success;
}

  /**
   * Print a position
   */
  static void printPose(const geometry_msgs::Pose& currentPose) {
    std::cout << "position" << " x:" << currentPose.position.x << " y:" << currentPose.position.y << " z:" << currentPose.position.z << std::endl;
    std::cout << "orientation" << " x:" << currentPose.orientation.x << " y:" << currentPose.orientation.y << " z:" << currentPose.orientation.z << " w:" << currentPose.orientation.w << std::endl;
  }


