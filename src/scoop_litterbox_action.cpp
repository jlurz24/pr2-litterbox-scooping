#include <litterbox/utils.h>
#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_arm_msgs/MoveArmAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <move_arm_msgs/utils.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <planning_environment_msgs/GetRobotState.h>
#include <boost/math/constants/constants.hpp>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>

// TODO: Put attaching the scoop in a separate action.

// TODO: Define pre and post conditions.

// Generated messages
#include <litterbox/ScoopLitterboxAction.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> MoveArmClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

using namespace std;

static const double PI = boost::math::constants::pi<double>();

/**
 * Cleans a litterbox
 */
class ScoopLitterboxAction {
public:
  ScoopLitterboxAction(const string& name): as(nh, name, boost::bind(&ScoopLitterboxAction::scoopLitterbox, this, _1), false),
    actionName(name){
       
    ROS_INFO("Starting initialization");
    
    as.registerPreemptCallback(boost::bind(&ScoopLitterboxAction::preemptCB, this));
    torsoClient = initClient<TorsoClient>("torso_controller/position_joint_action");
    rightArmClient = initClient<MoveArmClient>("move_right_arm");
    trajectoryClient =initClient<TrajClient>("r_arm_controller/joint_trajectory_action"); //LZ
    pointHeadClient = initClient<PointHeadClient>("/head_traj_controller/point_head_action"); 
    as.start();
    ROS_INFO("Initialization complete");
  }
  
  void preemptCB(){
    // TODO: Need much smarter post cancel behavior to put
    //       the robot back in a known state
    ROS_INFO("Scoop litterbox preempted");
    torsoClient->cancelGoal();
    pointHeadClient->cancelGoal();
    rightArmClient->cancelGoal();
    as.setPreempted();
  }

  /**
   * Main function to scoop the litterbox.
   */
  void scoopLitterbox(const litterbox::ScoopLitterboxGoalConstPtr& goal){
    ROS_INFO("Scooping the litterbox");
    
    if(!as.isActive()){
      ROS_INFO("Scoop litterbox cancelled prior to start");
      return;
    }
    torsoDown();
    
    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }
    
    pointHeadAt(goal->position);
    
    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }

    moveArmOverLitterboxJ();
    
    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }

    performScoop();
   
    if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
    }
 
    moveArmOverLitterboxJ();
    
    ROS_INFO("Litterbox cleaned successfully");
    as.setSucceeded(result);
  }

  ~ScoopLitterboxAction(){
  }
  
  private:
    auto_ptr<TorsoClient> torsoClient;
    auto_ptr<MoveArmClient> rightArmClient;
    auto_ptr<PointHeadClient> pointHeadClient;
    auto_ptr<TrajClient> trajectoryClient;
    ros::NodeHandle nh;

    // Actionlib classes
    actionlib::SimpleActionServer<litterbox::ScoopLitterboxAction> as;
    string actionName;
  
    // create messages that are used to published feedback/result
    litterbox::ScoopLitterboxFeedback feedback;
    litterbox::ScoopLitterboxResult result;

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
   
  /**
   * Point the head at a given point
   */
  void pointHeadAt(const geometry_msgs::PointStamped point){
    ROS_INFO("Pointing head");
    pr2_controllers_msgs::PointHeadGoal goal;

    goal.target = point;

    goal.pointing_frame = "wide_stereo_optical";
    goal.pointing_axis.x = 1;
    goal.pointing_axis.y = 0;
    goal.pointing_axis.z = 0;

    // Take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    // Go no faster than 1 rad/s
    goal.max_velocity = 1.0;

    sendGoal(pointHeadClient, goal, nh);
    ROS_INFO("Completed pointing head");

  }

  /**
   * Move the torso to the lowest possible height.
   */
  void torsoDown(){
    ROS_INFO("Lowering the torso");
    pr2_controllers_msgs::SingleJointPositionGoal down;
    down.position = 0.01;
    down.min_duration = ros::Duration(2.0);
    down.max_velocity = 1.0;

    sendGoal(torsoClient, down, nh);
    ROS_INFO("Torso down movement completed");
  }
  
  /**
   * Perform the scoop of the litterbox
   */
  bool performScoop(){
   ROS_INFO("Performing scoop");

 
    // Distance from each side to avoid.
    const double BOX_BUFFER = 0.02;
    
    // First move behind the litterbox.
    const double positions[] = {0.221012,
                                0.95,
                                0,
                                -0.4,
                                0.15,
                                -0.52,
                                1.45};
    std::vector<double> positionVec(&positions[0], &positions[7]);
    moveArmToJointPositions(positionVec);
    
    ROS_INFO("Arm moved to back of litterbox");

    // Now move to a random horizontal position.
    geometry_msgs::Point position;
    
    // Add randomness to pick a spot in the litter box.
    const double BOX_WIDTH = 0.31 - 2 * BOX_BUFFER;

    double horizontal = BOX_WIDTH / double(2) - ((double(rand()) / RAND_MAX) * BOX_WIDTH);
    position.x = 0;
    position.y = 0;
    position.z = horizontal;
    ROS_INFO("Adjusting horizontal by random %f", horizontal);
    moveRightArmRelative(position, identityOrientation());
    
    // Now lower the scoop.
    positionVec = getJointState(nh);
    positionVec[5] = -0.2;
    ROS_INFO("Lowering scoop");
    moveArmToJointPositions(positionVec);
    
    // Now scoop
    positionVec[1] = 0.75;
    positionVec[3] = -0.2;
    ROS_INFO("Scooping");
    moveArmToJointPositions(positionVec);

    ROS_INFO("Scoop complete");
    return true;
  }
  
  /**
   * Joint positions that place the arm over the litterbox
   */ 
  const std::vector<double> overLitterboxPositions() const {
    const static double positions[] = {0.221012,
                                0.400292,
                                -0.000006,
                                -0.2,
                                0.751933,
                                -0.326723,
                                0.845995};
    return std::vector<double>(&positions[0], &positions[7]);
  }
 
  /**
   * Move the arm over the litterbox 
   */
  void moveArmOverLitterboxJ(){
    ROS_INFO("Moving arm over litterbox");
    moveArmToJointPositions(overLitterboxPositions());
    ROS_INFO("Move arm complete");
  }
 
  /**
   * Move the arm according to a vector of joint positions. Must contain 7 positions in the following order:
   * r_shoulder_pan_joint - 0.56 - -2.135
   * r_shoulder_lift_joint - 1.29 - -0.35 deg
   * r_upper_arm_roll_joint - 0.65 - -3.75 deg
   * r_elbow_flex_joint - - -0.15 -2.12
   * r_forearm_roll_joint - Continuous
   * r_wrist_flex_joint - -2.0 - -0.1
   * r_wrist_roll_joint - Continuous
   */
  void moveArmToJointPositions(const std::vector<double>& positions){

    pr2_controllers_msgs::JointTrajectoryGoal goalB;

    // First, the joint names, which apply to all waypoints
    goalB.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goalB.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goalB.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goalB.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goalB.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goalB.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goalB.trajectory.joint_names.push_back("r_wrist_roll_joint");

    goalB.trajectory.points.resize(1);

    // Move arm over litterbox position
    // Positions
    goalB.trajectory.points[0].positions.resize(7);

    for(unsigned int i = 0; i < positions.size(); ++i){
      goalB.trajectory.points[0].positions[i] = positions[i];
    }

    // Velocities
    goalB.trajectory.points[0].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goalB.trajectory.points[0].velocities[j] = 0.0;
    }
    // To be reached 4 second after starting along the trajectory
    goalB.trajectory.points[0].time_from_start = ros::Duration(2.0);
    goalB.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    
    sendGoal(trajectoryClient, goalB, nh);
  }

  /**
   * Move the right arm relative to the current position
   * @param position Position to move the arm to
   * @param orientation Orientation to move the arm to
   */
  bool moveRightArmRelative(const geometry_msgs::Point position, const geometry_msgs::Quaternion orientation){
    return moveRightArm(position, orientation, "r_wrist_roll_link");
  }

  /**
   * Move the right arm relative to a given frame.
   * @param position Position to move the arm to
   * @param orientation Orientation to move the arm to
   * @param referenceFrame Frame the movement is relative to
   */
  bool moveRightArm(const geometry_msgs::Point position, const geometry_msgs::Quaternion orientation, const std::string referenceFrame){
     pr2_controllers_msgs::JointTrajectoryGoal goalB;
     // define the IK service messages
     kinematics_msgs::GetKinematicSolverInfo::Request request;
     kinematics_msgs::GetKinematicSolverInfo::Response response;
     kinematics_msgs::GetPositionIK::Request  gpik_req;
     kinematics_msgs::GetPositionIK::Response gpik_res;

     // First, the joint names, which apply to all waypoints
     goalB.trajectory.joint_names.push_back("r_shoulder_pan_joint");
     goalB.trajectory.joint_names.push_back("r_shoulder_lift_joint");
     goalB.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
     goalB.trajectory.joint_names.push_back("r_elbow_flex_joint");
     goalB.trajectory.joint_names.push_back("r_forearm_roll_joint");
     goalB.trajectory.joint_names.push_back("r_wrist_flex_joint");
     goalB.trajectory.joint_names.push_back("r_wrist_roll_joint");
     goalB.trajectory.points.resize(1);
     
     //IK
     ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
     ros::service::waitForService("pr2_right_arm_kinematics/get_ik");
     ros::ServiceClient query_client = nh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
     ros::ServiceClient ik_client = nh.serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");
     query_client.call(request,response);
      
     gpik_req.timeout = ros::Duration(4.0);
     gpik_req.ik_request.ik_link_name = "r_wrist_roll_link";

     gpik_req.ik_request.pose_stamped.header.frame_id = referenceFrame;
     gpik_req.ik_request.pose_stamped.pose.position = position;
     gpik_req.ik_request.pose_stamped.pose.orientation = orientation;
     gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
     gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
     for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
         {
         gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
         }
      
     if(ik_client.call(gpik_req, gpik_res))
        {
         while (gpik_res.error_code.val != gpik_res.error_code.SUCCESS)
              ROS_ERROR("Inverse kinematics failed");        
         goalB.trajectory.points[0].positions = gpik_res.solution.joint_state.position;
        }
      else
        ROS_ERROR("Inverse kinematics service call failed");

     // Velocities
     goalB.trajectory.points[0].velocities.resize(7);
     for (size_t j = 0; j < 7; ++j)
        {
        goalB.trajectory.points[0].velocities[j] = 0.0;
        }
     // To be reached 2 second after starting along the trajectory
     goalB.trajectory.points[0].time_from_start = ros::Duration(2.0);
     goalB.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    
     return sendGoal(trajectoryClient, goalB, nh);
  }

  /**
   * Return the identity orientation
   * @return The identity orientation
   */
  geometry_msgs::Quaternion identityOrientation() const {
    return tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "scoop_litterbox");

  ScoopLitterboxAction scoopAction(ros::this_node::getName());
  ros::spin();

  return 0;
}

