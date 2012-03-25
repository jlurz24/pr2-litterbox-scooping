#include <litterbox/utils.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_arm_msgs/MoveArmAction.h>
#include <move_arm_msgs/utils.h>
#include <boost/math/constants/constants.hpp>

// TODO: Define pre and post conditions.

// Generated messages
#include <litterbox/DumpPoopAction.h>

typedef actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> MoveArmClient;

using namespace std;

/**
 * Dumps poop into a trash can
 */
class DumpPoopAction {
public:
  DumpPoopAction(const string& name): as(nh, name, boost::bind(&DumpPoopAction::dumpPoop, this, _1), false), actionName(name){
    
    ROS_INFO("Starting initialization");
    
    rightArmClient = initClient<MoveArmClient>("move_right_arm");
    as.registerPreemptCallback(boost::bind(&DumpPoopAction::preemptCB, this));
    as.start();
    
    ROS_INFO("Initialization complete");
  }
  
  void preemptCB(){
    rightArmClient->cancelGoal();
    as.setPreempted();
  }

  /**
   * Main function to dump poop
   */
  void dumpPoop(const litterbox::DumpPoopGoalConstPtr& goal){
   if(!as.isActive()){
     ROS_INFO("Dump poop action cancelled before started");
     return;
   }
    
   ROS_INFO("Dumping Poop");
    
   std::vector<double> flipped = overTrashJoints();
   flipped[6] -= boost::math::constants::pi<double>();
   moveArmToJointPositions(flipped);
   
   if(as.isPreemptRequested() || !ros::ok()){
      as.setPreempted();
      return;
   }

   ROS_INFO("Poop dumped. Returning to base orientation.");
   moveArmToJointPositions(overTrashJoints());
   ROS_INFO("Grabber returned to normal orientation");
    
   as.setSucceeded(result);
  }

  ~DumpPoopAction(){
  }
  
  private:
    auto_ptr<MoveArmClient> rightArmClient;
    
    ros::NodeHandle nh;

    // Actionlib classes
    actionlib::SimpleActionServer<litterbox::DumpPoopAction> as;
    string actionName;
  
    // create messages that are used to published feedback/result
    litterbox::DumpPoopFeedback feedback;
    litterbox::DumpPoopResult result;

  /**
   * Joint positions that place the arm over the trash
   */
   std::vector<double> overTrashJoints(){
     const static double positions[] = {-0.5,
                                 0.4,
                                 0,
                                -0.2,
                                 0.752,
                                -0.327,
                                 0.9
                               };

    return std::vector<double> (&positions[0], &positions[7]);
  }

   /* Move the arm according to a vector of joint positions. Must contain 7 positions in the following order:
   * r_shoulder_pan_joint - 0.56 - -2.135
   * r_shoulder_lift_joint - 1.29 - -0.35 deg
   * r_upper_arm_roll_joint - 0.65 - -3.75 deg
   * r_elbow_flex_joint - - -0.15 -2.12
   * r_forearm_roll_joint - Continuous
   * r_wrist_flex_joint - -2.0 - -0.1
   * r_wrist_roll_joint - Continuous
   */
  void moveArmToJointPositions(const std::vector<double>& positions){

    move_arm_msgs::MoveArmGoal goalB;
    std::vector<std::string> names(7);
    names[0] = "r_shoulder_pan_joint";
    names[1] = "r_shoulder_lift_joint";
    names[2] = "r_upper_arm_roll_joint";
    names[3] = "r_elbow_flex_joint";
    names[4] = "r_forearm_roll_joint";
    names[5] = "r_wrist_flex_joint";
    names[6] = "r_wrist_roll_joint";

    goalB.motion_plan_request.group_name = "right_arm";
    goalB.motion_plan_request.num_planning_attempts = 1;
    goalB.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

    goalB.motion_plan_request.planner_id= std::string("");
    goalB.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
    goalB.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

    for (unsigned int i = 0 ; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size(); ++i){
      goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
      goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
      goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.01;
      goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.01;
    }
    
    for(unsigned int i = 0; i < positions.size(); ++i){
      goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = positions[i];
    }
    sendGoal(rightArmClient, goalB, nh);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "dump_poop");

  DumpPoopAction dumpPoopAction(ros::this_node::getName());
  ros::spin();

  return 0;
}

