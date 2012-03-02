#include <litterbox/utils.h>
#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_arm_msgs/MoveArmAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <move_arm_msgs/utils.h>

// TODO: Put attaching the scoop in a separate action.

// TODO: Define pre and post conditions.

// Generated messages
#include <litterbox/ScoopLitterboxAction.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> MoveArmClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

using namespace std;

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
    
    ros::NodeHandle nh;

    // Actionlib classes
    actionlib::SimpleActionServer<litterbox::ScoopLitterboxAction> as;
    string actionName;
  
    // create messages that are used to published feedback/result
    litterbox::ScoopLitterboxFeedback feedback;
    litterbox::ScoopLitterboxResult result;

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
      const static double TOLERANCE = 0.02;
    
      move_arm_msgs::MoveArmGoal goal;
      goal.motion_plan_request.group_name = "right_arm";
      goal.motion_plan_request.num_planning_attempts = 1;
      goal.motion_plan_request.planner_id = std::string("");
      goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
      goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
      
      
      motion_planning_msgs::SimplePoseConstraint desired_pose;
      desired_pose.header.frame_id = referenceFrame;
      desired_pose.link_name = "r_wrist_roll_link";

      desired_pose.pose.position = position;
      desired_pose.pose.orientation = orientation;

      desired_pose.absolute_position_tolerance.x = TOLERANCE;
      desired_pose.absolute_position_tolerance.y = TOLERANCE;
      desired_pose.absolute_position_tolerance.z = TOLERANCE;

      desired_pose.absolute_roll_tolerance = TOLERANCE;
      desired_pose.absolute_pitch_tolerance = TOLERANCE;
      desired_pose.absolute_yaw_tolerance = TOLERANCE;

      move_arm_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goal);

      // execute it
      return sendGoal(rightArmClient, goal, nh);
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

