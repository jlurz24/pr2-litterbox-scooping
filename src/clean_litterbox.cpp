#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm_msgs/MoveArmAction.h>
#include <move_arm_msgs/utils.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometric_shapes_msgs/Shape.h>
#include <mapping_msgs/CollisionObject.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <tf/tf.h>
#include <boost/math/constants/constants.hpp>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_common_action_msgs/TuckArmsAction.h>
#include <planning_environment_msgs/GetRobotState.h>
#include <cmath>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> MoveArmClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
typedef actionlib::SimpleActionClient<pr2_common_action_msgs::TuckArmsAction> TuckArmsClient;

using namespace std;

const static double TARGET_DISTANCE_FROM_LB = 0.9; // This includes the robots base
const static double PI = boost::math::constants::pi<double>();

/**
 * Cleans a litterbox
 */
class LitterboxCleaner {
public:
  LitterboxCleaner(){
    torsoClient = NULL;
    rightArmClient = NULL;
    baseClient = NULL;
    gripperClient = NULL;
    pointHeadClient = NULL;
    tuckArmsClient = NULL;
  }

  /**
   * Initialize the litterbox cleaner. Must be called first.
   */
  void init(){
    ROS_INFO("Starting initialization");
    
    torsoClient = initClient<TorsoClient>("torso_controller/position_joint_action");
    rightArmClient = initClient<MoveArmClient>("move_right_arm");
    baseClient = initClient<MoveBaseClient>("move_base");
    gripperClient = initClient<GripperClient>("r_gripper_controller/gripper_action");
    pointHeadClient = initClient<PointHeadClient>("/head_traj_controller/point_head_action");

    collisionPublisher = nh.advertise<mapping_msgs::AttachedCollisionObject>("attached_collision_object", 10);
    
    tuckArmsClient = initClient<TuckArmsClient>("/tuck_arms");

    // Initialize the random number generator to a fixed seed for repeatability
    srand(1000);

    ROS_INFO("Initialization complete");
  }
 
  /**
   * Print a position
   */
  static void printPose(const geometry_msgs::Pose& currentPose) {
    cout << "position" << " x:" << currentPose.position.x << " y:" << currentPose.position.y << " z:" << currentPose.position.z << endl;
    cout << "orientation" << " x:" << currentPose.orientation.x << " y:" << currentPose.orientation.y << " z:" << currentPose.orientation.z << " w:" << currentPose.orientation.w << endl;  
  }
   
  /**
   * Main function to clean the litterbox.
   * @return Whether the litterbox was cleaned successfully.
   */
  bool cleanLitterbox(){
    ROS_INFO("Cleaning the litterbox");

    const unsigned int NUM_SCOOPS = 5;
    bool success = false;
    
    torsoDown();
    pointHeadForward();
    
    // Get ready to scoop.
    untuckArms();
    openGripper();
    attachScoop();

    // Allow time to place scoop.
    // ros::Duration(10.0).sleep();
    closeGripper();
    
    approachLitterbox();

    for(unsigned int i = 0; i < NUM_SCOOPS; ++i){
      
      ROS_INFO("Starting scoop cycle");
      
      moveArmOverLitterboxJ();
      if(!performScoop()){
        ROS_INFO("Could not perform scoop. Aborting cycle");
        continue;
      }
      
      moveArmOverLitterboxJ();
      
      shakeRightArmJ();
      
      moveArmOverTrashJ();
      
      dumpPoop();
      
      success = true;
      ROS_INFO("Ending scoop cycle");
    }
    
    moveAwayFromLitterbox();
    
    ROS_INFO("Litterbox cleaned successfully");
    return success;
  }

  ~LitterboxCleaner(){
    delete torsoClient;
    delete rightArmClient;
    delete baseClient;
    delete gripperClient;
    delete pointHeadClient;
    delete tuckArmsClient;
  }
  
  private:
    TorsoClient* torsoClient;
    MoveArmClient* rightArmClient;
    MoveBaseClient* baseClient;
    GripperClient* gripperClient;
    PointHeadClient* pointHeadClient;
    TuckArmsClient* tuckArmsClient;

    /**
     * Publisher to publish information about the scoop.
     */
    ros::Publisher collisionPublisher;
    
    /**
     * Core nodehandle
     */
    ros::NodeHandle nh;

    /**
     * Initialize a client of the specified type and service name.
     * @param serviceName Name of the service.
     * @return Pointer to an initialized client.
     */
    template<class T>
    T* initClient(const std::string& serviceName){
      ROS_INFO("Initilizing client for %s", serviceName.c_str());
      T* client = new T(serviceName, true);
      
      // Wait for the action server to come up
      while(!client->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for service %s to come up", serviceName.c_str());
      }
      ROS_INFO("Service %s connected", serviceName.c_str());
      return client;
    }
  
  // Print the current state of all the robots joints.
  void printJointState(){
    std::vector<double> positions = getJointState();
    for(unsigned int i = 0; i < positions.size(); ++i){
      ROS_INFO("Joint State: %f", positions[i]);
    }
  }

  /**
   * Get the current right arm joint positions
   */
  const std::vector<double> getJointState(){
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
  
  // Point the head to the initial orientation.
  void pointHeadForward(){
    ROS_INFO("Pointing head forward");

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = "base_link";
    point.point.x = 3;
    point.point.y = 0;
    point.point.z = 0;
    pointHeadAt(point); 
   ROS_INFO("Completed pointing head forward");
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

    sendGoal(pointHeadClient, goal);
    ROS_INFO("Completed pointing head");

  }

  // Locate the litterbox
  geometry_msgs::PointStampedConstPtr locateLitterbox(){
    return ros::topic::waitForMessage<geometry_msgs::PointStamped>("litterbox_location", nh);
  }
 
  // Open the gripper
  void openGripper(){
    ROS_INFO("Opening gripper");
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;
    sendGoal(gripperClient, open);
    ROS_INFO("Gripper opened");
  }

  // Close the gripper
  void closeGripper(){
    ROS_INFO("Closing gripper");
    pr2_controllers_msgs::Pr2GripperCommandGoal close;
    close.command.position = 0;
    close.command.max_effort = 50.0;
    sendGoal(gripperClient, close);
    ROS_INFO("Gripper closed");
  }
  
  // Tuck both arms
  void tuckArms(){
    ROS_INFO("About to tuck arms");
    pr2_common_action_msgs::TuckArmsGoal goal;
    goal.tuck_left = true;
    goal.tuck_right = true;
    sendGoal(tuckArmsClient, goal);
    ROS_INFO("Arms tucked");
  }
  
  // Untuck the right arm.
  void untuckArms(){
    ROS_INFO("Untucking the arms");
    pr2_common_action_msgs::TuckArmsGoal goal;
    goal.tuck_left = false;
    goal.tuck_right = false;
    sendGoal(tuckArmsClient, goal);
    ROS_INFO("Arms untucked");
  }

  // Add the scoop to the environment
  void attachScoop(){
    ROS_INFO("Attaching the scoop");

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
    ROS_INFO("** Insert the scoop now **");
  }

  // Approach the litterbox
  void approachLitterbox(){
    ROS_INFO("Moving forward to litterbox");
    bool notReached = true;
    const double MAX_DISTANCE = 1.0;

    while(notReached){
      const geometry_msgs::PointStampedConstPtr location = locateLitterbox();
      // Point head at the target.
      pointHeadAt(*location);

      ROS_INFO("Litterbox at location %f %f", location->point.x, location->point.y);
      
      notReached = false;
      double xLocation = location->point.x - TARGET_DISTANCE_FROM_LB;
      double yLocation = location->point.y;

      if(fabs(xLocation) > MAX_DISTANCE){
        notReached = true;
        xLocation = xLocation > 0 ? MAX_DISTANCE : -MAX_DISTANCE;
      }
      if(fabs(yLocation) > MAX_DISTANCE){
        notReached = true;
        yLocation = yLocation > 0 ? MAX_DISTANCE : -MAX_DISTANCE;
      }
      
      ROS_INFO("Moving to point %f %f", xLocation, yLocation);
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "base_link";
      goal.target_pose.header.stamp = ros::Time::now();
    
      goal.target_pose.pose.position.x = xLocation;
      goal.target_pose.pose.position.y = yLocation;

      // TODO: Do not currently handle orientation.
      goal.target_pose.pose.orientation.w = 1;
    
      ROS_INFO("Moving to target position");
      printPose(goal.target_pose.pose);
      sendGoal(baseClient, goal);
      ROS_INFO("Target position reached");
    }

    ROS_INFO("Approach to litter box move completed");
  }
  
  // Back away from the litterbox.
  void moveAwayFromLitterbox(){
    ROS_INFO("Moving back from litterbox");
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    // Back away from the litterbox
    goal.target_pose.pose.position.x = -2;
    goal.target_pose.pose.position.y = 0;
    goal.target_pose.pose.orientation.w = 1;
    
    ROS_INFO("Moving to target position");
    printPose(goal.target_pose.pose);
    sendGoal(baseClient, goal);
    ROS_INFO("Move away from litter box move completed");
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

    sendGoal(torsoClient, down);
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
    positionVec = getJointState();
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

  /**
   * Move the arm over the trash
   */
  void moveArmOverTrashJ(){
    ROS_INFO("Moving arm over trash");
    moveArmToJointPositions(overTrashJoints());
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
    sendGoal(rightArmClient, goalB);
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
      bool success = sendGoal(rightArmClient, goal);
      printJointState();
      return success;
  }

  /**
   * Return the identity orientation
   * @return The identity orientation
   */
  geometry_msgs::Quaternion identityOrientation() const {
    return tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  }


  /**
   * Return the base orientation
   * @return The base orientation
   */
  geometry_msgs::Quaternion baseOrientation() const {
    return tf::createQuaternionMsgFromRollPitchYaw(boost::math::constants::pi<double>() / 2, 0, 0);
  }
  
  /**
   * Return the flipped orientation
   * @return The flipped orientation
   */
  geometry_msgs::Quaternion flippedOrientation() const {
    return tf::createQuaternionMsgFromRollPitchYaw(3 * boost::math::constants::pi<double>() / 2, 0, 0);
  }

    /**
   * Return the base orientation
   * @return The base orientation
   */
  geometry_msgs::Quaternion scoopingOrientation() const {
    return tf::createQuaternionMsgFromRollPitchYaw(boost::math::constants::pi<double>() / 2, boost::math::constants::pi<double>() / 6, 0);
  }
  
  /**
   * Return the identity position
   * @return The identity position
   */
  geometry_msgs::Point identityPosition() const {
    geometry_msgs::Point identity;
    identity.x = identity.y = identity.z = 0;
    return identity;
  }
 
  /**
   * Shake the right arm to dislodge any litter in the shoop.
   */
  void shakeRightArmJ(){
    ROS_INFO("Shaking the right arm");
    const unsigned int NUM_SHAKES = 2;
    const static double SHAKE_DISTANCE = 0.04; // Distance to move the shoulder joint.
    for(unsigned int i = 0; i < NUM_SHAKES; ++i){
      ROS_INFO("Starting a shake");

      // Start from the base orientation
      std::vector<double> positions = overLitterboxPositions();
      positions[0] += (i % 2 == 0) ? SHAKE_DISTANCE : -SHAKE_DISTANCE;
      moveArmToJointPositions(positions);
      ROS_INFO("Ending a shake");
    }

    // Return to the starting pose
    moveArmOverLitterboxJ();
 
    // Ensure all movement complete.
    ROS_INFO("Arm shake completed");
  }
  
  /**
   * Flip the scoop over to dump the contents of the shovel. Return the shovel
   * to the starting orientation when complete.
   */
  void dumpPoop(){
    ROS_INFO("Dumping poop");
    std::vector<double> flipped = overTrashJoints();
    flipped[6] -= boost::math::constants::pi<double>();
    moveArmToJointPositions(flipped);
    ROS_INFO("Poop dumped. Returning to base orientation.");
    moveArmToJointPositions(overTrashJoints());
    ROS_INFO("Grabber returned to normal orientation");
  }

  /**
   * Send a goal to an action client, wait for a result, and 
   * report success or failure.
   * @param client Action client to send the goal to
   * @param goal Goal to send
   * @return Whether the goal was executed successfully.
   */
  template<class T, class U>
  bool sendGoal(const T& client, const U& goal){
    bool success = false;
    if (nh.ok()){
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
  return success;
}
};

// Main function.
int main(int argc, char** argv){
  ros::init(argc, argv, "litter_cleaner");

  LitterboxCleaner cleaner;
  cleaner.init();

  cleaner.cleanLitterbox();
  ros::shutdown();

  return 0;
}
