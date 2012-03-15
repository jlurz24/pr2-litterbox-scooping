#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <litterbox/ScooperAttached.h>

using namespace std;
using namespace litterbox;

static const double CLOSED_GRIPPER_POSITION = 0.01;
static const double HELD_SCOOP_EFFORT = -0.5;

class DetectScooperAttached {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    string jointName;
    auto_ptr<message_filters::Subscriber<sensor_msgs::JointState> > jointSub;
    
    // Publisher for the resulting attached
    ros::Publisher pub; 
    
 public:
    DetectScooperAttached() : privateHandle("~"){
      
      ROS_INFO("Initializing the scooper attached detector");

      ROS_INFO("Waiting for joint state subscription");
      
      privateHandle.getParam("joint_name", jointName);
      ROS_INFO("Detecting scooper in joint name %s", jointName.c_str());

      jointSub.reset(new message_filters::Subscriber<sensor_msgs::JointState>(nh, "joint_states", 1));
      
      ROS_INFO("Waiting for joint state subscription");
      
      
      jointSub->registerCallback(boost::bind(&DetectScooperAttached::jointDataCallback, this, _1)); 
     
      // Publish whether the scooper is attached location
      ROS_INFO("Setting up publisher");
      pub = nh.advertise<ScooperAttached>("litterbox/scooper_attached", 1000);
      ROS_INFO("Initialization complete");
    }
    
    ~DetectScooperAttached(){
    }

    void jointDataCallback(const sensor_msgs::JointStateConstPtr& jointsMsg){
      if(pub.getNumSubscribers() == 0){
        return;
      }
      
      ScooperAttachedPtr msg = ScooperAttachedPtr(new ScooperAttached());
      
      // Find the matching joint.
      msg->attached = false;
      for(unsigned int i = 0; i < jointsMsg->name.size(); ++i){
        if(jointsMsg->name[i] == jointName){
          // Note: Effort when closing the scoop is negative.
          if(jointsMsg->position[i] > CLOSED_GRIPPER_POSITION && jointsMsg->effort[i] < HELD_SCOOP_EFFORT){
            ROS_INFO("Scoop attached");
            msg->attached = true;
          }
          break;
        }

      }
      pub.publish(msg);
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "DetectScooperAttached");
  DetectScooperAttached sad;
  ros::spin();
  return 0;
}

