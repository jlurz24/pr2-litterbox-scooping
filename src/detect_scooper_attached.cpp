#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <litterbox/ScooperAttached.h>

using namespace std;
using namespace litterbox;

static const double CLOSED_GRIPPER_POSITION = 0.01;
static const double HELD_SCOOP_EFFORT = 0.01; // TODO: EFfort is incorrect in sim -0.5;

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
      
      privateHandle.getParam("joint_name", jointName);
     
      // Publish whether the scooper is attached location 
       ros::SubscriberStatusCallback cb = boost::bind(&DetectScooperAttached::connectCB, this);
      pub = nh.advertise<ScooperAttached>("litterbox/scooper_attached", 1, cb, cb);
      
      ROS_INFO("Initialization complete of the scooper attached detector");
    }

 private:
    void connectCB(){
      if(pub.getNumSubscribers() == 1){
        startListening();
      }
      else if(pub.getNumSubscribers() == 0) {
        stopListening();
      }
    }

    void startListening(){
      if(jointSub.get() == NULL){
        jointSub.reset(new message_filters::Subscriber<sensor_msgs::JointState>(nh, "joint_states", 1));

        jointSub->registerCallback(boost::bind(&DetectScooperAttached::jointDataCallback, this, _1));
      }
      else {
        jointSub->subscribe();
      }
    }
    
    void stopListening(){
      jointSub->unsubscribe();
    }

    void jointDataCallback(const sensor_msgs::JointStateConstPtr& jointsMsg){
      
      ScooperAttachedPtr msg = ScooperAttachedPtr(new ScooperAttached());
      
      // Find the matching joint.
      msg->attached = false;
      for(unsigned int i = 0; i < jointsMsg->name.size(); ++i){
        if(jointsMsg->name[i] == jointName){
          // Note: Effort when closing the scoop is negative.
          if(jointsMsg->position[i] > CLOSED_GRIPPER_POSITION && jointsMsg->effort[i] <= HELD_SCOOP_EFFORT){
            msg->attached = true;
          }
          break;
        }

      }
      if(!msg->attached){
        ROS_INFO("Scoop is not attached");
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

