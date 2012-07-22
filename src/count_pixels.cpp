#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <arm_navigation_msgs/CollisionMap.h>

using namespace std;

class PixelCounter {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    auto_ptr<message_filters::Subscriber<arm_navigation_msgs::CollisionMap> > collisionSub;
    double totalVolume;
    bool timeSet;
    ros::Time startTime;

    void pixelCB(const arm_navigation_msgs::CollisionMap::ConstPtr& collisionMap){
      ROS_INFO("Received a collision map msg with %lu boxes", collisionMap->boxes.size());

      if(!timeSet){
        startTime = ros::Time::now();
      }
      for(unsigned int i = 0; i < collisionMap->boxes.size(); ++i){
        geometry_msgs::Point32 extents = collisionMap->boxes[i].extents;
        double volume = extents.x * extents.y * extents.z;
        ROS_INFO("Adding volume %f", volume);
        totalVolume += volume;
      }
      ROS_INFO("Total volume: %f", totalVolume);

      ros::Duration diff = ros::Time::now() - startTime;
      ROS_INFO("Total time %f", diff.toSec());
      ROS_INFO("V/S %f", totalVolume / double(diff.toSec()));
  }
   
  public:
   PixelCounter(): privateHandle("~"), totalVolume(0.0), timeSet(false){
    collisionSub.reset(new message_filters::Subscriber<arm_navigation_msgs::CollisionMap>(nh, "collision_map_in", 1));
    collisionSub->registerCallback(boost::bind(&PixelCounter::pixelCB, this, _1));
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "pixel_counter");
  PixelCounter pc;
  ros::spin();
  return 0;
}

