#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <arm_navigation_msgs/CollisionMap.h>

using namespace std;

class PixelCounter {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    auto_ptr<message_filters::Subscriber<arm_navigation_msgs::CollisionMap> > collisionSub;
    double totalDelta;
    double actualVolume;
    int frames;

    void pixelCB(const arm_navigation_msgs::CollisionMap::ConstPtr& collisionMap){
      ROS_INFO("Received a collision map msg with %lu boxes", collisionMap->boxes.size());

      double totalVolume = 0;
      float lowestX = 0;
      float highestX = 0;
      float lowestY = 0;
      float highestY = 0;
      frames++;
      for(unsigned int i = 0; i < collisionMap->boxes.size(); ++i){
        geometry_msgs::Point32 extents = collisionMap->boxes[i].extents;
        double volume = extents.x * extents.y * extents.z;
        highestX = std::max(highestX, collisionMap->boxes[i].center.x);
        lowestX = std::min(lowestX, collisionMap->boxes[i].center.x);
        highestY = std::max(highestY, collisionMap->boxes[i].center.y);
        lowestY = std::min(lowestY, collisionMap->boxes[i].center.y);
        totalVolume += volume;
      }
      ROS_INFO("Highest X: %f Lowest X: %f, Highest Y: %f, Lowest Y: %f", highestX, lowestX, highestY, lowestY);
      totalDelta += fabs(totalVolume - actualVolume);
      ROS_INFO("Total volume: %f actual volume: %f", totalVolume, actualVolume);
      ROS_INFO("Delta volume per frame %f frames %i", totalDelta / frames, frames);
  }
   
  public:
   PixelCounter(): privateHandle("~"), totalDelta(0.0), actualVolume(0.0), frames(0){
    collisionSub.reset(new message_filters::Subscriber<arm_navigation_msgs::CollisionMap>(nh, "collision_map_in", 1));
    collisionSub->registerCallback(boost::bind(&PixelCounter::pixelCB, this, _1));
    privateHandle.param<double>("actual_volume", actualVolume, 0.0);
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "pixel_counter");
  PixelCounter pc;
  ros::spin();
  return 0;
}

