#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelStates.h>

using namespace std;

class PixelOccupied {
  private:
    tf::TransformListener tf;
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    geometry_msgs::Point32 referencePoint;
    string modelName;
    string referenceFrame;
    auto_ptr<message_filters::Subscriber<arm_navigation_msgs::CollisionMap> > collisionSub;
    auto_ptr<message_filters::Subscriber<gazebo_msgs::ModelStates> > statesSub;
    double minHeight;
    bool unoccupiedMode;
    bool modelChangeReported;
    bool occupancyChangeReported;

    void modelCB(const gazebo_msgs::ModelStates::ConstPtr& models){
      // TODO: Could unsubscribe instead
      if(modelChangeReported){
        return;
      }
      bool found = false;
      for(unsigned int i = 0; i < models->name.size(); ++i){
        if(models->name[i] == modelName){
          found = true;
          break;
        }
      }

      if(unoccupiedMode && !found){
        modelChangeReported = true;
        ROS_INFO("Model was removed @ time %f", ros::Time::now().toSec());
      }
      else if(!unoccupiedMode && found){
        modelChangeReported = true;
        ROS_INFO("Model was added @ time %f", ros::Time::now().toSec());
      }     
    }
    void pixelCB(const arm_navigation_msgs::CollisionMap::ConstPtr& collisionMap){
      // TODO: Could unsubscribe.
      if(occupancyChangeReported){
        return;
      }

      ROS_INFO("Received a collision map with %lu boxes in %s frame", collisionMap->boxes.size(), collisionMap->header.frame_id.c_str());
      
      // Avoid transforming as the time to wait for the transform
      // could skew results.
      if(collisionMap->header.frame_id != referenceFrame){
        ROS_ERROR("Collision map and reference point frames don't match");
        return;
      }
      
      bool found = false;
      for(unsigned int i = 0; i < collisionMap->boxes.size(); ++i){
        const arm_navigation_msgs::OrientedBoundingBox& box = collisionMap->boxes[i];
        if(box.center.x + box.extents.x / 2.0 > referencePoint.x &&
           box.center.x - box.extents.x / 2.0 < referencePoint.x &&
           box.center.y + box.extents.y / 2.0 > referencePoint.y &&
           box.center.y - box.extents.y / 2.0 < referencePoint.y &&
           box.center.z > minHeight){
           // Intentionally ignoring z to avoid timing issues related
           // to the orientation of the laser scanner.
             found = true;
             break;
        }
      }
     
      if(found && !unoccupiedMode){
        occupancyChangeReported = true;
        ROS_INFO("Reference point was occupied @ time %f", ros::Time::now().toSec());
      }
      else if(!found && unoccupiedMode){
        occupancyChangeReported = true;
        ROS_INFO("Reference point was unoccupied @ time %f", ros::Time::now().toSec());
      }
    }
   
  public:
   PixelOccupied(): privateHandle("~") , unoccupiedMode(false), modelChangeReported(false), occupancyChangeReported(false){
    double temp;
    privateHandle.param<double>("reference_x", temp, 0.0);
    referencePoint.x = float(temp);
    privateHandle.param<double>("reference_y", temp, 0.0);
    referencePoint.y = float(temp);
    privateHandle.param<bool>("unoccupied_mode", unoccupiedMode, false);
    privateHandle.param<string>("reference_frame", referenceFrame, "/odom_combined");
    privateHandle.param<string>("model_name", modelName, "obstacle");
    privateHandle.param<double>("min_height", minHeight, 0.5);
    collisionSub.reset(new message_filters::Subscriber<arm_navigation_msgs::CollisionMap>(nh, "collision_map_in", 1));
    collisionSub->registerCallback(boost::bind(&PixelOccupied::pixelCB, this, _1)); 
    statesSub.reset(new message_filters::Subscriber<gazebo_msgs::ModelStates>(nh, "gazebo/model_states", 1));
    statesSub->registerCallback(boost::bind(&PixelOccupied::modelCB, this, _1));
  }
};

int main(int argc, char **argv){
  ROS_INFO("Initializing pixel occupied");
  ros::init(argc, argv, "pixel_occupied");
  PixelOccupied po;
  ros::spin();
  return 0;
}

