#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;

class PixelOccupied {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    pcl::PointXYZ referencePoint;
    auto_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > cloudSub;
    size_t numPixels;
    bool unoccupiedMode;

    void pixelCB(const sensor_msgs::PointCloud2ConstPtr& cloudMsg){
      PointCloudPtr cloud;
      pcl::fromROSMsg(*cloudMsg, *cloud);
      ROS_INFO("Received a cloud msg with %lu points in %s frame", cloud->size(), cloud->header.frame_id.c_str());

      bool found = false;
      for(unsigned int i = 0; i < cloud->points.size(); ++i){
        if(cloud->points[i].x == referencePoint.x && cloud->points[i].y == referencePoint.y && cloud->points[i].z == referencePoint.z){
          found = true;
          break;
        }
      }
     
      if(found && !unoccupiedMode){
        ROS_INFO("Reference point was found");
      }
      else if(!found && unoccupiedMode){
        ROS_INFO("Reference point was not found");
      }
    }
   
  public:
   PixelOccupied(): privateHandle("~"), numPixels(0), unoccupiedMode(false){
    string sensorTopic;
    privateHandle.getParam("sensor_topic", sensorTopic);
    double temp;
    privateHandle.param<double>("reference_x", temp, 0.0);
    referencePoint.x = float(temp);
    privateHandle.param<double>("reference_y", temp, 0.0);
    referencePoint.y = float(temp);
    privateHandle.param<double>("reference_z", temp, 0.0);
    referencePoint.z = float(temp);
    privateHandle.param<bool>("unoccupied_mode", unoccupiedMode, false);

    cloudSub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, sensorTopic, 1));
    cloudSub->registerCallback(boost::bind(&PixelOccupied::pixelCB, this, _1));
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "pixel_occupied");
  PixelOccupied po;
  ros::spin();
  return 0;
}

