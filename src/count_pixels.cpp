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

class PixelCounter {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    auto_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > cloudSub;
    size_t numPixels;

    void pixelCB(const sensor_msgs::PointCloud2ConstPtr& cloudMsg){
      PointCloudPtr cloud;
      pcl::fromROSMsg(*cloudMsg, *cloud);
      ROS_INFO("Received a cloud msg with %lu points", cloud->size());

      PointCloudPtr filteredCloud(new PointCloud);
      const double Z_FILTER_MIN = 0.0;
      const double Z_FILTER_MAX = 5.0;

      pcl::PassThrough<pcl::PointXYZ> filter;
      filter.setFilterFieldName("z");
      filter.setFilterLimits(Z_FILTER_MIN, Z_FILTER_MAX);
      filter.setInputCloud(cloud);
      filter.filter(*filteredCloud);
      ROS_INFO("Adding %lu points to the total", filteredCloud->size());
      numPixels += filteredCloud->size();
    }
   
  public:
   PixelCounter(): privateHandle("~"), numPixels(0){
    string sensorTopic;
    privateHandle.getParam("sensor_topic", sensorTopic);
    cloudSub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, sensorTopic, 1));
    cloudSub->registerCallback(boost::bind(&PixelCounter::pixelCB, this, _1));
  }
  
   ~PixelCounter() {
     ROS_INFO("Pixel counter detected %lu pixels", numPixels);
   }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "pixel_counter");
  PixelCounter pc;
  ros::spin();
  return 0;
}

