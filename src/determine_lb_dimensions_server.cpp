#include <ros/ros.h>
#include <litterbox/DetermineLBDimensions.h>
#include <sensor_msgs/Image.h>
#include <litterbox/find_rectangles.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

using namespace cv;
using namespace litterbox;

bool compareMaxAngle(const RectangleInfo& l, const RectangleInfo& r){
  return r.maxAngle > l.maxAngle;
}

class DetermineLBDimensionsServer {
  public:
    DetermineLBDimensionsServer(): privateNh("~"){
      privateNh.getParam("debug", debug);
      service = nh.advertiseService("determine_lb_dimensions", &DetermineLBDimensionsServer::determineDimensions, this);
    }

    bool determineDimensions(litterbox::DetermineLBDimensions::Request& req,
                         litterbox::DetermineLBDimensions::Response& res){
   
   ROS_INFO("Waiting for image message");
   sensor_msgs::ImageConstPtr image = ros::topic::waitForMessage<sensor_msgs::Image>("/narrow_stereo/left/image_rect_color", nh);
  
   ROS_INFO("Received an image with width %u height %u", image->width, image->height);

   // Wait for a 3d points message
   bool haveValid3d = false;
   sensor_msgs::PointCloud2ConstPtr imagePoints;
   while(!haveValid3d){
     ROS_INFO("Waiting for 3d points message");
     imagePoints = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/narrow_stereo/points2");

     // Determine if the message is valid
     // TODO: Accept valid sparse messages
     if(imagePoints->width != 640 && imagePoints->height != 480){
       ROS_INFO("Ignoring an invalid stereo points message with width %d and height %d", imagePoints->width, imagePoints->height);
       continue;
     }
     haveValid3d = true;
   }

   // Begin processing the image.
   cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(image);

   ROS_INFO("Finding rectangles");
   std::vector<RectangleInfo> rects = findRectangles(cvImg->image);
   
   if(rects.size() == 0){
     // TODO: Set error message here
     ROS_INFO("Could not locate any rectangles that might be the litterbox");
     return false;
   }

   // Determine which rectangle to use.
   std::vector<cv::Point> contour;
   if(rects.size() > 1){
     contour = std::min_element(rects.begin(), rects.end(), compareMaxAngle)->contour;
   }
   else {
     contour = rects[0].contour;
   }

   // Find the points in the 3d point plane.
   pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::fromROSMsg(*imagePoints, *depthCloud);

   // TODO: Apply downsampling here. Probably a voxel grid.
   std::vector<pcl::PointXYZ> contour3d;
   contour3d.resize(4);
   for(unsigned int i = 0; i < contour3d.size(); ++i){
     ROS_INFO("Pixel position x %f y %f", contour[i].x, contour[i].y);
     contour3d[i] = depthCloud->at(contour[i].x, contour[i].y);
     ROS_INFO("X: %f Y:%f Z: %f", contour3d[i].x, contour3d[i].y, contour3d[i].z);
   }

   double d1 = calc3DDistance(contour3d[0], contour3d[1]);
   double d2 = calc3DDistance(contour3d[1], contour3d[2]);
   ROS_INFO("d1 %f d2 %f", d1, d2);

   res.width = std::min(d1, d2);
   res.depth = std::max(d1, d2);
   res.height = 0; // TODO: Implement

   ROS_INFO("Litterbox width %f depth %f height %f", res.width, res.depth, res.height);
   return true;
  }
 
  private:
     ros::NodeHandle nh;
     ros::NodeHandle privateNh;
     ros::ServiceServer service;
     bool debug;

     static double calc3DDistance(pcl::PointXYZ& point1, pcl::PointXYZ& point2){
       return sqrt((point1.getVector3fMap() - point2.getVector3fMap()).squaredNorm());
     }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "determine_lb_dimensions_server");
  
  DetermineLBDimensionsServer determineLBDimensionsServer;
 
  ros::spin();

  return 0;
}
