#include <ros/ros.h>
#include <litterbox/DetermineLBDimensions.h>
#include <sensor_msgs/Image.h>
#include <litterbox/find_rectangles.h>
#include <cv_bridge/cv_bridge.h>

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
   sensor_msgs::ImageConstPtr image = ros::topic::waitForMessage<sensor_msgs::Image>("/narrow_stereo/left/image_mono", nh);
  
   ROS_INFO("Received an image with width %u height %u", image->width, image->height);
   
   cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(image);
  
   ROS_INFO("Finding rectangles");
   std::vector<RectangleInfo> rects = findRectangles(cvImg->image);
   
   if(rects.size()){
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
   return true;
  }
 
  private:
     ros::NodeHandle nh;
     ros::NodeHandle privateNh;
     ros::ServiceServer service;
     bool debug;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "determine_lb_dimensions_server");
  
  DetermineLBDimensionsServer determineLBDimensionsServer;
 
  ros::spin();

  return 0;
}
