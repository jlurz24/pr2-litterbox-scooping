#include <ros/ros.h>
#include <litterbox/DetermineLBDimensions.h>
#include <sensor_msgs/Image.h>
#include <litterbox/find_rectangles.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

using namespace cv;
using namespace litterbox;

bool compareMaxAngle(const RectangleInfo& l, const RectangleInfo& r){
  return r.maxAngle > l.maxAngle;
}

bool isTooLargeRectangle(const RectangleInfo& l) {
  // Likely the bogus outer perimeter rectangle.
  return l.area > 300000;
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

   // Wait for a 3d points message
   sensor_msgs::PointCloud2ConstPtr imagePoints;
   ROS_INFO("Waiting for 3d points message");
   imagePoints = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/narrow_stereo/left/points");

   // Begin processing the image.
   cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(image);

   ROS_INFO("Finding rectangles");
   std::vector<RectangleInfo> rects = findRectangles(cvImg->image);
   
   // Remove bogus outer rectangle.
   rects.erase(std::remove_if(rects.begin(), rects.end(), isTooLargeRectangle));

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
   pcl::PointCloud<pcl::PointXYZ> depthCloud;
   pcl::fromROSMsg(*imagePoints, depthCloud);

   // TODO: Apply downsampling here. Probably a voxel grid.
   std::vector<pcl::PointXYZ> contour3d;
   contour3d.resize(4);
   for(unsigned int i = 0; i < contour3d.size(); ++i){
     contour3d[i] = depthCloud.points.at(contour[i].y * image->width + contour[i].x);
   }

   double d1 = calc3DDistance(contour3d[0], contour3d[1]);
   double d2 = calc3DDistance(contour3d[1], contour3d[2]);

   res.width = std::min(d1, d2);
   res.depth = std::max(d1, d2);

   // Convert to map coordinates.
   // Now convert from image from camera frame to world
   ros::Time now = ros::Time::now();
   tf.waitForTransform("/base_footprint", image->header.frame_id, now, ros::Duration(10.0));
   
   for(unsigned int i = 0; i < 4; ++i){   
     geometry_msgs::PointStamped resultPoint;
     resultPoint.header.frame_id = "/base_footprint";
     geometry_msgs::PointStamped imagePoint;
     imagePoint.header.frame_id = image->header.frame_id;
     imagePoint.header.stamp = now;
     imagePoint.point.x = contour3d[i].x;
     imagePoint.point.y = contour3d[i].y;
     imagePoint.point.z = contour3d[i].z;
     tf.transformPoint("/base_footprint", imagePoint, resultPoint);
     res.contour.push_back(resultPoint);
   }

   // Use average height of the corners.
   res.height = (res.contour[0].point.z + res.contour[1].point.z + res.contour[2].point.z + res.contour[3].point.z) / 4.0;
   ROS_INFO("Litterbox width %f depth %f height %f", res.width, res.depth, res.height);
   return true;
  }
 
  private:
     ros::NodeHandle nh;
     ros::NodeHandle privateNh;
     ros::ServiceServer service;
     tf::TransformListener tf;
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
