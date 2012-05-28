#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <cmath>
#include <octomap/OcTreeStamped.h>
#include <octomap_ros/OctomapROS.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <arm_navigation_msgs/CollisionMap.h>

using namespace std;

typedef message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> > LRCameraSync;

typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraSubscriber;

class OctomapUpdater {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    bool cameraModelInitialized;
    std::string fixedFrame;
    std::string sensorName;
    double maxRange;

    tf::TransformListener tf;
    auto_ptr<octomap::OcTreeROS> octomap;
    ros::Subscriber depthPointsSub;

    auto_ptr<CameraSubscriber> lCameraSubscriber;
    auto_ptr<CameraSubscriber> rCameraSubscriber;
    auto_ptr<image_geometry::StereoCameraModel> cameraModel;
    auto_ptr<LRCameraSync> cameraSync;
    ros::Publisher occupiedPub;
    ros::Publisher cmapPub;
    ros::Publisher pointCloudPub;

 public:
    OctomapUpdater() : privateHandle("~"), cameraModelInitialized(false), lCameraSubscriber(new CameraSubscriber), rCameraSubscriber(new CameraSubscriber), cameraSync(new LRCameraSync(3)){

      double resolution;
      privateHandle.param<double>("resolution", resolution, 0.1);
      privateHandle.param<double>("max_range", maxRange, 5.0);

      privateHandle.param<std::string>("fixed_frame", fixedFrame, "/base_link");
      privateHandle.param<std::string>("sensor_name", sensorName, "/narrow_stereo/left/points");
      octomap.reset(new octomap::OcTreeROS(resolution));

      cameraModel.reset(new image_geometry::StereoCameraModel);

      // List for the depth messages
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(sensorName, nh);
      depthPointsSub = nh.subscribe(sensorName, 1, &OctomapUpdater::depthCloudCallback, this);
      lCameraSubscriber->subscribe(nh, "/narrow_stereo/left/camera_info", 3);
      rCameraSubscriber->subscribe(nh, "/narrow_stereo/right/camera_info", 3);
      cameraSync->connectInput(*lCameraSubscriber, *rCameraSubscriber);

      cameraSync->registerCallback(boost::bind(&OctomapUpdater::cameraInfoCallback, this, _1, _2));

      occupiedPub = nh.advertise<visualization_msgs::Marker>("occupied_cells", 1, false);
      cmapPub = nh.advertise<arm_navigation_msgs::CollisionMap>("collision_map_out", 1, true);
      pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_out", 1, true);

      ROS_INFO("Initialization complete of OctomapUpdater");
    }
    
    ~OctomapUpdater(){
    }

    void depthCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud){

      ROS_INFO("Depth cloud callback received");
     
      // Gazebo fails to properly set the frame id on cloud.
      sensor_msgs::PointCloud2 cloud2(*cloud);
      cloud2.header.frame_id = "narrow_stereo_optical_frame"; 
      ros::WallTime beginTransformTime = ros::WallTime::now();
      sensor_msgs::PointCloud2 worldCloud;
      worldCloud.header = cloud2.header;
      worldCloud.header.frame_id = fixedFrame;
      bool haveTransform = tf.waitForTransform(cloud2.header.frame_id, fixedFrame, cloud2.header.stamp, ros::Duration(3));

      if(!haveTransform){
        ROS_INFO("Failed to retrieve transform");
        return;
      }
      pcl_ros::transformPointCloud(fixedFrame, cloud2, worldCloud, tf);

      ros::WallTime endTransformTime = ros::WallTime::now();
      ROS_INFO("Transform took %f seconds", (endTransformTime - beginTransformTime).toSec());

      // Get the sensor origin.
      octomap->insertScan(worldCloud, getSensorOrigin(cloud2.header), maxRange);
      ros::WallTime endInsertTime = ros::WallTime::now();
      ROS_INFO("Insert scan took %f seconds", (endInsertTime - endTransformTime).toSec());      

      // Publish updates.
      ROS_INFO("Preparing to publish");
      std::vector<geometry_msgs::Point> occPoints;
      if(occupiedPub.getNumSubscribers() > 0 || cmapPub.getNumSubscribers() > 0 || pointCloudPub.getNumSubscribers() > 0){
        occPoints = getOccupiedPoints();
      }

      if(occupiedPub.getNumSubscribers() > 0){
        publishOccupiedPoints(occPoints, worldCloud.header);
        ROS_INFO("Publishing an update took %f seconds", (ros::WallTime::now() - endInsertTime).toSec());
      }
      if(cmapPub.getNumSubscribers() > 0){
        publishCollisionMap(occPoints, worldCloud.header);
        ROS_INFO("Publishing an update took %f seconds", (ros::WallTime::now() - endInsertTime).toSec());
      }
      if(pointCloudPub.getNumSubscribers() > 0){
        publishPointCloud(occPoints, worldCloud.header);
        ROS_INFO("Publishing an update too %f seconds", (ros::WallTime::now() - endInsertTime).toSec());
      }
      ROS_INFO("Callback complete");
    }

    void publishPointCloud(const std::vector<geometry_msgs::Point>& points, const std_msgs::Header &header){
     if(points.size() <= 1){
       ROS_INFO("No points to publish");
       return;
     }

     pcl::PointCloud<pcl::PointXYZ> cloud;
     cloud.points.reserve(points.size());

     for(std::vector<geometry_msgs::Point>::const_iterator it = points.begin(); it != points.end(); ++it){
       pcl::PointXYZ point;
       point.x = it->x;
       point.y = it->y;
       point.z = it->z;
       cloud.points.push_back(point);
    }

    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(cloud, cloud2);
    cloud2.header = header;
    pointCloudPub.publish(cloud2);
  }
    void publishOccupiedPoints(const std::vector<geometry_msgs::Point>& points, const std_msgs::Header& header){
      if(points.size() <= 1){
        ROS_INFO("No points to publish");
        return;
      }

      visualization_msgs::Marker occupiedCellsVis;
      occupiedCellsVis.header = header;
      occupiedCellsVis.ns = "map";
      occupiedCellsVis.id = 0;
      occupiedCellsVis.action = visualization_msgs::Marker::ADD;
      occupiedCellsVis.type = visualization_msgs::Marker::CUBE_LIST;
      occupiedCellsVis.scale.x = octomap->octree.getResolution();
      occupiedCellsVis.scale.y = octomap->octree.getResolution();
      occupiedCellsVis.scale.z = octomap->octree.getResolution();
      occupiedCellsVis.color.r = 1.0f;
      occupiedCellsVis.color.g = 0.0f;
      occupiedCellsVis.color.b = 0.0f;
      occupiedCellsVis.color.a = 0.5f;
      occupiedCellsVis.points = points;
      occupiedCellsVis.lifetime = ros::Duration();
      occupiedPub.publish(occupiedCellsVis); 
    }

    void publishCollisionMap(const std::vector<geometry_msgs::Point>& points, const std_msgs::Header& header) {
      if(points.size() <= 1){
        ROS_INFO("No points to publish");
        return;
      }

     arm_navigation_msgs::CollisionMap cmap;
     cmap.header = header;

     arm_navigation_msgs::OrientedBoundingBox box;
     box.extents.x = box.extents.y = box.extents.z = octomap->octree.getResolution();
     box.axis.x = box.axis.y = 0.0; box.axis.z = 1.0;
     box.angle = 0.0;
     cmap.boxes.reserve(points.size());

     for (std::vector<geometry_msgs::Point>::const_iterator it = points.begin(); it != points.end(); ++it) {
       box.center.x = it->x;
       box.center.y = it->y;
       box.center.z = it->z;
       cmap.boxes.push_back(box);
     }
     cmapPub.publish(cmap);
    }
 
    std::vector<geometry_msgs::Point> getOccupiedPoints() const {
      std::vector<geometry_msgs::Point> points;
      points.reserve(octomap->octree.size() / 2.0);
      for (octomap::OcTreeROS::OcTreeType::iterator it = octomap->octree.begin(), end = octomap->octree.end(); it != end; ++it){
        if (octomap->octree.isNodeOccupied(*it)){
          geometry_msgs::Point p;
          p.x = it.getX();
          p.y = it.getY();
          p.z = it.getZ();
          points.push_back(p);
        }
      }
      return points;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& leftCameraInfo, const sensor_msgs::CameraInfo::ConstPtr& rightCameraInfo){
      ROS_INFO("Got camera info: %d x %d, %d x %d\n", leftCameraInfo->height, leftCameraInfo->width, rightCameraInfo->height, rightCameraInfo->width);
      cameraModel->fromCameraInfo(*leftCameraInfo, *rightCameraInfo);
      cameraModelInitialized = true;
      lCameraSubscriber->unsubscribe();
      rCameraSubscriber->unsubscribe();
    }

    pcl::PointXYZ getSensorOrigin(const std_msgs::Header& sensorHeader) const {
      geometry_msgs::PointStamped stampedIn;
      stampedIn.header = sensorHeader;
      geometry_msgs::PointStamped stampedOut;
      tf.transformPoint(fixedFrame, stampedIn, stampedOut);
    
      pcl::PointXYZ result;
      result.x = stampedOut.point.x;
      result.y = stampedOut.point.y;
      result.z = stampedOut.point.z;
      return result;
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "octomap_updater");
  OctomapUpdater ocu;
  ros::spin();
  return 0;
}

