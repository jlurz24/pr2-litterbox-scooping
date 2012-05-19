#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <cmath>
#include <octomap/OcTreeStamped.h>
#include <octomap_ros/OctomapROS.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

typedef octomap::OcTreeStamped OcTreeType;
typedef message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> > LRCameraSync;

typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraSubscriber;

class OctomapUpdater {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    bool cameraModelInitialized;
    std::string fixedFrame;
    tf::TransformListener tf;
    auto_ptr<OcTreeType> octomap;
    auto_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > depthPointsSub;

    auto_ptr<CameraSubscriber> lCameraSubscriber;
    auto_ptr<CameraSubscriber> rCameraSubscriber;
    auto_ptr<image_geometry::StereoCameraModel> cameraModel;
    auto_ptr<LRCameraSync> cameraSync;
    ros::Publisher occupiedPub;

 public:
    OctomapUpdater() : privateHandle("~"), cameraModelInitialized(false), lCameraSubscriber(new CameraSubscriber), rCameraSubscriber(new CameraSubscriber), cameraSync(new LRCameraSync(3)){
      ROS_INFO("Octomap Updater");
      double resolution;
      privateHandle.param<double>("resolution", resolution, 0.1);
      privateHandle.param<std::string>("fixed_frame", fixedFrame, "base_link");
      octomap.reset(new OcTreeType(resolution));
     
      cameraModel.reset(new image_geometry::StereoCameraModel);

      // List for the depth messages
      depthPointsSub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/wide_stereo/points2", 1));

      depthPointsSub->registerCallback(boost::bind(&OctomapUpdater::depthCloudCallback, this, _1));
      lCameraSubscriber->subscribe(nh, "/wide_stereo/left_camera_info", 3);
      rCameraSubscriber->subscribe(nh, "/wide_stereo/right_camera_info", 3);
      cameraSync->connectInput(*lCameraSubscriber, *rCameraSubscriber);

      cameraSync->registerCallback(boost::bind(&OctomapUpdater::cameraInfoCallback, this, _1, _2));

      occupiedPub = nh.advertise<visualization_msgs::Marker>("occupied_cells", 1, false);
      ROS_INFO("Initialization complete of OctomapUpdater");
    }
    
    ~OctomapUpdater(){
    }

    void depthCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud){

      ROS_INFO("Depth cloud callback received");

      sensor_msgs::PointCloudPtr cloud1(new sensor_msgs::PointCloud);
      sensor_msgs::convertPointCloud2ToPointCloud(*cloud, *cloud1);

      // Convert to world frame
      ros::Duration timeout(10.0);
      bool haveTransform = tf.waitForTransform(cloud1->header.frame_id, fixedFrame, cloud1->header.stamp, timeout);

      if(!haveTransform){
        ROS_INFO("Failed to retrieve transform prior to timeout. Skipping message");
        return;
      }

      sensor_msgs::PointCloudPtr worldCloud(new sensor_msgs::PointCloud);
      tf.transformPointCloud(fixedFrame, cloud1->header.stamp, *cloud1, cloud1->header.frame_id, *worldCloud);
      
      // copy data to octomap pointcloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud (new pcl::PointCloud<pcl::PointXYZ>);

      sensor_msgs::PointCloud2Ptr worldCloud2(new sensor_msgs::PointCloud2);
      sensor_msgs::convertPointCloudToPointCloud2(*worldCloud, *worldCloud2);
      pcl::fromROSMsg(*worldCloud2, *pclCloud);

      octomap::Pointcloud octoCloud;
      octomap::pointcloudPCLToOctomap(*pclCloud, octoCloud);

      // Get the sensor origin.
      geometry_msgs::PointStamped sensorOrigin = getSensorOrigin(cloud->header);      octomap::point3d octoSensorOrigin(sensorOrigin.point.x, sensorOrigin.point.y, sensorOrigin.point.z);

      octomap->insertScan(octoCloud, octoSensorOrigin);
      
      ROS_INFO("Octomap update complete");

      // Publish updates.
      if(occupiedPub.getNumSubscribers() > 0){
        ROS_INFO("Publishing an update");
        std::vector<geometry_msgs::Point> occPoints = getOccupiedPoints();
        publishOccupiedPoints(occPoints, cloud->header);
      }
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
      occupiedCellsVis.type = visualization_msgs::Marker::CUBE_LIST;
      occupiedCellsVis.scale.x = octomap->getResolution();
      occupiedCellsVis.scale.y = octomap->getResolution();
      occupiedCellsVis.scale.z = octomap->getResolution();
      occupiedCellsVis.color.r = 0;
      occupiedCellsVis.color.g = 0;
      occupiedCellsVis.color.b = 1.0;
      occupiedCellsVis.color.a = 0.5;
      occupiedCellsVis.points = points;
      occupiedPub.publish(occupiedCellsVis); 
    }


    std::vector<geometry_msgs::Point> getOccupiedPoints() const {
      std::vector<geometry_msgs::Point> points;
      points.reserve(octomap->size() / 2.0);
      for (OcTreeType::iterator it = octomap->begin(), end = octomap->end(); it != end; ++it){
        if (octomap->isNodeOccupied(*it)){
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
      ROS_DEBUG("Got camera info: %d x %d, %d x %d\n", leftCameraInfo->height, leftCameraInfo->width, rightCameraInfo->height, rightCameraInfo->width);
      cameraModel->fromCameraInfo(*leftCameraInfo, *rightCameraInfo);
      cameraModelInitialized = true;
      cameraSync.release();
      lCameraSubscriber.release();
      rCameraSubscriber.release();
    }

    geometry_msgs::PointStamped getSensorOrigin(const std_msgs::Header& sensorHeader) const {
    geometry_msgs::PointStamped stampedIn;
    stampedIn.header = sensorHeader;
    geometry_msgs::PointStamped stampedOut;
    tf.transformPoint(fixedFrame, stampedIn, stampedOut);
    return stampedOut;
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "octomap_updater");
  OctomapUpdater ocu;
  ros::spin();
  return 0;
}

