#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <cmath>
#include <octomap/OcTreeStamped.h>
#include <octomap_ros/OctomapROS.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <litterbox/Reset.h>
using namespace std;

typedef octomap::OctomapROS<octomap::OcTreeStamped> OctomapType;

class OctomapUpdater {
  private:
    ros::NodeHandle mNH;
    ros::NodeHandle mPNH;

    std::string mFixedFrame;
    double mMaxRange;
    double mDegradeTolerance;

    tf::TransformListener mTf;
    auto_ptr<OctomapType> mOctoMap;
    ros::Subscriber mLaserPointsSub;
    ros::Subscriber mDepthPointsSub;

    ros::Publisher mOccupiedPub;
    ros::Publisher mCMapPub;
    ros::Publisher mPointCloudPub;
    ros::Timer mDisplayTimer;
    ros::ServiceServer resetService;
 public:
    OctomapUpdater() : mPNH("~"){

      mPNH.param("max_sensor_range", mMaxRange, 3.0);
      mPNH.param<std::string>("fixed_id", mFixedFrame, "/odom_combined");
      mPNH.param("degrade_tolerance", mDegradeTolerance, 1.0);

      initOctomap();

      bool useStereo = false;
      bool useLaser = true;
      mPNH.param("use_stereo", useStereo, useStereo);
      mPNH.param("use_laser", useLaser, useLaser);
      
      // List for the depth messages
      if(useStereo){
        std::string imageSensorName = "stereo_cloud_in";
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(imageSensorName, mNH);
        mDepthPointsSub = mNH.subscribe(imageSensorName, 1, &OctomapUpdater::depthCloudCallback, this);
      }

      if(useLaser){
        std::string laserSensorName = "laser_cloud_in";
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(laserSensorName, mNH);
        mLaserPointsSub = mNH.subscribe(laserSensorName, 1, &OctomapUpdater::depthCloudCallback, this);
      }

      // Publishers for visualization.
      mOccupiedPub = mNH.advertise<visualization_msgs::Marker>("occupied_cells", 1, false);
      mCMapPub = mNH.advertise<arm_navigation_msgs::CollisionMap>("collision_map_out", 1, true);
      mPointCloudPub = mNH.advertise<sensor_msgs::PointCloud2>("point_cloud_out", 1, true);

      // Register the reset service
      resetService = mPNH.advertiseService("reset", &OctomapUpdater::resetCallback, this);
      ROS_INFO("Initialization complete of OctomapUpdater");
    }
 
    bool resetCallback(litterbox::Reset::Request& request, litterbox::Reset::Response& response){

      ROS_INFO("Octomap reset");
      initOctomap();
      publishUpdates();
      return true;
    }

    void initOctomap(){
      // Defaults are from octoMap.
      double occupancyThresh = 0.5;
      double probHit = 0.8;
      double probMiss = 0.31;
      double threshMin = 0.12;
      double threshMax = 0.95;
      double resolution = 0.1;

      mPNH.param("sensor_model_occ_thresh", occupancyThresh, occupancyThresh);
      mPNH.param("sensor_model_hit", probHit, probHit);
      mPNH.param("sensor_model_miss", probMiss, probMiss);
      mPNH.param("sensor_model_thresh_min", threshMin, threshMin);
      mPNH.param("sensor_model_thresh_max", threshMax, threshMax);
      mPNH.param("resolution", resolution, resolution);

      mOctoMap.reset(new OctomapType(resolution));
      mOctoMap->octree.setOccupancyThres(occupancyThresh);
      mOctoMap->octree.setProbHit(probHit);
      mOctoMap->octree.setProbMiss(probMiss);
      mOctoMap->octree.setClampingThresMin(threshMin);
      mOctoMap->octree.setClampingThresMax(threshMax);
    }

    /*
     * Callback called from the sensor listener.
     */
    void depthCloudCallback(const sensor_msgs::PointCloud2ConstPtr& aCloud){
     
      // Gazebo fails to properly set the frame id on cloud.
      sensor_msgs::PointCloud2 cloud2(*aCloud);
      if(cloud2.header.frame_id == ""){
        cloud2.header.frame_id = "narrow_stereo_optical_frame";
      }

      // Transform the cloud to global coordinates.
      ros::WallTime beginTransformTime = ros::WallTime::now();
      sensor_msgs::PointCloud2 worldCloud;
      worldCloud.header = cloud2.header;
      worldCloud.header.frame_id = mFixedFrame;
      bool haveTransform = mTf.waitForTransform(cloud2.header.frame_id, mFixedFrame, cloud2.header.stamp, ros::Duration(3));

      if(!haveTransform){
        ROS_INFO("Failed to retrieve transform");
        return;
      }
      pcl_ros::transformPointCloud(mFixedFrame, cloud2, worldCloud, mTf);
      ros::WallTime endTransformTime = ros::WallTime::now();

      // Insert the scan.
      mOctoMap->insertScan(worldCloud, getSensorOrigin(cloud2.header), mMaxRange);

      // Degrade outdated information.
      degradeOutdatedPoints();

      ros::WallTime endInsertTime = ros::WallTime::now();
      // ROS_INFO("Insert scan took %f seconds", (endInsertTime - endTransformTime).toSec());

      publishUpdates();
  }
  
  /**
   * Degrade points that have not been observed
   * recently.
   */
  void degradeOutdatedPoints(){
    mOctoMap->octree.degradeOutdatedNodes(mDegradeTolerance);
  }
 
  /*
   * Publish updates
   */
  void publishUpdates() const {
      std_msgs::Header header;
      header.frame_id = mFixedFrame;
      header.stamp = ros::Time::now();

      ros::WallTime publishStartTime = ros::WallTime::now();
      
      // Publish updates.
      std::vector<geometry_msgs::Point> occPoints;
      if(mOccupiedPub.getNumSubscribers() > 0 || mCMapPub.getNumSubscribers() > 0){
        occPoints = getOccupiedPoints();
      }

      if(mOccupiedPub.getNumSubscribers() > 0){
        publishOccupiedPoints(occPoints, header);
      }
      if(mCMapPub.getNumSubscribers() > 0){
        publishCollisionMap(occPoints, header);
      }
      if(mPointCloudPub.getNumSubscribers() > 0){
        publishPointCloud(header);
      }
      // ROS_INFO("Publishing an update took %f seconds", (ros::WallTime::now() - publishStartTime).toSec());
    }

    /*
     * Publish the output as a point cloud.
     */
  void publishPointCloud(const std_msgs::Header& aHeader) const {
      pcl::PointCloud<pcl::PointXYZI> cloud = getIntensityPoints();
      sensor_msgs::PointCloud2 cloud2;
      pcl::toROSMsg(cloud, cloud2);
      cloud2.header = aHeader;
      mPointCloudPub.publish(cloud2);
  }

  /*
   * Publish the occupied cells.
   */
  void publishOccupiedPoints(const std::vector<geometry_msgs::Point>& aPoints, const std_msgs::Header& aHeader) const {

      visualization_msgs::Marker occupiedCellsVis;
      occupiedCellsVis.header = aHeader;
      occupiedCellsVis.ns = "map";
      occupiedCellsVis.id = 0;
      occupiedCellsVis.action = visualization_msgs::Marker::ADD;
      occupiedCellsVis.type = visualization_msgs::Marker::CUBE_LIST;
      occupiedCellsVis.scale.x = mOctoMap->octree.getResolution();
      occupiedCellsVis.scale.y = mOctoMap->octree.getResolution();
      occupiedCellsVis.scale.z = mOctoMap->octree.getResolution();
      occupiedCellsVis.color.r = 1.0f;
      occupiedCellsVis.color.g = 0.0f;
      occupiedCellsVis.color.b = 0.0f;
      occupiedCellsVis.color.a = 0.5f;
      occupiedCellsVis.points = aPoints;
      occupiedCellsVis.lifetime = ros::Duration();
      mOccupiedPub.publish(occupiedCellsVis); 
    }

    /**
     * Publish the output as a collision map.
     */
    void publishCollisionMap(const std::vector<geometry_msgs::Point>& aPoints, const std_msgs::Header& aHeader) const {

     arm_navigation_msgs::CollisionMap cmap;
     cmap.header = aHeader;

     arm_navigation_msgs::OrientedBoundingBox box;
     box.extents.x = box.extents.y = box.extents.z = mOctoMap->octree.getResolution();
     box.axis.x = box.axis.y = 0.0; box.axis.z = 1.0;
     box.angle = 0.0;
     cmap.boxes.reserve(aPoints.size());

     for (std::vector<geometry_msgs::Point>::const_iterator it = aPoints.begin(); it != aPoints.end(); ++it) {
       box.center.x = it->x;
       box.center.y = it->y;
       box.center.z = it->z;
       cmap.boxes.push_back(box);
     }
     mCMapPub.publish(cmap);
    }
 
    /**
     * Get the occupied points.
     * @return The occupied points.
     */
    std::vector<geometry_msgs::Point> getOccupiedPoints() const {
      std::vector<geometry_msgs::Point> points;
      points.reserve(mOctoMap->octree.size() / 2.0);
      for (OctomapType::OcTreeType::iterator it = mOctoMap->octree.begin(), end = mOctoMap->octree.end(); it != end; ++it){
        if (mOctoMap->octree.isNodeOccupied(*it)){
          geometry_msgs::Point p;
          p.x = it.getX();
          p.y = it.getY();
          p.z = it.getZ();
          points.push_back(p);
        }
      }
      return points;
    }

    pcl::PointCloud<pcl::PointXYZI> getIntensityPoints() const {
      pcl::PointCloud<pcl::PointXYZI> points;
      for (OctomapType::OcTreeType::iterator it = mOctoMap->octree.begin(), end = mOctoMap->octree.end(); it != end; ++it){
          // TODO: Need to generate multiple points for larger
          //       ranges. Also need to limit updates to some range
          //       away from the robot.
          // TODO: Could compress Z.
          pcl::PointXYZI p;
          p.x = it.getX();
          p.y = it.getY();
          p.z = it.getZ();
          p.intensity = mOctoMap->octree.isNodeOccupied(*it) ? 0.0 : 1.0;
          points.push_back(p);
      }
      return points;
    }


    /**
     * Get the sensor origin.
     * @return Sensor origin
     */
    pcl::PointXYZ getSensorOrigin(const std_msgs::Header& aSensorHeader) const {
      geometry_msgs::PointStamped stampedIn;
      stampedIn.header = aSensorHeader;
      geometry_msgs::PointStamped stampedOut;
      mTf.transformPoint(mFixedFrame, stampedIn, stampedOut);
    
      pcl::PointXYZ result;
      result.x = stampedOut.point.x;
      result.y = stampedOut.point.y;
      result.z = stampedOut.point.z;
      return result;
    }
};

/**
 * Main function to start the updater.
 */
int main(int argc, char **argv){
  ros::init(argc, argv, "octomap_updater");
  OctomapUpdater ocu;
  ros::spin();
  return 0;
}

