#include <ros/ros.h>
#include <pcl/octree/octree.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <gazebo/GetModelState.h>
#include <tf/transform_listener.h>

using namespace std;

typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Octree;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef Octree::AlignedPointTVector VoxelVector;

static const double EPSILON = 1e-2;

bool pointXYZEqual(const pcl::PointXYZ& left, const pcl::PointXYZ& right){
  bool result = fabs(left.x - right.x) < EPSILON &&
         fabs(left.y - right.y) < EPSILON &&
         fabs(left.z - right.z) < EPSILON;

  return result;
}

bool pointXYZGreaterThan(const pcl::PointXYZ &left, const pcl::PointXYZ& right){
  if(fabs(left.x - right.x) > EPSILON){
    return left.x > right.x;
  }
  if(fabs(left.y - right.y) > EPSILON){
    return left.y > right.y;
  }
  return left.z > right.z; 
}

// TODO: Make this a service?
class ObservableVolume {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    tf::TransformListener tf;
    ros::Publisher origPclPublisher;

    ros::Publisher intersectedPclPublisher;
  public:
   ObservableVolume() : privateHandle("~") {

      origPclPublisher = nh.advertise<sensor_msgs::PointCloud2>("original_point_cloud_out", 1, true);
      intersectedPclPublisher = nh.advertise<sensor_msgs::PointCloud2>("intersected_point_cloud_out", 1, true);

      // Get the resolution of the map.
      double resolution;
      privateHandle.param<double>("resolution", resolution, 0.1);
      Octree octree(resolution);
      
      // Get the current sensor origin.
      
      geometry_msgs::PointStamped laserScannerInLaserFrame;
      laserScannerInLaserFrame.header.frame_id = "/laser_tilt_mount_link";
      laserScannerInLaserFrame.header.stamp = ros::Time::now();
      geometry_msgs::PointStamped originPoint;

      string referenceFrame;
      privateHandle.param<string>("reference_frame", referenceFrame, "/odom_combined");
      ROS_INFO("Waiting for transform between %s and %s", referenceFrame.c_str(), laserScannerInLaserFrame.header.frame_id.c_str());
      bool found = false;
      while(!found){
        found = tf.waitForTransform(referenceFrame, laserScannerInLaserFrame.header.frame_id, laserScannerInLaserFrame.header.stamp, ros::Duration(10.0));
        if(!found){
          ROS_INFO("Failed to lookup the transform");
          ros::Duration(1.0).sleep();
        }
      }

      ROS_INFO("Transforming point");
      tf.transformPoint(referenceFrame, laserScannerInLaserFrame.header.stamp, laserScannerInLaserFrame, laserScannerInLaserFrame.header.frame_id, originPoint);

      Eigen::Vector3f origin(originPoint.point.x, originPoint.point.y, originPoint.point.z);
      ROS_INFO("Origin is x: %f y: %f z: %f", originPoint.point.x, originPoint.point.y, originPoint.point.z);

      // Get the name of the object to measure.
      string objectName;
      privateHandle.param<string>("object_name", objectName, "box1");

      // Fetch the object from gazebo.
      ros::ServiceClient getModelStateClient = nh.serviceClient<gazebo::GetModelState> ("/gazebo/get_model_state", true);
     
      gazebo::GetModelState requestResponse;
      requestResponse.request.model_name = "box1";
      getModelStateClient.call(requestResponse);
      geometry_msgs::Point center;
      if(requestResponse.response.success){
        center = requestResponse.response.pose.position;
        ROS_INFO("Center of model is x: %f y: %f z: %f", center.x, center.y, center.z);
      }
      else {
        ROS_ERROR("Failed to get the model state");
        return;
      }
      // Fetch the height, width, and depth from the configuration. No
      // way to get this from the exposed API.
      double width, height, depth, simulResolution;
      privateHandle.param<double>("object_width", width, 1.0);
      privateHandle.param<double>("object_height", height, 1.0);
      privateHandle.param<double>("object_depth", depth, 1.0);
      privateHandle.param<double>("simul_resolution", simulResolution, 0.05);

      ROS_INFO("Size of object is w: %f h: %f d: %f", width, height, depth);

      // Iterate over the entire cube and create points.
      PointCloudXYZ::Ptr pclCloud(new PointCloudXYZ);
      for(double x = center.x - depth / 2.0; x < center.x + depth / 2.0; x += simulResolution / 2.0){
        for(double y = center.y - width / 2.0; y < center.y + width / 2.0; y += simulResolution / 2.0){
          for(double z = center.z - height / 2.0; z < center.z + height / 2.0; z += simulResolution / 2.0){
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = z;
            pclCloud->points.push_back(point);
          }
        }
      }

      ROS_INFO("Number of points in cloud %lu", pclCloud->points.size());

      // Visualize
      sensor_msgs::PointCloud2 origCloud;
      pcl::toROSMsg(*pclCloud, origCloud);
      origCloud.header.stamp = ros::Time::now();
      origCloud.header.frame_id = referenceFrame;
      origPclPublisher.publish(origCloud);

      ROS_INFO("Number of points in cloud %lu", pclCloud->points.size());
      octree.setInputCloud(pclCloud);

      // TODO: Is this right?
      octree.defineBoundingBox();
      octree.addPointsFromInputCloud();

      VoxelVector occupiedVoxels;
      octree.getOccupiedVoxelCenters(occupiedVoxels);
      ROS_INFO("Number of occupied voxel centers %lu", occupiedVoxels.size());

      PointCloudXYZ::Ptr visibleCloud(new PointCloudXYZ);
      for(unsigned int i = 0; i < occupiedVoxels.size(); ++i){
        Eigen::Vector3f endPoint;
        endPoint[0] = occupiedVoxels[i].x;
        endPoint[1] = occupiedVoxels[i].y;
        endPoint[2] = occupiedVoxels[i].z;
        VoxelVector intersected;
        octree.getIntersectedVoxelCenters(origin, (endPoint - origin).normalized(), intersected);
        if(intersected.size() > 0){
          // ROS_INFO("Ray to point %f %f %f intersected %f %f %f", endPoint[0], endPoint[1], endPoint[2], intersected[0].x, intersected[0].y, intersected[0].z);
          visibleCloud->points.push_back(intersected[0]);
        }
        else {
          ROS_INFO("Ray did not intercept any voxels");
        }
      }

      // Eliminate duplicates
      ROS_INFO("Number of points prior to removing duplicates: %lu", visibleCloud->points.size());
      sort(visibleCloud->points.begin(), visibleCloud->points.end(), pointXYZGreaterThan);
      visibleCloud->points.erase(unique(visibleCloud->points.begin(), visibleCloud->points.end(), pointXYZEqual), visibleCloud->points.end());
      ROS_INFO("Number of points after removing duplicates: %lu", visibleCloud->points.size());
      
     // for(unsigned int i = 0; i < visibleCloud->points.size(); ++i){
     //   ROS_INFO("%f, %f, %f", visibleCloud->points[i].x, visibleCloud->points[i].y, visibleCloud->points[i].z);
      //}

      // Visualize
      sensor_msgs::PointCloud2 visiblePubCloud;
      pcl::toROSMsg(*visibleCloud, visiblePubCloud);
      visiblePubCloud.header.stamp = ros::Time::now();
      visiblePubCloud.header.frame_id = referenceFrame;
      intersectedPclPublisher.publish(visiblePubCloud);

      double volume = visibleCloud->points.size() * pow(resolution, 3);
      ROS_INFO("Total volume is %f", volume);
   }
};

int main(int argc, char **argv){
  ROS_INFO("Initializing observable_volume");
  ros::init(argc, argv, "observable_volume");
  ObservableVolume ov;
  ros::spin();
  return 0;
}

