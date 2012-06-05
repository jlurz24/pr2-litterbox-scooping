#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <cmvision/Blobs.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/feature.h>
#include <cmath>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;

class ObjectDetector {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    tf::TransformListener tf;
    std::string objectName;
    
    auto_ptr<message_filters::Subscriber<cmvision::Blobs> > blobsSub;
    auto_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > depthPointsSub;
    
    // Publisher for the resulting position event.
    ros::Publisher pub; 
    
    auto_ptr<message_filters::TimeSynchronizer<cmvision::Blobs, sensor_msgs::PointCloud2> > sync;
 public:
    ObjectDetector() : privateHandle("~"){
      
      ROS_INFO("Initializing the object detector");

      ROS_INFO("Waiting for blob subscription");
      
      privateHandle.getParam("object_name", objectName);
      ROS_INFO("Detecting blob with object name %s", objectName.c_str());

      // Listen for message from cm vision when it sees an object.
      blobsSub.reset(new message_filters::Subscriber<cmvision::Blobs>(nh, "/blobs", 3));
      
      ROS_INFO("Waiting for depth point subscription");
      
      // List for the depth messages
      depthPointsSub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/wide_stereo/left/points", 3));
      
      // Sync the two messages
      sync.reset(new message_filters::TimeSynchronizer<cmvision::Blobs, sensor_msgs::PointCloud2>(*blobsSub, *depthPointsSub, 10));
      
      sync->registerCallback(boost::bind(&ObjectDetector::finalBlobCallback, this, _1, _2));
     
      // Publish the object location
      pub = nh.advertise<geometry_msgs::PoseStamped>("object_location/" + objectName, 1000);
      ROS_INFO("Initialization complete");
    }
    
    ~ObjectDetector(){
    }

    void finalBlobCallback(const cmvision::BlobsConstPtr& blobsMsg, const sensor_msgs::PointCloud2ConstPtr& depthPointsMsg){
      if(pub.getNumSubscribers() == 0){
        return;
      }

      // Check if there is a detected blob.
      if(blobsMsg->blobs.size() == 0){
        ROS_INFO("No blobs detected");
        return;
      }
      
      PointCloudPtr depthCloud(new PointCloud);
      pcl::fromROSMsg(*depthPointsMsg, *depthCloud);

      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

      const PointCloudConstPtr blobsCloud = detectPlane(depthCloud, blobsMsg, inliers, coefficients);
      if(blobsCloud.get() == NULL || inliers->indices.size() == 0){
        ROS_INFO("No inliers to use for centroid detection");
        return;
      }

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*blobsCloud, *inliers, centroid);

      // TODO: Confirm don't need to convert to non homogenous centroid. 
      geometry_msgs::Vector3Stamped normalInImageFrame;
      normalInImageFrame.vector.x = coefficients->values[0];
      normalInImageFrame.vector.y = coefficients->values[1];
      normalInImageFrame.vector.z = coefficients->values[2];
      normalInImageFrame.header.frame_id = "wide_stereo_optical_frame";
      normalInImageFrame.header.stamp = depthPointsMsg->header.stamp;

      // Convert to world frame.
      ros::Duration timeout(10.0);
      tf.waitForTransform("wide_stereo_optical_frame", "map", depthPointsMsg->header.stamp, timeout);

      geometry_msgs::Vector3Stamped normalStamped;
      tf.transformVector("/map", normalInImageFrame, normalStamped);
      
      double yaw = atan(normalStamped.vector.x / -normalStamped.vector.y);
      geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);

      // Convert the centroid and quaternion to a PoseStamped
      geometry_msgs::PoseStamped resultPose;
      resultPose.header.frame_id = "wide_stereo_optical_frame";
      resultPose.header.stamp = depthPointsMsg->header.stamp;

      // Convert the centroid to a geometry msg point
      resultPose.pose.position.x = centroid[0];
      resultPose.pose.position.y = centroid[1];
      resultPose.pose.position.z = centroid[2];
      tf::quaternionTFToMsg(tf::Quaternion::getIdentity(), resultPose.pose.orientation);

      geometry_msgs::PoseStamped resultPoseMap;
      resultPoseMap.header.frame_id = "/map";
      resultPoseMap.header.stamp = depthPointsMsg->header.stamp;
      tf.transformPose("/map", resultPose, resultPoseMap);
      resultPoseMap.pose.orientation = q;

      ROS_INFO("Point in map frame: %f, %f, %f", resultPoseMap.pose.position.x, resultPoseMap.pose.position.y, resultPoseMap.pose.position.z);
      ROS_INFO("Q in map frame: %f %f %f %f", resultPoseMap.pose.orientation.x, resultPoseMap.pose.orientation.y, resultPoseMap.pose.orientation.z, resultPoseMap.pose.orientation.w);
      // Broadcast the result
      pub.publish(resultPoseMap);
    }

    const PointCloudPtr detectPlane(const PointCloudConstPtr& depthCloud, const cmvision::BlobsConstPtr& blobsMsg, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients){
         
       PointCloudPtr depthCloudFiltered(new PointCloud);

       depthCloudFiltered->header = depthCloud->header;
       depthCloudFiltered->is_dense = false;
       depthCloudFiltered->height = 1;

       for(unsigned int k = 0; k < blobsMsg->blobs.size(); ++k){
          const cmvision::Blob blob = blobsMsg->blobs[k];
          if(objectName.size() > 0 && objectName != blob.colorName){
            continue;
          }
          for(unsigned int i = blob.left; i <= blob.right; ++i){
            for(unsigned int j = blob.top; j <= blob.bottom; ++j){
              pcl::PointXYZ point = depthCloud->points.at(j * blobsMsg->image_width + i);
              depthCloudFiltered->push_back(point);
            }
          }
       }
        
       if(depthCloudFiltered->points.size() == 0){
          ROS_INFO("No blob points found.");
          return depthCloudFiltered;
       }

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
  
        // Optional
        seg.setOptimizeCoefficients(true);
        
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);

        seg.setInputCloud(depthCloudFiltered);
        seg.segment(*inliers, *coefficients);

        if(inliers->indices.size () == 0){
          ROS_INFO("Could not estimate a planar model for the given dataset.");
          return depthCloudFiltered;
        }

        ROS_INFO("%s: Inliers: %lu, Coefficients %f %f %f %f", objectName.c_str(), inliers->indices.size(), coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
        return depthCloudFiltered;
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "object_detector");
  ObjectDetector obd;
  ros::spin();
  return 0;
}

