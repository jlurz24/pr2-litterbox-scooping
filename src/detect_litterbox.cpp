#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <cmvision/Blobs.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/time_synchronizer.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/feature.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;

class LitterboxDetector {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    tf::TransformListener tf;
    std::string objectName;
    
    message_filters::Subscriber<cmvision::Blobs>* blobsSub;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* depthPointsSub;
    
    // Publisher for the resulting position event.
    ros::Publisher pub; 
    
    message_filters::TimeSynchronizer<cmvision::Blobs, sensor_msgs::PointCloud2>* sync;
 public:
    LitterboxDetector() : privateHandle("~"){
      
      ROS_INFO("Initializing the litterbox detector");

      ROS_INFO("Waiting for blob subscription");
      
      privateHandle.getParam("object_name", objectName);
      ROS_INFO("Detecting blob with object name %s", objectName.c_str());

      // Listen for message from cm vision when it sees the litterbox.
      blobsSub = new message_filters::Subscriber<cmvision::Blobs>(nh, "/blobs", 1);
      
      ROS_INFO("Waiting for depth point subscription");
      
      // List for the depth messages
      depthPointsSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/wide_stereo/points2", 1);
      
      // Sync the two messages
      ROS_INFO("Setting up sync");
      sync = new message_filters::TimeSynchronizer<cmvision::Blobs, sensor_msgs::PointCloud2>(*blobsSub, *depthPointsSub, 1000);
      
      sync->registerCallback(boost::bind(&LitterboxDetector::finalBlobCallback, this, _1, _2)); 
     
      // Publish the litterbox location
      ROS_INFO("Setting up publisher");
      pub = nh.advertise<geometry_msgs::PointStamped>("litterbox_location", 1000);
      ROS_INFO("Initialization complete");
    }
    
    ~LitterboxDetector(){
      delete blobsSub;
      delete depthPointsSub;
      delete sync;
    }

    void finalBlobCallback(const cmvision::BlobsConstPtr& blobsMsg, const sensor_msgs::PointCloud2ConstPtr& depthPointsMsg){

      // Determine if the message is valid
      // TODO: Accept valid sparse messages
      if(depthPointsMsg->width != 640 && depthPointsMsg->height != 480){
        ROS_INFO("Ignoring an invalid stereo points message with width %d and height %d", depthPointsMsg->width, depthPointsMsg->height);
        return;
      }

      // Check if there is a detected blob.
      if(blobsMsg->blobs.size() == 0){
        ROS_INFO("No blobs detected");
        return;
      }
      
      PointCloudPtr depthCloud(new PointCloud);
      pcl::fromROSMsg(*depthPointsMsg, *depthCloud);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      const PointCloudPtr blobsCloud = detectPlane(depthCloud, blobsMsg, inliers, coefficients);
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*blobsCloud, *inliers, centroid);
      
      geometry_msgs::PointStamped resultPoint;
      resultPoint.point.x = centroid[0];
      resultPoint.point.y = centroid[1];
      resultPoint.point.z = centroid[2];
      resultPoint.header.frame_id = "wide_stereo_optical_frame";

      // Now convert from image from camera frame to base_link
      ros::Duration timeout(10.0);
      tf.waitForTransform("wide_stereo_optical_frame", "base_link", ros::Time(0), timeout);
      
      geometry_msgs::PointStamped resultPoint3;
      tf.transformPoint("/base_link", resultPoint, resultPoint3);

      ROS_INFO("Point in base link frame: %f, %f, %f", resultPoint3.point.x, resultPoint3.point.y, resultPoint3.point.z);
      
      // TODO: Do something with the vector.

      // Broadcast the result
      pub.publish(resultPoint3);
    }

    const PointCloudPtr detectPlane(const PointCloudConstPtr& depthCloud, const cmvision::BlobsConstPtr& blobsMsg, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients){
         
       PointCloudPtr depthCloudFiltered(new PointCloud());
       depthCloudFiltered->header = depthCloud->header;
       depthCloudFiltered->is_dense = false;
       depthCloudFiltered->height = 1;

       for(unsigned int k = 0; k < blobsMsg->blobs.size(); ++k){
          const cmvision::Blob blob = blobsMsg->blobs[k];
          if(objectName.size() > 0 && objectName != blob.colorName){
            continue;
          }

          for(unsigned int i = blob.left; i <= blob.right; ++i){
            for(unsigned int j =  blob.top; j <= blob.bottom; ++j){
              depthCloudFiltered->push_back(depthCloud->at(i, j));
            }
          }
        }
  
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
  
        // Optional
        seg.setOptimizeCoefficients (true);
        
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (depthCloudFiltered);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0){
          ROS_INFO("Could not estimate a planar model for the given dataset.");
        }

        ROS_INFO("%s: Inliers: %lu, Coefficients %f %f %f %f", objectName.c_str(), inliers->indices.size(), coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
        return depthCloudFiltered;
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "LitterboxDetector");
  LitterboxDetector lbd;
  ros::spin();
  return 0;
}

