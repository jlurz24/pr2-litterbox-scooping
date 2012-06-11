#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class LaserScanToPointCloud {
     public:
        LaserScanToPointCloud();
     private:
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

        ros::NodeHandle nh;
        laser_geometry::LaserProjection projector;
        tf::TransformListener tf;

        ros::Publisher pub;
        ros::Subscriber sub;
};

LaserScanToPointCloud::LaserScanToPointCloud(){
        ROS_INFO("Initializing the laser scan to point cloud converter");

        sub = nh.subscribe<sensor_msgs::LaserScan> ("/scan_in", 100, &LaserScanToPointCloud::scanCallback, this);
        pub = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_out", 1, false);
        tf.setExtrapolationLimit(ros::Duration(0.1));
        ROS_INFO("Laser scan to point cloud converter intitialization complete");
}

void LaserScanToPointCloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    ROS_INFO("Received a laser scan");
    if(pub.getNumSubscribers() == 0){
      ROS_INFO("Ignoring scan callback. No subscribers.");
    }
    sensor_msgs::PointCloud2 cloud;
    projector.transformLaserScanToPointCloud(scan->header.frame_id, *scan, cloud, tf);
    pub.publish(cloud);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "laser_scan_to_point_cloud");
    LaserScanToPointCloud filter;
    ros::spin();
    return 0;
}

