#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <gazebo_msgs/ModelStates.h>
#include <costmap_2d/voxel_costmap_2d.h>
#include <voxel_grid/voxel_grid.h>

#define VOXEL_DEBUG 0

using namespace std;

class PixelOccupied {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    geometry_msgs::Point32 referencePoint;
    string modelName;
    string referenceFrame;
    string inputType;
    auto_ptr<message_filters::Subscriber<arm_navigation_msgs::CollisionMap> > collisionSub;
    auto_ptr<message_filters::Subscriber<gazebo_msgs::ModelStates> > statesSub;
    auto_ptr<message_filters::Subscriber<costmap_2d::VoxelGrid> > voxelSub;

    double minHeight;
    bool unoccupiedMode;
    bool modelChangeReported;
    bool occupancyChangeReported;
    bool modelEverFound;
    bool pixelEverFound;
    ros::Time modelChangeReportedTime;

    void modelCB(const gazebo_msgs::ModelStates::ConstPtr& models){
      // Reset if the occupancy change has been reported and the model
      // has updated.
      string oldModelName = modelName;
      privateHandle.param<string>("model_name", modelName, "obstacle");
      if(occupancyChangeReported && oldModelName != modelName){
        occupancyChangeReported = false;
        modelChangeReported = false;
        modelEverFound = false;
      }

      if(modelChangeReported){
        return;
      }

      bool found = false;
      for(unsigned int i = 0; i < models->name.size(); ++i){
        if(models->name[i] == modelName){
          modelEverFound = true;
          found = true;
          break;
        }
      }

      if(unoccupiedMode && !found && modelEverFound){
        modelChangeReported = true;
        occupancyChangeReported = false;
        ROS_INFO("Model was removed @ time %f", ros::Time::now().toSec());
        modelChangeReportedTime = ros::Time::now();
      }
      else if(!unoccupiedMode && found){
        modelChangeReported = true;
        occupancyChangeReported = false;
        ROS_INFO("Model was added @ time %f", ros::Time::now().toSec());
        modelChangeReportedTime = ros::Time::now();
      }
      else {
        // ROS_INFO("No change to report found %i unoccupiedMode %i modelEverFound %i model name %s", found, unoccupiedMode, modelEverFound, modelName.c_str()); 
      } 
    }

    static bool worldToMap3D(const costmap_2d::VoxelGridConstPtr& grid, double wx, double wy, double wz, double& mx, double& my, double& mz){
      if(wx < grid->origin.x || wy < grid->origin.y || wz < grid->origin.z)
        return false;

      mx = ((wx - grid->origin.x) / grid->resolutions.x);
      my = ((wy - grid->origin.y) / grid->resolutions.y);
      mz = ((wz - grid->origin.z) / grid->resolutions.z);

      return (mx < grid->size_x && my < grid->size_y && mz < grid->size_z);
    }

    void voxelCB(const costmap_2d::VoxelGridConstPtr& grid){
      if(occupancyChangeReported){
        return;
      }

      if(grid->data.empty()){
        ROS_INFO("Received an empty grid");
        return;
      }

      #if VOXEL_DEBUG
      for (uint32_t y_grid = 0; y_grid < grid->size_y; ++y_grid){
        for (uint32_t x_grid = 0; x_grid < grid->size_x; ++x_grid){
          for (uint32_t z_grid = 0; z_grid < grid->size_z; ++z_grid){
            voxel_grid::VoxelStatus status = voxel_grid::VoxelGrid::getVoxel(x_grid, y_grid, z_grid, grid->size_x, grid->size_y, grid->size_z, &grid->data.front());
            if (status == voxel_grid::MARKED){
              double x, y, z;
              costmap_2d::VoxelCostmap2D::mapToWorld3D(x_grid, y_grid, z_grid, grid->origin.x, grid->origin.y, grid->origin.z, grid->resolutions.x, grid->resolutions.y, grid->resolutions.z, x, y, z);
              ROS_INFO("Found an occupied cell @ %u %u %u %f %f %f", x_grid, y_grid, z_grid, x, y, z);
            }
          }
        }
      }
      #endif

      // Avoid transforming as the time to wait for the transform
      // could skew results.
      if(grid->header.frame_id != referenceFrame){
        ROS_ERROR("voxel grid frame %s and reference point frame %s don't match", grid->header.frame_id.c_str(), referenceFrame.c_str());
        return;
      }

      double x, y, z;
      if(!worldToMap3D(grid, referencePoint.x, referencePoint.y, 0.5, x, y, z)){
        ROS_ERROR("Failed to translate map point to grid");
      }

      voxel_grid::VoxelStatus status = voxel_grid::VoxelGrid::getVoxel(ceil(x), ceil(y), ceil(z), grid->size_x, grid->size_y, grid->size_z, &grid->data.front());
      ROS_INFO("Status is %i for %f %f %f", status, ceil(x), ceil(y), ceil(z));
      if(status == voxel_grid::MARKED && unoccupiedMode){
        pixelEverFound = true;
      }
      if(!modelChangeReported){
        return;
      }
      if(status == voxel_grid::MARKED && !unoccupiedMode){
        occupancyChangeReported = true;
        ROS_INFO("Reference point was occupied @ time %f", ros::Time::now().toSec());
        ROS_INFO("UPDATE TIME: %f", ros::Time::now().toSec() - modelChangeReportedTime.toSec());
      }
      else if(status == voxel_grid::FREE && unoccupiedMode){
        occupancyChangeReported = true;
        if(!pixelEverFound){
          ROS_INFO("Error: Pixel was never detected");
        }
        else {
          ROS_INFO("Reference point was unoccupied @ time %f", ros::Time::now().toSec());
          ROS_INFO("UPDATE TIME: %f", ros::Time::now().toSec() - modelChangeReportedTime.toSec());
        }
        pixelEverFound = false;
      }
      else {
        // ROS_INFO("No change to report");
      }
}

    void pixelCB(const arm_navigation_msgs::CollisionMap::ConstPtr& collisionMap){
      if(occupancyChangeReported || !modelChangeReported){
        return;
      }
      
      // Avoid transforming as the time to wait for the transform
      // could skew results.
      if(collisionMap->header.frame_id != referenceFrame){
        ROS_ERROR("Collision map and reference point frames don't match");
        return;
      }
      
      bool found = false;
      for(unsigned int i = 0; i < collisionMap->boxes.size(); ++i){
        const arm_navigation_msgs::OrientedBoundingBox& box = collisionMap->boxes[i];
        if(box.center.x + box.extents.x / 2.0 > referencePoint.x &&
           box.center.x - box.extents.x / 2.0 < referencePoint.x &&
           box.center.y + box.extents.y / 2.0 > referencePoint.y &&
           box.center.y - box.extents.y / 2.0 < referencePoint.y &&
           box.center.z > minHeight){
           // Intentionally ignoring z to avoid timing issues related
           // to the orientation of the laser scanner.
             found = true;
             pixelEverFound = true;
             break;
        }
      }
     
      if(found && !unoccupiedMode){
        occupancyChangeReported = true;
        ROS_INFO("Reference point was occupied @ time %f", ros::Time::now().toSec());
        ROS_INFO("UPDATE TIME: %f", ros::Time::now().toSec() - modelChangeReportedTime.toSec());
      }
      else if(!found && unoccupiedMode){
        if(!pixelEverFound){
          ROS_INFO("ERROR: Object was never detected");
        }
        occupancyChangeReported = true;
        pixelEverFound = false;
        ROS_INFO("Reference point was unoccupied @ time %f", ros::Time::now().toSec());
        ROS_INFO("UPDATE TIME: %f", ros::Time::now().toSec() - modelChangeReportedTime.toSec());
      }
      else {
        // ROS_INFO("No change to report");
      }
    }
   
  public:
   PixelOccupied(): privateHandle("~") , unoccupiedMode(false), modelChangeReported(false), occupancyChangeReported(false), modelEverFound(false), pixelEverFound(false){
    double temp;
    privateHandle.param<double>("reference_x", temp, 0.0);
    referencePoint.x = float(temp);
    privateHandle.param<double>("reference_y", temp, 0.0);
    referencePoint.y = float(temp);
    privateHandle.param<bool>("unoccupied_mode", unoccupiedMode, false);
    privateHandle.param<string>("reference_frame", referenceFrame, "/odom_combined");
    privateHandle.param<string>("model_name", modelName, "obstacle");
    privateHandle.param<double>("min_height", minHeight, 0.5);
    privateHandle.param<string>("input_type", inputType, "collision");

    if(inputType == "collision"){
      collisionSub.reset(new message_filters::Subscriber<arm_navigation_msgs::CollisionMap>(nh, "collision_map_in", 1));
      collisionSub->registerCallback(boost::bind(&PixelOccupied::pixelCB, this, _1)); 
    }
    
    statesSub.reset(new message_filters::Subscriber<gazebo_msgs::ModelStates>(nh, "gazebo/model_states", 1));
    statesSub->registerCallback(boost::bind(&PixelOccupied::modelCB, this, _1));

    if(inputType == "voxel"){
      voxelSub.reset(new message_filters::Subscriber<costmap_2d::VoxelGrid>(nh, "voxel_grid_in", 1));
      voxelSub->registerCallback(boost::bind(&PixelOccupied::voxelCB, this, _1));
    }
  }
};

int main(int argc, char **argv){
  ROS_INFO("Initializing pixel occupied");
  ros::init(argc, argv, "pixel_occupied");
  PixelOccupied po;
  ros::spin();
  return 0;
}

