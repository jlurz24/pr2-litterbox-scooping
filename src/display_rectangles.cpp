#include <ros/ros.h>
#include <litterbox/find_rectangles.h>

int main(int argc, char** argv){
  if(argc < 2){
    ROS_INFO("You must specify the file name on the command line");
    return 0;
  }
  ROS_INFO("Finding rectangles in file: %s", argv[1]);
  litterbox::showRectanglesInFile(argv[1]);
}

