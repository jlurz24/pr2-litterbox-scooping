#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

namespace litterbox {
  struct RectangleInfo {
    std::vector<cv::Point> contour;
    double perimeter;
    double area;
    double maxAngle;
  };

  // returns sequence of squares detected on the image.
  const std::vector<RectangleInfo> findRectangles(const cv::Mat& img);

  // the function draws all the squares in the image
  void drawRectangles(const cv::Mat& img, const std::vector<std::vector<cv::Point> >& rectangles, const std::string& windowName);

  void showRectanglesInImage(const cv::Mat& img);
  void showRectanglesInFile(const std::string& fileName);
};
