#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <litterbox/find_rectangles.h>

namespace litterbox {

const unsigned int THRESH = 50;
const unsigned int THRESH_LEVELS = 1;

typedef std::vector<cv::Point> Contour;

// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle(const cv::Point pt1, const cv::Point pt2, const cv::Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

// returns sequence of rectangles detected on the image.
const std::vector<RectangleInfo> findRectangles(const cv::Mat& img){

    // Results
    std::vector<RectangleInfo> rectangles;

    // select the maximum ROI in the image
    // with the width and height divisible by 2
    cv::Mat timgMat = img.clone();
    cv::Rect imgRect = cv::Rect(0, 0, img.size().width & -2, img.size().height & -2);
    timgMat.adjustROI(imgRect.tl().y, imgRect.br().y, imgRect.tl().x, imgRect.br().x);

    // down-scale and upscale the image to filter out the noise
    cv::Mat pyrMat;
    cv::pyrDown(timgMat, pyrMat);
    cv::pyrUp(pyrMat, timgMat);
    
    // find rectangles in every color plane of the image
    for(int c = 0; c < timgMat.channels(); ++c){
        // extract the c-th color plane
        cv::Mat tgrayMat;
        IplImage timgIpl = timgMat;
        cv::extractImageCOI(&timgIpl, tgrayMat, c);
        
        // try several threshold levels
        cv::Mat grayMat;
        for(unsigned int l = 0; l < THRESH_LEVELS; l++){
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if(l == 0){
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                cv::Canny(tgrayMat, grayMat, 0, THRESH, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                cv::dilate(grayMat, grayMat, cv::Mat());
            } else {
                // apply threshold if l!=0:
                cv::threshold(tgrayMat, grayMat, (l + 1) * 255 / THRESH_LEVELS, 255, CV_THRESH_BINARY);
            }

            // find contours and store them all as a list
            std::vector<Contour> contours;
            cv::findContours(grayMat, contours,
                CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            // test each contour
            for(unsigned int k = 0; k < contours.size(); ++k){
                Contour contour = contours[k];
                // approximate contour with accuracy proportional
                // to the contour perimeter
                double perimeter = cv::arcLength(cv::Mat(contour), true);
                std::vector<cv::Point> result;
                cv::approxPolyDP(cv::Mat(contour), result, perimeter * 0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if(result.size()== 4){
                    double area = std::fabs(cv::contourArea(cv::Mat(result)));
                    if(area > 1000 && cv::isContourConvex(cv::Mat(result))){

                    // find minimum angle between joint
                    // edges (maximum of cosine)
                    double t1 = std::fabs(angle(
                            result[3],
                            result[1],
                            result[0]));

                    double t2 = std::fabs(angle(
                            result[0],
                            result[2],
                            result[1]));
                    double t3 = std::fabs(angle(
                            result[1],
                            result[3],
                            result[2]));
                    double s = std::max(t3, std::max(t1, t2));

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if(s < 0.3){
                        RectangleInfo rectInfo = {result, perimeter, area, s};
                        rectangles.push_back(rectInfo);
                    }
                  }
                }
            }
        }
    }
    ROS_INFO("Located %lu rectangles", rectangles.size());
    return rectangles;
}


// the function draws all the rectangles in the image
void drawRectangles(const cv::Mat& img, const std::vector<std::vector<cv::Point> >& rectangles, const std::string& windowName){

    cv::Mat cpy = img.clone();

    // draw the square as a closed polyline
    cv::drawContours(cpy, rectangles, -1, CV_RGB(0, 255, 0), 3, CV_AA);

    // show the resultant image
    cv::imshow(windowName, cpy);
}

void showRectanglesInFile(const std::string& fileName){

  cv::Mat img = cv::imread(fileName);
  showRectanglesInImage(img);
}

void showRectanglesInImage(const cv::Mat& img){
  static const std::string windowName = "Rectangles";
  cv::namedWindow(windowName, 1);

  // find and draw the rectangles
  const std::vector<RectangleInfo> rects = findRectangles(img);
  std::vector<Contour> contours;
  for(unsigned int i = 0; i < rects.size(); ++i){
    contours.push_back(rects[i].contour);
  }
  drawRectangles(img, contours, windowName);

  // cv::waitKey takes care of event processing
  cv::waitKey(0);
}
}

