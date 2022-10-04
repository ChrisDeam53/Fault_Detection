// ImageScanner.cc
#include "ImageScanner.hh"

#include <opencv2/imgproc/imgproc.hpp>
#include <glog/logging.h>
#include <opencv2/core/mat.hpp>


using namespace utils;

/////////////////////////////////////////////////////////////////////////////////////
cv::Mat ImageScanner::FindHsvValues(cv::Mat inputImage, const cv::Scalar HSVLowerValue, const cv::Scalar HSVUpperValue)
{
    // Convert Colours into HSV.
    cv::cvtColor(inputImage, inputImage, cv::COLOR_BGR2HSV);

    // Finds the values within the threshold.
    cv::Mat returningImage;
    // Check if elements lie within the HSV ranges.
    cv::inRange(inputImage, HSVLowerValue, HSVUpperValue, returningImage);

    GetHsvPixelLocation(returningImage);

    return returningImage;
}
/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
cv::Mat ImageScanner::GetHsvPixelLocation(cv::Mat hsvImage)
{
    // cv::Mat to contain non-zero points.
    cv::Mat nonZeroCoords;

    // Finds all non-zero values from the cv::Mat object.
    cv::findNonZero(hsvImage, nonZeroCoords);

    for(int i = 0; i < nonZeroCoords.total(); i++)
    {
        LOG(INFO) << "Zero#" << i << ": " << nonZeroCoords.at<cv::Point>(i).x << ", " << nonZeroCoords.at<cv::Point>(i).y;
    }
    
    // drawOnImage(hsvImage, cv::Point(2404,2675), cv::Point(2402,2676), cv::Scalar(255,0,0), 2);
    drawOnImage(hsvImage, cv::Point(0,0), cv::Point(500,500), cv::Scalar(255,0,0), 2);

    // Return the coordinates of non-zero values.
    return nonZeroCoords;
}
/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
void ImageScanner::drawOnImage(cv::Mat image, cv::Point startPoint, cv::Point endPoint, cv::Scalar lineColour, int thickness)
{
    cv::line(image, startPoint, endPoint, lineColour, 2, thickness);
}