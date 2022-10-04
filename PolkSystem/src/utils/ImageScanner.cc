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
    ConvertImageToHsv(inputImage);

    // Find the values within the threshold.
    cv::Mat returningImage;
    cv::Mat tempImage = inputImage;
    // Check if elements lie within the HSV ranges.
    cv::inRange(inputImage, HSVLowerValue, HSVUpperValue, returningImage);

    cv::Mat nonZeroCoords = GetHsvPixelLocation(returningImage);

    for(int i = 0; i < nonZeroCoords.total(); i++)
    {
        LOG(INFO) << "Non-Zero#" << i << ": " << nonZeroCoords.at<cv::Point>(i).x << ", " << nonZeroCoords.at<cv::Point>(i).y;
        cv::Point tempPoint(nonZeroCoords.at<cv::Point>(i).x, nonZeroCoords.at<cv::Point>(i).y);
        cv::circle(tempImage, tempPoint, 50, cv::Scalar(255,0,0), 1);
    }

    LOG(INFO) << "Number of channels in testing: " << nonZeroCoords.channels();
    LOG(INFO) << "Number of channels in inputImage: " << inputImage.channels();
    
    return tempImage;
}
/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
void ImageScanner::ConvertImageToHsv(cv::Mat image)
{
    // Convert RGB into HSV.
    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
}
/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
void ImageScanner::ConvertImageToRgb(cv::Mat image)
{
    // Convert HSV back into RGB.
    cv::cvtColor(image, image, cv::COLOR_HSV2BGR, 3);
}
/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
cv::Mat ImageScanner::GetHsvPixelLocation(cv::Mat hsvImage)
{
    // cv::Mat to contain non-zero points.
    cv::Mat nonZeroCoords;

    // Finds all non-zero values from the cv::Mat object.
    cv::findNonZero(hsvImage, nonZeroCoords);

    // for(int i = 0; i < nonZeroCoords.total(); i++)
    // {
    //     LOG(INFO) << "Non-Zero#" << i << ": " << nonZeroCoords.at<cv::Point>(i).x << ", " << nonZeroCoords.at<cv::Point>(i).y;
    //     cv::Point tempPoint(nonZeroCoords.at<cv::Point>(i).x, nonZeroCoords.at<cv::Point>(i).y);
    //     cv::circle(hsvImage, tempPoint, 50, cv::Scalar(255,0,0), 1);
    // }

    // Return the coordinates of non-zero values.
    return nonZeroCoords;
}
/////////////////////////////////////////////////////////////////////////////////////