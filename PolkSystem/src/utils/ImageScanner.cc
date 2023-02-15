// ImageScanner.cc
#include "ImageScanner.hh"

#include <opencv2/imgproc/imgproc.hpp>
#include <glog/logging.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/fast_math.hpp>

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

    cv::Mat separateChannels[3];

    cv::split(inputImage, separateChannels);

    cv::Mat hueChannel = separateChannels[0];
    cv::Mat saturationChannel = separateChannels[1];
    cv::Mat valueChannel = separateChannels[2];

    cv::Scalar(0, 0, 30);
    cv::Scalar(0, 0, 39);
    const float channelOneMin = 0.037;
    const float channelOneMax = 0.037;

    cv::Scalar(0, 31.2, 0);
    cv::Scalar(0, 46.8, 0);
    const float channelTwoMin = 0.175;
    const float channelTwoMax = 0.543;

    cv::Scalar(0, 0, 37.6);
    cv::Scalar(0, 0, 35.3);
    const float channelThreeMin = 0.343;
    const float channelThreeMax = 1.000;

    cv::Mat outputChannelOne;
    cv::Mat outputChannelTwo;
    cv::Mat outputChannelThree;

    cv::inRange(hueChannel, cv::Scalar(0, 0, 30), cv::Scalar(0, 0, 39), outputChannelOne);
    cv::inRange(hueChannel, cv::Scalar(0, 31.2, 0), cv::Scalar(0, 46.8, 0), outputChannelTwo);
    cv::inRange(hueChannel, cv::Scalar(0, 0, 35.3), cv::Scalar(0, 0, 35.3), outputChannelThree);

    cv::Mat hsvChannels[3] = {outputChannelOne, outputChannelTwo, outputChannelThree};
    cv::Mat combinedChannels;
    cv::merge(hsvChannels, 1, combinedChannels);

    LOG(INFO) << "Number of channels in combinedChannels: " << combinedChannels.channels();
    LOG(INFO) << "Number of channels in returningImage: " << returningImage.channels();

    cv::Mat nonZeroCoords = GetHsvPixelLocation(combinedChannels);

    // Uncomment when required
    // for(int i = 0; i < nonZeroCoords.total(); i++)
    // {
    //     LOG(INFO) << "Non-Zero#" << i << ": " << nonZeroCoords.at<cv::Point>(i).x << ", " << nonZeroCoords.at<cv::Point>(i).y;
    //     cv::Point tempPoint(nonZeroCoords.at<cv::Point>(i).x, nonZeroCoords.at<cv::Point>(i).y);
    //     cv::circle(tempImage, tempPoint, 10, cv::Scalar(255,0,0), 1);
    // }

    LOG(INFO) << "Number of channels in testing: " << nonZeroCoords.channels();
    LOG(INFO) << "Number of channels in inputImage: " << inputImage.channels();
    LOG(INFO) << "Number of channels in outputImage: " << tempImage.channels();

    cv::Mat testImage = SegmentImageColours(tempImage, inputImage);

    cv::Mat testImage2 = ApplyEdgeDetection(inputImage);

    // Returns the tresholded image - The wooden blocks are white.
    cv::Mat testImage3 = ThresholdWoodenShape(inputImage);

    cv::Mat testingBitwiseAnd;

    // Need to convert the mask into a 3 channel image as the original image is a 3 channel image.
    cv::Mat convertedMat;
    cv::cvtColor(testImage3, convertedMat, cv::COLOR_GRAY2BGR, 3);

    LOG(INFO) << "";
    LOG(INFO) << "";
    LOG(INFO) << "Number of channels in testImage3: " << testImage3.channels();
    LOG(INFO) << "Number of channels in inputImage: " << inputImage.channels();
    LOG(INFO) << "Number of channels in convertedMat: " << convertedMat.channels();

    // Applies the mask atop the original image.
    ConvertImageToRgb(inputImage);
    cv::bitwise_and(inputImage, convertedMat, testingBitwiseAnd);

    cv::Mat edgeMat = ApplyEdgeDetection(testingBitwiseAnd);

    // Output_Test_32_Morphology
    // DetectBoltCircles(edgeMat);

    cv::Mat newMaskMat;
    cv::bitwise_and(testImage2, edgeMat, newMaskMat);

    cv::Mat circlesMatrix;
    circlesMatrix = DetectBoltCircles(newMaskMat);
    LOG(INFO) << "Number of channels in circlesMatrix: " << circlesMatrix.channels();

    cv::Mat rgbImageMatrix;
    cv::cvtColor(circlesMatrix, rgbImageMatrix, cv::COLOR_GRAY2BGR, 3);

    // Covert all white contour lines to green.
    cv::Mat whiteToGreenMask;
    cv::inRange(rgbImageMatrix, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), whiteToGreenMask);
    rgbImageMatrix.setTo(cv::Scalar(0, 255, 0), whiteToGreenMask);

    // Impose the Green circles mask atop the original image.
    cv::Mat detectedBoltsMatrix;
    cv::bitwise_or(inputImage, rgbImageMatrix, detectedBoltsMatrix);
    return detectedBoltsMatrix;
}


/////////////////////////////////////////////////////////////////////////////////////
void ImageScanner::ConvertImageToHsv(cv::Mat image)
{
    if(image.empty())
    {
        // Image matrix is empty.
        LOG(ERROR) << "Could not read the image.";
    }
    // Convert RGB into HSV.
    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
}


/////////////////////////////////////////////////////////////////////////////////////
void ImageScanner::ConvertImageToRgb(cv::Mat image)
{
    if(image.empty())
    {
        // Image matrix is empty.
        LOG(ERROR) << "Could not read the image.";
    }
    // Convert HSV back into RGB.
    cv::cvtColor(image, image, cv::COLOR_HSV2BGR, 3);
}

/////////////////////////////////////////////////////////////////////////////////////
cv::Mat ImageScanner::SegmentImageColours(cv::Mat colourImage, cv::Mat hsvImage)
{
    // NOTE: As inputted image is HSV not RGB (I think) - Comment out for now
    // const cv::Scalar lightBrown{43, 26.1, 87.1};
    // const cv::Scalar darkBrown{38, 38.4, 79.6};
    const cv::Scalar lightBrownHsv{17, 90.6, 83.1};
    const cv::Scalar darkBrownHsv{23, 90.4, 77.3};

    cv::Mat returningImage;
    cv::Mat tempImage;

    // Threshold the image.
    cv::inRange(hsvImage, lightBrownHsv, darkBrownHsv, tempImage);

    cv::Mat nonZeroCoords = GetHsvPixelLocation(tempImage);
    
    // Impose the mask atop the original image - Keeps every pixel if the corresponding mask value is 1.
    cv::bitwise_and(hsvImage, hsvImage, returningImage, nonZeroCoords);

    return tempImage;
}


/////////////////////////////////////////////////////////////////////////////////////
cv::Mat ImageScanner::GetHsvPixelLocation(cv::Mat hsvImage)
{
    // cv::Mat to contain non-zero points.
    cv::Mat nonZeroCoords;

    // Finds all non-zero values from the cv::Mat object.
    cv::findNonZero(hsvImage, nonZeroCoords);

    // Return the coordinates of non-zero values.
    return nonZeroCoords;
}


/////////////////////////////////////////////////////////////////////////////////////
cv::Mat ImageScanner::ApplyEdgeDetection(cv::Mat hsvImage)
{
    // cv::Mat to return with canny Edge applied.
    cv::Mat cannyEdgeImage;

    // Image 14 -> 100,200,3,false
    // Image 15 -> 100,200,3,true
    // Image 16 -> 150,200,3,false
    // Image 17 -> 350,400,3,false -> Does not show wooden blocks
    // Image 18 -> 450,500,3,false
    // Image 19 -> 450,500,3,false
    // Image 20 -> 450,500,3,true

    // OG
    // cv::Canny(hsvImage, cannyEdgeImage, 650, 700, 3, true);

    cv::Canny(hsvImage, cannyEdgeImage, 550, 700, 3, true);

    return cannyEdgeImage;
}


/////////////////////////////////////////////////////////////////////////////////////
cv::Mat ImageScanner::ThresholdWoodenShape(cv::Mat rgbImage)
{
    const std::string shapeType = "Hexagon";

    cv::Mat greyscaleImage, returnImage;

    // Convert RGB to Greyscale for Omotsu's Thresholding.
    cv::cvtColor(rgbImage, greyscaleImage, cv::COLOR_BGR2GRAY);

    // Image 21 -> (non-adaptive threshold)
    // Apply a fixed-level threshold to each array element.
    cv::threshold(greyscaleImage, returnImage, 127, 255, cv::THRESH_BINARY_INV);

    return ApplyErosion(returnImage);
}


/////////////////////////////////////////////////////////////////////////////////////
cv::Mat ImageScanner::ApplyErosion(cv::Mat inputImage)
{
    cv::Mat erodedImage;

    cv::erode(inputImage, erodedImage, cv::Mat(), cv::Point(-1,-1), 30);

    cv::rectangle(erodedImage, cv::Point(0,0), cv::Point(3024, (erodedImage.rows / 100) * 30), cv::Scalar(0,0,0), cv::FILLED);

    return erodedImage;
}


/////////////////////////////////////////////////////////////////////////////////////
cv::Mat ImageScanner::DetectBoltCircles(cv::Mat edgeMat)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat circlesMatrix;
    std::vector<cv::Vec3f> circles;

    cv::findContours(edgeMat, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat zerosMatrix = cv::Mat::zeros(edgeMat.rows, edgeMat.cols, CV_8UC3);
    
    cv::drawContours(zerosMatrix, contours, hierarchy[0][1], cv::Scalar(255, 255, 255), 2);

    cv::cvtColor(zerosMatrix, circlesMatrix, cv::COLOR_BGR2GRAY, 1);

    // smooth it, otherwise a lot of false circles may be detected
    cv::GaussianBlur(circlesMatrix, circlesMatrix, cv::Size(9, 9), 2, 2);

    cv::HoughCircles(circlesMatrix, circles, cv::HOUGH_GRADIENT, 1, 60, 200, 20, 0, 0);

    LOG(INFO) << "--eeee- circles.size() " << circles.size();

    // NOTE:
    // Code below inspired by: https://stackoverflow.com/questions/20698613/detect-semicircle-in-opencv
    // Answered by: Micka, Dec 20, 2013 at 14:43.
    for(size_t i = 0; i < circles.size(); i++)
    {
        // Draw on inputImage to draw atop original image.
        LOG(INFO) << "i 0: " << circles[i][0];
        LOG(INFO) << "i 1: " << circles[i][1];

        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // Draw the circle center
        cv::circle(circlesMatrix, center, 3, cv::Scalar(255,0,0), -1, 8, 0 );
        // Draw the circle outline
        cv::circle(circlesMatrix, center, radius, cv::Scalar(255,0,0), 3, 8, 0 );
    }

    ///////////////////////////////////

    // Compute distance transform.
    cv::Mat dt;
    cv::distanceTransform(255 - (edgeMat > 0), dt, cv::DIST_MASK_3 ,3);

    // Test for semi-circles.
    float minInlierDist = 2.0f;

    for(size_t i = 0; i < circles.size(); i++ ) 
    {
        // Test inlier percentage:
        // Sample the circle and check for distance to the next edge
        unsigned int counter = 0;
        unsigned int inlier = 0;

        cv::Point2f center((circles[i][0]), (circles[i][1]));
        float radius = (circles[i][2]);

        // maximal distance of inlier might depend on the size of the circle
        float maxInlierDist = radius/25.0f;

        if(maxInlierDist < minInlierDist)
        {
            maxInlierDist = minInlierDist;
        }

        //TODO: maybe paramter incrementation might depend on circle size!
        for(float t = 0; t < 2 * 3.14159265359f; t += 0.1f)
        {
            counter++;
            float cX = radius * cos(t) + circles[i][0];
            float cY = radius * sin(t) + circles[i][1];

            if(dt.at<float>(cY,cX) < maxInlierDist) 
            {
                inlier++;
                cv::circle(cv::Scalar(255,0,0), cv::Point2i(cX,cY),3, cv::Scalar(255,0,0));
                // Green: 0, 255, 0
            } 
            else
            {
                cv::circle(cv::Scalar(255,0,0), cv::Point2i(cX,cY),3, cv::Scalar(255,0,0));
                // Green: 0, 255, 0
            }
        }
        LOG(INFO) << 100.0f*(float)inlier/(float)counter << " % of a circle with radius " << radius << " detected" << std::endl;
    }

    return circlesMatrix;
}