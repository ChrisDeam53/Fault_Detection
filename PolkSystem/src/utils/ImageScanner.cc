// ImageScanner.cc
#include "ImageScanner.hh"

#include <opencv2/imgproc/imgproc.hpp>
#include <glog/logging.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/fast_math.hpp>
#include <vector>

// Temp imread:
#include <opencv2/imgcodecs.hpp>
// Temp cv::TermCriteria
#include <opencv2/core/types.hpp>

// Temp CV_32FC2
#include <opencv2/core/hal/interface.h>

// Will need to eventually remove for MIN
#include <opencv2/core/cvdef.h>

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

    cv::Mat nonZeroCoords = GetHsvPixelLocation(combinedChannels);

    // Uncomment when required
    // for(int i = 0; i < nonZeroCoords.total(); i++)
    // {
    //     LOG(INFO) << "Non-Zero#" << i << ": " << nonZeroCoords.at<cv::Point>(i).x << ", " << nonZeroCoords.at<cv::Point>(i).y;
    //     cv::Point tempPoint(nonZeroCoords.at<cv::Point>(i).x, nonZeroCoords.at<cv::Point>(i).y);
    //     cv::circle(tempImage, tempPoint, 10, cv::Scalar(255,0,0), 1);
    // }

    cv::Mat segmentColoursMat = SegmentImageColours(tempImage, inputImage);

    cv::Mat edgeDetectionMat = ApplyEdgeDetection(inputImage);

    // Returns the tresholded image - The wooden blocks are white.
    cv::Mat thresholdMat = ThresholdWoodenShape(inputImage);

    cv::Mat bitwiseAndMat;

    // Need to convert the mask into a 3 channel image as the original image is a 3 channel image.
    cv::Mat convertedMat;
    cv::cvtColor(thresholdMat, convertedMat, cv::COLOR_GRAY2BGR, 3);

    // Applies the mask atop the original image.
    ConvertImageToRgb(inputImage);
    cv::bitwise_and(inputImage, convertedMat, bitwiseAndMat);

    cv::Mat edgeMat = ApplyEdgeDetection(bitwiseAndMat);

    cv::Mat newMaskMat;
    cv::bitwise_and(edgeDetectionMat, edgeMat, newMaskMat);

    cv::Mat circlesMatrix;
    circlesMatrix = DetectBoltCircles(newMaskMat);

    cv::Mat rgbImageMatrix;
    cv::cvtColor(circlesMatrix, rgbImageMatrix, cv::COLOR_GRAY2BGR, 3);

    // Covert all white contour lines to green.
    cv::Mat whiteToGreenMask;
    cv::inRange(rgbImageMatrix, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), whiteToGreenMask);
    rgbImageMatrix.setTo(cv::Scalar(0, 255, 0), whiteToGreenMask);

    // Impose the Green circles mask atop the original image.
    cv::Mat detectedBoltsMatrix;
    cv::bitwise_or(inputImage, rgbImageMatrix, detectedBoltsMatrix);
    // return detectedBoltsMatrix;

    // TESTING ONWARDS:
    cv::Mat colourSegmentMat;
    colourSegmentMat = SegmentImageColours(bitwiseAndMat, bitwiseAndMat);
    // return colourSegmentMat;

    // Works:
    cv::Mat kMeansMat;
    kMeansMat = ApplyKMeansAlgorithm(bitwiseAndMat);
    // return kMeansMat;

////////////////
    cv::blur(kMeansMat, kMeansMat, cv::Size(3,3), cv::Point(-1,-1), cv::BORDER_DEFAULT);

    cv::Mat greyMat;
    cv::cvtColor(kMeansMat, greyMat, cv::COLOR_BGR2GRAY);

    // cv::threshold(greyMat, greyMat, 200, 255, cv::THRESH_BINARY);
    cv::threshold(greyMat, greyMat, 160, 200, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(greyMat, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	cv::Scalar red(0,0,255);
    cv::Mat zerosMatrix = cv::Mat::zeros(greyMat.rows, greyMat.cols, CV_8UC3);
    cv::drawContours(zerosMatrix, contours, -1, red, 2);

    cv::Mat zerosMatrixCircles;

    // return zerosMatrix;
    return zerosMatrixCircles;

    // OG WORKING
	// cv::drawContours(kMeansMat, contours, -1, red, 2);
    // return kMeansMat;

////////////////

    // IDEA: Retrieve all and only the black pixels
    // Inverse that image, so they're white
    // HoughCircle on them

    // bitwiseAndMat returns ONLY the product in the image.
    // return bitwiseAndMat;

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
    cv::Mat3b img = colourImage;

    cv::Mat3b hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    cv::Mat1b mask;
    // Uses HSV Values.
    // Works with eye.
    // cv::inRange(hsv, cv::Scalar(2, 100, 65), cv::Scalar(12, 170, 100), mask);

    // NOTE: Code Inspired by: https://stackoverflow.com/questions/31760302/detect-brown-colour-object-using-opencv
    // Post from: Miki answered Aug 1, 2015 at 12:30.

    cv::Scalar lowerBrown = cv::Scalar(15,255,150);
    cv::Scalar upperBrown = cv::Scalar(26,88.5,54.5);

    cv::Scalar lowerBrown2 = cv::Scalar(0, 100, 20);
    cv::Scalar upperBrown2 = cv::Scalar(10, 255, 255);

    cv::Scalar whiteBoundary = cv::Scalar(0, 0, 0);
    cv::Scalar whiteBoundary2 = cv::Scalar(30, 100, 100);

    cv::inRange(hsv, whiteBoundary2, upperBrown, mask);

    cv::Mat1b kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(mask, contours, hierarchy, cv::RETR_TREE , cv::CHAIN_APPROX_SIMPLE);

    int idx_largest_blob = 0;
    if (contours.size() > 1)
    {
        int size_largest_blob = contours[0].size();

        for (int i = 0; i < contours.size(); ++i)
        {
            if (size_largest_blob < contours[i].size())
            {
                size_largest_blob = contours[i].size();
                idx_largest_blob = i;
            }
        }
    }

    cv::Mat3b res = img.clone();

    cv::drawContours(res, contours, idx_largest_blob, cv::Scalar(0, 255, 0));

    if(contours.size() > 0)
    {
        cv::RotatedRect r = cv::minAreaRect(contours[idx_largest_blob]);
    }
    
    cv::Point2f pts[4];

    // r.cv::points(pts);

    for (int j = 0; j < 4; ++j)
    {
        cv::line(res, pts[j], pts[(j + 1) % 4], cv::Scalar(0, 0, 255));
    }

    if(contours.size() > 0)
    {
        cv::Rect box = cv::boundingRect(contours[idx_largest_blob]);
        cv::rectangle(res, box, cv::Scalar(255, 0, 0));
    }

    return res;
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

    cv::Canny(hsvImage, cannyEdgeImage, 400, 700, 3, true);

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

    const std::string TOP_LEFT("Top Left");
    const std::string TOP_RIGHT("Top Right");
    const std::string BOTTOM_LEFT("Bottom Left");
    const std::string BOTTOM_RIGHT("Bottom Right");

    cv::findContours(edgeMat, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat zerosMatrix = cv::Mat::zeros(edgeMat.rows, edgeMat.cols, CV_8UC3);
    
    cv::drawContours(zerosMatrix, contours, hierarchy[0][1], cv::Scalar(255, 255, 255), 2);

    cv::cvtColor(zerosMatrix, circlesMatrix, cv::COLOR_BGR2GRAY, 1);

    // Median Blur.
    // cv::medianBlur(circlesMatrix, circlesMatrix, 3);

    // Smooth the image - Prevent false circles detection.
    cv::GaussianBlur(circlesMatrix, circlesMatrix, cv::Size(9, 9), 2, 2);

    // Apply HoughCircles to find circles.
    cv::HoughCircles(circlesMatrix, circles, cv::HOUGH_GRADIENT, 1, 60, 200, 20, 0, 0);

    LOG(INFO) << "--eeee- circles.size() " << circles.size();

    const int imageWidth = circlesMatrix.cols;
    const int imageHeight = circlesMatrix.rows;

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
        cv::circle(circlesMatrix, center, radius, cv::Scalar(255,0,0), 5, 8, 0 );

        // If x < 1/2 of imageWidth = Left 
        // If x > 1/2 of imageWidth = Right
        // If y < 1/2 of imageHeight = Top
        // If y < 1/2 of imageHeight = Bottom

        if(circles[i][0] < (imageWidth / 2))
        {
            // Left
            if(circles[i][1] < (imageHeight / 1.5))
            {
                // Top
                cv::Point textPoint(circles[i][0], circles[i][1]+100);
                cv::putText(circlesMatrix, TOP_LEFT, textPoint, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 5);
            }
            else
            {
                // Bottom
                cv::Point textPoint(circles[i][0], circles[i][1]+100);
                cv::putText(circlesMatrix, BOTTOM_LEFT, textPoint, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 5);
            }
 
        }
        else
        {
            // Right
            if(circles[i][1] < (imageHeight / 1.5))
            {
                // Top
                cv::Point textPoint(circles[i][0], circles[i][1]+100);
                cv::putText(circlesMatrix, TOP_RIGHT, textPoint, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 5);
            }
            else
            {
                // Bottom
                cv::Point textPoint(circles[i][0], circles[i][1]+100);
                cv::putText(circlesMatrix, BOTTOM_RIGHT, textPoint, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 5);
            }
        }   
    }

    ///////////////////////////////////

    // Compute distance transform.
    cv::Mat dt;
    cv::distanceTransform(255 - (edgeMat > 0), dt, cv::DIST_MASK_3 ,3);

    // Test for semi-circles.
    float minInlierDist = 2.0f;

    for(size_t i = 0; i < circles.size(); i++ ) 
    {
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

        // Perhaps look at circle size - If too big, do not include it.
        for(float t = 0; t < 2 * 3.14159265359f; t += 0.1f)
        {
            counter++;
            float cX = radius * cos(t) + circles[i][0];
            float cY = radius * sin(t) + circles[i][1];

            if(dt.at<float>(cY,cX) < maxInlierDist) 
            {
                inlier++;
                cv::circle(cv::Scalar(255,0,0), cv::Point2i(cX,cY),3, cv::Scalar(255,0,0));
            } 
            else
            {
                cv::circle(cv::Scalar(255,0,0), cv::Point2i(cX,cY),3, cv::Scalar(255,0,0));
            }
        }
        LOG(INFO) << 100.0f*(float)inlier/(float)counter << " % of a circle with radius " << radius << " detected" << std::endl;
    }

    return circlesMatrix;
}


/////////////////////////////////////////////////////////////////////////////////////
cv::Mat ImageScanner::DetectMissingBoltCircles(cv::Mat rgbMat)
{
    // box blur -> threshold -> canny -> contours:
    cv::Mat returnImage = cv::Mat::zeros(rgbMat.rows, rgbMat.cols, CV_8UC1);
    cv::Mat thresholdImage;
    cv::Mat blurredImage;
    cv::Mat cannyImage;
    cv::Mat gray;

    std::vector<cv::Vec3f> circles;

    cv::blur(rgbMat, blurredImage, cv::Size(18.7,18.7), cv::Point(-1, -1), cv::BORDER_DEFAULT);

    // Convert image to grayscale
    cv::cvtColor(rgbMat, gray, cv::COLOR_BGR2GRAY);
    // Convert image to binary

    cv::adaptiveThreshold(gray, blurredImage, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 111, 17);

    // cannyImage = ApplyEdgeDetection(blurredImage);
    cv::Canny(blurredImage, cannyImage, 100, 900, 3, true);

    // OG
    cv::threshold(blurredImage, thresholdImage, 1, 255, cv::THRESH_BINARY);

    // TEMP
    return thresholdImage;

    returnImage = ApplyEdgeDetection(thresholdImage);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // cv::findContours(returnImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(returnImage, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    return returnImage;
}


/////////////////////////////////////////////////////////////////////////////////////
cv::Mat ImageScanner::ApplyTemplate(cv::Mat rgbImage)
{
    // Notes:
    // UPDATE:
    // Perhaps have this method reutrn a CV::POINT
    // Then these are drawn in the constructor
    // Apply the scalar in the contructor too

    // UPDATE2:
    // OR I could get a vector of templates
    // Loop through each one, appent the points to that vector
    // Then loop through that vector and draw all rectangles

    const cv::Mat bolt01 = cv::imread("../images/Bolt_VER_1.PNG", cv::IMREAD_COLOR);
    const cv::Mat bolt02 = cv::imread("../images/Bolt_VER_2.PNG", cv::IMREAD_COLOR);
    const cv::Mat bolt03 = cv::imread("../images/Bolt_VER_3.PNG", cv::IMREAD_COLOR);
    const cv::Mat bolt04 = cv::imread("../images/Bolt_VER_4.PNG", cv::IMREAD_COLOR);

    cv::Mat imageArray[4];
    std::vector<cv::Point> pointsVector;
    
    cv::Mat greyscaleBolt01;
    cv::Mat greyscaleBolt02;
    cv::Mat greyscaleBolt03;
    cv::Mat greyscaleBolt04;

    cv::cvtColor(bolt01, greyscaleBolt01, cv::COLOR_BGR2GRAY);
    cv::cvtColor(bolt02, greyscaleBolt02, cv::COLOR_BGR2GRAY);
    cv::cvtColor(bolt03, greyscaleBolt03, cv::COLOR_BGR2GRAY);
    cv::cvtColor(bolt04, greyscaleBolt04, cv::COLOR_BGR2GRAY);

    imageArray[0] = greyscaleBolt01;
    imageArray[1] = greyscaleBolt02;
    imageArray[2] = greyscaleBolt03;
    imageArray[3] = greyscaleBolt04;

    cv::Mat matchingResultTemplate;
    cv::Mat greyscaleImage;

    double minVal;
    double maxVal;
    cv::Point minLocation;
    cv::Point maxLocation;
    cv::Point matchLocation;
    int columns =  rgbImage.cols - greyscaleBolt04.cols + 30;
    int rows = rgbImage.rows - greyscaleBolt04.rows + 30;

    cv::cvtColor(rgbImage, greyscaleImage, cv::COLOR_BGR2GRAY);

    // OG
    // cv::matchTemplate(greyscaleImage, greyscaleBolt04, matchingResultTemplate, cv::TM_SQDIFF, cv::noArray());

    cv::matchTemplate(greyscaleImage, greyscaleBolt03, matchingResultTemplate, cv::TM_CCOEFF_NORMED, cv::noArray());

    cv::normalize(matchingResultTemplate, matchingResultTemplate, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    cv::minMaxLoc(matchingResultTemplate, &minVal, &maxVal, &minLocation, &maxLocation, cv::noArray());

    // If using: cv::TM_SQDIFF or cv::TM_SQDIFF_NORMED 
    // matchLocation = minLocation;
    
    matchLocation = maxLocation;

    cv::rectangle(greyscaleImage, matchLocation, cv::Point(matchLocation.x + greyscaleBolt03.cols , matchLocation.y + greyscaleBolt03.rows), cv::Scalar::all(0), 2, 8, 0);

    cv::rectangle(matchingResultTemplate, matchLocation, cv::Point(matchLocation.x, matchLocation.y), cv::Scalar::all(0), 2, 8, 0);

    return greyscaleImage;
}


/////////////////////////////////////////////////////////////////////////////////////
cv::Mat ImageScanner::ApplyKMeansAlgorithm(cv::Mat rgbImage)
{
    // https://docs.opencv.org/3.4/d1/d5c/tutorial_py_kmeans_opencv.html
    // https://docs.opencv.org/3.4/d5/d38/group__core__cluster.html#ga9a34dc06c6ec9460e90860f15bcd2f88

    // NOTE:
    // Code inspired by: https://github.com/abubakr-shafique/K_Means_Clustering_CPP/blob/master/K_Means_Clustering.cpp

    // Change to float.
    cv::Mat samples(rgbImage.rows * rgbImage.cols, rgbImage.channels(), CV_32F);
    cv::Mat labels;
	int attempts = 5;
	cv::Mat centers;

	for(int y = 0; y < rgbImage.rows; y++)
    {
		for(int x = 0; x < rgbImage.cols; x++)
        {
            for(int z = 0; z < rgbImage.channels(); z++)
            {
				if (rgbImage.channels() == 3) {
					samples.at<float>(y + x * rgbImage.rows, z) = rgbImage.at<cv::Vec3b>(y, x)[z];
				}
				else
                {
                    samples.at<float>(y + x * rgbImage.rows, z) = rgbImage.at<uchar>(y, x);
				}
            }
        }
    }

    // 2nd Argument DEFAULT: 4 - WORKING
	cv::kmeans(samples, 4, labels, cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 10000, 0.0001), attempts, cv::KMEANS_PP_CENTERS, centers);

	cv::Mat newImage(rgbImage.size(), rgbImage.type());
	for (int y = 0; y < rgbImage.rows; y++)
    {
		for (int x = 0; x < rgbImage.cols; x++)
		{
			int clusterIDX = labels.at<int>(y + x * rgbImage.rows, 0);

			if (rgbImage.channels()==3)
            {
				for (int i = 0; i < rgbImage.channels(); i++)
                {
					newImage.at<cv::Vec3b>(y, x)[i] = centers.at<float>(clusterIDX, i);
				}
			}
			else
            {
				newImage.at<uchar>(y, x) = centers.at<float>(clusterIDX, 0);
			}
		}
    }
    
    return newImage;
}