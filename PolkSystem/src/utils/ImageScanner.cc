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

    cv::Mat segmentColoursMat = SegmentImageColours(tempImage, inputImage);

    cv::Mat edgeDetectionMat = ApplyEdgeDetection(inputImage);

    // Returns the tresholded image - The wooden blocks are white.
    cv::Mat thresholdMat = ThresholdWoodenShape(inputImage);

    cv::Mat bitwiseAndMat;

    // Need to convert the mask into a 3 channel image as the original image is a 3 channel image.
    cv::Mat convertedMat;
    cv::cvtColor(thresholdMat, convertedMat, cv::COLOR_GRAY2BGR, 3);

    LOG(INFO) << "";
    LOG(INFO) << "";
    LOG(INFO) << "Number of channels in thresholdMat: " << thresholdMat.channels();
    LOG(INFO) << "Number of channels in inputImage: " << inputImage.channels();
    LOG(INFO) << "Number of channels in convertedMat: " << convertedMat.channels();

    // Applies the mask atop the original image.
    ConvertImageToRgb(inputImage);
    cv::bitwise_and(inputImage, convertedMat, bitwiseAndMat);

    cv::Mat edgeMat = ApplyEdgeDetection(bitwiseAndMat);

    cv::Mat newMaskMat;
    cv::bitwise_and(edgeDetectionMat, edgeMat, newMaskMat);

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

    // cv::Mat kMeansMat;
    // kMeansMat = ApplyKMeansAlgorithm(inputImage);
    // return kMeansMat;

    // Temp testing
    // cv::Mat appliedTemplateMat;
    // appliedTemplateMat = ApplyTemplate(bitwiseAndMat);
    // appliedTemplateMat = ApplyTemplate(inputImage);
    // return appliedTemplateMat;

    // Returns only the circles on black background. (Green Circles)
    // return rgbImageMatrix;

    // return edgeMat;
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

    // MEDIAN BLUR
    // cv::medianBlur(circlesMatrix, circlesMatrix, 3);

    // Smooth the image - Prevent false circles detection.
    cv::GaussianBlur(circlesMatrix, circlesMatrix, cv::Size(9, 9), 2, 2);

    // Apply HoughCircles to find circles.
    cv::HoughCircles(circlesMatrix, circles, cv::HOUGH_GRADIENT, 1, 60, 200, 20, 0, 0);

    LOG(INFO) << "--eeee- circles.size() " << circles.size();

    const int imageWidth = circlesMatrix.cols;
    const int imageHeight = circlesMatrix.rows;
    LOG(INFO) << "Width : " << imageWidth;
    LOG(INFO) << "Height: " << imageHeight;

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


        // Draw text to show *which* bolt has been detected.
        
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

    LOG(INFO) << "rgbImage: " <<  rgbImage.depth();
    LOG(INFO) << "greyscaleImage: " <<  greyscaleImage.depth();
    LOG(INFO) << "matchingResultTemplate: " <<  matchingResultTemplate.depth();

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


    cv::Mat labels;
    double compactness;

    cv::Scalar colourTab[] =
    {
        cv::Scalar(0, 0, 255),
        cv::Scalar(0,255,0),
        cv::Scalar(255,100,100),
        cv::Scalar(255,0,255),
        cv::Scalar(0,255,255)
    };

    std::vector<cv::Point2f> centers;

    cv::Mat matPoints(50, 1, CV_32FC2);

    // compactness = cv::kmeans(matPoints, 4, labels, cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0), 3, cv::KMEANS_USE_INITIAL_LABELS, centers);

    compactness = cv::kmeans(matPoints, 4, labels, cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0), 30, cv::KMEANS_PP_CENTERS , cv::noArray());

    // May need to comment out:
    rgbImage = cv::Scalar::all(0);

    for (int i = 0; i < centers.size(); i++)
    {
        cv::Point2f centre = centers[i];
        cv::circle(rgbImage, centre, 40, colourTab[i], 1, cv::LINE_AA);
    }
    
    return rgbImage;


    /////////////////////////////////////////////////////

    // const int MAX_CLUSTERS = 5;
    // cv::Scalar colorTab[] =
    // {
    //     cv::Scalar(0, 0, 255),
    //     cv::Scalar(0,255,0),
    //     cv::Scalar(255,100,100),
    //     cv::Scalar(255,0,255),
    //     cv::Scalar(0,255,255)
    // };
    // cv::Mat img(500, 500, CV_8UC3);
    // cv::RNG rng(12345);

    // int k, clusterCount = rng.uniform(2, MAX_CLUSTERS+1);
    // int i, sampleCount = rng.uniform(1, 1001);
    // cv::Mat points(sampleCount, 1, CV_32FC2), labels;

    // clusterCount = MIN(clusterCount, sampleCount);



    // std::vector<cv::Point2f> centers;
    // /* generate random sample from multigaussian distribution */
    // for( k = 0; k < clusterCount; k++ )
    // {
    //     cv::Point center;
    //     center.x = rng.uniform(0, img.cols);
    //     center.y = rng.uniform(0, img.rows);
    //     cv::Mat pointChunk = points.rowRange(k*sampleCount/clusterCount,
    //                                         k == clusterCount - 1 ? sampleCount :
    //                                         (k+1)*sampleCount/clusterCount);
    //     rng.fill(pointChunk, cv::RNG::NORMAL, cv::Scalar(center.x, center.y), cv::Scalar(img.cols*0.05, img.rows*0.05));
    // }
    // cv::randShuffle(points, 1, &rng);
    
    // double compactness = kmeans(points, clusterCount, labels,
    //     cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0),
    //         3, cv::KMEANS_PP_CENTERS, centers);

    // img = cv::Scalar::all(0);

    // for(int i = 0; i < sampleCount; i++)
    // {
    //     int clusterIdx = labels.at<int>(i);
    //     cv::Point ipt = points.at<cv::Point2f>(i);
    //     cv::circle( img, ipt, 2, colorTab[clusterIdx], cv::FILLED, cv::LINE_AA );
    // }

    // for (int i = 0; i < (int)centers.size(); ++i)
    // {
    //     cv::Point2f c = centers[i];
    //     circle( img, c, 40, colorTab[i], 1, cv::LINE_AA );
    // }


    // return img;

}