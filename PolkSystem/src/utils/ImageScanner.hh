// ImageScanner.hh
#pragma once

#include <opencv2/core/mat.hpp>

namespace utils
{

    class ImageScanner
    {
        ////////////////////////////////////////////////////////////////////////////////////
        /// @brief Deals with image processing.
        ////////////////////////////////////////////////////////////////////////////////////

        public:

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Default constructor.
        ////////////////////////////////////////////////////////////////////////////////////
        ImageScanner() = default;

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief ImageScanner constructor. 
        /// @param image - OpenCV matrix object | The image to be checked.
        ////////////////////////////////////////////////////////////////////////////////////
        // Original: HSVLowerValue{0,0,38}, HSVUpperValue{0, 0, 75}
        // Upper Value: Lighter
        // HSVLowerValue{39,46.8,35.3}, HSVUpperValue{30,31.2,37.6}
        ImageScanner(cv::Mat image) : inputImage(image), HSVLowerValue{39,46.8,35.3}, HSVUpperValue{30,31.2,37.6}
        {
            scannedImage = FindHsvValues(inputImage, HSVLowerValue, HSVUpperValue);
        };

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Coverts normal RGB image supplied into HSV.
        /// @note Method exists in anticipation that more steps may be required.
        /// @param inputImage - OpenCV matrix object.
        ////////////////////////////////////////////////////////////////////////////////////
        void ConvertImageToHsv(cv::Mat image);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Converts the supplied HSV image into the original state (RGB).
        /// @note Method exists in anticipation that more steps may be required.
        /// @param inputImage - OpenCV matrix object.
        ////////////////////////////////////////////////////////////////////////////////////
        void ConvertImageToRgb(cv::Mat image);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Returns the updated image matrix.
        /// @return scannedImage - OpenCV matrix object.
        ////////////////////////////////////////////////////////////////////////////////////
        inline cv::Mat getImage()
        {
            return scannedImage;
        } 

        private:
        // Image provided by the system.
        cv::Mat inputImage;
        // Image returned by the system - With applied results.
        cv::Mat scannedImage;
        cv::Mat nonZeroCoordinates;

        // Total count of bolts for the product.
        int productBoltCount;
        // Total count of bolts expected.
        const int  productBoltCountExpected = 4;
        // Total count of missing bolts.
        int productBoltMissingCount;
        // True if product is faulty.
        bool isProductFaulty;

        // Vector of Vec3f circles to store found bolts.
        std::vector<cv::Vec3f> productBoltCirclesFound;

        /// Threshold values - Allowing for a range of HSV values to encompass a the colour range.
        /// Hue, Saturation, Value.
        const cv::Scalar HSVLowerValue; // Original: 0,0,38 - Dark Grey
        const cv::Scalar HSVUpperValue; // Original: 0,0, 75 - Light Grey

        // Light on bolt: rgba(246,244,233,255) -> hsv(51, 5.3, 96.5)
        // Slightly darker bolt with light: rgba(173,166,153,255) -> hsv(39, 11.6, 67.8)

        // Silver rgb(191,191,191) -> hsv(0, 0, 75)

        // Light Silver: rgb(224,224,224) -> hsv(0, 0, 87.8)
        // Dark Silver:  rgb(160,160,160) -> hsv(0, 0, 62.7)

        // Light Brown[Shadow] rgb(96,81,66) -> hsv(30,31.2,37.6)
        // Dark Brown[Shadow]  rgb(90,75,48) -> hsv(39,46.8,35.3)

        // Light Brown[Wood] rgb(235,218,181) -> hsv(41,23,92.2)
        // Dark Brown[Wood]  rgb(200,176,132) -> hsv(39,34,78.4)


        // LightGray rgb(211,211,211) -> hsv(0, 0, 82.7)
        // Gray      rgb(128,128,128) -> hsv(0, 0, 50.2)

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Used to find the HSV values for bolts within the image.
        /// @param inputImage - OpenCV matrix object.
        /// @param HSVLowerValue - CV Scalar object - 4-element vector containing the HSV lower bounds.
        /// @param HSVUpperValue - CV Scalar object - 4-element vector containing the HSV upper bounds.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat FindHsvValues(cv::Mat inputImage, const cv::Scalar HSVLowerValue, const cv::Scalar HSVUpperValue);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Method to draw & outline specific areas on the returned image.
        /// @param hsvImage - OpenCV matrix object - HSV Image.
        /// @note Will want to get coordinates of white pixels: https://stackoverflow.com/questions/34978705/get-coordinates-of-white-pixels-opencv
        /// @note 255 is returned from FindHsvValues if the colour is in range, 0 if it is not.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat GetHsvPixelLocation(cv::Mat hsvImage);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Returns the updated image matrix.
        /// @param hsvImage - OpenCV matrix object.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat SegmentImageColours(cv::Mat colourImage, cv::Mat hsvImage);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Applies edge-detection algorithms.
        /// @param hsvImage - OpenCV matrix object a containing a HSV image.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat ApplyEdgeDetection(cv::Mat hsvImage);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Applies edge-detection algorithms - Attempts to detect the shape.
        /// @param hsvImage - OpenCV matrix object a containing an RGB image.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat ThresholdWoodenShape(cv::Mat rgbImage);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Applies erosion against the image, removing noise along the edges.
        /// @param hsvImage - OpenCV matrix object containing a HSV image.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat ApplyErosion(cv::Mat inputImage);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Applies HoughCircles algorithm on edges that are assumed to be circles.
        /// @param edgeMat - OpenCV matrix object containing a binary image.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat DetectBoltCircles(cv::Mat edgeMat);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Applies HoughCircles algorithm on edges that are assumed to be circles.
        /// @param rgbMat - OpenCV matrix object containing an RGB image with reduced ROI.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat DetectMissingBoltCircles(cv::Mat rgbMat);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Applies Template Matching against bolts in an image.
        /// @param hsvImage - OpenCV matrix object containing a HSV image.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat ApplyTemplate(cv::Mat rgbImage);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Applies the K-Means Algorithm against the image to reduce ROI.
        /// @param hsvImage - OpenCV matrix object containing a HSV image.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat ApplyKMeansAlgorithm(cv::Mat rgbImage);

        
        ////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Used to find the HSV values for bolts within the image.
        /// @param boltVector -Value containing the bolt positions.
        ////////////////////////////////////////////////////////////////////////////////////
        inline void SetProductBoltCirclesFound(std::vector<cv::Vec3f> boltVector)
        {
            productBoltCirclesFound = boltVector;
        }

        inline std::vector<cv::Vec3f> GetProductBoltCirclesFound()
        {
            return productBoltCirclesFound;
        }
    };

}