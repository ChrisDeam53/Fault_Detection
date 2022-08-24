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
        /// @brief Default constructor.
        ////////////////////////////////////////////////////////////////////////////////////
        ImageScanner() = default;

        ////////////////////////////////////////////////////////////////////////////////////
        /// @brief ImageScanner constructor. 
        /// @param image - OpenCV matrix object | The image to be checked.
        ////////////////////////////////////////////////////////////////////////////////////
        ImageScanner(cv::Mat image) : inputImage(image), HSVLowerValue{0,0,0}, HSVUpperValue{0, 0, 75}
        {
            scannedImage = FindHsvValues(inputImage, HSVLowerValue, HSVUpperValue);
        };

        ////////////////////////////////////////////////////////////////////////////////////
        /// @brief Coverts normal RGB image supplied into HSV.
        /// @param inputImage - OpenCV matrix object.
        ////////////////////////////////////////////////////////////////////////////////////
        void CovertImageToHsv(cv::Mat image);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @brief Returns the updated image matrix.
        /// @return inputImage - OpenCV matrix object.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat getImage()
        {
            return scannedImage;
        }

        private:
        cv::Mat inputImage;
        cv::Mat scannedImage;

        /// Threshold values - Allowing for a range of HSV values to encompass a the colour range.
        // Hue, Saturation, Value.
        const cv::Scalar HSVLowerValue; // Original: 0,0,38
        const cv::Scalar HSVUpperValue; // Original: 0,0, 75

        ////////////////////////////////////////////////////////////////////////////////////
        /// @brief Used to find the HSV values for bolts within the image.
        /// @param inputImage - OpenCV matrix object.
        /// @param HSVLowerValue - CV Scalar object - 4-element vector containing the HSV lower bounds.
        /// @param HSVUpperValue - CV Scalar object - 4-element vector containing the HSV upper bounds.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat FindHsvValues(cv::Mat inputImage, const cv::Scalar HSVLowerValue, const cv::Scalar HSVUpperValue);
        
    };

}