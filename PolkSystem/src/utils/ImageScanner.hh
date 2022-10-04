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
        /// @note Method exists in anticipation that more steps may be required.
        /// @param inputImage - OpenCV matrix object.
        ////////////////////////////////////////////////////////////////////////////////////
        void ConvertImageToHsv(cv::Mat image);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @brief Converts the supplied HSV image into the original state (RGB).
        /// @note Method exists in anticipation that more steps may be required.
        /// @param inputImage - OpenCV matrix object.
        ////////////////////////////////////////////////////////////////////////////////////
        void ConvertImageToRgb(cv::Mat image);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @brief Returns the updated image matrix.
        /// @return scannedImage - OpenCV matrix object.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat getImage()
        {
            return scannedImage;
        }

        private:
        cv::Mat inputImage;
        cv::Mat scannedImage;
        cv::Mat nonZeroCoordinates;

        /// Threshold values - Allowing for a range of HSV values to encompass a the colour range.
        /// Hue, Saturation, Value.
        const cv::Scalar HSVLowerValue; // Original: 0,0,38 - Dark Grey
        const cv::Scalar HSVUpperValue; // Original: 0,0, 75 - Light Grey

        ////////////////////////////////////////////////////////////////////////////////////
        /// @brief Used to find the HSV values for bolts within the image.
        /// @param inputImage - OpenCV matrix object.
        /// @param HSVLowerValue - CV Scalar object - 4-element vector containing the HSV lower bounds.
        /// @param HSVUpperValue - CV Scalar object - 4-element vector containing the HSV upper bounds.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat FindHsvValues(cv::Mat inputImage, const cv::Scalar HSVLowerValue, const cv::Scalar HSVUpperValue);

        ////////////////////////////////////////////////////////////////////////////////////
        /// @brief Method to draw & outline specific areas on the returned image.
        /// @param hsvImage - OpenCV matrix object.
        /// @note Will want to get coordinates of white pixels: https://stackoverflow.com/questions/34978705/get-coordinates-of-white-pixels-opencv
        /// @note 255 is returned from FindHsvValues if the colour is in range, 0 if it is not.
        ////////////////////////////////////////////////////////////////////////////////////
        cv::Mat GetHsvPixelLocation(cv::Mat hsvImage);
    };

}