// ImageViewer.hh
#pragma once

#include <opencv2/core/mat.hpp>

namespace utils
{
    class ImageViewer
    {
        ////////////////////////////////////////////////////////////////////////////////////
        /// @brief Open and deal with images.      
        ////////////////////////////////////////////////////////////////////////////////////
        
        public:
        /////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Reads an image that is supplied.
        /// @param image - OpenCV matrix object. Image to be passed in from "/images".
        /// @return True if image can be opened.
        /////////////////////////////////////////////////////////////////////////////////////
        bool OpenImage(cv::Mat image); 

        /////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Writes an image into "/images".
        /// @param image - OpenCV matrix object. Image to be passed in from "/images".
        /// @param imageName - Name supplied for image.
        /////////////////////////////////////////////////////////////////////////////////////
        void WriteImage(cv::Mat image, const std::string imageName);

        inline std::string GetImageName()
        {
            return imageName;
        }

        private:

        inline void SetImageName(std::string inputImageName)
        {
            imageName = inputImageName;
        }

        std::string imageName;
    };
}