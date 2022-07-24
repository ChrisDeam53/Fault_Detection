// ImageViewer.hh
#pragma once

#include <opencv2/core/mat.hpp>

namespace utils
{
    class ImageViewer
    {
        public:
        /////////////////////////////////////////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @brief Reads an image that is supplied & writes the image into "/images".
        /// @param image - OpenCV matrix object. Image to be passed in from "/images".
        /// @return True if image can be opened.
        /////////////////////////////////////////////////////////////////////////////////////
        bool OpenImage(cv::Mat image); 

        private:
    };
}