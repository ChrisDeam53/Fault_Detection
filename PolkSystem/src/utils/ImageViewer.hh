// ImageViewer.hh
#pragma once

#include <stdio.h>

namespace utils
{
    class ImageViewer
    {
        public:

        //////////////////////////////////////////////////
        /// @author Christopher Deam.
        /// @note OpenImage.
        /// @brief Reads an image that is supplied & writes the image into "/images".
        /// @return True if image can be opened.
        //////////////////////////////////////////////////
        bool OpenImage(); 

        private:
    };
}