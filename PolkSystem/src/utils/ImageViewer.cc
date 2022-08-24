// ImageViewer.cc
#include "ImageViewer.hh"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <glog/logging.h>

using namespace utils;

/////////////////////////////////////////////////////////////////////////////////////
bool ImageViewer::OpenImage(cv::Mat image)
{
    if(image.empty())
    {
        // Image matrix is empty.
        LOG(ERROR) << "Could not read the image.";
        return false;
    }

    const std::string filePath("../src/../images/writtenImage.png");
    cv::imwrite(filePath, image);
    LOG(INFO) << "Image has been written to: " << filePath;

    return true;
}
/////////////////////////////////////////////////////////////////////////////////////