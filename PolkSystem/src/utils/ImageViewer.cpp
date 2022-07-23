// ImageViewer.cpp
#include "ImageViewer.hh"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

using namespace utils;

/////////////////////////////////////////////////////////////////////////////////////
bool ImageViewer::OpenImage()
{
    cv::Mat img = cv::imread("token_1.png", cv::IMREAD_COLOR);

    if(img.empty())
    {
        std::cout << "Could not read the image." << std::endl;
        return false;
    }

    cv::imwrite("../../images/test.png", img);

    return true;
}
/////////////////////////////////////////////////////////////////////////////////////