// ImageViewer.cpp
#include "ImageViewer.hh"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

using namespace utils;

int ImageViewer::OpenImage()
{
    cv::Mat img = cv::imread("/src/utils/token_1.png", cv::IMREAD_COLOR);

    if(img.empty())
    {
        std::cout << "Could not read the image." << std::endl;
        return 1;
    }

    cv::imwrite("/src/utils/test.png", img);

    return 0;
}