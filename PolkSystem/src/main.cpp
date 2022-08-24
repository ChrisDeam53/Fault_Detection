#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include "utils/ImageViewer.hh"

int main(int, char**)
{
    cv::Mat image = cv::imread("../images/Test_Image_2.jpg", cv::IMREAD_COLOR);
    utils::ImageViewer viewer;
    viewer.OpenImage(image);

    
    return 0;
}