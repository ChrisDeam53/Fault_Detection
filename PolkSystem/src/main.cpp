#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <glog/logging.h>

#include "utils/ImageViewer.hh"
#include "utils/ImageScanner.hh"

int main(int, char**)
{
    cv::Mat image = cv::imread("../images/Test_Image_2.jpg", cv::IMREAD_COLOR);
    utils::ImageViewer viewer;
    utils::ImageScanner scanner(image);

     if(viewer.OpenImage(image))
     {
        cv::Mat updatedImage = scanner.getImage();
        viewer.WriteImage(updatedImage, "Output_Test_2");
     }

    return 0;
}