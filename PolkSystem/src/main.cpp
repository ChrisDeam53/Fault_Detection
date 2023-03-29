#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <glog/logging.h>

#include "utils/ImageViewer.hh"
#include "utils/ImageScanner.hh"

#include "products/WoodenBoltProduct.hh"
#include <string>

#include "parser/JSONParser.hh"

int main(int, char**)
{
    cv::Mat image = cv::imread("../images/IMG_2122.JPG", cv::IMREAD_COLOR);
    utils::ImageViewer viewer;
    utils::ImageScanner scanner(image);

    const int NUMBER_OF_BOLTS_EXPECTED = 4;
    int numberOfFoundBolts = scanner.GetProductBoltCirclesFound().size();
    bool isFaulty = false;
    if(numberOfFoundBolts != 4)
    {
        isFaulty = true;
    }
    int numberOfFaults = NUMBER_OF_BOLTS_EXPECTED - numberOfFoundBolts;
    std::string imageProcessTime = scanner.GetImageTimeProcessed();
    imageProcessTime.erase(std::remove(imageProcessTime.begin(), imageProcessTime.end(), '\n'), imageProcessTime.cend());

    if(viewer.OpenImage(image))
    {
        cv::Mat updatedImage = scanner.getImage();
        viewer.WriteImage(updatedImage, "IMG_2122.JPG_Scanned");

        std::string imageName = viewer.GetImageName();
        products::WoodenBoltProduct product(imageName, std::to_string(numberOfFoundBolts), isFaulty, std::to_string(numberOfFaults), imageProcessTime);

        LOG(INFO) << "<:: - Program Executed - ::>";
    }

    return 0;
}