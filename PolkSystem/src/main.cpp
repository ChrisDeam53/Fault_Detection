#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <glog/logging.h>

#include "utils/ImageViewer.hh"
#include "utils/ImageScanner.hh"

#include "products/WoodenBoltProduct.hh"

// Temp includes
#include "parser/JSONParser.hh"

int main(int, char**)
{
    cv::Mat image = cv::imread("../images/IMG_1679.JPG", cv::IMREAD_COLOR);
    utils::ImageViewer viewer;
    utils::ImageScanner scanner(image);
    // std::string imageName, std::string boltNumber, bool isFaulty, std::string numberOfFaults
    products::WoodenBoltProduct product("1671", "4", false, "0");

    // Instead of creating a JSON parser
    // Create a product that uses the JSON parser to write it's own JSON file.
    // JSON parser is only used to parse data into the JSON format.

    // NOTE: Could mention in report, that as shown in 1948, due to incorrect lighting
    // Algorithm works, but as paramaters change in the image, it is not fully effective

    // NOTE: I have changed the canny Edge algorithm params
    // Winners: 1668 | 1669 | 1679 | 1949 | 1950 | 1952
    // 
    // Stats:
    // 1668: 4/4, 100% | 1669: 4/4, 100% NOTE: May need to tweak to remove BIG circles
    // 1670: 2/4, 50% | 1671: 1/4, 25% | 1672: 2/4, 50% | 1673: 1/4, 25% | 1674: 1/4, 25% 
    // 1675: 1/3, 33% | 1676: 0/3, 0% | 1677: 1/3, 33% | 1678: 2/3, 66% | 1679: 2/2, 100% [LOOK AT MISSING BOLTS DETECTION]
    // 1680: 1/2, 50% | 1681: 1/2, 50% | 1682: 1/2, 50% | 1947, 2/4, 50% | 1948: 2/4, 50% | 
    // 1949: 4/4, 100% | 1950: 4/4, 100% | 1951: 3/4, 75% | 1952: 4/4, 100% [Although barely]

    if(viewer.OpenImage(image))
    {
        cv::Mat updatedImage = scanner.getImage();
        viewer.WriteImage(updatedImage, "Output_Test_51_KMeans_Test");
        LOG(INFO) << "<:: - Program Executed - ::>";
    }

    return 0;
}