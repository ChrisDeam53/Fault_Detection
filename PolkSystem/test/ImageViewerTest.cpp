#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include "../src/utils/ImageViewer.hh"

/////////////////////////////////////////////////////////////////////////////////////
/// @test TestCVImread
/// @brief True when a valid image & pathfile is provided.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageViewerTest, TestCVImread)
{
    utils::ImageViewer viewer;
    cv::Mat image = cv::imread("../images/token_1.png", cv::IMREAD_COLOR);
    bool response = viewer.OpenImage(image);
    EXPECT_TRUE(response);
}