#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include "../src/utils/ImageViewer.hh"


/////////////////////////////////////////////////////////////////////////////////////
/// @test Test OpenImage
/// @brief Expect true when a valid image & pathfile is provided.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageViewerTest, Test_OpenImage_With_Correct_Matrix)
{
    utils::ImageViewer viewer;
    cv::Mat image = cv::imread("../images/Test_Image_2.jpg", cv::IMREAD_COLOR);
    bool response = viewer.OpenImage(image);
    EXPECT_TRUE(response);
}

/////////////////////////////////////////////////////////////////////////////////////
/// @test Test OpenImage - Ensures false is returned if empty matrix provided.
/// @brief Expects an image to be written.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageViewerTest, Test_OpenImage_With_Empty_Matrix)
{
    utils::ImageViewer viewer;
    cv::Mat image = cv::imread("../images/NonExistentImageMatrix.jpg", cv::IMREAD_COLOR);
    bool response = viewer.OpenImage(image);
    EXPECT_FALSE(response);
}

/////////////////////////////////////////////////////////////////////////////////////
/// @test Test OpenImage
/// @brief Expects an image to be written.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageViewerTest, Test_WriteImage)
{
    utils::ImageViewer viewer;
    cv::Mat image = cv::imread("../images/eye.jpg", cv::IMREAD_COLOR);
    viewer.WriteImage(image, "Test_WriteImage");
    
    cv::Mat writtenImage = cv::imread("../images/scannedImages/Test_WriteImage.jpg", cv::IMREAD_COLOR);
    bool response = viewer.OpenImage(writtenImage);
    EXPECT_TRUE(response);
}

