#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include "../src/utils/ImageScanner.hh"

/////////////////////////////////////////////////////////////////////////////////////
/// @test Test that the correct number of bolts are found.
/// @brief Expect true when a correct number ofg bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_Returned_With_2_Bolts)
{
    cv::Mat image = cv::imread("../images/IMG_1679.JPG", cv::IMREAD_COLOR);

    utils::ImageScanner scanner(image);

    const int BOLT_COUNT_EXPECTED = 2;

    int numberOfFoundBolts = scanner.GetProductBoltCirclesFound().size();

    bool correctNumberOfBoltsFound = false;

    if(numberOfFoundBolts == BOLT_COUNT_EXPECTED)
    {
        correctNumberOfBoltsFound = true;
    }

    EXPECT_TRUE(correctNumberOfBoltsFound);
}

/////////////////////////////////////////////////////////////////////////////////////
/// @test Test that the correct number of bolts are found.
/// @brief Expect true when a correct number ofg bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_Returned_With_4_Bolts)
{
    cv::Mat image = cv::imread("../images/IMG_1668.JPG", cv::IMREAD_COLOR);

    utils::ImageScanner scanner(image);

    const int BOLT_COUNT_EXPECTED = 4;

    int numberOfFoundBolts = scanner.GetProductBoltCirclesFound().size();

    bool correctNumberOfBoltsFound = false;

    if(numberOfFoundBolts == BOLT_COUNT_EXPECTED)
    {
        correctNumberOfBoltsFound = true;
    }

    EXPECT_TRUE(correctNumberOfBoltsFound);
}

/////////////////////////////////////////////////////////////////////////////////////
/// @test Test that the correct number of bolts are found.
/// @brief Expect true when a correct number ofg bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_Returned_With_4_Bolts_Ver_2)
{
    cv::Mat image = cv::imread("../images/IMG_1950.JPG", cv::IMREAD_COLOR);

    utils::ImageScanner scanner(image);

    const int BOLT_COUNT_EXPECTED = 5;

    int numberOfFoundBolts = scanner.GetProductBoltCirclesFound().size();

    bool correctNumberOfBoltsFound = false;

    if(numberOfFoundBolts == BOLT_COUNT_EXPECTED)
    {
        correctNumberOfBoltsFound = true;
    }

    EXPECT_TRUE(correctNumberOfBoltsFound);
}





