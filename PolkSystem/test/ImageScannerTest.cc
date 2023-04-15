#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <glog/logging.h>
#include "../src/utils/ImageScanner.hh"


/////////////////////////////////////////////////////////////////////////////////////
/// @test Test that the correct number of bolts are found.
/// @brief Expect true when a correct number of bolts are found.
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
/// @brief Expect true when a correct number of bolts are found.
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
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_Returned_With_2_Bolts_Ver_4)
{
    cv::Mat image = cv::imread("../images/IMG_1950.JPG", cv::IMREAD_COLOR);

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
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_With_No_Product)
{
    cv::Mat image = cv::imread("../images/IMG_2189.JPG", cv::IMREAD_COLOR);

    utils::ImageScanner scanner(image);

    const int BOLT_COUNT_EXPECTED = 0;

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
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_With_Diagonal_Product)
{
    cv::Mat image = cv::imread("../images/IMG_2193.JPG", cv::IMREAD_COLOR);

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
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_With_3_Bolts)
{
    cv::Mat image = cv::imread("../images/IMG_2152.JPG", cv::IMREAD_COLOR);

    utils::ImageScanner scanner(image);

    const int BOLT_COUNT_EXPECTED = 3;

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
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_With_3_Bolts_Ver_2)
{
    cv::Mat image = cv::imread("../images/IMG_2153.JPG", cv::IMREAD_COLOR);

    utils::ImageScanner scanner(image);

    const int BOLT_COUNT_EXPECTED = 3;

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
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_Returned_With_4_Bolts_Ver_4)
{
    cv::Mat image = cv::imread("../images/IMG_1669.JPG", cv::IMREAD_COLOR);

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
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_Returned_With_4_Bolts_Ver_3)
{
    cv::Mat image = cv::imread("../images/IMG_1949.JPG", cv::IMREAD_COLOR);

    utils::ImageScanner scanner(image);

    const int BOLT_COUNT_EXPECTED = 4;

    int numberOfFoundBolts = scanner.GetProductBoltCirclesFound().size();

    LOG(INFO) << "Ver_3" << numberOfFoundBolts;

    bool correctNumberOfBoltsFound = false;

    if(numberOfFoundBolts == BOLT_COUNT_EXPECTED)
    {
        correctNumberOfBoltsFound = true;
    }

    EXPECT_TRUE(correctNumberOfBoltsFound);
}

/////////////////////////////////////////////////////////////////////////////////////
/// @test Test that the correct number of bolts are found.
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_Returned_With_2_Bolts_Ver_2)
{
    cv::Mat image = cv::imread("../images/IMG_2122.JPG", cv::IMREAD_COLOR);

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
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_Returned_With_3_Bolts_Ver_3)
{
    cv::Mat image = cv::imread("../images/IMG_2126.JPG", cv::IMREAD_COLOR);

    utils::ImageScanner scanner(image);

    const int BOLT_COUNT_EXPECTED = 3;

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
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_Returned_With_2_Bolts_Ver_3)
{
    cv::Mat image = cv::imread("../images/IMG_2128.JPG", cv::IMREAD_COLOR);

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
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_Returned_With_3_Bolts_Ver_4)
{
    cv::Mat image = cv::imread("../images/IMG_2131.JPG", cv::IMREAD_COLOR);

    utils::ImageScanner scanner(image);

    const int BOLT_COUNT_EXPECTED = 3;

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
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_Returned_With_0_Bolts)
{
    cv::Mat image = cv::imread("../images/Test_Image_2.jpg", cv::IMREAD_COLOR);

    utils::ImageScanner scanner(image);

    const int BOLT_COUNT_EXPECTED = 0;

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
/// @brief Expect true when a correct number of bolts are found.
/////////////////////////////////////////////////////////////////////////////////////
TEST(ImageScannerTest, Test_Correct_Number_Of_Bolts_Returned_With_0_Bolts_Ver_2)
{
    cv::Mat image = cv::imread("../images/IMG_2141.JPG", cv::IMREAD_COLOR);

    utils::ImageScanner scanner(image);

    const int BOLT_COUNT_EXPECTED = 0;

    int numberOfFoundBolts = scanner.GetProductBoltCirclesFound().size();

    bool correctNumberOfBoltsFound = false;

    if(numberOfFoundBolts == BOLT_COUNT_EXPECTED)
    {
        correctNumberOfBoltsFound = true;
    }

    EXPECT_TRUE(correctNumberOfBoltsFound);
}
