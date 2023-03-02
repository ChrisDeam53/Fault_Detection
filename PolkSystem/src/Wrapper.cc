// Wrapper.cc
#pragma once
#include <math.h>

class Wrapper
{
public:
    
    ////////////////////////////////////////////////////////////////////////////////////
    /// @brief Coverts RGB values into HSV. Helper function for when calculations are needed on the fly.
    /// @note Method exists in anticipation that conversions will be required.
    /// @param red - Red Channel.
    /// @param green - Green Channel.
    /// @param blue - Blue Channel.
    ////////////////////////////////////////////////////////////////////////////////////
    float calculateRGBNumbersToHSV(double red, double green, double blue)
    {
        // Divide all values by 255 -> Range changes from 0->255 to 0->1.
        red = red / 255.0;
        green = green / 255.0;
        blue = blue / 255.0;

        // HSV - Hue Saturation Value.

        // Maximum of red, green, blue.
        double maxVal = std::max(red, std::max(green, blue));
        // Minimum of red, green, blue.
        double minVal = std::min(red, std::min(green, blue));
        // Difference between the maximum and minimum.
        double difference = maxVal - minVal;

        // Set hue & saturation to -1.
        double hue = -1;
        double saturation = -1;

        if (maxVal == minVal)
        {
            // If values are the same, then the hue = 0.
            hue = 0;
        }
        else if (maxVal == red)
        {
            // If maxVal == red then compute hue.
            hue = fmod(60 * ((green - blue) / difference) + 360, 360);
        }
        else if (maxVal == green)
        {
            // if maxVal == green then compute hue.
            hue = fmod(60 * ((blue - red) / difference) + 120, 360);
        }
        else if (maxVal == blue)
        {
            // If maxVal == b then compute hue.
            hue = fmod(60 * ((red - green) / difference) + 240, 360);
        }

        // Compute vue.
        double vue = maxVal * 100;
    }
};