// Product.h
#pragma once

#include <string>

// Base Class.

namespace products
{

    class Product
    {
        public:
        void setProductInformation(int boltNumber)
        {
            numberOfBolts = std::to_string(boltNumber);
        }

        void createProductInformationJSONFile();

        private:
        std::string productId, numberOfBolts, IsFaulty, numberOfFaults;
        bool isFaulty = false;
        std::string setProductInformation();  
        std::string timeProcessed;
    };
}