// WoodenBoltProduct.hh
#pragma once

#include <string>

#include "Product.hh"

#include "../parser/JSONParser.hh"



namespace products
{

    class WoodenBoltProduct : public Product
    {
        public:

        WoodenBoltProduct() = default;


        WoodenBoltProduct(std::string imageName, std::string boltNumber, bool isFaulty, std::string numberOfFaults, std::string timeProcessed) 
        : productId(imageName), numberOfBolts(boltNumber), isFaulty(isFaulty), numberOfFaults(numberOfFaults), timeProcessed(timeProcessed)
        {  
            LOG(INFO) << "Product JSON: " << productJSON;
            LOG(INFO) << "Time Processed: " << timeProcessed;
            parser::JSONParser parser(productId, productJSON);
        }

        private: 
        std::string productId, numberOfBolts, numberOfFaults;
        bool isFaulty = false;
        std::string timeProcessed;
        const std::string quote = "\"";

        inline std::string BoolToString(bool boolValue)
        {
            // Using Ternary Operators.
            return boolValue ? "true" : "false";
        }

        inline std::string GetProductJSON()
        {
            return productJSON;
        }

        std::string productJSON = 
        "{\n\t\"ProductID\": " + quote + productId + quote + "," +
            "\n\t\"ImageDetails\": " + "{" +
            "\n\t\t\"NumberOfBolts\": " + numberOfBolts + "," +
            "\n\t\t\"IsFaulty\": " + BoolToString(isFaulty) + "," +
            "\n\t\t\"NumberOfFaults\": " + numberOfFaults + "," +
            "\n\t\t\"TimeProcessed\": " + quote + timeProcessed + quote + 
            "\n\t}"
        "\n}";    
    };
}

/*
{
    "ProductID": "",
    "ImageDetails:": {
      "NumberOfBolts": "",
      "IsFaulty": "",
      "numberOfFaults": ""
    }
}

*/