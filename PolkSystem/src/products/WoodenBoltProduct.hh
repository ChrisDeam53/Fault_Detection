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


        WoodenBoltProduct(std::string imageName, std::string boltNumber, bool isFaulty, std::string numberOfFaults) : productId(imageName), numberOfBolts(boltNumber),
        isFaulty(isFaulty), numberOfFaults(numberOfFaults)
        {  
            LOG(INFO) << "Product JSON: " << productJSON;
            parser::JSONParser parser(productId, productJSON);
        }

        private: 
        std::string productId, numberOfBolts, numberOfFaults;
        bool isFaulty = false;

        inline std::string BoolToString(bool boolValue)
        {
            //using ternary operators
            return boolValue ? "true" : "false";
        }

        inline std::string GetProductJSON()
        {
            return productJSON;
        }

        std::string productJSON = 
        "{\n\t\"ProductID\": " + productId + "," +
            "\n\t\"ImageDetails\": " + "{" +
            "\n\t\t\"NumberOfBolts\": " + numberOfBolts + "," +
            "\n\t\t\"IsFaulty\": " + BoolToString(isFaulty) + "," +
            "\n\t\t\"numberOfFaults\": " + numberOfFaults +
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