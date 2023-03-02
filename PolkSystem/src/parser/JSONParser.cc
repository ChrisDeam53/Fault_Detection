#include <string>

#include "JSONParser.hh"

using namespace parser;

void JSONParser::CreateJSONFile(std::string fileName, std::string textString)
{
    // Create and open a text file
    std::ofstream jsonFile(fileName + ".json");

    jsonFile << textString;

    // Close the file
    jsonFile.close();

}