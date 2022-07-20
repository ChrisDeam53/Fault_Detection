#include <iostream>

#include "utils/ImageViewer.hh"

int main(int, char**)
{
    utils::ImageViewer viewer;
    viewer.OpenImage();
    std::cout << "Hello, world!\n";
    return 0;
}