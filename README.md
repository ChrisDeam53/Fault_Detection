# Fault Detection Program.
Author: Christopher Deam

# What is the project?
My Final year project, inspired by the Poka Yoke system designed in Japan.

I will be attempting to create a piece of software that can be used as a device to find faults and errors on an object.
The idea is that this product would be used to find faults in various shop floor products - A static camera would be positioned above a shop floor production line. An image would be taken for each object that passes through.

The system will then flag up and errors and faults to the User, who can then deal with the object appropriately.

# Setup Guide:

1) Install WSL - https://docs.microsoft.com/en-us/windows/wsl/install
2) sudo apt update
3) Install OPENCV
- sudo apt install libopencv-dev python3-opencv
- To check that it has installed:
- python3 -c "import cv2; print(cv2.__version__)"
4) Install CMake & Make (Can use VSCode Extensions) Or:
- CMake Install Guide: https://cmake.org/install/
- Make Install Guide: https://linuxhint.com/install-make-ubuntu/
5) VSCode Install:
- https://code.visualstudio.com/
- sudo apt install g++ gdb make ninja-build rsync zip
- sudo apt-get update
- sudo apt-get install wget ca-certificates
6) VSCode Extensions Active in ``WSL:Ubuntu - Installed``:
- C/C++
- C/C++ Extension Pack
- CMake
- CMake Tools
- Docker (Not in use)
- Docker Compose (Not in use)
- Docker Explorer (Not in use)
- Doxygen Documentation Generator
- Git Blame
- Git Extension Pack
- Git History
- gitignore
- GitLens -- Git supercharged
- Makefile Tools
- Open in GitHub, Bitbucket, Gitlab, VisualStudio.com !
7) GLOG:
- git clone https://github.com/google/glog.git
- cd glog
- cmake -S .-B build -G "Unix Makefiles"
- cmake --build build
- cmake --build build --target test
- sudo cmake --build build --target install


# Additional Notes:

Start Guide:

NOTE: GTest in CMake causes some issues when attempting to build the program with ``cmake ..`` and ``make``.

You may be required to comment out any/all GTest CMake entries to allow for the build to compile.

Whilst in ``Fault_Detection/PolkSystem``:

``./createBuildFiles.sh`` - Runs the Polk script in DEBUG mode (which can be found in the /build directory)

Click ``Build`` -> Cog Icon at the bottom of VSCode screen to build the project if necessary

Click ``Debug`` -> At the bottom of VSCode screen to debug the project if necessary.

``./runUnitTests.sh`` - Runs the UnitTests script found in the /build directory if necessary.

