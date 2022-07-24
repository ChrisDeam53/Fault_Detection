cmake_minimum_required(VERSION 3.0.0)
project(Polk VERSION 0.1.0)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 17)

# include(CTest)
include(FetchContent)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

################################################################
# PTHREADS
################################################################
# set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
# set(THREADS_PREFER_PTHREAD_FLAG TRUE)
# find_package(Threads REQUIRED)

################################################################
# GLog
################################################################
find_package (glog 0.6.0 REQUIRED)

################################################################
# Polk
# Sets the main executable file. Polk. Run "./Polk" in "/build" -> To execute the binary file.
# Currently, to execute OpenCV properly, you need to run "./Polk" in the top-level directory.
################################################################
add_executable( ${PROJECT_NAME} src/main.cpp src/utils/ImageViewer.hh src/utils/ImageViewer.cpp )

################################################################
# GTest
################################################################

# include(FetchContent)
# FetchContent_Declare(
#   googletest
#   GIT_REPOSITORY https://github.com/google/googletest.git
#   GIT_TAG release-1.12.1
# )
# # For Windows: Prevent overriding the parent project's compiler/linker settings
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# FetchContent_MakeAvailable(googletest)

# enable_testing()

# add_executable(UnitTests test/ImageViewerTest.cpp)

# target_link_libraries(UnitTests GTest::gtest_main)

# include(GoogleTest)
# gtest_discover_tests(UnitTests)

################################################################
# Specifies libs.
# Polk executable.
################################################################
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} glog::glog )

################################################################
# Output Directories
################################################################
# Sets the archive files to be moved into /lib
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_NAME}/lib )
# Sets the lib files to be moved into /lib
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_NAME}/lib )
# Sets the bin files to be moved into /bin
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_NAME}/build )

################################################################
# Setting Project Name & Version
################################################################
set( CPACK_PROJECT_NAME ${PROJECT_NAME} )
set( CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
