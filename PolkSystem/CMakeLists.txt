cmake_minimum_required(VERSION 3.0.0)
project(Polk VERSION 0.1.0)

include(CTest)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
FetchContent_MakeAvailable(googletest)

enable_testing()

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# (Not currently in use)
file( GLOB UTILS_SOURCES src/utils/ImageViewer.cpp) 

# (Not currently in use)
file( GLOB UTILS_HEADERS src/utils/ImageViewer.hh)

# (Not currently in use)
add_library( UTILS_LIB ${UTILS_SOURCES} ${UTILS_HEADERS} )

# Sets the main executable file. Polk. Run "./Polk" in "/build" -> To execute the binary file.
# Currently, to execute OpenCV properly, you need to run "./Polk" in the top-level directory.
add_executable( ${PROJECT_NAME} src/main.cpp src/utils/ImageViewer.hh src/utils/ImageViewer.cpp )

# Sets the test binary for ImageViewerTest.
add_executable( ImageViewerTest test/ImageViewerTest.cpp )

# Links ImageViewerTest to Google Test.
target_link_libraries( ImageViewerTest GTest::gtest_main )

include(GoogleTest)
gtest_discover_tests( ImageViewerTest )

# Sets Util files. (Not currently in use)
set( UTILS src/utils/ImageViewer.hh src/utils/ImageViewer.c )

# Specifies libs.
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

# Sets the archive files to be moved into /lib
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_NAME}/lib )
# Sets the lib files to be moved into /lib
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_NAME}/lib )
# Sets the bin files to be moved into /bin
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_NAME}/build )

set( CPACK_PROJECT_NAME ${PROJECT_NAME} )
set( CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
