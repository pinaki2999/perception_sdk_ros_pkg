cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# add library directories (-L)
LINK_DIRECTORIES(
    /home/pinaki/hack-arena/BRICS_3D/lib/
)

# add include directories (-I)
INCLUDE_DIRECTORIES(
     /home/pinaki/hack-arena/BRICS_3D/src
     /home/pinaki/hack-arena/BRICS_3D/external
     ${PROJECT_SOURCE_DIR}/lib/core
     /home/pinaki/hack-arena/Others/libfreenect/include
     /home/pinaki/hack-arena/Others/libfreenect/wrappers/c_sync
     
)


ADD_SUBDIRECTORY(./lib) #compile everything in lib to create the libraries

rosbuild_add_executable(gopalColorBasedRoiExtractor src/ColorBasedRoiExtractionNode.cpp 
						src/examples/ColorBasedRoiExtractor)
target_link_libraries(gopalColorBasedRoiExtractor perception_sdk_core)
