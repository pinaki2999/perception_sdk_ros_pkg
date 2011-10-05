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
)



# define sources lists for libraries (cpp files)
SET (CORE_LIBRARY_SOURCES
    ./lib/core/ColorSpaceConvertor
    ./lib/core/ColorBasedROIExtractorHSV
)

# compile libraries
ADD_LIBRARY(perception_sdk_core ${CORE_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(Perception_SDK_Core ${CORE_LIBRARY_LIBS})


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})