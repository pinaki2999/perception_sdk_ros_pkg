cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# add library directories (-L)
LINK_DIRECTORIES(
    /home/pinaki/hack-arena/BRICS_3D/lib/
)

# add include directories (-I)
INCLUDE_DIRECTORIES(
     /home/pinaki/hack-arena/BRICS_3D/src
     /home/pinaki/hack-arena/BRICS_3D/external
     ./core
)



# define sources lists for libraries (cpp files)
SET (CORE_LIBRARY_SOURCES
    ./core/util/ColorSpaceConvertor
    ./core/filtering/ColorBasedROIExtractorHSV
    ./core/util/SimplePointCloudGeneratorCube
    ./core/util/PCLTypecaster
)

# compile libraries
ADD_LIBRARY(perception_sdk_core ${CORE_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(Perception_SDK_Core ${CORE_LIBRARY_LIBS})
