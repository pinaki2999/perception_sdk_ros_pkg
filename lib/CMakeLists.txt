cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# define sources lists for libraries (cpp files)
SET (CORE_LIBRARY_SOURCES
    ./core/util/ColorSpaceConvertor
    ./core/filtering/IColorBasedROIExtractor
    ./core/filtering/ColorBasedROIExtractorHSV
    ./core/util/SimplePointCloudGeneratorCube
    ./core/util/PCLTypecaster
    ./core/depthPerception/kinectDataGrabber
    ./core/segmentation/EuclideanClusterExtraction
 #   ./core/features/Centroid3DEstimation
)

# compile libraries
ADD_LIBRARY(perception_sdk_core ${CORE_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(perception_sdk_core ${CORE_LIBRARY_LIBS} brics_3dpm_core)
