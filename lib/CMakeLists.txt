cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# define sources lists for libraries (cpp files)
SET (CORE_LIBRARY_SOURCES
    ./brics_3d_sdk/util/ColorSpaceConvertor
    ./brics_3d_sdk/filtering/IColorBasedROIExtractor
    ./brics_3d_sdk/filtering/ColorBasedROIExtractorHSV
    ./brics_3d_sdk/util/SimplePointCloudGeneratorCube
    ./brics_3d_sdk/util/PCLTypecaster
    ./brics_3d_sdk/segmentation/EuclideanClusterExtraction
    ./brics_3d_sdk/features/Centroid3DEstimation
)

# compile libraries
ADD_LIBRARY(perception_sdk_core ${CORE_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(perception_sdk_core ${CORE_LIBRARY_LIBS} brics_3dpm_core)
