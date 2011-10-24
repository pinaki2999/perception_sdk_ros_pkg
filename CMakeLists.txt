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


# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# add library directories (-L)
LINK_DIRECTORIES(
    /home/pinaki/hack-arena/BRICS_3D/lib/
)

# add include directories (-I)
INCLUDE_DIRECTORIES(
     /home/pinaki/hack-arena/BRICS_3D/src
     /home/pinaki/hack-arena/BRICS_3D/external
)


#ADD_SUBDIRECTORY(./lib) #compile everything in lib to create the libraries

#executable for color based region of interest extraction
rosbuild_add_executable(colorBasedRoiExtractor src/ColorBasedRoiExtractionNode.cpp 
						src/examples/ColorBasedRoiExtractor)
target_link_libraries(colorBasedRoiExtractor brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)

#executable for hsv configuration finder
rosbuild_add_executable(hsvLimitsFinder src/HSVLimitsFinderNode.cpp 
						src/examples/ColorBasedRoiExtractor)
target_link_libraries(hsvLimitsFinder  brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)

#executable for object cluster extraction
#rosbuild_add_executable(objectClusterExtractor src/ObjectClusterExtractionNode.cpp 
#						src/examples/EuclideanClusterExtractor)
#target_link_libraries(objectClusterExtractor  brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)
