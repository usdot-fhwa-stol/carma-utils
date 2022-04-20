cmake_minimum_required(VERSION 3.5)
project(rapidjson)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Includes
# Currently this is a header only library so nothing else is needed here
ament_export_include_directories(include)

# Install
ament_auto_package()
