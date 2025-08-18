cmake_minimum_required(VERSION 3.8)
project(autoware_lgsvl_interface)

set(CMAKE_CXX_STANDARD 17)


# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(lgsvl_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

include_directories(
  include
)


set(${PROJECT_NAME}_SRC
  src/autoware_lgsvl_interface.cpp
)

# Create the executable node
ament_auto_add_executable(autoware_lgsvl_interface
  ${${PROJECT_NAME}_SRC}
)

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  launch
  params
  include
)
ament_target_dependencies(autoware_lgsvl_interface
  rclcpp
  autoware_auto_vehicle_msgs
  geometry_msgs
  lgsvl_msgs    # <-- 新增
)