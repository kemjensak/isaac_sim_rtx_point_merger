cmake_minimum_required(VERSION 3.8)
project(isaac_sim_rtx_point_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  nav_msgs
  pcl_ros
  std_msgs
  geometry_msgs
  sensor_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
#
add_executable(merger src/merger.cpp)
ament_target_dependencies(merger ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_include_directories(merger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(merger PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS merger
  DESTINATION lib/${PROJECT_NAME})


ament_package()