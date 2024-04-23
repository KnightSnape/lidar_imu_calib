cmake_minimum_required(VERSION 3.8)
project(lidar_imu_calib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ndt_omp_ros2 REQUIRED) 

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(pcl_ros REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake")
message(STATUS "CMAKE_PREFIX_PATH:" ${CMAKE_PREFIX_PATH})
message(STATUS "LD_LIBRARY_PATH:" ${LD_LIBRARY_PATH})
message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

add_library(
  calibration_lib
  src/lib/calibExRLidar2Imu.cpp
)

target_link_libraries(
  calibration_lib
  ${PCL_LIBRARIES}
)

ament_target_dependencies(
  calibration_lib
  rclcpp
  sensor_msgs
  tf2_ros
  geometry_msgs
  ndt_omp_ros2
)

add_executable(
  lidar_imu_calib
  src/calib_exR_lidar2imu_node.cpp
)

target_link_libraries(
  lidar_imu_calib
  ${PCL_LIBRARIES}
  calibration_lib
)

ament_target_dependencies(
  lidar_imu_calib
  rclcpp
  sensor_msgs
  tf2_ros
  geometry_msgs
  ndt_omp_ros2
)

install(
  TARGETS
  lidar_imu_calib
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
  lidar_imu_calib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
