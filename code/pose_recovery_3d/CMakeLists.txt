cmake_minimum_required(VERSION 3.0.2)
project(pose_recovery_3d)

## 编译选项
add_compile_options(-std=c++14)

## 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
)

## catkin配置
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf tf2_ros
)

## 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 可执行文件
add_executable(pose_recovery_3d_node src/pose_recovery_3d_node.cpp)
target_link_libraries(pose_recovery_3d_node ${catkin_LIBRARIES})

add_executable(auto_relocalization_3d_node src/auto_relocalization_3d_node.cpp)
target_link_libraries(auto_relocalization_3d_node ${catkin_LIBRARIES})

## 安装
install(TARGETS pose_recovery_3d_node auto_relocalization_3d_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
