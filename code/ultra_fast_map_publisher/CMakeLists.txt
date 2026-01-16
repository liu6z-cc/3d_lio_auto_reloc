cmake_minimum_required(VERSION 3.0.2)
project(ultra_fast_map_publisher)

# ===================== 核心配置（适配Melodic） =====================
# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 优化编译参数：平衡编译速度和运行性能（去掉耗时的-flto，O3降为O2）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=native -Wall -Wextra")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")

# 显式设置编译模式为Release（Melodic必须显式指定）
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 统一输出目录（Melodic推荐写法）
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# ===================== 查找依赖（精简PCL组件） =====================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf
  std_msgs
  nav_msgs
)

# 只链接PCL核心组件（减少编译/链接耗时）
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# 启用OpenMP（Melodic兼容写法）
if(OpenMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# ===================== 包含/链接目录 =====================
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

# 禁用PCL冗余定义
add_definitions(${PCL_DEFINITIONS})

# ===================== 编译可执行文件 =====================
add_executable(${PROJECT_NAME}
  src/ultra_fast_map_publisher.cpp
)

# 显式设置输出目录（Melodic兼容）
set_target_properties(${PROJECT_NAME} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
)

# 链接库（只链接用到的PCL组件，避免冗余）
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

# ===================== Catkin包配置（Melodic必须） =====================
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp pcl_ros pcl_conversions sensor_msgs tf std_msgs nav_msgs
  DEPENDS PCL Eigen3 OpenMP
)

# ===================== 安装规则（修复核心错误） =====================
# 错误1修复：显式指定RUNTIME DESTINATION（Melodic要求绝对/明确路径）
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  # Melodic默认路径：lib/ultra_fast_map_publisher
)

# 安装头文件（显式指定DESTINATION）
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# 错误2修复：严谨的目录判断 + 显式DESTINATION
# 安装launch文件（仅当目录存在且非空时）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch" AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  file(GLOB LAUNCH_FILES "${CMAKE_CURRENT_SOURCE_DIR}/launch/*")
  if(LAUNCH_FILES)
    install(DIRECTORY launch/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
  endif()
endif()

# 安装rviz文件（仅当目录存在且非空时）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz" AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  file(GLOB RVIZ_FILES "${CMAKE_CURRENT_SOURCE_DIR}/rviz/*")
  if(RVIZ_FILES)
    install(DIRECTORY rviz/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    )
  endif()
endif()
