cmake_minimum_required(VERSION 3.0.2)
project(bz_plan)

# set(CMAKE_VERBOSE_MAKEFILE "false")
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# elseif(COMPILER_SUPPORTS_CXX0X)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# else()
#         message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()

# set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/sdf_tools)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  sensor_msgs
  visualization_msgs
  quadrotor_msgs
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
generate_dynamic_reconfigure_options( cfg/dynamicParam.cfg )


###################################
## catkin specific configuration ##
###################################
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES bezier_planner
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
link_directories( ${PROJECT_SOURCE_DIR}/third_party/mosek/lib/mosek8_1 )
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/mosek/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/sdf_tools/include)

include_directories(
  include 
  SYSTEM 
  third_party
  ${catkin_INCLUDE_DIRS} 
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -Wall") #c++编译器设置 -Wextra -Werror

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/bz_plan_node.cpp)

#节点1-路径规划器： plan_node 建图+前端路径搜索+后端轨迹优化
add_executable(
  bz_plan_node 
    src/plan_node.cpp
    src/bezier_base.cpp
    src/corridor_generator.cpp
    src/trajectory_generator.cpp
    src/a_star.cpp
    third_party/fast_methods/console/console.cpp
    third_party/fast_methods/fm/fmdata/fmcell.cpp
    third_party/fast_methods/ndgridmap/cell.cpp
)
target_link_libraries(bz_plan_node ${catkin_LIBRARIES} sdf_tools ${PCL_LIBRARIES} mosek64)

#节点2-轨迹服务器：b_traj_server，跟踪轨迹并生成相应的控制命令
add_executable (
  traj_server 
    src/traj_server.cpp 
    src/bezier_base.cpp
)
target_link_libraries( traj_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})


#添加动态参数配置 2，需指定添加到哪个节点上
add_dependencies(bz_plan_node ${PROJECT_NAME}_gencfg)