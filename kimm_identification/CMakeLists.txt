cmake_minimum_required(VERSION 2.8.3)
project(kimm_identification)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure  
  geometry_msgs    
  rospy
  roslib
  tf  
)

find_package(Eigen3 REQUIRED)
find_package(kimm_object_estimation REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/ekf_param.cfg  
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS    
    dynamic_reconfigure    
    geometry_msgs    
    roscpp    
)
###########
## Build ##
###########
## library
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}    
  ${kimm_object_estimation_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_simulation src/id_simul.cpp)
target_link_libraries(${PROJECT_NAME}_simulation
  ${catkin_LIBRARIES}
  ${kimm_object_estimation_LIBRARIES}
  -lpthread
)
add_dependencies(${PROJECT_NAME}_simulation ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg
)

#############
## Install ##
#############
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dyros_mobile_manipulator_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
