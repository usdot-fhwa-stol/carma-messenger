
# Copyright (C) 2020-2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 2.8.3)
project(j2735_convertor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
set(DEPS
  cav_msgs
  cav_srvs
  j2735_msgs
  roscpp
  carma_utils
)
find_package(catkin REQUIRED COMPONENTS
  ${DEPS}
)

## System dependencies
find_package(Boost REQUIRED COMPONENTS system thread)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES j2735_conversions
   CATKIN_DEPENDS ${DEPS}
   DEPENDS Boost
)

###########
## Build ##
###########

## Headers
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare library
add_library(j2735_conversions
  src/bsm_convertor.cpp
  src/spat_convertor.cpp
  src/map_convertor.cpp
  src/control_message_convertor.cpp
  src/control_request_convertor.cpp
)

target_link_libraries(j2735_conversions ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(j2735_conversions ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(j2735_convertor_node
         src/main.cpp
         src/j2735_convertor.cpp)

target_link_libraries(j2735_convertor_node j2735_conversions ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(j2735_convertor_node j2735_conversions ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS j2735_convertor_node j2735_conversions
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############

catkin_add_gmock(${PROJECT_NAME}-test
 test/test_main.cpp
 test/control_message_test.cpp
 test/control_request_test.cpp
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
)

if(TARGET ${PROJECT_NAME}-test)
 target_link_libraries(${PROJECT_NAME}-test j2735_conversions ${catkin_LIBRARIES})
endif()
