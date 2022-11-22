
# Copyright (C) 2022 LEIDOS.
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

cmake_minimum_required(VERSION 3.5)
project(cpp_message)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
find_package(asn1c REQUIRED)

carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Name build targets
set(node_exec cpp_message_node_exec)
set(node_lib cpp_message_node)

# Includes
include_directories(
	include
	include/generated       
)

ament_export_include_directories(include)

# Build
ament_auto_add_library(${node_lib} SHARED
        src/cpp_message.cpp
        src/MobilityOperation_Message.cpp
        src/EmergencyVehicleResponse_Message.cpp
        src/MobilityResponse_Message.cpp
        src/MobilityPath_Message.cpp
        src/MobilityRequest_Message.cpp
        src/BSM_Message.cpp
        src/SPAT_Message.cpp
        src/Map_Message.cpp
        src/PSM_Message.cpp
        )


add_library(testlib SHARED IMPORTED)

# Following is asn1c library install path outside the container, which is usually fixed as /opt/carma/install/
set(ASN1C_INSTALL_PATH ${asn1c_DIR}/../../../)

set_target_properties(testlib PROPERTIES
  IMPORTED_LOCATION ${ASN1C_INSTALL_PATH}/lib/libasn1c.so
)

target_link_libraries(${node_lib} testlib)

ament_auto_add_executable(${node_exec} 
        src/main.cpp 
)

# Register component
rclcpp_components_register_nodes(${node_lib} "cpp_message::Node")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_exec}
        ${node_lib}
)

# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_cpp_message_node 
                        test/test_MobilityOperations.cpp
                        test/test_EmergencyVehicleResponse.cpp
                        test/test_MobilityResponse.cpp
                        test/test_MobilityPath.cpp
                        test/test_MobilityRequest.cpp
                        test/test_BSM.cpp
                        test/test_MapMessage.cpp
                        test/test_SPAT.cpp
                        test/test_encode_decode.cpp
                        test/test_PSM.cpp
                        test/node_test.cpp
                        )

  ament_target_dependencies(test_cpp_message_node ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_cpp_message_node ${node_lib})

endif()

# Install
ament_auto_package(
        INSTALL_TO_SHARE launch
)
