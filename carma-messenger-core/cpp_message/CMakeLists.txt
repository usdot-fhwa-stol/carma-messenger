cmake_minimum_required(VERSION 2.8.3)
project(cpp_message)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	cav_msgs
	j2735_msgs
	bondcpp
	roscpp
	carma_utils
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
)

###########
## Build ##
###########
include_directories(
	${catkin_INCLUDE_DIRS}
	include
	include/generated
)

add_library(testlib SHARED IMPORTED)

## Following is asn1c library install path outside the container, which is usually fixed as /opt/carma/install/
set(ASN1C_INSTALL_PATH "/opt/carma/install")

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../lib_asn1c/third_party_lib/libasn1c_x64.so DESTINATION ${ASN1C_INSTALL_PATH}/lib)
	file(RENAME ${ASN1C_INSTALL_PATH}/lib/libasn1c_x64.so ${ASN1C_INSTALL_PATH}/lib/libasn1c.so)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
	file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../lib_asn1c/third_party_lib/libasn1c_x86.so DESTINATION ${ASN1C_INSTALL_PATH}/lib)
	file(RENAME ${ASN1C_INSTALL_PATH}/lib/libasn1c_x86.so ${ASN1C_INSTALL_PATH}/lib/libasn1c.so)
endif()

set_target_properties(testlib PROPERTIES
  IMPORTED_LOCATION ${ASN1C_INSTALL_PATH}/lib/libasn1c.so
)

## Declare a C++ executable
add_executable(cpp_message_node
	src/main.cpp
	src/cpp_message.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(cpp_message_node testlib ${Boost_LIBRARIES} ${catkin_LIBRARIES} cpp_message_library)

add_library(cpp_message_library src/cpp_message.cpp 
            src/MobilityOperation_Message.cpp
			src/MobilityResponse_Message.cpp
			src/MobilityPath_Message.cpp
			src/MobilityRequest_Message.cpp
			src/BSM_Message.cpp)
add_dependencies(cpp_message_library ${catkin_EXPORTED_TARGETS} testlib)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(cpp_message_node ${catkin_EXPORTED_TARGETS} )

#############
## Install ##
#############

install(TARGETS cpp_message_node cpp_message_library
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
	launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gmock(${PROJECT_NAME}-test test/test_encode_decode.cpp
	test/test_MobilityOperations.cpp
	test/test_MobilityResponse.cpp
	test/test_MobilityPath.cpp
	test/test_MobilityRequest.cpp
	test/test_BSM.cpp
)
target_link_libraries(${PROJECT_NAME}-test cpp_message_library testlib ${catkin_LIBRARIES})