cmake_minimum_required(VERSION 2.8.3)
project(asn1c)

################
## POST BUILD ##
################

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	install(FILES third_party_lib/libasn1c_x64.so DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RENAME libasn1c.so)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
	install(FILES third_party_lib/libasn1c_x86.so DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RENAME libasn1c.so)
endif()

