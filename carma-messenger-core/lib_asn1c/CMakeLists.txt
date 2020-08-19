cmake_minimum_required(VERSION 2.8.3)
project(asn1c)

################
## POST BUILD ##
################



#message(STATUS "The copy target path is: " ${CATKIN_INSTALL_PREFIX}/lib)
#message(STATUS "ASN1C would install to folder: " ${CMAKE_LIB_PREFIX}/lib)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	#message(STATUS "Install prefix is: " ${CMAKE_INSTALL_PREFIX}/lib)
	install(FILES third_party_lib/libasn1c_x64.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib RENAME libasn1c.so)
	#file(RENAME ${CMAKE_INSTALL_PREFIX}/lib/libasn1c_x64.so ${CMAKE_INSTALL_PREFIX}/lib/libasn1c.so)
	#install(FILES ${CATKIN_INSTALL_PREFIX}/lib/libasn1c.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
	install(FILES third_party_lib/libasn1c_x86.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib RENAME libasn1c.so)
	#file(COPY third_party_lib/libasn1c_x86.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)	
	#file(RENAME ${CMAKE_INSTALL_PREFIX}/lib/libasn1c_x86.so ${CMAKE_INSTALL_PREFIX}/lib/libasn1c.so)	
	#install(FILES ${CATKIN_INSTALL_PREFIX}/lib/libasn1c.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)
endif()
