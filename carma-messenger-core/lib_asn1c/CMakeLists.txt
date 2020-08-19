cmake_minimum_required(VERSION 2.8.3)
project(asn1c)

################
## POST BUILD ##
################




if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	install(FILES third_party_lib/libasn1c_x64.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib RENAME libasn1c.so)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
	install(FILES third_party_lib/libasn1c_x86.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib RENAME libasn1c.so)
endif()
