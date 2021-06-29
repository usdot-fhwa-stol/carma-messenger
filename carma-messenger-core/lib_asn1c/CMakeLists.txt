
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

