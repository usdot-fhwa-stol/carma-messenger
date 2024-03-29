
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

cmake_minimum_required(VERSION 3.5)
project(carma-messenger)

find_package(carma_cmake_common REQUIRED)
carma_package()

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1) # ROS 1

  find_package(catkin REQUIRED)

  ###################################
  ## catkin specific configuration ##
  ###################################
  catkin_package()

  #############
  ## Install ##
  #############

  ## Mark other files for installation (e.g. launch and bag files, etc.)
  install(DIRECTORY
    launch
    log-config
    scripts
    ui
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

else() # ROS 2

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()


  ament_auto_package(
          INSTALL_TO_SHARE launch log-config scripts ui
  )

endif()

