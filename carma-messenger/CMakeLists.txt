cmake_minimum_required(VERSION 2.8.3)
project(carma-messenger)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message
  j2735_convertor
)

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
