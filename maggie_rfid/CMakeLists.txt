cmake_minimum_required(VERSION 2.8.3)
project(maggie_rfid)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS maggie_rfid_msgs maggie_rfid_drivers roscpp)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
    INCLUDE_DIRS src
    CATKIN_DEPENDS roscpp
    DEPENDS maggie_rfid_msgs maggie_rfid_drivers
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include src
    ${catkin_INCLUDE_DIRS}
    ${maggie_rfid_drivers_INCLUDE_DIRS}
    /usr/local/include/rfid
)

ADD_SUBDIRECTORY(src)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
