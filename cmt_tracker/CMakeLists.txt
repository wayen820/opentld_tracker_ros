cmake_minimum_required(VERSION 2.8.3)
project(cmt_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs image_transport mycommbase  cv_bridge message_generation)

find_package(OpenCV REQUIRED)


generate_messages(DEPENDENCIES sensor_msgs mycommbase)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# generate_messages(
#    DEPENDENCIES
#    std_msgs  # Or other packages containing msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cmt_tracker
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
     ${catkin_INCLUDE_DIRS}
     ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(cmt_tracker
#   src/${PROJECT_NAME}/cmt_tracker.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(cmt_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(cmt src/common.cpp src/gui.cpp src/main.cpp
    src/CMT.cpp src/Consensus.cpp src/Fusion.cpp src/Matcher.cpp src/Tracker.cpp
    src/fastcluster/fastcluster.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(cmt_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(cmt
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)
add_executable(cmt_tracker_node src/common.cpp src/gui.cpp src/tracker_node.cpp
    src/CMT.cpp src/Consensus.cpp src/Fusion.cpp src/Matcher.cpp src/Tracker.cpp
    src/fastcluster/fastcluster.cpp)
target_link_libraries(cmt_tracker_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS cmt_tracker_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cmt_tracker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
