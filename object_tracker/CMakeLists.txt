cmake_minimum_required(VERSION 2.8.3)
project(object_tracker)

include(../dlib-19.4/dlib/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs image_transport mycommbase  cv_bridge)
find_package(OpenCV REQUIRED)

catkin_package()
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(object_tracker_node src/video_tracking_ex.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(object_tracker_node
    ${catkin_LIBRARIES}
    dlib::dlib
    ${OpenCV_LIBRARIES}
 )
#install(TARGETS object_tracker_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Declare a C++ executable
add_executable(object_tracker_node2 src/image_tracking_ex.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(object_tracker_node2
    ${catkin_LIBRARIES}
    dlib::dlib
    ${OpenCV_LIBRARIES}
 )
## Declare a C++ executable
add_executable(dlib_tracker src/dlib_tracker.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dlib_tracker
    ${catkin_LIBRARIES}
    dlib::dlib
    ${OpenCV_LIBRARIES}
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
# install(TARGETS object_tracker_node object_tracker_node2
#   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_object_tracker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
