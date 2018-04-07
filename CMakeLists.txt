cmake_minimum_required(VERSION 3.5)
project(twist_vesc)

set(NODE_NAME twist_vesc_node)
set(NODELET_NAME twist_vesc_nodelet)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        genmsg
        geometry_msgs
        tf_conversions
        cmake_modules
        sensor_msgs
		ackermann_msgs
        )

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)

catkin_package(
        INCLUDE_DIRS  /home/nvidia/ros_catkin_ws/install_isolated/include
        LIBRARIES ${NODE_NAME}
        CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
		ackermann_msgs
        tf_conversions
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	/home/nvidia/ros_catkin_ws/install_isolated/include
	${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

set(SOURCE_FILES src/twist_vesc_node.cpp)

# node executable
add_executable(${NODE_NAME} ${SOURCE_FILES})

add_dependencies(${NODE_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${NODE_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(TARGETS ${NODELET_NAME}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

install(DIRECTORY include/${NODE_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
#install(FILES ${NODELET_NAME}.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

