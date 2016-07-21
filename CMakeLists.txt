cmake_minimum_required(VERSION 2.8.3)
project(fotonic_3dcamera)

find_package(catkin REQUIRED COMPONENTS 
    camera_info_manager
    roscpp 
    tf
    std_msgs
    sensor_msgs 
    image_transport
    cv_bridge
    
    )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#  std_msgs sensor_msgs
# )

catkin_package(
  CATKIN_DEPENDS 
    camera_info_manager
    roscpp 
    tf
    std_msgs
    sensor_msgs
    image_transport

#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}

)

message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" )
  set(FZ_API ${PROJECT_SOURCE_DIR}/lib/x86_64/libfz_api.so.1)
else()
  set(FZ_API ${PROJECT_SOURCE_DIR}/lib/i386/libfz_api.so.1)
endif()

## Declare a cpp executable
add_executable(fotonic_3dcamera_node src/fotonic_3dcamera_node.cpp src/camera.cpp)

target_link_libraries(fotonic_3dcamera_node 
  ${catkin_LIBRARIES} 
  ${FZ_API})

# Install fz_api library
add_custom_command( TARGET fotonic_3dcamera_node POST_BUILD COMMAND cp ${FZ_API} ${CATKIN_DEVEL_PREFIX}/lib)
