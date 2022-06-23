cmake_minimum_required(VERSION 2.8.3)
project(steer_bot_control)



if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to RelWithDebInfo as none was specified.")
  set(CMAKE_BUILD_TYPE
      "RelWithDebInfo"
      CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_BUILD_TYPE)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             nav_msgs
             roscpp
             roslaunch
             tf2_ros
             tf2)

catkin_package(
    CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    roslaunch
    tf2_ros
    tf2)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
add_dependencies(tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})


install(
  TARGETS tf_broadcaster
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    roslaunch_add_file_check(launch)
endif()
