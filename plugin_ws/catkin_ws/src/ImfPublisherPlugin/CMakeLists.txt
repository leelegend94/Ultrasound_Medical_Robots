cmake_minimum_required(VERSION 3.3.0)
message(${CMAKE_VERSION})
message("ImFusion data publisher(ROS)")
project(imf_publisher_plugin)

include(cmake/Macros.cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# JZL
set(ROS_DISTRO "melodic" CACHE STRING "ROS distribution name (e.g. melodic)")
set(ROS_WORKSPACE_ROOT "/opt/ros/${ROS_DISTRO}" CACHE STRING "Root of the ROS workspace to be sourced (e.g. /opt/ros/melodic)")

message(STATUS "> Configuring ${PROJECT_NAME} for ROS ${ROS_DISTRO}")
if (EXISTS ${ROS_WORKSPACE_ROOT})
    message(STATUS "> Sourcing ROS workspace at ${ROS_WORKSPACE_ROOT}")
else()
    message(FATAL_ERROR "Could not find a ROS workspace root at ${ROS_WORKSPACE_ROOT}")
endif()

list(APPEND CMAKE_PREFIX_PATH ${ROS_WORKSPACE_ROOT})

#find_package(ImFusionRobotics ImFusionROS REQUIRED)
find_package(ImFusionLib COMPONENTS ImFusionRobotics ImFusionROS CephasonicsPlugin REQUIRED)
#JZL

find_package(ImFusionLib REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

foreach(plugin ${Qt5Network_PLUGINS})
  get_target_property(_loc ${plugin} LOCATION)
  message("Plugin ${plugin} is at location ${_loc}")
endforeach()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  CFG_EXTRAS definitions.cmake
)


include("cmake/definitions.cmake")
file(GLOB PLUGIN_HEADERS "include/*.h" "include/imf_publisher_plugin/*.h")

set(PLUGIN_SOURCES
  src/plugin.cpp
  src/factory.cpp
  src/algorithm.cpp
  src/controller.cpp
  src/controller.ui
)

add_library(${PROJECT_NAME} SHARED ${PLUGIN_SOURCES} ${PLUGIN_FORMS} ${PLUGIN_HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${IMFUSION_SDK_INCLUDE}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  )

target_link_libraries (${PROJECT_NAME} PRIVATE
	ImFusionLib
	ImFusionStream
	ImFusionUS
        ImFusionRobotics #JZL
        ImFusionROS  #JZL
        CephasonicsPlugin   #JZL
	${catkin_LIBRARIES}
        Qt5::Widgets
)

#JZL
set(OldPythonExecutable ${PYTHON_EXECUTABLE})
set(PYTHON_EXECUTABLE /usr/bin/python2)
include(cmake/Macros.cmake)
imfusion_ros_add_catkin_dependencies(${PROJECT_NAME} COMPONENTS tf2_ros tf2_eigen cv_bridge)
set(PYTHON_EXECUTABLE ${OldPythonExecutable})

if (UNIX)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        target_link_libraries(${PROJECT_NAME} PRIVATE "-Wl,--allow-multiple-definition")
#        target_link_options(${PROJECT_NAME} PRIVATE
#                "-Wl,--disable-new-dtags"		# Always use rpath instead of runpath, since it is used recursively. See http://blog.qt.io/blog/2011/10/28/rpath-and-runpath for an explanation.
#                )
    endif()

    # set the linker rpath so that libraries are found even if not install in system location (e.g. /usr)
    # see https://cmake.org/Wiki/CMake_RPATH_handling for more information
    set(RPATH "${RPATH}:$ORIGIN:${ROS_WORKSPACE_ROOT}/lib")
    if (NOT "${RPATH}" STREQUAL "")
        set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${RPATH}")
    endif()
endif()

#JZL


# Define output target directories
#imfusion_set_common_target_properties()

###### INSTALLATION
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
  EXPORT "${PROJECT_NAME}Targets"
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
